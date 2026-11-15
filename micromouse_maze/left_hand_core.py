#!/usr/bin/env python3
"""左墙跟随控制器核心（纯算法，不含任何 ROS 依赖）。

设计要点
--------
1. 决策层（左手法则）：机器人不保存地图，只靠当前时刻 360° 雷达判断
   "左/前/右"三个方向是否有墙，按 左手 → 直行 → 右手 → 掉头 的优先级选向。
   这就是电脑鼠竞赛里的经典左手法则。
2. 执行层（分三段）：
   DRIVE  在通道内沿目标格心行驶：航向由里程计闭环，横向偏差由左右墙
          等距约束修正（等距 = 通道居中，抑制里程计横向漂移）。
   TURN   在格心原地转向到目标朝向（里程计航向闭环，不是"转到看见墙为止"，
          避免窄通道里过冲撞墙）。
   EXIT   进入终点单元且朝北时，直行穿出北墙缺口，越界即判定出迷宫。
3. 参数全部走 ros2 参数，离线仿真脚本复用同一份类，保证"仿真整定 →
   实机/在环"一致，避免两套逻辑各调各的。

坐标约定：世界坐标，x 东、y 北；yaw 逆时针为正；单元格心见 maze_model。
"""

import math

# 方向：0=+x(东) 1=+y(北) 2=-x(西) 3=-y(南)
DX = (1, 0, -1, 0)
DY = (0, 1, 0, -1)
DIR_NAME = ('E', 'N', 'W', 'S')


def wrap(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def clamp(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)


# ---------------------------------------------------------------- 雷达分区
def make_sectors(angle_min, angle_inc, n):
    """按角度区间预计算索引区间，避免每帧重复算 arcsin。"""
    def idx(a_deg):
        a = math.radians(a_deg)
        k = int(round((a - angle_min) / angle_inc))
        return max(0, min(n - 1, k))

    # (起始索引, 结束索引)，区间按逆时针（角度递增）排列
    return {
        'front': (idx(-18), idx(18)),
        'left': (idx(72), idx(108)),
        'right': (idx(-108), idx(-72)),
    }


def sector_min(ranges, span):
    """取角度区间的最近距离，自动处理跨 ±180° 的环绕。"""
    i0, i1 = span
    n = len(ranges)
    if i0 <= i1:
        seg = ranges[i0:i1 + 1]
    else:
        seg = list(ranges[i0:]) + list(ranges[:i1 + 1])
    vals = [r for r in seg if r == r and r > 0.0]      # 过滤 NaN / 0
    return min(vals) if vals else 8.0


class LeftHandCore:
    """左手法则 + 通道居中 + 格心转向。"""

    DRIVE = 'DRIVE'
    TURN = 'TURN'
    EXIT = 'EXIT'
    DONE = 'DONE'
    TIMEOUT = 'TIMEOUT'

    def __init__(self, **kw):
        # 迷宫尺度（与 world 一致）
        self.cell = kw.get('cell', 0.60)
        self.x0 = kw.get('x0', 0.0)                       # 格心原点（网格对称时为 0）
        self.y0 = kw.get('y0', 0.0)
        self.entry = kw.get('entry', (0.0, 0.0))          # 起点格心（世界坐标）
        self.exit_cell = kw.get('exit_cell', (0.0, 0.0))
        self.half = kw.get('half', 1.50)                  # 外墙内表面到原点距离
        # 速度
        self.v_nom = kw.get('v_nom', 0.20)
        self.v_slow = kw.get('v_slow', 0.09)
        self.w_max = kw.get('w_max', 1.20)
        self.turn_w = kw.get('turn_w', 0.90)
        # 控制增益
        self.kp_head = kw.get('kp_head', 2.00)            # 航向环
        self.kp_lat = kw.get('kp_lat', 1.20)              # 居中横向环
        self.lat_gain = kw.get('lat_gain', 0.45)          # 横向修正注入比例
        # 感知
        self.wall_th = kw.get('wall_th', 0.50)            # < 该值认为该方向有墙
        self.arrive_tol = kw.get('arrive_tol', 0.04)      # 到达格心判据
        self.slow_dist = kw.get('slow_dist', 0.30)        # 接近格心降速距离
        self.timeout = kw.get('timeout', 180.0)

        self.phase = self.DRIVE
        self.target = self.entry                          # 当前目标格心
        self.goal_dir = 1                                 # 当前行进方向
        self.yaw_des = 0.0
        self.t = 0.0
        self.path_cells = []
        self.events = []
        self.turn_count = 0
        self.distance = 0.0
        self._prev_xy = None
        self.exit_start = None

    # ---------------- 对外接口 ----------------
    def step(self, scan, pose, dt):
        """scan: dict(ranges, angle_min, angle_inc) 或 None；pose: (x, y, yaw)。

        返回 (v, w, info)；info 含 phase / 事件文本，供日志打印。
        """
        self.t += dt
        x, y, yaw = pose
        if self._prev_xy is not None:
            self.distance += math.hypot(x - self._prev_xy[0], y - self._prev_xy[1])
        self._prev_xy = (x, y)

        info = ''
        if self.t > self.timeout:
            self.phase = self.TIMEOUT
            return 0.0, 0.0, 'TIMEOUT'

        if self.phase in (self.DONE, self.TIMEOUT):
            return 0.0, 0.0, self.phase

        if self.phase == self.EXIT:
            # 穿出北墙缺口后直行 0.55 m 停车
            tx, ty = self.exit_start[0], self.exit_start[1] + 0.55
            d = math.hypot(tx - x, ty - y)
            if d < 0.05 or y > self.half + 0.55:
                self.phase = self.DONE
                return 0.0, 0.0, 'DONE'
            yaw_des = math.atan2(ty - y, tx - x)
            w = clamp(self.kp_head * wrap(yaw_des - yaw), -self.w_max, self.w_max)
            v = clamp(0.6 * d + 0.05, 0.05, self.v_nom)
            return v, w, 'EXIT'

        if self.phase == self.TURN:
            err = wrap(self.yaw_des - yaw)
            w = clamp(2.2 * err, -self.turn_w, self.turn_w)
            if abs(err) < 0.035:
                self.phase = self.DRIVE      # target 已在 _decide 中设好，勿重复前进
                self.events.append('t=%.1f 转向完成 -> %s'
                                   % (self.t, DIR_NAME[self.goal_dir]))
                return 0.0, 0.0, 'TURN_DONE'
            # 小角度时给一点前进量，避免原地磨地
            return 0.0, w, 'TURN'

        # ---------------- DRIVE ----------------
        tx, ty = self.target
        d = math.hypot(tx - x, ty - y)
        s_along = (tx - x) * math.cos(yaw) + (ty - y) * math.sin(yaw)
        if d < self.arrive_tol or s_along < 0.01:
            return self._decide(pose, scan)

        yaw_des = math.atan2(ty - y, tx - x)
        err = wrap(yaw_des - yaw)
        w = clamp(self.kp_head * err, -self.w_max, self.w_max)

        # 横向修正：左右等距即居中；只在两侧都能看到墙时启用
        if scan is not None:
            sec = scan['sectors']
            left = sector_min(scan['ranges'], sec['left'])
            right = sector_min(scan['ranges'], sec['right'])
            if left < 0.55 and right < 0.55:
                e_lat = (left - right) * 0.5
                w = clamp(w + self.lat_gain * self.kp_lat * e_lat,
                          -self.w_max, self.w_max)

        v = self.v_nom if d > self.slow_dist else \
            clamp(self.v_slow + (self.v_nom - self.v_slow) * d / self.slow_dist,
                  0.05, self.v_nom)
        return v, clamp(w, -self.w_max, self.w_max), 'DRIVE'

    # ---------------- 内部 ----------------
    def _next_cell(self):
        x, y = self.target
        return (x + DX[self.goal_dir] * self.cell,
                y + DY[self.goal_dir] * self.cell)

    def _decide(self, pose, scan):
        """到达格心：按左手法则选方向。"""
        x, y, yaw = pose
        self.goal_dir = self._quantize(yaw)
        if scan is None:
            self.phase = self.TURN
            self.yaw_des = yaw
            return 0.0, 0.0, 'NO_SCAN'

        sec = scan['sectors']
        front = sector_min(scan['ranges'], sec['front'])
        left = sector_min(scan['ranges'], sec['left'])
        right = sector_min(scan['ranges'], sec['right'])

        d = self.goal_dir
        cx0, cy0 = self._snap_center(x, y)
        # 出口优先级最高：已走到终点格心、朝北且北面开阔 -> 直接出迷宫
        if (abs(cx0 - self.exit_cell[0]) < 1e-6
                and abs(cy0 - self.exit_cell[1]) < 1e-6
                and d == 1 and front > self.wall_th):
            self.events.append('t=%.1f 到达终点格心 -> EXIT' % self.t)
            self.exit_start = (cx0, cy0)
            self.phase = self.EXIT
            return 0.0, 0.0, 'EXIT_START'

        cands = [((d + 1) % 4, 'L'), (d, 'F'), ((d - 1) % 4, 'R'), ((d + 2) % 4, 'B')]
        dist = {(d + 1) % 4: left, d: front, (d - 1) % 4: right, (d + 2) % 4: 8.0}
        cx, cy = self._snap_center(x, y)
        chosen, tag = None, ''
        for cd, nm in cands:
            # 越界方向（走出迷宫边界）只允许北墙出口
            if not self._inside_from(cx, cy, cd):
                if cd == 1 and self._at_exit(cx, cy):
                    chosen, tag = 1, 'EXIT'
                    break
                continue
            if dist[cd] > self.wall_th:
                chosen, tag = cd, nm
                break
        if chosen is None:
            chosen, tag = (d + 2) % 4, 'B'

        self.events.append('t=%.1f (%.2f,%.2f) yaw=%+.0f F=%.2f L=%.2f R=%.2f -> %s'
                           % (self.t, x, y, math.degrees(yaw), front, left, right, tag))

        if tag == 'EXIT':
            self.exit_start = (cx, cy)
            self.phase = self.EXIT
            return 0.0, 0.0, 'EXIT_START'

        self.goal_dir = chosen
        self.target = (cx + DX[chosen] * self.cell, cy + DY[chosen] * self.cell)
        if chosen == d:
            return self.v_slow, 0.0, 'GO_' + tag

        self.yaw_des = self._snap_yaw(chosen)
        self.phase = self.TURN
        self.turn_count += 1
        return 0.0, 0.0, 'TURN_' + tag

    def _snap_center(self, x, y):
        """把当前位置吸附到最近的格心（格心 = x0 + k*cell）。"""
        kx = int(round((x - self.x0) / self.cell))
        ky = int(round((y - self.y0) / self.cell))
        return (self.x0 + kx * self.cell, self.y0 + ky * self.cell)

    def _quantize(self, yaw):
        """把连续航向量化到最近的 90° 方向序号。"""
        return int(round(yaw / (math.pi / 2.0))) % 4

    def _snap_yaw(self, d):
        return d * math.pi / 2.0

    def _inside_from(self, cx, cy, d):
        nx = cx + DX[d] * self.cell
        ny = cy + DY[d] * self.cell
        lim = self.half - self.cell / 2.0 + 1e-3   # 最外圈格心的坐标上限
        return -lim <= nx <= lim and -lim <= ny <= lim

    def _at_exit(self, cx, cy):
        return (abs(cx - self.exit_cell[0]) < 1e-6
                and abs(cy - self.exit_cell[1]) < 1e-6)
