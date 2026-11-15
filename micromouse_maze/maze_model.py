#!/usr/bin/env python3
"""地老鼠走迷宫：网格迷宫模型（生成 / 校验 / 射线仿真 / 出 SDF / 出布局图）。

竞赛规则对齐（MicroMouse 经典规则）：
  * 迷宫为 N×N 方格，单元边长 CELL，墙厚 THICK，墙高 HEIGHT；
  * 迷宫外框闭合，起点单元南墙开入口、终点单元北墙开出口；
  * 内部墙中心线落在网格线上，形成 T 形 / 十字交叉；
  * 起点在西南角单元、朝北进入迷宫，出口在东北角单元北墙。

坐标：单元 (i, j)，i 为 x 方向列号，j 为 y 方向行号，均从 0 开始。
单元中心 (x, y) = (X0 + i*CELL, Y0 + j*CELL)，X0 = Y0 = -(N-1)*CELL/2。
"""

import json
import math
import random

# ---- 可配置参数（set_grid 修改） ----
CELL = 0.70          # 单元边长 (m)，即通道宽度（车体 0.42x0.24，留足转向余量）
THICK = 0.04         # 墙厚 (m)
HEIGHT = 0.45        # 墙高 (m)：雷达扫描面高 0.38 m，可稳定扫到
N = 5                # 每边单元数

X0 = Y0 = -(N - 1) * CELL / 2.0
HALF = N * CELL / 2.0
ENTRY = (0, 0)
EXIT = (N - 1, N - 1)

# 方向：0=+x(东) 1=+y(北) 2=-x(西) 3=-y(南)
DX = (1, 0, -1, 0)
DY = (0, 1, 0, -1)
DIR_NAME = ('E', 'N', 'W', 'S')


def set_grid(n, cell=CELL, thick=THICK, height=HEIGHT):
    """切换网格规格（同时刷新派生常量）。"""
    global N, CELL, THICK, HEIGHT, X0, Y0, HALF, ENTRY, EXIT
    N, CELL, THICK, HEIGHT = n, cell, thick, height
    X0 = Y0 = -(N - 1) * CELL / 2.0
    HALF = N * CELL / 2.0
    ENTRY = (0, 0)
    EXIT = (N - 1, N - 1)


def cell_center(c):
    return (X0 + c[0] * CELL, Y0 + c[1] * CELL)


def in_grid(c):
    return 0 <= c[0] < N and 0 <= c[1] < N


def world_to_cell(x, y):
    return (int(round((x - X0) / CELL)), int(round((y - Y0) / CELL)))


class Maze:
    """walls[(i, j, d)] = True 表示单元 (i,j) 朝 d 方向有墙。"""

    def __init__(self, walls=None):
        self.walls = dict(walls) if walls else {}

    # ---------------- 构造 ----------------
    @staticmethod
    def perfect(seed):
        """随机深度优先（递归回溯）生成单连通迷宫（无环路）。"""
        rng = random.Random(seed)
        walls = {}
        for i in range(N):
            for j in range(N):
                for d in range(4):
                    if not in_grid((i + DX[d], j + DY[d])):
                        walls[(i, j, d)] = True
        visited = {(0, 0)}
        stack = [(0, 0)]
        while stack:
            i, j = stack[-1]
            nbrs = [(d, (i + DX[d], j + DY[d])) for d in range(4)
                    if in_grid((i + DX[d], j + DY[d]))
                    and (i + DX[d], j + DY[d]) not in visited]
            rng.shuffle(nbrs)
            if not nbrs:
                stack.pop()
                continue
            d, nb = nbrs[0]
            walls[(i, j, d)] = False
            walls[(nb[0], nb[1], (d + 2) % 4)] = False
            visited.add(nb)
            stack.append(nb)
        return Maze(walls)

    def knock(self, cell, d):
        """拆掉单元 cell 朝 d 的内墙，制造环路（竞赛迷宫常见）。"""
        nb = (cell[0] + DX[d], cell[1] + DY[d])
        if not in_grid(nb) or not self.walls.get((cell[0], cell[1], d), True):
            return False
        self.walls[(cell[0], cell[1], d)] = False
        self.walls[(nb[0], nb[1], (d + 2) % 4)] = False
        return True

    def open_entry_exit(self):
        self.walls[(ENTRY[0], ENTRY[1], 3)] = False     # 南墙开口
        self.walls[(EXIT[0], EXIT[1], 1)] = False       # 北墙开口

    def clone(self):
        return Maze(dict(self.walls))

    # ---------------- 查询 ----------------
    def can_go(self, cell, d):
        return in_grid(cell) and not self.walls.get((cell[0], cell[1], d), True)

    def n_walls(self):
        return sum(1 for i in range(N) for j in range(N) for d in (0, 1)
                   if self.walls.get((i, j, d), True))

    def dead_ends(self):
        out = []
        for i in range(N):
            for j in range(N):
                deg = sum(1 for d in range(4) if self.can_go((i, j), d))
                if deg == 1 and (i, j) not in (ENTRY, EXIT):
                    out.append((i, j))
        return out

    def loops(self):
        edges = sum(1 for i in range(N) for j in range(N) for d in (0, 1)
                    if self.can_go((i, j), d))
        return max(0, edges - N * N + 1)

    def ascii(self, path=None):
        path = path or []
        lines = []
        for j in range(N - 1, -1, -1):
            top = ''
            for i in range(N):
                top += '+' + ('   ' if self.can_go((i, j), 1) else '---')
            lines.append(top + '+')
            mid = ''
            for i in range(N):
                if (i, j) == ENTRY:
                    mark = ' S '
                elif (i, j) == EXIT:
                    mark = ' E '
                elif (i, j) in path:
                    mark = ' . '
                else:
                    mark = '   '
                mid += (' ' if self.can_go((i, j), 2) else '|') + mark
            mid += ' ' if self.can_go((N - 1, j), 0) else '|'
            lines.append(mid)
        bot = ''
        for i in range(N):
            bot += '+' + ('   ' if self.can_go((i, 0), 3) else '---')
        lines.append(bot + '+')
        return '\n'.join(lines)


# ---------------------------------------------------------------- 左手法则
def left_dir(d):
    """左手方向（逆时针 90°）。"""
    return (d + 1) % 4


def right_dir(d):
    return (d - 1) % 4


def left_hand_path(maze, start=None, heading=1, limit=400):
    """离散步进左手法则：左手 -> 直行 -> 右手 -> 掉头。

    返回 (path, cell, exited)；path 元素为 (单元, 进入该单元时的朝向)。
    """
    cell = start or ENTRY
    d = heading
    path = [(cell, d)]
    for _ in range(limit):
        if cell == EXIT and d == 1 and maze.can_go(cell, 1):
            return path, cell, True
        for cand in (left_dir(d), d, right_dir(d), (d + 2) % 4):
            if maze.can_go(cell, cand):
                cell = (cell[0] + DX[cand], cell[1] + DY[cand])
                d = cand
                path.append((cell, d))
                break
        else:
            return path, cell, False
    return path, cell, False


# ---------------------------------------------------------------- 几何 / 仿真
def wall_segments(maze):
    """生成带入口/出口缺口的墙体线段，用于射线仿真与绘图。"""
    segs = []
    lo, hi = -HALF, HALF
    half_t = THICK / 2.0

    def vline(x, y1, y2):
        segs.append(((x, y1), (x, y2)))

    def hline(y, x1, x2):
        segs.append(((x1, y), (x2, y)))

    # 内部竖墙：x = 网格线，连续的行合并
    for i in range(1, N):
        j = 0
        while j < N:
            if maze.walls.get((i - 1, j, 0), True):
                j2 = j
                while j2 + 1 < N and maze.walls.get((i - 1, j2 + 1, 0), True):
                    j2 += 1
                cx = -HALF + i * CELL          # 网格线在单元之间，不是格心
                vline(cx, Y0 + j * CELL - CELL / 2, Y0 + j2 * CELL + CELL / 2)
                j = j2 + 1
            else:
                j += 1
    # 内部横墙
    for j in range(1, N):
        i = 0
        while i < N:
            if maze.walls.get((i, j - 1, 1), True):
                i2 = i
                while i2 + 1 < N and maze.walls.get((i2 + 1, j - 1, 1), True):
                    i2 += 1
                cy = -HALF + j * CELL          # 同上：横墙落在网格线上
                hline(cy, X0 + i * CELL - CELL / 2, X0 + i2 * CELL + CELL / 2)
                i = i2 + 1
            else:
                i += 1

    # 外墙（南/北墙留缺口）
    ex = cell_center(ENTRY)[0]
    gx = cell_center(EXIT)[0]
    hline(-HALF, -HALF - half_t, ex - CELL / 2)
    hline(-HALF, ex + CELL / 2, HALF + half_t)
    hline(HALF, -HALF - half_t, gx - CELL / 2)
    hline(HALF, gx + CELL / 2, HALF + half_t)
    vline(-HALF, -HALF, HALF)
    vline(HALF, -HALF, HALF)
    return segs


def raycast(segs, ox, oy, angles, max_range=8.0):
    out = []
    for a in angles:
        ca, sa = math.cos(a), math.sin(a)
        best = max_range
        for (x1, y1), (x2, y2) in segs:
            dx, dy = x2 - x1, y2 - y1
            den = ca * dy - sa * dx
            if abs(den) < 1e-12:
                continue
            t = ((x1 - ox) * dy - (y1 - oy) * dx) / den
            u = ((x1 - ox) * sa - (y1 - oy) * ca) / den
            if t > 1e-4 and -1e-9 <= u <= 1 + 1e-9 and t < best:
                best = t
        out.append(best)
    return out


# ---------------------------------------------------------------- SDF 输出
def _box(cx, cy, sx, sy, k):
    return f"""      <collision name="c{k}">
        <pose>{cx:.4f} {cy:.4f} {HEIGHT/2:.4f} 0 0 0</pose>
        <geometry><box><size>{sx:.4f} {sy:.4f} {HEIGHT:.3f}</size></box></geometry>
        <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction>
          <bounce><restitution_factor>0.0</restitution_factor></bounce></surface>
      </collision>
      <visual name="v{k}">
        <pose>{cx:.4f} {cy:.4f} {HEIGHT/2:.4f} 0 0 0</pose>
        <geometry><box><size>{sx:.4f} {sy:.4f} {HEIGHT:.3f}</size></box></geometry>
        <material><ambient>0.80 0.82 0.86 1</ambient><diffuse>0.88 0.90 0.94 1</diffuse>
          <specular>0.04 0.04 0.04 1</specular></material>
      </visual>"""


def build_world_sdf(maze, world_name='maze_world'):
    """网格墙合并成长条后输出 SDF（碰撞体更少，Gazebo 求解更快）。"""
    boxes = []
    for (p, q) in wall_segments(maze):
        (x1, y1), (x2, y2) = p, q
        if abs(y1 - y2) < 1e-9:
            boxes.append(((x1 + x2) / 2, y1, abs(x2 - x1), THICK))
        else:
            boxes.append((x1, (y1 + y2) / 2, THICK, abs(y2 - y1)))
    body = "\n".join(_box(cx, cy, sx, sy, k)
                     for k, (cx, cy, sx, sy) in enumerate(boxes))

    sx, sy = cell_center(ENTRY)
    gx, gy = cell_center(EXIT)
    return f"""<?xml version="1.0" encoding="UTF-8"?>
<!-- 机器人感知与控制综合实践 —— 地老鼠网格迷宫（maze_model.py 自动生成，请勿手改）
     {N}x{N} 格，单元 {CELL:.2f} m，墙厚 {THICK:.2f} m，墙高 {HEIGHT:.2f} m；
     内部墙 {maze.n_walls()} 段，死胡同 {len(maze.dead_ends())} 个，环路 {maze.loops()} 个。
     入口：单元 {ENTRY} 南墙缺口；出口：单元 {EXIT} 北墙缺口。 -->
<sdf version="1.6">
  <world name="{world_name}">
    <include><uri>model://sun</uri></include>
    <include><uri>model://ground_plane</uri></include>
    <gravity>0 0 -9.8</gravity>
    <physics name="default_physics" default="0" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>
    <!-- 固定初始相机：俯视偏前，整座迷宫与小车都在画面内 -->
    <gui fullscreen="0">
      <camera name="user_camera">
        <pose>0 -4.300 4.400 0 0.780 1.5708</pose>
        <view_controller>orbit</view_controller>
      </camera>
    </gui>
    <scene><ambient>0.75 0.75 0.75 1</ambient><background>0.88 0.91 0.95 1</background>
      <shadows>false</shadows></scene>

    <!-- 起点（蓝）/ 终点（绿）地面标识，仅视觉无碰撞 -->
    <model name="start_mark"><static>true</static><link name="m">
      <visual name="v"><pose>{sx:.3f} {sy:.3f} 0.004 0 0 0</pose>
        <geometry><box><size>{CELL-0.08:.2f} {CELL-0.08:.2f} 0.008</size></box></geometry>
        <material><ambient>0.15 0.45 0.90 1</ambient><diffuse>0.18 0.50 0.95 1</diffuse></material></visual>
    </link></model>
    <model name="goal_mark"><static>true</static><link name="m">
      <visual name="v"><pose>{gx:.3f} {gy:.3f} 0.004 0 0 0</pose>
        <geometry><box><size>{CELL-0.08:.2f} {CELL-0.08:.2f} 0.008</size></box></geometry>
        <material><ambient>0.12 0.72 0.30 1</ambient><diffuse>0.15 0.78 0.34 1</diffuse></material></visual>
    </link></model>

    <model name="maze_walls"><static>true</static><pose>0 0 0 0 0 0</pose>
      <link name="walls_link">
{body}
      </link>
    </model>
  </world>
</sdf>
"""


def layout_json(maze, extra=None):
    d = {
        'N': N, 'cell': CELL, 'thick': THICK, 'height': HEIGHT,
        'entry': list(ENTRY), 'exit': list(EXIT),
        'internal_walls': maze.n_walls(), 'loops': maze.loops(),
        'dead_ends': [list(c) for c in maze.dead_ends()],
        'walls': [[i, j, dd] for (i, j, dd), w in sorted(maze.walls.items()) if w],
    }
    if extra:
        d.update(extra)
    return d


if __name__ == '__main__':
    m = Maze.perfect(1)
    m.open_entry_exit()
    path, cell, ok = left_hand_path(m)
    print(m.ascii([p[0] for p in path]))
    print('walls', m.n_walls(), 'dead_ends', len(m.dead_ends()))
    print('left-hand exit:', ok, 'steps', len(path) - 1)
