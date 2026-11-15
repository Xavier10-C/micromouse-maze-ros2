#!/usr/bin/env python3
"""迷宫左墙跟随节点（ROS 2 封装）。

算法本体在 left_hand_core.LeftHandCore（纯 Python，可离线用射线仿真单测），
本节点只做：话题订阅 / 参数加载 / 定时调用 / 状态与日志输出。

状态机（详见 left_hand_core 文档）：
    DRIVE  通道内沿目标格心行驶（航向由里程计闭环，横向由左右等距修正）
    TURN   格心原地转向（左手法则选向：左手 -> 直行 -> 右手 -> 掉头）
    EXIT   终点格心朝北，直行穿出北墙缺口
    DONE / TIMEOUT  结束

感知只用 /scan（判断左/前/右是否有墙），位置与航向只用 /odom，
不使用 SLAM、不使用预先地图，符合"未知迷宫 + 反应式决策"的竞赛设定。
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from micromouse_maze.left_hand_core import (DIR_NAME, LeftHandCore, make_sectors,
                                            wrap)


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class LeftWallFollower(Node):

    def __init__(self):
        super().__init__('left_wall_follower')

        # ---- 迷宫尺度（必须与 world 一致）----
        self.declare_parameter('grid_n', 5)
        self.declare_parameter('cell', 0.60)
        self.declare_parameter('half', 1.50)
        # ---- 速度与增益 ----
        self.declare_parameter('v_nom', 0.24)
        self.declare_parameter('v_slow', 0.09)
        self.declare_parameter('w_max', 1.20)
        self.declare_parameter('turn_w', 1.30)
        self.declare_parameter('kp_head', 2.00)
        self.declare_parameter('kp_lat', 1.20)
        self.declare_parameter('lat_gain', 0.45)
        # ---- 感知 / 到达判据 ----
        self.declare_parameter('wall_th', 0.50)
        self.declare_parameter('arrive_tol', 0.04)
        self.declare_parameter('slow_dist', 0.30)
        self.declare_parameter('timeout', 180.0)
        # ---- 日志 ----
        self.declare_parameter('print_state', True)

        p = {n: self.get_parameter(n).value for n in (
            'grid_n', 'cell', 'half', 'v_nom', 'v_slow', 'w_max', 'turn_w',
            'kp_head', 'kp_lat', 'lat_gain', 'wall_th', 'arrive_tol',
            'slow_dist', 'timeout')}

        cell = float(p['cell'])
        n = int(p['grid_n'])
        half = float(p['half'])
        x0 = -((n - 1) * cell) / 2.0
        self.exit_cell_xy = (x0 + (n - 1) * cell, x0 + (n - 1) * cell)
        self.entry_cell_xy = (x0, x0)

        self.core = LeftHandCore(
            cell=cell, x0=0.0, y0=0.0, half=half,
            entry=self.entry_cell_xy, exit_cell=self.exit_cell_xy,
            v_nom=float(p['v_nom']), v_slow=float(p['v_slow']),
            w_max=float(p['w_max']), turn_w=float(p['turn_w']),
            kp_head=float(p['kp_head']), kp_lat=float(p['kp_lat']),
            lat_gain=float(p['lat_gain']), wall_th=float(p['wall_th']),
            arrive_tol=float(p['arrive_tol']), slow_dist=float(p['slow_dist']),
            timeout=float(p['timeout']))
        self.core.target = self.entry_cell_xy
        self.core.goal_dir = 1                     # 起点朝北进迷宫
        self.core.yaw_des = math.pi / 2.0

        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.state_pub = self.create_publisher(String, 'wall_state', 10)
        self.create_subscription(LaserScan, 'scan', self.scan_cb, 10)
        self.create_subscription(Odometry, 'odom', self.odom_cb, 10)
        self.timer = self.create_timer(0.05, self.control)

        self.ranges = None
        self.angle_min = -math.pi
        self.angle_inc = 2 * math.pi / 360.0
        self.sectors = make_sectors(self.angle_min, self.angle_inc, 360)
        self.x = self.y = self.yaw = 0.0
        self.has_odom = False
        self.t_prev = self.get_clock().now()
        self.visits = []
        self._done_logged = False

        self.get_logger().info(
            '左墙跟随状态机启动：%dx%d 格 / 单元 %.2f m / 目标速度 %.2f m/s；'
            '起点格心 (%.2f, %.2f) 朝北'
            % (n, n, cell, float(p['v_nom']), self.entry_cell_xy[0],
               self.entry_cell_xy[1]))

    # ---------------- 回调 ----------------
    def scan_cb(self, msg):
        if self.ranges is None:
            self.angle_min = msg.angle_min
            self.angle_increment = msg.angle_increment
            self.sectors = make_sectors(self.angle_min,
                                        msg.angle_increment, len(msg.ranges))
        self.ranges = list(msg.ranges)

    def odom_cb(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = yaw_from_quat(msg.pose.pose.orientation)
        self.has_odom = True

    # ---------------- 主循环 ----------------
    def control(self):
        now = self.get_clock().now()
        dt = (now - self.t_prev).nanoseconds * 1e-9
        self.t_prev = now
        if not self.has_odom or self.ranges is None:
            return
        dt = max(1e-3, min(0.2, dt))

        scan = {'ranges': self.ranges, 'sectors': self.sectors}
        v, w, info = self.core.step(scan, (self.x, self.y, self.yaw), dt)

        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.pub.publish(msg)

        st = String()
        st.data = '%s v=%.2f w=%+.2f odom=(%.2f,%.2f) yaw=%+.0f' % (
            self.core.phase, v, w, self.x, self.y, math.degrees(self.yaw))
        self.state_pub.publish(st)

        if info.startswith('GO_') or info.startswith('TURN_'):
            cell = self.core._snap_center(self.x, self.y)
            if not self.visits or self.visits[-1] != cell:
                self.visits.append(cell)
                self.get_logger().info(
                    '经过格心 (%.2f, %.2f) -> %s' % (cell[0], cell[1], info))

        if self.core.phase in (self.core.DONE, self.core.TIMEOUT) \
                and not self._done_logged:
            self._done_logged = True
            self.pub.publish(Twist())
            if self.core.phase == self.core.DONE:
                self.get_logger().info(
                    '到达迷宫出口！用时 %.2f s，里程 %.2f m，转向 %d 次，'
                    '经过 %d 个格心，终点 (%.3f, %.3f)'
                    % (self.core.t, self.core.distance, self.core.turn_count,
                       len(self.visits), self.x, self.y))
            else:
                self.get_logger().warn('超时 %.0f s 未出迷宫，停车' %
                                       float(self.get_parameter('timeout').value))
            for e in self.core.events:
                self.get_logger().info('  ' + e)


def main(args=None):
    rclpy.init(args=args)
    node = LeftWallFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
