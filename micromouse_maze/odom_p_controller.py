#!/usr/bin/env python3
"""基于 /odom 的闭环 P 控制器：点到点运动 + 航点序列跟踪。

控制律：
    e_d = sqrt((gx-x)^2 + (gy-y)^2)
    e_h = wrap(atan2(gy-y, gx-x) - yaw)
    v   = min(v_max, kp_linear * e_d)        （航向误差大时降速先转向）
    w   = clip(kp_angular * e_h, -w_max, w_max)
到达半径 goal_tol 内即切换下一航点，全部完成后停车。
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


def yaw_from_quat(q):
    """四元数 -> 平面航向角 yaw（绕 z 轴）。"""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    """把角度归一化到 (-pi, pi]，避免跨 +-pi 时出现 2*pi 的突跳。"""
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class OdomPController(Node):
    """里程计闭环 P 控制器 + 航点序列。"""

    def __init__(self):
        super().__init__('odom_p_controller')
        self.declare_parameter('waypoints',
                               [1.0, 0.0, 1.0, 1.0, 0.0, 1.0, 0.0, 0.0])
        self.declare_parameter('kp_linear', 0.45)
        self.declare_parameter('kp_angular', 1.60)
        self.declare_parameter('v_max', 0.20)
        self.declare_parameter('w_max', 1.00)
        self.declare_parameter('goal_tol', 0.10)

        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.sub = self.create_subscription(Odometry, 'odom', self.odom_cb, 10)
        self.timer = self.create_timer(0.05, self.control)

        flat = [float(v) for v in self.get_parameter('waypoints').value]
        self.waypoints = [(flat[i], flat[i + 1])
                          for i in range(0, len(flat) - 1, 2)]
        self.idx = 0
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.has_odom = False
        self.done = False
        self.get_logger().info('P 控制器启动，航点序列：%s' % str(self.waypoints))

    def odom_cb(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.x = p.x
        self.y = p.y
        self.yaw = yaw_from_quat(q)
        self.has_odom = True

    def control(self):
        if self.done or not self.has_odom or self.idx >= len(self.waypoints):
            self.pub.publish(Twist())
            return

        gx, gy = self.waypoints[self.idx]
        dx, dy = gx - self.x, gy - self.y
        dist = math.hypot(dx, dy)
        tol = float(self.get_parameter('goal_tol').value)
        if dist < tol:
            self.get_logger().info(
                '到达航点 %d/%d 目标(%.2f, %.2f) 实际(%.3f, %.3f) 位置误差 %.3f m'
                % (self.idx + 1, len(self.waypoints), gx, gy,
                   self.x, self.y, dist))
            self.idx += 1
            if self.idx >= len(self.waypoints):
                self.done = True
                self.get_logger().info('全部航点跟踪完成，停车')
            self.pub.publish(Twist())
            return

        kp_l = float(self.get_parameter('kp_linear').value)
        kp_a = float(self.get_parameter('kp_angular').value)
        v_max = float(self.get_parameter('v_max').value)
        w_max = float(self.get_parameter('w_max').value)

        e_h = wrap(math.atan2(dy, dx) - self.yaw)
        v = min(v_max, kp_l * dist)
        if abs(e_h) > 0.5:          # 先原地转向，避免大航向误差下画出大弧线
            v *= 0.35
        w = max(-w_max, min(w_max, kp_a * e_h))

        msg = Twist()
        msg.linear.x = v
        msg.angular.z = w
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = OdomPController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
