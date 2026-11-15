#!/usr/bin/env python3
"""拓展任务：Pure Pursuit 纯追踪轨迹跟踪。

在给定折线路径上取前视点（lookahead），由几何关系
    alpha = atan2(dy, dx) - yaw
    kappa = 2*sin(alpha)/Ld
    w     = v * kappa
得到角速度指令，与左墙跟随形成"反应式 / 轨迹式"两种策略对照。
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


class PurePursuit(Node):
    """纯追踪控制器。"""

    def __init__(self):
        super().__init__('pure_pursuit')
        self.declare_parameter('path', [0.0, 0.0, 1.0, 0.0, 2.0, 0.0,
                                        2.6, 0.6, 2.6, 1.4, 2.0, 2.0,
                                        1.0, 2.0, 0.0, 2.0])
        self.declare_parameter('lookahead', 0.55)
        self.declare_parameter('speed', 0.15)
        self.declare_parameter('w_max', 1.00)
        self.declare_parameter('goal_tol', 0.18)

        flat = [float(v) for v in self.get_parameter('path').value]
        self.path = [(flat[i], flat[i + 1]) for i in range(0, len(flat) - 1, 2)]
        self.target_idx = 0
        # 路径累计弧长，用于"最近点 + 前视距离"的标准纯追踪搜索
        self.arc = [0.0]
        for k in range(1, len(self.path)):
            self.arc.append(self.arc[-1] + dist(self.path[k - 1], self.path[k]))

        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Odometry, 'odom', self.odom_cb, 10)
        self.timer = self.create_timer(0.05, self.control)
        self.x = self.y = self.yaw = 0.0
        self.has_odom = False
        self.done = False
        self.get_logger().info('Pure Pursuit 启动：%d 个路径点，前视距离 %.2f m'
                               % (len(self.path),
                                  float(self.get_parameter('lookahead').value)))

    def odom_cb(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = yaw_from_quat(msg.pose.pose.orientation)
        self.has_odom = True

    def control(self):
        if self.done or not self.has_odom:
            self.pub.publish(Twist())
            return
        ld = float(self.get_parameter('lookahead').value)
        v = float(self.get_parameter('speed').value)
        w_max = float(self.get_parameter('w_max').value)
        tol = float(self.get_parameter('goal_tol').value)

        if dist((self.x, self.y), self.path[-1]) < tol:
            self.done = True
            self.get_logger().info(
                '纯追踪到达路径终点 (%.3f, %.3f)，终点误差 %.3f m'
                % (self.x, self.y, dist((self.x, self.y), self.path[-1])))
            self.pub.publish(Twist())
            return

        # 1) 找路径上离机器人最近的点（全局搜索，避免索引单调前进后锁死在远处）
        near = min(range(len(self.path)),
                   key=lambda k: dist((self.x, self.y), self.path[k]))
        # 2) 从最近点沿路径前进 Ld 的弧长，得到前视点
        goal = self.path[-1]
        for k in range(near, len(self.path)):
            if self.arc[k] - self.arc[near] >= ld:
                goal = self.path[k]
                self.target_idx = k
                break

        alpha = wrap(math.atan2(goal[1] - self.y, goal[0] - self.x) - self.yaw)

        msg = Twist()
        if abs(alpha) > 0.35:
            # 前视点偏差超过 20°：先原地对准，窄通道里边走边打舵会蹭墙
            msg.linear.x = 0.0
            msg.angular.z = max(-w_max, min(w_max, 2.0 * alpha))
        else:
            kappa = 2.0 * math.sin(alpha) / ld
            w = max(-w_max, min(w_max, v * kappa))
            msg.linear.x = v * max(0.55, math.cos(alpha))
            msg.angular.z = w
        self.pub.publish(msg)

        if int(self.get_clock().now().nanoseconds * 1e-9 * 2) % 4 == 0:
            self.get_logger().info(
                'DBG pose=(%.3f,%.3f,%.0fdeg) near=%d goal=(%.2f,%.2f) '
                'alpha=%.2f cmd v=%.3f w=%.3f'
                % (self.x, self.y, math.degrees(self.yaw), near,
                   goal[0], goal[1], alpha, msg.linear.x, msg.angular.z))


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuit()
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
