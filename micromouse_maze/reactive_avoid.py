#!/usr/bin/env python3
"""反应式避障：前方检测到障碍物时停车并原地右转，前方开阔后恢复直行。

用于单独验证任务书 4.2 的避障要求；带滞回（stop_dist / clear_dist）
避免在阈值附近反复启停。
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


def sector_min(msg, a1_deg, a2_deg):
    a1, a2 = math.radians(a1_deg), math.radians(a2_deg)
    best = float(msg.range_max)
    for i, r in enumerate(msg.ranges):
        ang = msg.angle_min + i * msg.angle_increment
        ang = math.atan2(math.sin(ang), math.cos(ang))
        if a1 <= ang <= a2 and math.isfinite(r) \
                and msg.range_min <= r <= msg.range_max:
            best = min(best, float(r))
    return best


class ReactiveAvoid(Node):
    """前方障碍 -> 停车右转，带滞回的反应式避障节点。"""

    def __init__(self):
        super().__init__('reactive_avoid')
        self.declare_parameter('stop_dist', 0.60)
        self.declare_parameter('clear_dist', 0.95)
        self.declare_parameter('cruise_speed', 0.12)
        self.declare_parameter('turn_speed', 0.80)

        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.sub = self.create_subscription(LaserScan, 'scan', self.scan_cb, 10)
        self.timer = self.create_timer(0.10, self.control)

        self.front = 8.0
        self.left = 8.0
        self.right = 8.0
        self.has_scan = False
        self.avoiding = False
        self.clear_count = 0
        self.trigger_count = 0
        self.turn_dir = -1.0        # 转向方向：+1 左 / -1 右（按两侧余量选）
        self.avoid_t0 = 0.0         # 本轮避障起始时间
        self.yaw = 0.0              # 里程计航向
        self.yaw0 = 0.0             # 本轮避障起始航向
        self.turn_step = 0
        self.get_logger().info('反应式避障节点启动')

    def scan_cb(self, msg):
        self.front = sector_min(msg, -25, 25)
        self.left = sector_min(msg, 60, 120)
        self.right = sector_min(msg, -120, -60)
        self.has_scan = True

    def odom_cb(self, msg):
        self.yaw = yaw_from_quat(msg.pose.pose.orientation)

    def control(self):
        if not self.has_scan:
            return
        stop_d = float(self.get_parameter('stop_dist').value)
        clear_d = float(self.get_parameter('clear_dist').value)
        cruise = float(self.get_parameter('cruise_speed').value)
        turn = float(self.get_parameter('turn_speed').value)

        if not self.avoiding and self.front < stop_d:
            self.avoiding = True
            self.trigger_count += 1
            self.clear_count = 0
            self.avoid_t0 = self.get_clock().now().nanoseconds * 1e-9
            # 反式避障的关键：往余量更大的一侧转，而不是固定右转
            self.turn_dir = 1.0 if self.left > self.right else -1.0
            self.yaw0 = self.yaw
            self.turn_step = 0
            self.get_logger().warn(
                '前方 %.2f m 检测到障碍（第 %d 次，左 %.2f / 右 %.2f）'
                ' -> 停车并向%s转'
                % (self.front, self.trigger_count, self.left, self.right,
                   '左' if self.turn_dir > 0 else '右'))

        if self.avoiding:
            # 用里程计把车原地转过 90°，再判断前方是否够走：比"转到看见远处"更可控
            turned = abs(math.atan2(math.sin(self.yaw - self.yaw0),
                                    math.cos(self.yaw - self.yaw0)))
            if turned >= 1.45:
                if self.front > stop_d + 0.05:
                    self.avoiding = False
                    self.get_logger().info(
                        '已转过 %.0f°，前方 %.2f m 可通行 -> 恢复直行'
                        % (math.degrees(turned), self.front))
                    v, w = cruise, 0.0
                else:
                    self.turn_step += 1
                    self.yaw0 = self.yaw
                    if self.turn_step >= 2:
                        self.turn_dir = -self.turn_dir
                        self.turn_step = 0
                    self.get_logger().warn(
                        '转过 90° 仍被挡（前 %.2f m）-> 继续向%s转 90°'
                        % (self.front, '左' if self.turn_dir > 0 else '右'))
                    v, w = 0.0, self.turn_dir * turn
            else:
                v, w = 0.0, self.turn_dir * turn
        else:
            v, w = cruise, 0.0

        msg = Twist()
        msg.linear.x = v
        msg.angular.z = w
        self.pub.publish(msg)


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def main(args=None):
    rclpy.init(args=args)
    node = ReactiveAvoid()
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
