#!/usr/bin/env python3
"""记录 /odom 真实轨迹到 CSV，供报告中的实验曲线与误差分析使用。

输出列：t, x, y, yaw, v, w，其中 t 为节点启动后的秒数，
x/y 是 /odom 的世界坐标（gazebo_ros_diff_drive 以 spawn 位姿初始化里程计），
不是 SLAM 全局定位结果。
"""

import atexit
import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class TrajectoryRecorder(Node):
    """里程计轨迹记录节点。"""

    def __init__(self):
        super().__init__('trajectory_recorder')
        self.declare_parameter('output', '/home/zyy/ros2_ws/trajectory.csv')
        self.declare_parameter('sample_period', 0.10)
        path = str(self.get_parameter('output').value)
        self.fp = open(path, 'w', encoding='utf-8')
        self.fp.write('# 地老鼠走迷宫实验 里程计轨迹（来源：Gazebo 差速驱动插件 /odom）\n')
        self.fp.write('t,x,y,yaw,v,w\n')
        self.fp.flush()
        atexit.register(self.close_file)

        self.t0 = self.get_clock().now()
        self.last = -1.0
        self.rows = 0
        self.create_subscription(Odometry, 'odom', self.odom_cb, 10)
        self.get_logger().info('轨迹记录节点启动，输出：%s' % path)

    def close_file(self):
        try:
            self.fp.flush()
            self.fp.close()
        except Exception:
            pass

    def odom_cb(self, msg):
        t = (self.get_clock().now() - self.t0).nanoseconds * 1e-9
        period = float(self.get_parameter('sample_period').value)
        if t - self.last < period:
            return
        self.last = t
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        v = msg.twist.twist.linear.x
        w = msg.twist.twist.angular.z
        self.fp.write('%.3f,%.5f,%.5f,%.5f,%.5f,%.5f\n'
                      % (t, p.x, p.y, yaw, v, w))
        self.fp.flush()
        self.rows += 1
        if self.rows % 100 == 0:
            self.get_logger().info('已记录 %d 行轨迹' % self.rows)


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close_file()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
