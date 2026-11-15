#!/usr/bin/env python3
"""开环运动控制：直线行驶 + 原地转向。

不读取任何传感器，仅按时间轴给 /cmd_vel 发指令，用于和后面的
里程计闭环 P 控制做对照，说明开环控制存在累积误差。
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class OpenLoopDrive(Node):
    """按预定时序发布速度指令的开环控制器。"""

    def __init__(self):
        super().__init__('open_loop_drive')
        self.declare_parameter('linear_speed', 0.15)
        self.declare_parameter('angular_speed', 0.60)
        self.declare_parameter('straight_time', 3.0)

        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        v = float(self.get_parameter('linear_speed').value)
        w = float(self.get_parameter('angular_speed').value)
        t = float(self.get_parameter('straight_time').value)
        turn_t = 1.5707963 / w if w > 1e-6 else 2.62

        # (持续时间 s, 线速度 m/s, 角速度 rad/s, 说明)
        self.steps = [
            (t, v, 0.0, '直线行驶'),
            (0.8, 0.0, 0.0, '停车'),
            (turn_t, 0.0, w, '原地左转 90 度'),
            (0.8, 0.0, 0.0, '停车'),
            (t, v, 0.0, '换向后再直线行驶'),
            (0.8, 0.0, 0.0, '停车'),
            (turn_t, 0.0, -w, '原地右转 90 度回正'),
        ]
        self.idx = 0
        self.t0 = None
        self.done = False
        self.timer = self.create_timer(0.05, self.tick)
        self.get_logger().info('开环控制节点启动，共 %d 个动作' % len(self.steps))

    def tick(self):
        now = self.get_clock().now()
        if self.idx >= len(self.steps):
            self.pub.publish(Twist())
            if not self.done:
                self.get_logger().info('开环时序全部执行完毕，已停车')
                self.done = True
                self.timer.cancel()
            return

        dur, v, w, label = self.steps[self.idx]
        if self.t0 is None:
            self.t0 = now
            self.get_logger().info('步骤 %d/%d：%s（计划 %.2f s）'
                                   % (self.idx + 1, len(self.steps), label, dur))
        elapsed = (now - self.t0).nanoseconds * 1e-9
        if elapsed >= dur:
            self.get_logger().info('步骤 %d 完成，实测 %.2f s' % (self.idx + 1, elapsed))
            self.idx += 1
            self.t0 = None
            return

        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = OpenLoopDrive()
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
