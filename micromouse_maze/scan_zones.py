#!/usr/bin/env python3
"""激光雷达 /scan 极坐标消息解析：360 度分区提取前 / 左 / 右墙体距离。

sensor_msgs/LaserScan 的 ranges[i] 对应角度 angle_min + i*angle_increment，
本节点把 360 度扫描切成 5 个扇区，取每个扇区内的最小有效测距值
（滤除 inf / NaN / 越界值），发布到 /scan_zones 供避障与墙跟随节点复用。
"""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray

# (名称, 起始角 deg, 结束角 deg)；0 度为正前方 +x，正角度为左侧 +y
ZONES = [
    ('front', -20, 20),
    ('front_left', 20, 70),
    # 左/右墙扇区收窄到正侧方 +-5 度：70~110 度过宽时，出口缺口的
    # 斜后方墙体会被当成"左墙仍在"，导致左墙跟随错过出口而绕圈。
    ('left', 85, 95),
    ('front_right', -70, -20),
    ('right', -95, -85),
]


def sector_min(msg, a1_deg, a2_deg):
    """返回指定角度扇区内的最小有效测距，无有效点则返回量程上限。"""
    a1, a2 = math.radians(a1_deg), math.radians(a2_deg)
    best = float(msg.range_max)
    for i, r in enumerate(msg.ranges):
        ang = msg.angle_min + i * msg.angle_increment
        ang = math.atan2(math.sin(ang), math.cos(ang))   # 归一化到 (-pi, pi]
        if not (a1 <= ang <= a2):
            continue
        if math.isfinite(r) and msg.range_min <= r <= msg.range_max:
            best = min(best, float(r))
    return best


class ScanZones(Node):
    """雷达分区节点。"""

    def __init__(self):
        super().__init__('scan_zones')
        self.declare_parameter('print_every', 10)
        self.pub = self.create_publisher(Float32MultiArray, 'scan_zones', 10)
        self.sub = self.create_subscription(LaserScan, 'scan', self.cb, 10)
        self.count = 0
        self.get_logger().info('雷达分区节点启动：front / front_left / left / right / front_right')

    def cb(self, msg):
        d = {name: sector_min(msg, lo, hi) for name, lo, hi in ZONES}
        out = Float32MultiArray()
        out.data = [d['front'], d['front_left'], d['left'],
                    d['right'], d['front_right']]
        self.pub.publish(out)

        self.count += 1
        every = int(self.get_parameter('print_every').value)
        if every > 0 and self.count % every == 0:
            self.get_logger().info(
                '第 %4d 帧 (共 %d 点): 前 %.2f | 左前 %.2f | 左 %.2f '
                '| 右 %.2f | 右前 %.2f (m)'
                % (self.count, len(msg.ranges), d['front'],
                   d['front_left'], d['left'], d['right'], d['front_right']))


def main(args=None):
    rclpy.init(args=args)
    node = ScanZones()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
