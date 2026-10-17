#!/usr/bin/env python3
"""/planning_map을 받아 첫 맵(grid_first)과 최신 맵(grid_last)을 raw int8 + meta json으로 저장."""
import json
import sys

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node


class Recorder(Node):
    def __init__(self, out_prefix):
        super().__init__('grid_recorder')
        self.out = out_prefix
        self.count = 0
        self.create_subscription(OccupancyGrid, '/planning_map', self.cb, 10)

    def save(self, msg, name):
        data = np.array(msg.data, dtype=np.int8)
        data.tofile(f'{self.out}/{name}.bin')
        meta = {
            'width': msg.info.width,
            'height': msg.info.height,
            'resolution': msg.info.resolution,
            'origin_x': msg.info.origin.position.x,
            'origin_y': msg.info.origin.position.y,
            'occupied_cells': int((data > 50).sum()),
        }
        with open(f'{self.out}/{name}.json', 'w') as f:
            json.dump(meta, f)

    def cb(self, msg):
        self.count += 1
        if self.count == 1:
            self.save(msg, 'grid_first')
        self.save(msg, 'grid_last')


def main():
    rclpy.init()
    rclpy.spin(Recorder(sys.argv[1]))


if __name__ == '__main__':
    main()
