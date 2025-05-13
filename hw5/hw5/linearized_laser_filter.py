import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
from enum import Enum
import math
import itertools as it
from math import factorial as fac

FILTER_DIST = .1

class FilterType(Enum):
    NAIVE_OUTLIERS = 1
    SCAN_MATCH = 2


class FilterNode(Node):
    def __init__(self):
        super().__init__('minimal_node')

        # noise profile
        self.filter_type = FilterType.NAIVE_OUTLIERS
        self.scan_snapshot = None

        # kruft supporting the lesser of the least bad middlewares
        self.get_logger().info('Node started')
        self.sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.pub = self.create_publisher(LaserScan, '/filtered_scan', 10)
        self.pub


    def ranges_to_coords(self, ranges, angle_increment):
        coords = np.zeros((ranges.shape[0], 2))
        for i, r in enumerate(ranges):
            if r == np.inf:
                coords[i] = [np.inf, np.inf]
            else:
                theta = angle_increment * i
                dx = r * np.cos(theta)
                dy = r * np.sin(theta)
                coords[i] = [dx, dy]
        return coords

    def scan_callback(self, msg):
        ranges = np.array(msg.ranges)

        # create permutation matrix if snapshot does not yet exist
        if self.scan_snapshot is None:
            self.scan_snapshot = np.zeros((ranges.shape[0], ranges.shape[0], 2))
            coords = self.ranges_to_coords(ranges, msg.angle_increment)
            for i in range(ranges.shape[0]):
                self.scan_snapshot[i, :, 0] = coords[:, 0]
                self.scan_snapshot[i, :, 1] = coords[:, 1]
            return

        # arrange new points into coords if they haven't already been
        if 'coords' not in locals(): coords = self.ranges_to_coords(ranges, msg.angle_increment)
        

        coords_compare = np.full((coords.shape[0], coords.shape[0], 2), coords)
        coords_diff = coords_compare - self.scan_snapshot

        dists = np.zeros((coords_diff.shape[0], coords_diff.shape[1]))
        dists = np.nan_to_num(coords_diff[:, :, 0]**2 + coords_diff[:, :, 1]**2, np.inf)
        dists[dists == 0] = np.inf
        # print(dists)
        
        min_dists = np.zeros(coords_diff.shape[0])

        for i in range(dists.shape[0]):
            min_dists[i] = np.min(dists[i])
        # print(np.median(min_dists))
        
        # print(min_dists)

        

            # for i, r in enumerate(ranges):
            #     if r == np.inf:
            #         scan_snapshot[i] = [np.inf, np.inf]
            #     else:
            #         theta = msg.angle_increment * i
            #         dx = r * np.cos(theta)
            #         dy = r * np.sin(theta)
            #         scan_snapshot[i] = [dx, dy]

        # # convert polar readings to x, y coordinates
        # coords = np.zeros((ranges.shape[0], 2))
        # for i, r in enumerate(ranges):
        #     if r == np.inf:
        #         coords[i] = [np.inf, np.inf]
        #     else:
        #         theta = msg.angle_increment * i
        #         dx = r * np.cos(theta)
        #         dy = r * np.sin(theta)
        #         coords[i] = [dx, dy]
        
        # # replace msg contents and repub
        # msg.ranges = ranges.tolist()
        self.pub.publish(msg)
        # self.get_logger().info(f"removed {removed_points} points /scan")


def main(args=None):
    rclpy.init(args=args)
    node = FilterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()