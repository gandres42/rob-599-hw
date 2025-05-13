import rclpy
from rclpy.node import Node
from rclpy.parameter_event_handler import ParameterEventHandler
from sensor_msgs.msg import LaserScan
import numpy as np
from enum import Enum
import math
import random
from hw5_msgs.srv import ScanSnapshot

NEIGHBOR_DIST = .4

class FilterNode(Node):
    def __init__(self):
        super().__init__('laser_filter')

        # scan dist parameter
        self.declare_parameters(namespace='', parameters=[
            ('scan_dist', .2),
        ])
        self.scan_dist = 0.2
        self.handler = ParameterEventHandler(self)
        self.callback_handle = self.handler.add_parameter_callback(
            parameter_name = 'scan_dist',
            node_name = self.get_name(),
            callback = self.scan_dist_callback
        )

        # snapshot service
        self.create_service(ScanSnapshot, 'scan_snapshot', self.snap_srv_callback)
        self.snap_next = True

        # noise profile
        self.scan_snapshot = None

        # kruft supporting the lesser of the least bad middlewares
        self.get_logger().info('Node started')
        self.sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 1)
        self.pub = self.create_publisher(LaserScan, '/filtered_scan', 10)
        self.pub

        self.create_timer(60, self.small_talk)
        self.start_time = self.get_clock().now()

    def scan_dist_callback(self, param):
        print(param)
        self.scan_dist = param.value.double_value

    def snap_srv_callback(self, request, response):
        self.snap_next = True
        return response

    def small_talk(self):
        if random.random() > .5:
            self.get_logger().info(random.choice([
                "So, how's your day going?",
                "Crazy weather we're having, huh?",
                "Did you do anything fun over the weekend?",
                "How was your commute?",
                "Did you catch the game last night?",
                "This week is flying by, isn't it?",
                "Can you believe it's already May?",
                "How was your lunch?",
                "Try any good restaurants lately?",
                "How's the family?",
                "You been keeping busy?",
                "So, where are you from?",
                "Been working on anything interesting?",
                "Do you live around here?"
            ]))

    def scan_capture_snapshot(self, ranges, angle_increment):
        scan_snapshot = np.zeros((ranges.shape[0], 2))
        for i, r in enumerate(ranges):
            if r == math.inf:
                scan_snapshot[i] = [math.inf, math.inf]
            else:
                theta = angle_increment * i
                dx = r * math.cos(theta)
                dy = r * math.sin(theta)
                scan_snapshot[i] = [dx, dy]
        self.scan_snapshot = scan_snapshot

    def scan_callback(self, msg):
        ranges = np.array(msg.ranges)

        # capture scan snapshot if one does not yet exist
        if self.snap_next:
            self.scan_capture_snapshot(ranges, msg.angle_increment)
            self.snap_next = False

        # convert polar readings to x, y coordinates
        coords = np.zeros((ranges.shape[0], 2))
        for i, r in enumerate(ranges):
            if r == math.inf:
                coords[i] = [math.inf, math.inf]
            else:
                theta = msg.angle_increment * i
                dx = r * math.cos(theta)
                dy = r * math.sin(theta)
                coords[i] = [dx, dy]
        
        # remove outliers based on distance to snapshot
        removed_points = 0
        for i, (x_a, y_a) in enumerate(coords):
            if abs(x_a) == math.inf or abs(y_a) == math.inf:
                ranges[i] = math.inf

            # calc min distance to snapshot scan
            snapshot_dist = math.inf
            for (x_b, y_b) in self.scan_snapshot:
                if x_a != x_b or y_a != y_b:
                    if abs(x_b) == math.inf or abs(y_b) == math.inf:
                        continue
                    snapshot_dist = min(math.sqrt((x_a - x_b)**2 + (y_a - y_b)**2), snapshot_dist)

            # calc min distance to nearest neighbor
            neighbor_dist = math.inf
            for (x_b, y_b) in coords:
                if x_a != x_b or y_a != y_b:
                    if abs(x_b) == math.inf or abs(y_b) == math.inf:
                        continue
                    neighbor_dist = min(math.sqrt((x_a - x_b)**2 + (y_a - y_b)**2), neighbor_dist)
            
            # inf out if too far from neighbor
            if snapshot_dist >= self.scan_dist:# and neighbor_dist <= NEIGHBOR_DIST:
                r = math.hypot(x_a, y_a)
                ranges[i] = r
            else:
                ranges[i] = 0
                removed_points += 1


        # replace msg contents and repub
        msg.ranges = ranges.tolist()
        self.pub.publish(msg)
        # self.get_logger().info(f"removed {removed_points} points from /scan")


def main(args=None):
    rclpy.init(args=args)
    node = FilterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()