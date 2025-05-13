import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2, PointField, Image
from std_msgs.msg import Int8
import math
import numpy as np
import struct
from PIL import ImageDraw, ImageFont
from PIL import Image as PilImage
import time

BIN_DIST = 1
INTRUDER_RADIUS = 2

class PersonDetector(Node):
    def __init__(self):
        super().__init__('person_detector')
        # subscribe to scan
        self.create_subscription(LaserScan, '/filtered_scan', self.scan_callback, 10)

        # subcribe to camera so we can snapshot it when needed
        self.create_subscription(Image, '/astra_ros/devices/default/color/image_color', self.camera_callback, 10)
        self.current_camera_msg = 0
        
        # human identification publushers
        self.cloud_pub = self.create_publisher(PointCloud2, '/people_cloud', 10)
        self.headcount_img_pub = self.create_publisher(Image, '/people_count_img', 10)
        self.headcount_pub = self.create_publisher(Int8, '/people_count', 10)
        
        # create point cloud representing detection radius for rviz
        self.radius_pub = self.create_publisher(PointCloud2, '/intrusion_detection/radius', 10)
        self.radius_timer = self.create_timer(1, self.no_no_square)
        
        # maintain human count
        self.intruder_count = 0
        self.are_you_sure = 0

    def camera_callback(self, msg):
        self.current_camera_msg = msg

    def no_no_square(self):
        angles = np.linspace(0, 2 * np.pi, 5000, endpoint=False)
        x = INTRUDER_RADIUS * np.cos(angles)
        y = INTRUDER_RADIUS * np.sin(angles)
        points = np.zeros((len(x), 3))
        points[:, 0] = x
        points[:, 1] = y
        rad_cloud = self.create_pointcloud2(points)
        self.radius_pub.publish(rad_cloud)

    def text_to_image_pillow(self, text, image_size=(500, 150), font_size=56):
        font = ImageFont.load_default(size=font_size)
        img = PilImage.new('RGB', image_size, color='white')
        draw = ImageDraw.Draw(img)

        lines = text.split('\n')
        lines = [line.replace('\t', '    ') for line in lines]
        line_heights = [draw.textbbox((0, 0), line, font=font)[3] for line in lines]
        total_height = sum(line_heights)

        y = (image_size[1] - total_height) // 2

        for line, h in zip(lines, line_heights):
            w = draw.textbbox((0, 0), line, font=font)[2]
            x = (image_size[0] - w) // 2
            draw.text((x, y), line, font=font, fill='black')
            y += h
        np_img = np.array(img)
        
        msg = Image()
        msg.height = np_img.shape[0]
        msg.width = np_img.shape[1]
        msg.encoding = 'rgb8'
        msg.is_bigendian = 0
        msg.step = np_img.shape[1] * 3
        msg.data = np_img.tobytes()

        return msg

    def create_pointcloud2(self, points, frame_id="ramsis/base_laser_scanner"):
        msg = PointCloud2()
        msg.header.stamp = rclpy.time.Time().to_msg()
        msg.header.frame_id = frame_id
        msg.height = 1
        msg.width = len(points)

        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True

        buffer = []
        for p in points:
            buffer.append(struct.pack('fff', *p))

        msg.data = b''.join(buffer)
        return msg

    def laserscan_to_xy(self, msg):
        ranges = np.array(msg.ranges)
        coords = np.zeros((ranges.shape[0], 2))
        for i, r in enumerate(ranges):
            if r == math.inf:
                coords[i] = [math.inf, math.inf]
            else:
                theta = msg.angle_increment * i
                dx = r * math.cos(theta)
                dy = r * math.sin(theta)
                coords[i] = [dx, dy]
        return coords 
    
    def scan_callback(self, msg):
        # convert msg ranges to coords     
        coords = self.laserscan_to_xy(msg)

        # cluster into bins based on proximity
        clusters = []
        for (x_a, y_a) in coords:
            if x_a == 0 and y_a == 0:
                continue
            cluster_bin = -1
            for i, cluster in enumerate(clusters):
                min_dist = math.inf
                for (x_b, y_b) in cluster:
                    if x_a != x_b or y_a != y_b:
                        min_dist = min(math.sqrt((x_a - x_b)**2 + (y_a - y_b)**2), min_dist)
                if min_dist < BIN_DIST:
                    cluster_bin = i
                    break
            if cluster_bin != -1:
                clusters[i].append((x_a, y_a))
            else:
                clusters.append([(x_a, y_a)])
                
        # decry the foolish knaves that infringe upon my sightline
        intruder_count = 0
        centroids = []
        for cluster in clusters:
            if len(cluster) >= 7:
                cluster_arr = np.array(cluster)
                mx = -np.mean(cluster_arr[:, 0])
                my = -np.mean(cluster_arr[:, 1])
                centroids.append((mx, my, 0))
                if math.sqrt(mx**2 + my**2) < INTRUDER_RADIUS:
                    intruder_count += 1

        # save image if violation count has changed
        if self.intruder_count != intruder_count:
            arr = np.frombuffer(self.current_camera_msg.data, dtype=np.uint8).reshape((self.current_camera_msg.height, self.current_camera_msg.width, 3))
            img = PilImage.fromarray(arr, 'RGB')
            filename = f'violation_{round(time.monotonic())}.png'
            img.save(filename)

        # publish count as an int for sane humans to use
        count_msg = Int8(data=intruder_count)
        self.headcount_pub.publish(count_msg)

        # convert to an image for visualization in rviz
        self.intruder_count = intruder_count
        img_msg = self.text_to_image_pillow(f"nearby humans:\n{intruder_count}")
        self.headcount_img_pub.publish(img_msg)

        # publish cloud of identified humans
        cloud_msg = self.create_pointcloud2(centroids)
        self.cloud_pub.publish(cloud_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PersonDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()