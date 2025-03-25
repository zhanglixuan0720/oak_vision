import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import numpy as np
import cv2
from builtin_interfaces.msg import Time
from oak_vision.oak_camera import OAKCamera  
from oak_vision.oak_camera import OAKFrame, OAKVIOFrame


class OAKVisionNode(Node):
    def __init__(self, name):
        super().__init__(name)
        self.init_params()
        self.camera = OAKCamera(self.width, self.height, self.fps, self.mask_vio)
        self.bridge = CvBridge()

        self.pub_color_raw = self.create_publisher(Image, '/camera/color/image_raw', 10)
        self.pub_color_compressed = self.create_publisher(CompressedImage, '/camera/color/image_compressed', 10)
        self.pub_color_info = self.create_publisher(CameraInfo, '/camera/color/camera_info', 10)
        self.pub_depth_raw = self.create_publisher(Image, '/camera/depth/image_raw', 10)
        self.pub_depth_compressed = self.create_publisher(CompressedImage, '/camera/depth/image_compressed', 10)
        self.pub_depth_info = self.create_publisher(CameraInfo, '/camera/depth/camera_info', 10)
        

        self.timer_rgb = self.create_timer(1.0 / self.camera.fps, self.publish_rgb)
        self.timer_depth = self.create_timer(1.0 / self.camera.fps, self.publish_depth)
        if not self.mask_vio:
            self.pub_odom = self.create_publisher(Odometry, '/camera/odom', 10)
            self.timer_vio = self.create_timer(1 / 100., self.publish_vio)
        
        # buffer for persistent data
        self.color_intrinsics_msg = None
        self.depth_intrinsics_msg = None
        self.get_logger().info(f'Node {name} has started')
        
    def init_params(self):
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30)
        self.mask_vio = self.declare_parameter('mask_vio', False)
        self.width = self.get_parameter('width').get_parameter_value().integer_value
        self.height = self.get_parameter('height').get_parameter_value().integer_value
        self.fps = self.get_parameter('fps').get_parameter_value().integer_value
        self.mask_vio = self.get_parameter('mask_vio').get_parameter_value().bool_value

    def publish_rgb(self):
        # Color
        color_frame = self.camera.read_color_frame()
        if color_frame:
            self.publish_color_msg(color_frame)

    def publish_depth(self):
        # Depth
        depth_frame = self.camera.read_depth_frame()
        if depth_frame:
            self.publish_depth_msg(depth_frame)
    
    def publish_vio(self):
        # VIO
        vio_frame = self.camera.read_vio_frame()
        if vio_frame:
            self.publish_odometry(vio_frame)

    def to_ros_time(self, stamp_sec: float) -> Time:
        secs = int(stamp_sec)
        nsecs = int((stamp_sec - secs) * 1e9)
        return Time(sec=secs, nanosec=nsecs)
    
    def create_camera_info(self, intrinsics, width, height, frame_id) -> CameraInfo:
        camera_info = CameraInfo()
        camera_info.header.frame_id = frame_id
        camera_info.width = width
        camera_info.height = height

        fx = intrinsics[0][0]
        fy = intrinsics[1][1]
        cx = intrinsics[0][2]
        cy = intrinsics[1][2]

        camera_info.k = [fx, 0., cx, 0., fy, cy, 0., 0., 1.]

        camera_info.p = [fx, 0., cx, 0., 0., fy, cy, 0., 0., 0., 1., 0.]

        camera_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        camera_info.distortion_model = 'plumb_bob'

        return camera_info

    def publish_color_msg(self, oak_frame: OAKFrame):
        stamp = self.to_ros_time(oak_frame.stamp)
        # raw image
        msg = self.bridge.cv2_to_imgmsg(oak_frame.frame, encoding='bgr8')
        msg.header.stamp = stamp
        msg.header.frame_id = 'oak_color'
        self.pub_color_raw.publish(msg)

        # compressed image
        comp_msg = CompressedImage()
        comp_msg.header = msg.header
        comp_msg.format = "jpeg"
        comp_msg.data = cv2.imencode(".jpg", oak_frame.frame)[1].tobytes()
        self.pub_color_compressed.publish(comp_msg)
        
        # camera info
        info = self.create_camera_info(oak_frame.intrinsics, oak_frame.frame.shape[1], oak_frame.frame.shape[0], 'oak_color') if self.color_intrinsics_msg is None else self.color_intrinsics_msg
        info.header.stamp = msg.header.stamp
        self.pub_color_info.publish(info)

    def publish_depth_msg(self, oak_frame: OAKFrame):
        stamp = self.to_ros_time(oak_frame.stamp)
        # raw depth
        msg = self.bridge.cv2_to_imgmsg(oak_frame.frame.astype(np.uint16), encoding='16UC1')
        msg.header.stamp = stamp
        msg.header.frame_id = 'oak_depth'
        self.pub_depth_raw.publish(msg)

        # compressed depth (PNG)
        comp_msg = CompressedImage()
        comp_msg.header = msg.header
        comp_msg.format = "png"
        comp_msg.data = cv2.imencode(".png", oak_frame.frame.astype(np.uint16))[1].tobytes()
        self.pub_depth_compressed.publish(comp_msg)
        
        # camera info
        info = self.create_camera_info(oak_frame.intrinsics, oak_frame.frame.shape[1], oak_frame.frame.shape[0], 'oak_depth') if self.depth_intrinsics_msg is None else self.depth_intrinsics_msg
        info.header.stamp = msg.header.stamp
        self.pub_depth_info.publish(info)

    def publish_odometry(self, vio_frame: OAKVIOFrame):
        stamp = self.to_ros_time(vio_frame.stamp)

        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'camera_link'

        msg.pose.pose.position.x = vio_frame.position[0]
        msg.pose.pose.position.y = vio_frame.position[1]
        msg.pose.pose.position.z = vio_frame.position[2]

        msg.pose.pose.orientation.x = vio_frame.orientation[0]
        msg.pose.pose.orientation.y = vio_frame.orientation[1]
        msg.pose.pose.orientation.z = vio_frame.orientation[2]
        msg.pose.pose.orientation.w = vio_frame.orientation[3]

        self.pub_odom.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = OAKVisionNode('oak_camera')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
