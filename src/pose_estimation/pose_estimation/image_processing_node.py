#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from geometry_msgs.msg import Pose, PoseArray
from cv_bridge import CvBridge
import mediapipe as mp
from message_filters import TimeSynchronizer, Subscriber

class ImageProcessingNode(Node):
    def __init__(self):
        super().__init__('image_processing_node')
        self._initialize_parameters()
        self._initialize_mediapipe()
        self._initialize_subscribers()
        self._initialize_publishers()

    def _initialize_parameters(self):
        """Initialize node parameters and variables."""
        self.bridge = CvBridge()
        self.depth_image = None
        self.camera_info = None
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
        self.skeleton_detected = False
        self.scale_x = -1.0
        self.scale_y = 1.0
        self.scale_z = -1.0
        self.shoulder_offset_y = 0.2
        self.get_logger().info("Initialized parameters")

    def _initialize_mediapipe(self):
        """Initialize MediaPipe Pose model."""
        self.mp_pose = mp.solutions.pose
        self.mp_drawing = mp.solutions.drawing_utils
        self.pose = self.mp_pose.Pose(
            min_detection_confidence=0.7,
            min_tracking_confidence=0.7,
            model_complexity=1
        )
        self.get_logger().info("Initialized MediaPipe Pose model")

    def _initialize_subscribers(self):
        """Initialize ROS subscribers."""
        self.color_sub = Subscriber(self, CompressedImage, '/kinect2/qhd/image_color/compressed')
        self.depth_sub = Subscriber(self, CompressedImage, '/kinect2/qhd/image_depth_rect/compressed')
        self.ts = TimeSynchronizer([self.color_sub, self.depth_sub], queue_size=5)
        self.ts.registerCallback(self._synced_callback)
        self.camera_info_sub = self.create_subscription(CameraInfo, '/kinect2/qhd/camera_info', self._camera_info_callback, 10)
        self.get_logger().info("Initialized subscribers")

    def _initialize_publishers(self):
        """Initialize ROS publishers."""
        self.image_pub = self.create_publisher(Image, '/pose/annotated_image', 10)
        self.poses_pub = self.create_publisher(PoseArray, '/pose/joint_poses', 10)
        self.camera_info_pub = self.create_publisher(CameraInfo, '/sync/camera_info', 10)
        self.get_logger().info("Initialized publishers")

    def _camera_info_callback(self, msg):
        """Handle camera info message."""
        self.camera_info = msg
        self.fx = msg.k[0] if msg.k[0] > 0 else 540.686
        self.fy = msg.k[4] if msg.k[4] > 0 else 540.686
        self.cx = msg.k[2] if msg.k[2] > 0 else 479.75
        self.cy = msg.k[5] if msg.k[5] > 0 else 269.75
        self.camera_info_pub.publish(msg)
        self.get_logger().info(f"Camera info: fx={self.fx}, fy={self.fy}, cx={self.cx}, cy={self.cy}")

    def _process_images(self, color_msg, depth_msg):
        """Decode and validate color and depth images."""
        color_stamp = color_msg.header.stamp.sec + color_msg.header.stamp.nanosec * 1e-9
        depth_stamp = depth_msg.header.stamp.sec + depth_msg.header.stamp.nanosec * 1e-9
        self.get_logger().info(f"Synchronization time difference: {abs(color_stamp - depth_stamp):.6f} seconds")

        # Decode depth image
        np_arr = np.frombuffer(depth_msg.data, np.uint8)
        self.depth_image = cv2.imdecode(np_arr, cv2.IMREAD_UNCHANGED)
        if self.depth_image is None or self.depth_image.size == 0:
            self.get_logger().error("Depth image is not decoded, empty or invalid")
            return None, None
        # Decode color image
        np_arr = np.frombuffer(color_msg.data, np.uint8)
        color_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if color_image is None or color_image.size == 0:
            self.get_logger().error("Color image is not decoded, empty or invalid")
            return None, None

        color_image_rgb = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
        self.get_logger().info(f"Depth image shape: {self.depth_image.shape}")
        self.get_logger().info(f"Color image shape: {color_image.shape}")
        return color_image, color_image_rgb

    def _get_depth_value(self, x_pixel, y_pixel, image_width, image_height):
        """Calculate depth value from depth image."""
        if not (0 <= x_pixel < image_width and 0 <= y_pixel < image_height):
            self.get_logger().warn(f"Pixel out of bounds: ({x_pixel}, {y_pixel})")
            return 0.5
        patch = self.depth_image[max(0, y_pixel-3):y_pixel+4, max(0, x_pixel-3):x_pixel+4]
        valid_depths = patch[patch > 0]
        depth = np.median(valid_depths) / 1000.0 if valid_depths.size > 0 else 0.5
        if not (0.3 <= depth <= 5.0):
            self.get_logger().warn(f"Depth {depth} out of valid range, using fallback")
            return 0.5
        return depth

    def _mediapipe_to_rviz(self, x, y, z, joint_id):
        """Transform MediaPipe coordinates to RViz coordinates."""
        rviz_x = z * self.scale_x
        rviz_y = x * self.scale_y
        rviz_z = y * self.scale_z
        self.get_logger().info(
            f"Joint {joint_id} transform: MediaPipe (x={x:.3f}, y={y:.3f}, z={z:.3f}) -> "
            f"RViz (x={rviz_x:.3f}, y={rviz_y:.3f}, z={rviz_z:.3f})"
        )
        return rviz_x, rviz_y, rviz_z

    def _process_pose(self, color_image_rgb, color_msg):
        """Process MediaPipe pose and generate annotated image and pose array."""
        results = self.pose.process(color_image_rgb)
        pose_array = PoseArray()
        pose_array.header = color_msg.header
        pose_array.header.frame_id = "kinect2_link"
        return results, pose_array

    def _process_joints(self, results, color_image, pose_array):
        """Process joint positions and calculate 3D coordinates."""
        joint_positions = {}
        annotated_image = color_image.copy()

        if not results.pose_landmarks:
            if self.skeleton_detected:
                self.skeleton_detected = False
                self.get_logger().warn("No skeleton detected")
            return None, annotated_image, pose_array

        self.skeleton_detected = True
        self.get_logger().info("Skeleton detected")
        self.mp_drawing.draw_landmarks(annotated_image, results.pose_landmarks, self.mp_pose.POSE_CONNECTIONS)

        required_joints = [0, 11, 12, 15, 16, 23, 24, 27, 28]
        invalid_joints = [idx for idx in required_joints if results.pose_landmarks.landmark[idx].visibility <= 0.2]
        if invalid_joints:
            self.get_logger().warn(f"Invalid visibility for joints {invalid_joints}")
            return None, annotated_image, pose_array

        h, w = color_image.shape[:2]
        shoulder_left = results.pose_landmarks.landmark[11]
        shoulder_right = results.pose_landmarks.landmark[12]
        middle_shoulder_x_pixel = int((shoulder_left.x + shoulder_right.x) * w / 2)
        middle_shoulder_y_pixel = int((shoulder_left.y + shoulder_right.y) * h / 2)
        shoulder_depth = self._get_depth_value(middle_shoulder_x_pixel, middle_shoulder_y_pixel, w, h)
        if shoulder_depth == 0.0:
            self.get_logger().warn("Invalid shoulder depth, skipping frame")
            return None, annotated_image, pose_array

        middle_shoulder_z = shoulder_depth
        middle_shoulder_x = (middle_shoulder_x_pixel - self.cx) * middle_shoulder_z / self.fx
        middle_shoulder_y = (middle_shoulder_y_pixel - self.cy) * middle_shoulder_z / self.fy
        origin_x = middle_shoulder_x
        origin_y = middle_shoulder_y + self.shoulder_offset_y
        origin_z = middle_shoulder_z

        for joint_id in [0, 15, 16, 27, 28]:
            joint = results.pose_landmarks.landmark[joint_id]
            x_pixel = int(joint.x * w)
            y_pixel = int(joint.y * h)
            depth = self._get_depth_value(x_pixel, y_pixel, w, h)
            joint_name = {0: "Nose", 15: "Right Wrist", 16: "Left Wrist", 27: "Right Foot", 28: "Left Foot"}.get(joint_id)
            if depth == 0.0:
                self.get_logger().warn(f"Invalid depth for {joint_name} at ({x_pixel}, {y_pixel}), skipping coordinates")
                continue
            x_world = (x_pixel - self.cx) * depth / self.fx
            y_world = (y_pixel - self.cy) * depth / self.fy
            z_world = depth
            x_offset = x_world - origin_x
            y_offset = y_world - origin_y
            z_offset = z_world - origin_z
            rviz_x, rviz_y, rviz_z = self._mediapipe_to_rviz(x_offset, y_offset, z_offset, joint_id)
            if not all(np.isfinite([rviz_x, rviz_y, rviz_z])):
                self.get_logger().warn(f"Invalid RViz coordinates for {joint_name}: x={rviz_x}, y={rviz_y}, z={rviz_z}")
                continue
            self.get_logger().info(f"{joint_name} 3D coordinates (RViz): x={rviz_x:.3f}m, y={rviz_y:.3f}m, z={rviz_z:.3f}m")
            joint_positions[joint_id] = [rviz_x, rviz_y, rviz_z]
            cv2.putText(
                annotated_image, f"{joint_name}: ({rviz_x:.2f}, {rviz_y:.2f}, {rviz_z:.2f})m",
                (x_pixel, y_pixel - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            pose = Pose()
            pose.position.x = float(rviz_x)
            pose.position.y = float(rviz_y)
            pose.position.z = float(rviz_z)
            pose_array.poses.append(pose)

        return joint_positions, annotated_image, pose_array

    def _publish_results(self, color_msg, pose_array, annotated_image):
        """Publish pose array and annotated image."""
        self.poses_pub.publish(pose_array)
        self.get_logger().info(f"Published {len(pose_array.poses)} joint poses")

        annotated_image_msg = self.bridge.cv2_to_imgmsg(annotated_image, encoding="bgr8")
        annotated_image_msg.header = color_msg.header
        self.image_pub.publish(annotated_image_msg)
        self.get_logger().info("Published annotated image")

    def _synced_callback(self, color_msg, depth_msg):
        """Handle synchronized color and depth image messages."""
        self.get_logger().info("Synced callback triggered")
        if self.camera_info is None:
            self.get_logger().warn("Missing camera info, skipping frame")
            return

        try:
            color_image, color_image_rgb = self._process_images(color_msg, depth_msg)
            if color_image is None or color_image_rgb is None:
                self.get_logger().warn("Invalid images, skipping frame")
                return

            results, pose_array = self._process_pose(color_image_rgb, color_msg)
            joint_positions, annotated_image, pose_array = self._process_joints(results, color_image, pose_array)
            if joint_positions is None:
                self._publish_results(color_msg, pose_array, annotated_image)
                return

            self._publish_results(color_msg, pose_array, annotated_image)

        except Exception as e:
            self.get_logger().error(f"Error processing frame: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ImageProcessingNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error occurred: {e}")
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()