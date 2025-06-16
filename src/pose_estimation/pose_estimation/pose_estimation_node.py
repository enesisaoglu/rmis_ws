#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image, CompressedImage, CameraInfo, JointState
from geometry_msgs.msg import Pose, PoseArray
from cv_bridge import CvBridge
import mediapipe as mp
from message_filters import TimeSynchronizer, Subscriber
from ikpy.chain import Chain
import os
from uuid import uuid4

class PoseEstimationNode(Node):
    def __init__(self):
        super().__init__('pose_estimation_node')
        self._initialize_parameters()
        self._initialize_mediapipe()
        self._initialize_subscribers()
        self._initialize_publishers()
        self._initialize_ik_chains()
        self._publish_zero_joint_states()
        self._initialize_head_tracking()

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
        self.smooth_factor = 0.3
        self.previous_angles = [0.0] * 22
        self.scale_x = -1.0
        self.scale_y = 1.0
        self.scale_z = -1.0
        self.shoulder_offset_z = 0.2
        self.servo_names = [
            'neck_link_joint', 'head_link_joint',
            'r_upper_shoulder_link_joint', 'r_lower_shoulder_link_joint',
            'r_upper_arm_link_joint', 'r_lower_arm_link_joint', 'r_hand_link_joint',
            'l_upper_shoulder_link_joint', 'l_lower_shoulder_link_joint',
            'l_upper_arm_link_joint', 'l_lower_arm_link_joint', 'l_hand_link_joint',
            'r_upper_hip_link_joint', 'r_lower_hip_link_joint',
            'r_upper_leg_outer_link_joint', 'r_lower_leg_outer_link_joint', 'r_lower_ankle_link_joint',
            'l_upper_hip_link_joint', 'l_lower_hip_link_joint',
            'l_upper_leg_outer_link_joint', 'l_lower_leg_outer_link_joint', 'l_lower_ankle_link_joint'
        ]

    def _initialize_head_tracking(self):
        """Initialize head tracking parameters."""
        self.neutral_head_position = None
        self.head_yaw_range = np.radians(45.0)
        self.head_pitch_range = np.radians(30.0)
        self.neck_yaw_joint_range = np.radians(45.0)
        self.head_pitch_joint_range = np.radians(30.0)
        self.previous_yaw = 0.0
        self.previous_pitch = 0.0
        self.get_logger().info("Head tracking initialized")

    def _initialize_mediapipe(self):
        """Initialize MediaPipe Pose model."""
        self.mp_pose = mp.solutions.pose
        self.mp_drawing = mp.solutions.drawing_utils
        self.pose = self.mp_pose.Pose(
            min_detection_confidence=0.7,
            min_tracking_confidence=0.7,
            model_complexity=1
        )

    def _initialize_subscribers(self):
        """Initialize ROS subscribers."""
        self.color_sub = Subscriber(self, CompressedImage, '/kinect2/qhd/image_color/compressed')
        self.depth_sub = Subscriber(self, CompressedImage, '/kinect2/qhd/image_depth_rect/compressed')
        self.ts = TimeSynchronizer([self.color_sub, self.depth_sub], queue_size=5)
        self.ts.registerCallback(self._synced_callback)
        self.color_debug_sub = self.create_subscription(CompressedImage, '/kinect2/qhd/image_color/compressed', self._color_debug_callback, 10)
        self.depth_debug_sub = self.create_subscription(CompressedImage, '/kinect2/qhd/image_depth_rect/compressed', self._depth_debug_callback, 10)
        self.camera_info_sub = self.create_subscription(CameraInfo, '/kinect2/qhd/camera_info', self._camera_info_callback, 10)

    def _initialize_publishers(self):
        """Initialize ROS publishers."""
        self.image_pub = self.create_publisher(Image, '/pose/annotated_image', 10)
        self.poses_pub = self.create_publisher(PoseArray, '/pose/joint_poses', 10)
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)

    def _initialize_ik_chains(self):
        """Load IKPy chains from JSON files."""
        json_path = "/home/enesisaoglu/rmis_ws/src/pose_estimation/json/"
        json_files = [
            "rmis_left_arm.json",
            "rmis_right_arm.json",
            "rmis_left_leg.json",
            "rmis_right_leg.json"
        ]
        for json_file in json_files:
            file_path = os.path.join(json_path, json_file)
            if not os.path.exists(file_path):
                raise FileNotFoundError(f"JSON file not found: {file_path}")

        try:
            self.left_arm_chain = Chain.from_json_file(os.path.join(json_path, "rmis_left_arm.json"))
            self.right_arm_chain = Chain.from_json_file(os.path.join(json_path, "rmis_right_arm.json"))
            self.left_leg_chain = Chain.from_json_file(os.path.join(json_path, "rmis_left_leg.json"))
            self.right_leg_chain = Chain.from_json_file(os.path.join(json_path, "rmis_right_leg.json"))
            self.get_logger().info("Successfully loaded IKPy chains")
            self.get_logger().info(f"Left leg chain links: {self.left_leg_chain.links}")
            self.get_logger().info(f"Right leg chain links: {self.right_leg_chain.links}")
        except Exception as e:
            self.get_logger().error(f"Failed to load IKPy chains: {e}")
            raise

    def _publish_zero_joint_states(self):
        """Publish zero joint states for initialization or reset."""
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = self.servo_names
        joint_state.position = [0.0] * len(self.servo_names)
        self.joint_state_pub.publish(joint_state)
        self.get_logger().info("Published initial zero joint states")

    def _camera_info_callback(self, msg):
        """Handle camera info message."""
        self.camera_info = msg
        self.fx = msg.k[0] if msg.k[0] > 0 else 540.686
        self.fy = msg.k[4] if msg.k[4] > 0 else 540.686
        self.cx = msg.k[2] if msg.k[2] > 0 else 479.75
        self.cy = msg.k[5] if msg.k[5] > 0 else 269.75
        self.get_logger().info(f"Camera info: fx={self.fx}, fy={self.fy}, cx={self.cx}, cy={self.cy}")

    def _color_debug_callback(self, msg):
        """Log receipt of color image for debugging."""
        self.get_logger().info("Received color image message")

    def _depth_debug_callback(self, msg):
        """Log receipt of depth image for debugging."""
        self.get_logger().info("Received depth image message")

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

    def _adjust_leg_target(self, target):
        """Adjust leg target coordinates."""
        x, y, z = target
        self.get_logger().info(f"Leg target: x={x:.3f}, y={y:.3f}, z={z:.3f}")
        return [x, y, z]

    def _smooth_angle(self, new_angle, index):
        """Apply smoothing to joint angles."""
        if not np.isfinite(new_angle):
            self.get_logger().warn(f"Invalid angle at index {index}: {new_angle}")
            return self.previous_angles[index]
        smoothed = (1 - self.smooth_factor) * self.previous_angles[index] + self.smooth_factor * new_angle
        self.previous_angles[index] = smoothed
        return smoothed

    def _compute_head_angles(self, nose_position, shoulder_x, shoulder_y, shoulder_z):
        """Compute head yaw and pitch angles relative to neutral position."""
        if not all(np.isfinite(nose_position)):
            self.get_logger().warn("Invalid nose position for head tracking")
            return 0.0, 0.0

        nose_x, nose_y, nose_z = nose_position

        if self.neutral_head_position is None:
            self.neutral_head_position = [nose_x, nose_y, nose_z]
            self.get_logger().info(f"Set neutral head position: {self.neutral_head_position}")
            return 0.0, 0.0

        rel_x = nose_x - self.neutral_head_position[0]
        rel_y = nose_y - self.neutral_head_position[1]
        rel_z = nose_z - self.neutral_head_position[2]

        self.get_logger().info(f"Relative nose position: rel_x={rel_x:.3f}, rel_y={rel_y:.3f}, rel_z={rel_z:.3f}")

        yaw = np.arctan2(rel_x, nose_z - shoulder_z)
        pitch = np.arctan2(rel_y, nose_z - shoulder_z)

        yaw = np.clip(yaw, -self.head_yaw_range, self.head_yaw_range)
        pitch = np.clip(pitch, -self.head_pitch_range, self.head_pitch_range)

        neck_yaw = (yaw / self.head_yaw_range) * self.neck_yaw_joint_range  # Added negative sign
        head_pitch = (pitch / self.head_pitch_range) * self.head_pitch_joint_range

        neck_yaw = self._smooth_angle(neck_yaw, 0)
        head_pitch = self._smooth_angle(head_pitch, 1)

        self.get_logger().info(f"Head angles: yaw={np.degrees(yaw):.2f}°, pitch={np.degrees(pitch):.2f}°")
        self.get_logger().info(f"Robot joint angles: neck_yaw={np.degrees(neck_yaw):.2f}°, head_pitch={np.degrees(head_pitch):.2f}°")

        return neck_yaw, head_pitch

    def _process_images(self, color_msg, depth_msg):
        """Decode and validate color and depth images."""
        color_stamp = color_msg.header.stamp.sec + color_msg.header.stamp.nanosec * 1e-9
        depth_stamp = depth_msg.header.stamp.sec + depth_msg.header.stamp.nanosec * 1e-9
        self.get_logger().info(f"Synchronization time difference: {abs(color_stamp - depth_stamp):.6f} seconds")

        np_arr = np.frombuffer(depth_msg.data, np.uint8)
        self.depth_image = cv2.imdecode(np_arr, cv2.IMREAD_UNCHANGED)
        if self.depth_image is None or self.depth_image.size == 0:
            self.get_logger().error("Depth image is empty or invalid")
            return None, None

        np_arr = np.frombuffer(color_msg.data, np.uint8)
        color_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if color_image is None or color_image.size == 0:
            self.get_logger().error("Color image decode failed")
            return None, None

        color_image_rgb = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
        self.get_logger().info(f"Depth image shape: {self.depth_image.shape}, min: {self.depth_image.min()}, max: {self.depth_image.max()}")
        self.get_logger().info(f"Color image shape: {color_image.shape}")
        return color_image, color_image_rgb

    def _process_pose(self, color_image, color_image_rgb, color_msg):
        """Process MediaPipe pose and generate annotated image and pose array."""
        results = self.pose.process(color_image_rgb)
        annotated_image = color_image.copy()
        pose_array = PoseArray()
        pose_array.header = color_msg.header
        pose_array.header.frame_id = "kinect2_link"
        return results, annotated_image, pose_array

    def _process_joints(self, results, color_image, pose_array):
        """Process joint positions and calculate 3D coordinates."""
        joint_positions = {}
        annotated_image = color_image.copy()
        if not results.pose_landmarks:
            if self.skeleton_detected:
                self.skeleton_detected = False
                self.get_logger().warn("No skeleton detected, resetting to zero joint states")
                self._publish_zero_joint_states()
            return None, annotated_image, pose_array, None, None

        self.skeleton_detected = True
        self.get_logger().info("Skeleton detected")
        self.mp_drawing.draw_landmarks(
            annotated_image, results.pose_landmarks, self.mp_pose.POSE_CONNECTIONS)
        
        required_joints = [0, 11, 12, 15, 16, 23, 24, 27, 28]
        invalid_joints = [idx for idx in required_joints if results.pose_landmarks.landmark[idx].visibility <= 0.2]
        if invalid_joints:
            self.get_logger().warn(f"Invalid visibility for joints {invalid_joints}")
            return None, annotated_image, pose_array, None, None

        h, w = color_image.shape[:2]
        shoulder_left = results.pose_landmarks.landmark[11]
        shoulder_right = results.pose_landmarks.landmark[12]
        shoulder_x_pixel = int((shoulder_left.x + shoulder_right.x) * w / 2)
        shoulder_y_pixel = int((shoulder_left.y + shoulder_right.y) * h / 2)
        shoulder_depth = self._get_depth_value(shoulder_x_pixel, shoulder_y_pixel, w, h)
        if shoulder_depth == 0.0:
            self.get_logger().warn("Invalid shoulder depth, skipping frame")
            return None, annotated_image, pose_array, None, None

        shoulder_z = shoulder_depth
        shoulder_x = (shoulder_x_pixel - self.cx) * shoulder_z / self.fx
        shoulder_y = (shoulder_y_pixel - self.cy) * shoulder_z / self.fy
        origin_x = shoulder_x
        origin_y = shoulder_y + self.shoulder_offset_z
        origin_z = shoulder_z

        shoulder_rviz_x, shoulder_rviz_y, shoulder_rviz_z = self._mediapipe_to_rviz(
            shoulder_x - origin_x, shoulder_y - origin_y, shoulder_z - origin_z, 0)
        self.get_logger().info(f"Shoulder center: y={shoulder_y - origin_y:.3f}")

        nose_position = None
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
            if joint_id == 0:
                nose_position = [x_world, y_world, z_world]
            cv2.putText(
                annotated_image, f"{joint_name}: ({rviz_x:.2f}, {rviz_y:.2f}, {rviz_z:.2f})m",
                (x_pixel, y_pixel - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1
            )
            pose = Pose()
            pose.position.x = float(x_world)
            pose.position.y = float(y_world)
            pose.position.z = float(z_world)
            pose_array.poses.append(pose)

        neck_yaw, head_pitch = self._compute_head_angles(nose_position, shoulder_x, shoulder_y, shoulder_z) if nose_position else (0.0, 0.0)

        return joint_positions, annotated_image, pose_array, neck_yaw, head_pitch

    def _compute_ik(self, joint_positions, neck_yaw, head_pitch):
        """Compute inverse kinematics for arms, legs, and head."""
        servo_angles = [0.0] * len(self.servo_names)
        left_arm_target = joint_positions.get(15, [0.0, 0.0, 0.0])
        right_arm_target = joint_positions.get(16, [0.0, 0.0, 0.0])
        left_leg_target = self._adjust_leg_target(joint_positions.get(27, [0.0, 0.0, -0.5]))
        right_leg_target = self._adjust_leg_target(joint_positions.get(28, [0.0, 0.0, -0.5]))

        self.get_logger().info(
            f"IK targets (RViz): LA={left_arm_target}, RA={right_arm_target}, LL={left_leg_target}, RL={right_leg_target}"
        )

        try:
            ik_left_arm = self.left_arm_chain.inverse_kinematics(left_arm_target, max_iter=500)
            self.get_logger().info(f"Left arm IK (degrees): {[np.degrees(a) for a in ik_left_arm]}")
        except Exception as e:
            self.get_logger().error(f"Left arm IK failed: {e}, target={left_arm_target}")
            ik_left_arm = [0.0] * 6

        try:
            ik_right_arm = self.right_arm_chain.inverse_kinematics(right_arm_target, max_iter=500)
            self.get_logger().info(f"Right arm IK (degrees): {[np.degrees(a) for a in ik_right_arm]}")
        except Exception as e:
            self.get_logger().error(f"Right arm IK failed: {e}, target={right_arm_target}")
            ik_right_arm = [0.0] * 6

        try:
            ik_left_leg = self.left_leg_chain.inverse_kinematics(left_leg_target, max_iter=500)
            self.get_logger().info(f"Left leg IK (degrees): {[np.degrees(a) for a in ik_left_leg]}")
        except Exception as e:
            self.get_logger().error(f"Left leg IK failed: {e}, target={left_leg_target}")
            ik_left_leg = [0.0] * 7

        try:
            ik_right_leg = self.right_leg_chain.inverse_kinematics(right_leg_target, max_iter=500)
            self.get_logger().info(f"Right leg IK (degrees): {[np.degrees(a) for a in ik_right_leg]}")
        except Exception as e:
            self.get_logger().error(f"Right leg IK failed: {e}, target={right_leg_target}")
            ik_right_leg = [0.0] * 7

        servo_angles[0] = neck_yaw
        servo_angles[1] = head_pitch
        servo_angles[7:12] = [self._smooth_angle(angle, i) for i, angle in enumerate(ik_left_arm[1:6], 7)]
        servo_angles[2:7] = [self._smooth_angle(angle, i) for i, angle in enumerate(ik_right_arm[1:6], 2)]
        servo_angles[17:22] = [self._smooth_angle(angle, i) for i, angle in enumerate(ik_left_leg[2:7], 17)]
        servo_angles[12:17] = [self._smooth_angle(angle, i) for i, angle in enumerate(ik_right_leg[2:7], 12)]

        return servo_angles

    def _publish_results(self, color_msg, pose_array, annotated_image, servo_angles):
        """Publish joint states, pose array, and annotated image."""
        joint_state = JointState()
        joint_state.header = color_msg.header
        joint_state.name = self.servo_names
        joint_state.position = servo_angles
        self.joint_state_pub.publish(joint_state)
        self.get_logger().info(f"Published joint states with {len(servo_angles)} joints")

        self.poses_pub.publish(pose_array)
        self.get_logger().info(f"Published {len(pose_array.poses)} joint poses")

        annotated_image_msg = self.bridge.cv2_to_imgmsg(annotated_image, encoding="bgr8")
        annotated_image_msg.header = color_msg.header
        self.image_pub.publish(annotated_image_msg)

    def _synced_callback(self, color_msg, depth_msg):
        """Handle synchronized color and depth image messages."""
        self.get_logger().info("Synced callback triggered")
        if self.camera_info is None:
            self.get_logger().warn("Missing camera info, skipping frame")
            return

        try:
            color_image, color_image_rgb = self._process_images(color_msg, depth_msg)
            if color_image is None or color_image_rgb is None:
                return

            results, annotated_image, pose_array = self._process_pose(color_image, color_image_rgb, color_msg)
            joint_positions, annotated_image, pose_array, neck_yaw, head_pitch = self._process_joints(results, color_image, pose_array)
            if joint_positions is None:
                self._publish_results(color_msg, pose_array, annotated_image, [0.0] * len(self.servo_names))
                return

            servo_angles = self._compute_ik(joint_positions, neck_yaw, head_pitch)
            self._publish_results(color_msg, pose_array, annotated_image, servo_angles)

        except Exception as e:
            self.get_logger().error(f"Error processing frame: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = PoseEstimationNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Hata oluştu: {e}")
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()