#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseArray
from ikpy.chain import Chain
import numpy as np
import os

class RmisMimicNode(Node):
    def __init__(self):
        super().__init__('rmis_mimic_node')
        self._initialize_parameters()
        self._initialize_ik_chains()
        self._initialize_subscribers()
        self._initialize_publishers()
        self._publish_zero_joint_states()

    def _initialize_parameters(self):
        """Initialize node parameters and variables."""
        self.smooth_factor = 0.3
        self.previous_angles = [0.0] * 22
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
        self.get_logger().info("Initialized parameters and servo names")

    def _initialize_ik_chains(self):
        """Load IKPy chains from JSON files."""
        json_path = "/home/enesisaoglu/rmis_ws/src/pose_estimation/json/"
        json_files = [
            "rmis_left_arm.json", "rmis_right_arm.json",
            "rmis_left_leg.json", "rmis_right_leg.json",
            "rmis_head.json"
        ]
        for json_file in json_files:
            file_path = os.path.join(json_path, json_file)
            if not os.path.exists(file_path):
                self.get_logger().error(f"JSON file not found: {file_path}")
                raise FileNotFoundError(f"JSON file not found: {file_path}")

        try:
            self.left_arm_chain = Chain.from_json_file(os.path.join(json_path, "rmis_left_arm.json"))
            self.right_arm_chain = Chain.from_json_file(os.path.join(json_path, "rmis_right_arm.json"))
            self.left_leg_chain = Chain.from_json_file(os.path.join(json_path, "rmis_left_leg.json"))
            self.right_leg_chain = Chain.from_json_file(os.path.join(json_path, "rmis_right_leg.json"))
            self.head_chain = Chain.from_json_file(os.path.join(json_path, "rmis_head.json"))
            self.get_logger().info("Successfully loaded IKPy chains including head chain")
        except Exception as e:
            self.get_logger().error(f"Failed to load IKPy chains: {e}")
            raise

    def _initialize_subscribers(self):
        """Initialize ROS subscribers."""
        self.poses_sub = self.create_subscription(PoseArray, '/pose/joint_poses', self._poses_callback, 10)
        self.get_logger().info("Initialized joint poses subscriber")

    def _initialize_publishers(self):
        """Initialize ROS publishers."""
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.get_logger().info("Initialized joint state publisher")

    def _publish_zero_joint_states(self):
        """Publish zero joint states for initialization or reset."""
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = self.servo_names
        joint_state.position = [0.0] * len(self.servo_names)
        self.joint_state_pub.publish(joint_state)
        self.get_logger().info("Published initial zero joint states")

    def _smooth_angle(self, new_angle, index):
        """Apply smoothing to joint angles."""
        if not np.isfinite(new_angle):
            self.get_logger().warn(f"Invalid angle at index {index}: {new_angle}")
            return self.previous_angles[index]
        smoothed = (1 - self.smooth_factor) * self.previous_angles[index] + self.smooth_factor * new_angle
        self.previous_angles[index] = smoothed
        return smoothed

    def _adjust_leg_target(self, target):
        """Adjust leg target coordinates."""
        x, y, z = target
        self.get_logger().info(f"Leg target: x={x:.3f}, y={y:.3f}, z={z:.3f}")
        return [x, y, z]
    
    def _compute_ik(self, joint_positions):
        """Compute inverse kinematics for arms, legs, and head."""
        servo_angles = [0.0] * len(self.servo_names)
        left_arm_target = joint_positions.get(15, [0.0, 0.0, 0.0])
        right_arm_target = joint_positions.get(16, [0.0, 0.0, 0.0])
        left_leg_target = self._adjust_leg_target(joint_positions.get(27, [0.0, 0.0, -0.5]))
        right_leg_target = self._adjust_leg_target(joint_positions.get(28, [0.0, 0.0, -0.5]))
        head_target = joint_positions.get(0, [0.0, 0.0, 0.0])  # Nose position
    
        self.get_logger().info(f"Head target: {head_target}")
    
        # Left Arm IK
        try:
            ik_left_arm = self.left_arm_chain.inverse_kinematics(left_arm_target, max_iter=100)
            self.get_logger().info(f"Left arm IK (degrees): {[np.degrees(a) for a in ik_left_arm]}")
        except Exception as e:
            self.get_logger().error(f"Left arm IK failed: {e}")
            ik_left_arm = [0.0] * 6
    
        # Right Arm IK
        try:
            ik_right_arm = self.right_arm_chain.inverse_kinematics(right_arm_target, max_iter=100)
            self.get_logger().info(f"Right arm IK (degrees): {[np.degrees(a) for a in ik_right_arm]}")
        except Exception as e:
            self.get_logger().error(f"Right arm IK failed: {e}")
            ik_right_arm = [0.0] * 6
    
        # Left Leg IK
        try:
            ik_left_leg = self.left_leg_chain.inverse_kinematics(left_leg_target, max_iter=100)
            self.get_logger().info(f"Left leg IK (degrees): {[np.degrees(a) for a in ik_left_leg]}")
        except Exception as e:
            self.get_logger().error(f"Left leg IK failed: {e}")
            ik_left_leg = [0.0] * 7
    
        # Right Leg IK
        try:
            ik_right_leg = self.right_leg_chain.inverse_kinematics(right_leg_target, max_iter=100)
            self.get_logger().info(f"Right leg IK (degrees): {[np.degrees(a) for a in ik_right_arm]}")
        except Exception as e:
            self.get_logger().error(f"Right leg IK failed: {e}")
            ik_right_leg = [0.0] * 7
    
        # Head: Manual yaw and pitch calculation
        yaw_angle = np.arctan2(head_target[1], head_target[0])  # y/x ratio for yaw
        yaw_angle = np.clip(yaw_angle, -1.57, 1.57)  # ±90° limit
        self.get_logger().info(f"Manual yaw angle (degrees): {np.degrees(yaw_angle)}")
    
        skala = 4.0  # Daha belirgin hareket için skala artırıldı
        neutral_z = 0.4  # Nötr pozisyon loglara göre uygun
        pitch_angle_raw = -(head_target[2] - neutral_z) * skala  # İşaret ters çevrildi
        self.get_logger().info(f"Raw pitch angle (degrees): {np.degrees(pitch_angle_raw)}")
        pitch_angle = np.clip(pitch_angle_raw, -1.0, 1.0)  # ±57° limit
        self.get_logger().info(f"Clipped pitch angle (degrees): {np.degrees(pitch_angle)}")
    
        # Assign servo angles
        servo_angles[0] = self._smooth_angle(yaw_angle, 0)  # neck_link_joint (yaw)
        servo_angles[1] = self._smooth_angle(pitch_angle, 1)  # head_link_joint (pitch)
        self.get_logger().info(f"Servo pitch angle (radians): {servo_angles[1]}")
        servo_angles[7:12] = [self._smooth_angle(angle, i) for i, angle in enumerate(ik_left_arm[1:6], 7)]
        servo_angles[2:7] = [self._smooth_angle(angle, i) for i, angle in enumerate(ik_right_arm[1:6], 2)]
        servo_angles[17:22] = [self._smooth_angle(angle, i) for i, angle in enumerate(ik_left_leg[2:7], 17)]
        servo_angles[12:17] = [self._smooth_angle(angle, i) for i, angle in enumerate(ik_right_leg[2:7], 12)]
    
        return servo_angles

    def _poses_callback(self, msg):
        """Handle joint poses message and compute IK."""
        self.get_logger().info("Received joint poses")
        try:
            if not msg.poses:
                self.get_logger().warn("No joint poses received, publishing zero joint states")
                self._publish_zero_joint_states()
                return

            joint_positions = {}
            for i, pose in enumerate(msg.poses):
                joint_id = [0, 15, 16, 27, 28][i] if i < 5 else None
                if joint_id is not None:
                    joint_positions[joint_id] = [pose.position.x, pose.position.y, pose.position.z]
                    self.get_logger().info(f"Joint {joint_id}: {joint_positions[joint_id]}")

            servo_angles = self._compute_ik(joint_positions)

            joint_state = JointState()
            joint_state.header = msg.header
            joint_state.name = self.servo_names
            joint_state.position = servo_angles
            self.joint_state_pub.publish(joint_state)
            self.get_logger().info(f"Published joint states with {len(servo_angles)} joints: {servo_angles}")

        except Exception as e:
            self.get_logger().error(f"Error in IK computation: {e}")
            self._publish_zero_joint_states()

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = RmisMimicNode()
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