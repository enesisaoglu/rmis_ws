#!/usr/bin/env python3
import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Image
from flask import Flask, render_template, Response
from flask_socketio import SocketIO, emit
import cv2
import base64
import numpy as np
import threading
import time
import json
import logging
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Define paths for files
HOME_DIR = os.path.expanduser("~")
ANGLES_FILE = os.path.join(HOME_DIR, "current_angles.txt")
COMMAND_FILE = os.path.join(HOME_DIR, "command.txt")

# Get package share directory for templates and static files
package_share_dir = get_package_share_directory('robot_control_ui')
template_dir = os.path.join(package_share_dir, 'templates')
static_dir = os.path.join(package_share_dir, 'static')

# Flask app setup
app = Flask(__name__, template_folder=template_dir, static_folder=static_dir)
app.config['SECRET_KEY'] = 'your-secret-key'
socketio = SocketIO(app, cors_allowed_origins="*")

# Store latest robot camera frame and annotated image
try:
    latest_robot_frame = cv2.imread(os.path.join(static_dir, 'sample_frame.jpg'))
    latest_annotated_image = cv2.imread(os.path.join(static_dir, 'sample_frame.jpg'))
except Exception as e:
    logger.error(f"Error loading sample_frame.jpg: {e}")
    latest_robot_frame = np.zeros((480, 640, 3), dtype=np.uint8)
    latest_annotated_image = np.zeros((480, 640, 3), dtype=np.uint8)

# Define all joints
JOINTS = [
    "neck_link_joint", "head_link_joint",
    "l_hand_link_joint", "l_lower_ankle_link_joint", "l_lower_arm_link_joint",
    "l_lower_hip_link_joint", "l_lower_leg_outer_link_joint", "l_lower_shoulder_link_joint",
    "l_upper_arm_link_joint", "l_upper_hip_link_joint", "l_upper_leg_outer_link_joint",
    "l_upper_shoulder_link_joint",
    "r_hand_link_joint", "r_lower_ankle_link_joint", "r_lower_arm_link_joint",
    "r_lower_hip_link_joint", "r_lower_leg_outer_link_joint", "r_lower_shoulder_link_joint",
    "r_upper_arm_link_joint", "r_upper_hip_link_joint", "r_upper_leg_outer_link_joint",
    "r_upper_shoulder_link_joint"
]

def get_servo_angles():
    """
    Retrieve or generate servo angles for all joints with +90 degree calibration offset.
    Maps to index.html sliders: shoulder -> l_upper_shoulder_link_joint,
    elbow -> l_lower_arm_link_joint, wrist -> l_hand_link_joint.
    """
    try:
        # Check if current_angles.txt exists
        if os.path.exists(ANGLES_FILE):
            with open(ANGLES_FILE, "r") as f:
                angles = json.load(f)
                # Validate angles and apply +90 degree offset
                if all(key in angles for key in JOINTS):
                    return {joint: min(max(int(angles[joint]), 0), 180) for joint in JOINTS}
        
        # Generate default angles with +90 offset
        angles = {joint: 90 + 90 for joint in JOINTS}  # Default to 180 degrees (90 + 90)
        return angles
    except Exception as e:
        logger.error(f"Error reading servo angles: {e}")
        # Return default angles with +90 offset on error
        return {joint: 180 for joint in JOINTS}

class FlaskNode(Node):
    def __init__(self, socketio):
        super().__init__('flask_node')
        self.socketio = socketio
        self.joint_angles = {joint: 180 for joint in JOINTS}  # Default to 180 (90 + 90)
        self.bridge = CvBridge()
        
        # Subscribe to /joint_states from pose_estimation
        self.joint_subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_states_callback,
            10
        )
        
        # Subscribe to /pose/annotated_image from pose_estimation
        self.image_subscription = self.create_subscription(
            Image,
            '/pose/annotated_image',
            self.annotated_image_callback,
            10
        )
        
        # Publisher for calibrated joint states
        self.publisher = self.create_publisher(
            JointState,
            '/calibrated_joint_states',
            10
        )
        
        self.get_logger().info("FlaskNode initialized")

    def joint_states_callback(self, msg):
        """Handle incoming joint states, apply +90 degree offset, and publish."""
        try:
            calibrated_angles = {}
            for name, angle in zip(msg.name, msg.position):
                # Convert radians to degrees and apply +90 offset
                angle_deg = np.degrees(angle) + 90
                calibrated_angle = min(max(int(angle_deg), 0), 180)
                calibrated_angles[name] = calibrated_angle
            
            # Update internal state
            self.joint_angles.update(calibrated_angles)
            
            # Save to file
            with open(ANGLES_FILE, "w") as f:
                json.dump(self.joint_angles, f)
            
            # Emit to UI
            self.socketio.emit('angles_update', self.joint_angles, namespace='/')
            self.get_logger().info(f"Emitted angles: {self.joint_angles}")
            
            # Publish calibrated joint states
            calibrated_msg = JointState()
            calibrated_msg.header = msg.header
            calibrated_msg.name = list(self.joint_angles.keys())
            calibrated_msg.position = [float(self.joint_angles[name]) for name in self.joint_angles]
            self.publisher.publish(calibrated_msg)
            self.get_logger().info("Published calibrated joint states")
            
        except Exception as e:
            self.get_logger().error(f"Error processing joint states: {e}")

    def annotated_image_callback(self, msg):
        """Handle incoming annotated image."""
        global latest_annotated_image
        try:
            latest_annotated_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.get_logger().info("Received annotated image")
        except Exception as e:
            self.get_logger().error(f"Error processing annotated image: {e}")

@app.route('/')
def index():
    try:
        logger.info(f"Rendering template from: {os.path.join(template_dir, 'index.html')}")
        return render_template('index.html')
    except Exception as e:
        logger.error(f"Error rendering index.html: {e}")
        return "Template rendering error", 500

@app.route('/mediapipe_feed')
def mediapipe_feed():
    def generate():
        global latest_annotated_image
        while True:
            if latest_annotated_image is None:
                frame = cv2.imread(os.path.join(static_dir, 'sample_frame.jpg'))
            else:
                frame = latest_annotated_image
            _, buffer = cv2.imencode('.jpg', frame)
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
            time.sleep(0.05)
    return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/robot_cam_feed')
def robot_cam_feed():
    def generate():
        global latest_robot_frame
        while True:
            _, buffer = cv2.imencode('.jpg', latest_robot_frame)
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
            time.sleep(0.05)
    return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')

@socketio.on('robot_cam_feed')
def handle_robot_cam_feed(data):
    try:
        global latest_robot_frame
        jpg_as_text = data.get('frame', '')
        jpg_buffer = base64.b64decode(jpg_as_text)
        nparr = np.frombuffer(jpg_buffer, np.uint8)
        latest_robot_frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
        logger.info("Received robot camera frame")
    except Exception as e:
        logger.error(f"Error handling robot camera frame: {e}")

@socketio.on('update_angles')
def handle_update_angles(data):
    try:
        # Map index.html sliders to specific joints
        angles = get_servo_angles()
        angles['l_upper_shoulder_link_joint'] = min(max(int(data.get('shoulder', angles['l_upper_shoulder_link_joint'])), 0), 180)
        angles['l_lower_arm_link_joint'] = min(max(int(data.get('elbow', angles['l_lower_arm_link_joint'])), 0), 180)
        angles['l_hand_link_joint'] = min(max(int(data.get('wrist', angles['l_hand_link_joint'])), 0), 180)
        with open(ANGLES_FILE, "w") as f:
            json.dump(angles, f)
        emit('angles_update', angles, broadcast=True)
        logger.info(f"Published angles: {angles}")
    except Exception as e:
        logger.error(f"Error updating angles: {e}")

@socketio.on('command')
def handle_command(data):
    try:
        action = data.get('action', '')
        if action in ['walk', 'stop']:
            with open(COMMAND_FILE, "w") as f:
                f.write(action)
            emit('command', {'action': action}, broadcast=True)
            logger.info(f"Published command: {action}")
        else:
            logger.warning(f"Invalid command received: {action}")
    except Exception as e:
        logger.error(f"Error handling command: {e}")

def run_flask():
    try:
        logger.info(f"Starting Flask server with template_dir: {template_dir}, static_dir: {static_dir}")
        socketio.run(app, host='0.0.0.0', port=5000, allow_unsafe_werkzeug=True)
    except Exception as e:
        logger.error(f"Error running Flask: {e}")

def main():
    rclpy.init()
    node = FlaskNode(socketio)
    
    # Run Flask in a separate thread
    flask_thread = threading.Thread(target=run_flask, daemon=True)
    flask_thread.start()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().error(f"Error occurred: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()