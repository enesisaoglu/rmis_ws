#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class TopicPublisher(Node):
    def __init__(self):
        super().__init__('topic_publisher')
        self.publisher_ = self.create_publisher(String, '/my_topic', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info('Publishing to /my_topic at 1 Hz')

    def timer_callback(self):
        msg = String()
        msg.data = 'hello I am RMIS'
        self.publisher_.publish(msg)
        self.get_logger().info('Published: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    node = TopicPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down publisher')
    except Exception as e:
        node.get_logger().error(f'Error: {e}')
    finally:
        node.destroy_node()
        if rclpy.ok():  # Only call shutdown if the context is still valid
            rclpy.shutdown()

if __name__ == '__main__':
    main()