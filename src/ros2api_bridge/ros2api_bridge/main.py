import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class RosBridgeNode(Node):
    def __init__(self):
        super().__init__('ros_bridge_node')
        self.publisher_ = self.create_publisher(String, 'bridge_topic', 10)

    def publish_msg(self, msg: str):
        ros_msg = String()
        ros_msg.data = msg
        self.publisher_.publish(ros_msg)
        self.get_logger().info(f"Published: {msg}")

def main(args=None):
    rclpy.init(args=args)
    node = RosBridgeNode()
    rate = node.create_rate(1)  # 1Hz (1초 간격)

    try:
        while rclpy.ok():
            node.publish_msg("Hello from ROS2!")
            rclpy.spin_once(node)
            rate.sleep()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

