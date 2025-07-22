import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class RosBridgeNode(Node):
    def __init__(self):
        super().__init__('ros_bridge_node')
        self.publisher_ = self.create_publisher(String, 'bridge_topic', 10)

    def publish_initialpose(self,x,y,yaw):
        ros_msg = String()
        ros_msg.data = str(x+y+yaw) #추후 오토웨어 메시지 타입으로 내부 필드 변경
        self.publisher_.publish(ros_msg)
        self.get_logger().info(f"Published: {ros_msg.data}")
