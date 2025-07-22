import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Point, Quaternion
from std_msgs.msg import Header
from builtin_interfaces.msg import Time
from tier4_system_msgs.srv import ChangeOperationMode
import math

class RosBridgeNode(Node):
    def __init__(self):
        super().__init__('ros_bridge_node')
        
        # Publishers
        self.initialpose_publisher = self.create_publisher(
            PoseWithCovarianceStamped, 
            '/initialpose', 
            10
        )
        
        self.goal_publisher = self.create_publisher(
            PoseStamped, 
            '/planning/mission_planning/goal', 
            10
        )
        
        # Service client
        self.operation_mode_client = self.create_client(
            ChangeOperationMode,
            '/system/operation_mode/change_operation_mode'
        )
    
    def publish_initialpose(self, x, y, yaw, frame_id="map"):
        """
        Initial pose를 발행하는 함수
        Args:
            x (float): x 좌표
            y (float): y 좌표  
            yaw (float): yaw 각도 (라디안)
            frame_id (str): 좌표계 프레임 ID
        """
        msg = PoseWithCovarianceStamped()
        
        # Header 설정
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        
        # Position 설정
        msg.pose.pose.position = Point()
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.position.z = 0.0
        
        # Orientation 설정 (yaw를 quaternion으로 변환)
        msg.pose.pose.orientation = self._yaw_to_quaternion(yaw)
        
        # Covariance 설정 (기본값 - 필요에 따라 조정)
        msg.pose.covariance = [0.0] * 36
        msg.pose.covariance[0] = 0.25   # x variance
        msg.pose.covariance[7] = 0.25   # y variance
        msg.pose.covariance[35] = 0.06854  # yaw variance
        
        self.initialpose_publisher.publish(msg)
        self.get_logger().info(f"Published initialpose: x={x}, y={y}, yaw={yaw}")
    
    def publish_goal(self, x, y, yaw, frame_id="map"):
        """
        Goal pose를 발행하는 함수
        Args:
            x (float): x 좌표
            y (float): y 좌표
            yaw (float): yaw 각도 (라디안)
            frame_id (str): 좌표계 프레임 ID
        """
        msg = PoseStamped()
        
        # Header 설정
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        
        # Position 설정
        msg.pose.position = Point()
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = 0.0
        
        # Orientation 설정
        msg.pose.orientation = self._yaw_to_quaternion(yaw)
        
        self.goal_publisher.publish(msg)
        self.get_logger().info(f"Published goal: x={x}, y={y}, yaw={yaw}")
    
    async def change_operation_mode(self, mode):
        """
        Operation mode를 변경하는 서비스 호출 함수
        Args:
            mode (int): 운영 모드 (1=STOP, 2=AUTONOMOUS, 3=LOCAL, 4=REMOTE)
        """
        if not self.operation_mode_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Operation mode service not available')
            return False
        
        request = ChangeOperationMode.Request()
        request.mode = int(mode)
        
        try:
            future = self.operation_mode_client.call_async(request)
            # await는 FastAPI에서 처리하므로 여기서는 future만 반환
            return future
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')
            return None
    
    def _yaw_to_quaternion(self, yaw):
        """
        Yaw 각도를 Quaternion으로 변환
        Args:
            yaw (float): yaw 각도 (라디안)
        Returns:
            Quaternion: 변환된 쿼터니언
        """
        quaternion = Quaternion()
        quaternion.x = 0.0
        quaternion.y = 0.0
        quaternion.z = math.sin(yaw / 2.0)
        quaternion.w = math.cos(yaw / 2.0)
        return quaternion