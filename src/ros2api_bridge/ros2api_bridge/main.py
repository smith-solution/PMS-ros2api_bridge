from fastapi import FastAPI, Request
import uvicorn
import threading
import rclpy
from ros2api_bridge.ros2api_bridge.ros_node import RosBridgeNode
from pydantic import BaseModel

app = FastAPI()

#Json Parcing with Pydantic
class GenericMessage(BaseModel):
    purpose: str
    payload: dict

# ROS2 노드를 백그라운드에서 실행
rclpy.init()
ros_node = RosBridgeNode()

def ros_spin():
    rclpy.spin(ros_node)

threading.Thread(target=ros_spin, daemon=True).start()

@app.post("/send_message")
async def send_message(msg: GenericMessage):
    if msg.purpose == "initialpose":
        x = msg.payload.get("x")
        y = msg.payload.get("y")
        yaw = msg.payload.get("yaw")
        ros_node.publish_initialpose(x,y,yaw)
    return {"status": "Message published", "content": msg.purpose}

@app.on_event("shutdown")
def shutdown_event():
    ros_node.destroy_node()
    rclpy.shutdown()
