from fastapi import FastAPI, HTTPException
import uvicorn
import threading
import rclpy
import asyncio
import math
from ros2api_bridge.ros2api_bridge.ros_node import RosBridgeNode
from pydantic import BaseModel
from typing import Optional

app = FastAPI(title="Autoware ROS2 Bridge API", version="1.0.0")

# Pydantic 모델들
class InitialPosePayload(BaseModel):
    x: float
    y: float
    yaw: float  # 라디안 단위
    frame_id: Optional[str] = "map"

class GoalPayload(BaseModel):
    x: float
    y: float
    yaw: float  # 라디안 단위
    frame_id: Optional[str] = "map"

class OperationModePayload(BaseModel):
    mode: int  # 1=STOP, 2=AUTONOMOUS, 3=LOCAL, 4=REMOTE

# ROS2 노드를 백그라운드에서 실행
rclpy.init()
ros_node = RosBridgeNode()

def ros_spin():
    rclpy.spin(ros_node)

threading.Thread(target=ros_spin, daemon=True).start()

@app.post("/initialpose")
async def set_initial_pose(pose: InitialPosePayload):
    """
    Initial pose 설정
    차량의 현재 위치를 지도 상에서 초기화합니다.
    """
    try:
        ros_node.publish_initialpose(pose.x, pose.y, pose.yaw, pose.frame_id)
        return {
            "status": "success", 
            "message": "Initial pose published",
            "data": {
                "x": pose.x,
                "y": pose.y,
                "yaw": pose.yaw,
                "frame_id": pose.frame_id
            }
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/goal")
async def set_goal(goal: GoalPayload):
    """
    Navigation goal 설정
    차량이 이동할 목표 지점을 설정합니다.
    """
    try:
        ros_node.publish_goal(goal.x, goal.y, goal.yaw, goal.frame_id)
        return {
            "status": "success",
            "message": "Goal published",
            "data": {
                "x": goal.x,
                "y": goal.y,
                "yaw": goal.yaw,
                "frame_id": goal.frame_id
            }
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/operation_mode")
async def change_operation_mode(mode_data: OperationModePayload):
    """
    Operation mode 변경
    Autoware의 동작 모드를 변경합니다.
    - 1: STOP - 정지
    - 2: AUTONOMOUS - 자율주행 
    - 3: LOCAL - 로컬 제어
    - 4: REMOTE - 원격 제어
    """
    try:
        # 모드 유효성 검사
        valid_modes = [1, 2, 3, 4]
        mode_names = {1: "STOP", 2: "AUTONOMOUS", 3: "LOCAL", 4: "REMOTE"}
        
        if mode_data.mode not in valid_modes:
            raise HTTPException(
                status_code=400, 
                detail=f"Invalid mode. Valid modes: {dict(zip(valid_modes, [mode_names[m] for m in valid_modes]))}"
            )
        
        future = await ros_node.change_operation_mode(mode_data.mode)
        if future is None:
            raise HTTPException(status_code=500, detail="Service call failed")
        
        response = await asyncio.create_task(asyncio.wrap_future(future))
        
        if response.status.success:
            return {
                "status": "success", 
                "message": f"Operation mode changed to {mode_names[mode_data.mode]}",
                "data": {
                    "mode": mode_data.mode,
                    "mode_name": mode_names[mode_data.mode]
                }
            }
        else:
            return {
                "status": "error", 
                "message": f"Failed to change mode: {response.status.message}",
                "data": {
                    "mode": mode_data.mode,
                    "error_code": response.status.code
                }
            }
            
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.get("/")
async def root():
    """
    API 정보 및 사용 가능한 엔드포인트 목록
    """
    return {
        "message": "Autoware ROS2 Bridge API",
        "version": "1.0.0",
        "description": "Autoware 제어를 위한 REST API",
        "endpoints": {
            "POST /initialpose": "차량 초기 위치 설정",
            "POST /goal": "내비게이션 목표 지점 설정", 
            "POST /operation_mode": "동작 모드 변경 (STOP/AUTONOMOUS/LOCAL/REMOTE)",
            "GET /health": "서비스 상태 확인"
        },
        "coordinate_system": "map frame (기본값)",
        "angle_unit": "radians"
    }

@app.get("/health")
async def health_check():
    """
    서비스 상태 확인
    """
    try:
        # ROS 노드가 정상적으로 작동하는지 확인
        node_name = ros_node.get_name()
        return {
            "status": "healthy",
            "message": "Service is running",
            "ros_node": node_name,
            "timestamp": ros_node.get_clock().now().to_msg()
        }
    except Exception as e:
        raise HTTPException(status_code=503, detail=f"Service unhealthy: {str(e)}")

@app.on_event("shutdown")
def shutdown_event():
    """
    애플리케이션 종료 시 ROS 노드 정리
    """
    ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000)