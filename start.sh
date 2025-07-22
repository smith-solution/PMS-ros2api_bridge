#!/bin/bash

set -e  # 에러 발생 시 스크립트 즉시 종료

echo "✅ [start.sh] Starting ROS2 FastAPI bridge..."

# 1. ROS 환경 로딩
echo "🔧 Sourcing ROS environment..."
source /opt/ros/humble/setup.bash
source /PMS_server/install/setup.bash

# export ROS 환경 + Python 경로 확실히 잡기 (source로만 잘 안되어서), 추후에 autoware msgs 패키지들 빌드한 경로 확인해서 여기에 추가
export PYTHONPATH="/opt/ros/humble/lib/python3.10/site-packages:/PMS_server/install:/PMS_server/src:$PYTHONPATH"
export LD_LIBRARY_PATH=/opt/ros/humble/lib:$LD_LIBRARY_PATH


# 4. Uvicorn 실행
echo "🚀 Launching FastAPI server..."
python3 -m uvicorn ros2api_bridge.ros2api_bridge.main:app --host 0.0.0.0 --port 8001
