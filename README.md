# 🚀 FastAPI + Uvicorn + ROS2 Bridge

> **Node.js**로부터 메시지를 받아  
> **ROS2 Autoware 메시지**로 가공한 뒤 퍼블리시하는 FastAPI 서버

---

## 🔧 1️⃣ 기능

- 📬 **Node.js**는 FastAPI의 엔드포인트에 `JSON` 형식의 **추상화된 Autoware 메시지**를 전송  

<img width="695" height="405" alt="Screenshot from 2025-07-22 21-02-10" src="https://github.com/user-attachments/assets/4a7d0bb2-3135-4696-a760-d981d921c551" />



- 🧠 FastAPI는 해당 메시지를 파싱하여, ROS2의 **Autoware custom message**로 변환

- 🌐 FastAPI ↔ ROS2(Autoware) 간 통신은 **CycloneDDS** 기반의 ROS 2 DDS 통신을 사용

---

## ⚙️ 2️⃣ 사용법

### 🖥️ 호스트에서 CycloneDDS 및 ROS 환경 설정

> 컨테이너와 동일한 환경이어야 통신 가능함  
> (컨테이너의 `ROS_DOMAIN_ID`는 `30`, `RMW_IMPLEMENTATION`은 `rmw_cyclonedds_cpp`)

```bash
# 호스트에서 실행
export ROS_DOMAIN_ID=30
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

### 🐳 서버 실행 (Docker Compose)

```bash
cd ~
mkdir PMS_server
cd PMS_server
git clone https://github.com/Lee-seokgwon/PMS-ros2api_bridge.git
docker compose up --build
```

### 테스트

```bash
# Fastapi 엔드포인트에 테스트용 json 전송, 실제로는 node.js가 이 역할을 대신함
# Initial pose 설정
curl -X POST "http://localhost:8001/initialpose" \
  -H "Content-Type: application/json" \
  -d '{"x": -33.086097717285156, "y": 28.541202545166016, "yaw": 1.5708, "frame_id": "map"}'


# Goal 설정  
curl -X POST "http://localhost:8001/goal" \
  -H "Content-Type: application/json" \
  -d '{"x": -33.186100006103516, "y": 36.441200256347656, "yaw": 1.482, "frame_id": "map"}'


# Autonomous mode로 변경
curl -X POST "http://localhost:8001/operation_mode" \
  -H "Content-Type: application/json" \
  -d '{"mode": 2}'

# 서비스 상태 확인
curl "http://localhost:8001/health"
```

```bash
#호스트에서 fastapi에 요청 들어올때 마다 실시간으로 발사되는 토픽 확인
# 동일 네트워크, 동일 DDS구현체, 동일 ROS_DOMAIN_ID 이면 어디서든지 확인가능
ros2 topic echo /initialpose
ros2 topic echo /planning/mission_planning/goal
```

---

## 3️⃣ 🧑‍💻 Json 수신 방식 (VTS 참고자료)

FastAPI 서버는 다음과 같은 JSON 형식의 요청을 받습니다.  
모든 요청은 `Content-Type: application/json` 으로 POST해야 하며, 좌표계는 기본적으로 `"map"` 프레임을 기준으로 합니다.

---


### 🚦 1. 초기 위치 설정 (POST `/initialpose`)

```json
{
  "x": 1.0,
  "y": 2.0,
  "yaw": 0.7854,
  "frame_id": "map"
}
```


| 필드명     | 타입   | 설명                       |
|------------|--------|----------------------------|
| x          | float  | 초기 위치의 X 좌표         |
| y          | float  | 초기 위치의 Y 좌표         |
| yaw        | float  | 회전각 (라디안 단위)       |
| frame_id   | string | 기준 좌표계 (기본값: map)  |


---

### 🎯 2. 목표 지점 설정 (POST `/goal`)

```json
{
  "x": 5.0,
  "y": 3.5,
  "yaw": 1.5708,
  "frame_id": "map"
}
```

> 구조는 `/initialpose`와 동일하며, 차량이 도달해야 할 목표 위치를 나타냅니다.

---

### 🕹️ 3. 동작 모드 변경 (POST `/operation_mode`)

```json
{
  "mode": 2
}
```


| 모드 번호 | 설명                    |
|-----------|-------------------------|
| 1         | STOP (정지)             |
| 2         | AUTONOMOUS (자율주행)   |
| 3         | LOCAL (로컬 제어)       |
| 4         | REMOTE (원격 제어)      |


> 유효하지 않은 모드를 전송하면 400 에러가 반환됩니다.

---

### 🔍 4. 상태 확인 (GET `/health`)

서버 및 ROS 노드 상태를 확인할 수 있습니다.

응답 예시:
```json
{
  "status": "healthy",
  "message": "Service is running",
  "ros_node": "ros_bridge_node",
  "timestamp": {
    "sec": 1721680000,
    "nanosec": 123456789
  }
}
```


---


## 👥 4️⃣ Contributors

| 👤 이름 | 🛠️ 역할       |
|--------|------------|
| 이석권 | Developer |



<img width="1342" height="952" alt="Screenshot from 2025-07-22 20-43-14" src="https://github.com/user-attachments/assets/455b0a9c-c580-4d4c-b71c-9f1867a8f9ae" />
