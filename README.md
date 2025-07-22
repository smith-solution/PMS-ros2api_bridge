# 🚀 FastAPI + Uvicorn + ROS2 Bridge

> **Node.js**로부터 메시지를 받아  
> **ROS2 Autoware 메시지**로 가공한 뒤 퍼블리시하는 FastAPI 서버

---

## 🔧 1️⃣ 기능

- 📬 **Node.js**는 FastAPI의 엔드포인트에 `JSON` 형식의 **추상화된 Autoware 메시지**를 전송  
  예시:  
  ```json
  {
    "purpose": "initialpose",
    "payload": {"x" : 2 , "y" : 3, "yaw" : 2.3}
  }
  ```

<img width="716" height="509" alt="Screenshot from 2025-07-22 17-18-41" src="https://github.com/user-attachments/assets/d92da969-df50-4cdf-94cb-5afd876517d6" />

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
docker compose up
```

```bash
# Fastapi 엔드포인트에 테스트용 json 전송
curl -X POST http://localhost:8000/send_message -H "Content-Type: application/json" -d '{"purpose": "initialpose", "payload": {"x": 1.23, "y": 4.56, "yaw": 1.57}}'

#호스트에서 fastapi에 요청 들어올때 마다 실시간으로 발사되는 토픽 확인
ros2 topic echo /bridge_topic
```

## 👥 3️⃣ Contributors

| 👤 이름 | 🛠️ 역할       |
|--------|------------|
| 이석권 | Developer |



<img width="1291" height="952" alt="Screenshot from 2025-07-22 16-42-52" src="https://github.com/user-attachments/assets/1b587297-64a5-4da3-b925-ee368fb72194" />
