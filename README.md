# 🐾 ROBOKRATES: 동물 수술용 보조 로봇

수의사 보조 인력 부족 문제 해결을 위한 ROS2 기반 협동로봇 프로젝트

---

## 🩺 프로젝트 개요

### 🎯 **프로젝트 소개 및 목적**

현재 수의사 의료 현장은 **보조 인력의 부족**, **높은 노동 강도** 등으로 많은 어려움을 겪고 있습니다.  
2024년, **두산로보틱스**는 사람을 대상으로 한 내시경 수술에서 협동로봇을 성공적으로 적용하여 보조 인력의 부담을 줄이고 수술의 정밀도를 향상시켰습니다.  

`ROBOKRATES`는 이와 같은 기술을 **동물 수술에 적용**함으로써, 수의사의 피로를 줄이고 수술 효율성과 정확도를 동시에 향상시키고자 합니다.

---

## 🤖 사용 장비 및 개발 환경

### 💻 **하드웨어**

| 장비 | 사양 |
|------|------|
| **로봇 제어용 PC** | RTX4060 탑재 MSI 노트북, Ubuntu 22.04, ROS 2 Humble |
| **두산 M0609 협동로봇** | 6축 고성능 모터, 가반하중 6kg, 작업반경 900mm, 반복정밀도 ±0.03mm |
| **RG2 그리퍼** | 그리핑 범위: 0~110mm, 힘: 40N, Payload: Force Fit 2kg / Form Fit 5kg |
| **Intel RealSense D435i** | 스테레오 Depth 센서 + IR Projector + RGB 카메라 |

---

## 🧠 소프트웨어 및 기술 스택

### 🧬 **AI 및 컴퓨터 비전**

- **YOLOv8n**: 로보플로우 기반 라벨링 및 학습, ultralytics 라이브러리 사용  
- **Deep SORT**: 객체 추적  
- **OpenCV**: 영상처리 (외곽선 검출 등)

### 🌐 **웹 UI 및 음성 인터페이스**

- **Flask + Flask-SocketIO**: 실시간 영상 스트리밍 및 감지 결과 전송
- **STT / TTS**: gTTS, playsound, openwakeword, langchain, pyaudio, sounddevice
- **DICOM Viewer**: pydicom, PIL 기반 의료 영상 뷰어 기능

### ⚙️ **운영체제 및 ROS2 통신**

- **OS**: Ubuntu 22.04 LTS
- **ROS2**: Humble (Python 기반 rclpy 멀티 노드 구조, 총 9개 노드 운영)
- **로봇 통신**: Doosan DRL API, pymodbus (Modbus TCP 기반 RG2 그리퍼 제어)
- **센서 통합**: Intel RealSense 연동 (cv_bridge, sensor_msgs/Image)

---

## 🔁 주요 기능 시나리오

1. 로봇이 **자체 음성 안내**로 환자의 병적 상태를 수의사에게 전달  
2. 수의사 음성 인식 후, 로봇이 **메스**를 집어 수의사의 손에 전달  
3. 음성 명령에 따라, 로봇이 **소독 스프레이**를 잡아 환부에 분사  
4. 음성 명령에 따라, 로봇이 **석션 도구**를 사용해 이물질 흡입  
5. 석션 도구를 제자리에 복귀시키고 **로봇은 홈 위치로 원복**

---

## 📡 ROS2 노드 구성 (요약)

※ 자세한 노드 간 통신 구조는 내부 문서 혹은 별도 아키텍처 다이어그램 참고

- **Keyword Node**: STT/명령어 처리
- **Tracking Node**: 객체 추적 및 검출 연동
- **Robot Control Node**: 로봇 제어 및 작업 명령 수행
- **Gripper Node**: RG2 그리퍼 제어
- **Detection Node**: YOLO 기반 객체 감지 및 추론
- **UI Node**: 웹 기반 인터페이스와 SocketIO 통신
- **Voice Output Node**: 수술 상태 음성 출력
- **Image Streaming Node**: RealSense 영상 송출
- **Medical Viewer Node**: DICOM 뷰어 연동

---

## 🧠 get_keyword 노드

사용자의 음성 명령을 인식하여 **도구(Object)** 및 **목적지(Target)** 정보를 추출하고, 이를 ROS2 서비스 형태로 다른 노드(예: robot_control)로 전달하는 **음성 기반 인터페이스 핵심 노드**입니다.

---

### 📌 주요 기능

1. **Wake Word 감지**
   - `"hello rokey"`를 감지하면 대기 상태에서 활성 상태로 전환
   - `"Yes, I'm ready"` TTS 응답으로 사용자와 인터랙션 시작

2. **STT → GPT-4o 기반 명령어 분석**
   - 사용자의 음성을 **텍스트로 변환 (STT)**
   - LangChain + GPT-4o 모델을 통해 텍스트에서 명령어 추출  
   - 예:  
     - `"메스"` → `object: scalpel`, `target: hands`  
     - `"트래킹 시작해줘"` → `object: start`, `target: tracking`

3. **명령어 전송 및 응답 처리**
   - 추출된 명령어를 ROS2 서비스 응답 및 Socket.IO를 통해 다른 모듈로 전달  
   - 명령 유형에 따라 **TTS 안내** 또는 **웹 UI 알림 송신**

---

### 🗣️ 지원 명령어 예시

| 사용자 입력 | Object | Target | 특이 처리 |
|-------------|--------|--------|------------|
| "메스" | scalpel | hands | TTS 응답 |
| "스프레이" | spray | scar | "소독을 시작합니다" |
| "석션" | suction | scar | TTS 응답 |
| "수술정보" | info | info | Socket.IO로 info emit |
| "트래킹 시작해줘" | start | tracking | TTS "tracking_start" |
| "트래킹 종료해줘" | stop | tracking | TTS "tracking_stop" |

---

### 🧬 AI 처리 파이프라인

```text
STT (MicController → OpenAI Whisper) 
→ LangChain Prompt (GPT-4o) 
→ 도구 및 목적지 추출 
→ ROS2 Service 응답 or SocketIO 이벤트 발행

---




## 🧑‍💻 팀 소개

**TEAM C-4조 - ROBOKRATES**  
- 이희우  
- 정석환  
- 김동건  
- 김재훈

---

## 📎 라이선스

본 프로젝트는 연구 및 학습 목적으로 개발되었으며, 상업적 사용을 금합니다.

---

## 📬 문의

기술 관련 문의는 팀원에게 직접 연락 부탁드립니다.
