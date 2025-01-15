## 드론&터틀봇 자율 협력 시스템 시뮬레이션

### 기록

| 기간 | 내용 | 폴더 |
| --- | --- | --- |
| 2024.12.17 ~ 2024.12.23 | RVIZ2와 Gazebo를 활용한 다중 로봇 자율 협력 시스템 시뮬레이션 | py_srvcli, multitb_interfaces, sjtu_drone, turtlebot3_multi_robot, yolo_human_navi |

### 기술 스택

| 분류 | 기술 |
| --- | --- |
| 개발 환경 | Ubuntu 22.04 |
| 개발 언어 | Python |
| 통신 프로토콜 | ROS2 |
| UI | Flask |
| 데이터베이스 | SQLite3 |
| 디자인 및 프로토타이핑 도구 | Figma |
| 시뮬레이션 및 시각화 도구 | gazebo, rviz |

### 시나리오

<aside>

원자력 발전소가 폭발한 위급한 상황에서, 터틀봇 3대가 방사선 위험 지역에 투입되어 생존자를 구조! 드론은 상공에서 현장 상황을 실시간으로 전달한다. 터틀봇과 드론의 협력 덕분에 구조팀은 생존자들을 무사히 구출한다.

</aside>

### **🎯 주요 기능**

| 패키지 | 설명 |
| --- | --- |
| multitb_interfaces | 서비스 인터페이스 패키지 |
| py_srvcli | 관제시스템 flask 생성 패키지 |
| turtlebot3_multi_robot | 다중 터틀봇 launch, world/모델 로드 패키지  |
| yolo_human_navi | 생존자 yolo 감지, 터틀봇 내비게이션 |
| sjtu_drone | 드론 패키지(https://github.com/NovoG93/sjtu_drone) |

### world

![world](https://github.com/user-attachments/assets/d79dce88-23d1-44ec-a2a7-bf6b70aab860)

### 결과물

[View PDF Document](./그룹E-2_7주차_협동-3_산출물.pdf)

### 실행영상

[![Watch the video](https://img.youtube.com/vi/H1p9tkmerFY/hqdefault.jpg)](https://www.youtube.com/watch?v=H1p9tkmerFY)

### 실행 방법

<aside>

1. Install Gazebo
    
    [https://classic.gazebosim.org/tutorials?tut=install_ubuntu](https://classic.gazebosim.org/tutorials?tut=install_ubuntu)
    
2. Install Simulation Package
    
    [https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#install-simulation-package](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#install-simulation-package)
    
3. Install Multi Robot Package
    
    [https://github.com/arshadlab/turtlebot3_multi_robot](https://github.com/arshadlab/turtlebot3_multi_robot)
    
</aside>

```python
# bashrc 설정
$ nano ~/.bashrc

source ~/turtlebot3_ws/install/setup.bash
export TURTLEBOT3_MODEL=waffle
source /usr/share/gazebo/setup.sh
```

```python
# Flask
$ ros2 run py_srvcli minimap 

# object detection
$ ros2 run yolo_human_navi human_detector 

# turtlebot navigation
$ ros2 run yolo_human_navi turtlebot_navi 

# gazebo
$ ros2 launch turtlebot3_multi_robot tmp.launch.py enable_drive:=False enable_rviz:=False
```

### 기여

- 진민혁: 드론 패키지 분석, urdf 수정, 전체 코드 연결
- 서연우: World 제작, object detection, 터틀봇 협력 시스템
- 배민지: Flask 관제 시스템 제작
