## 드론&터틀봇 자율 협력 시스템 시뮬레이션

### 기록

| 기간 | 내용 | 폴더 |
| --- | --- | --- |
| 2024.12.17 ~ 2024.12.23 | RVIZ2와 Gazebo를 활용한 다중 로봇 자율 협력 시스템 시뮬레이션 | multitb_interfaces, py_srvcli, turtlebot3_multi_robot, yolo_human_navi, sjtu_drone |

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

### map

![map](https://github.com/user-attachments/assets/e31e68f5-3164-440a-b1c9-e00f0fb911f7)

### model

![dog](https://github.com/user-attachments/assets/cd42b79d-3a58-4305-88cd-682042a3b284)
![bird](https://github.com/user-attachments/assets/75c51826-8150-4724-9a1c-8f7fbe8e22c0)

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

# gazebo
$ ros2 launch turtlebot3_multi_robot tmp.launch.py enable_drive:=False enable_rviz:=False

# turtlebot navigation
$ ros2 run yolo_human_navi turtlebot_navi 

# drone navigation
$ ros2 run sjtu_drone_control test_moveto
```

### 주의사항

아래 경로는 본인의 작업 환경에 맞게 수정하세요.

```python
# py_srvcli/minimap.py
app = Flask(
    __name__,
    template_folder="/home/ynu/Rokey/project7/prj7_git/src/py_srvcli/py_srvcli/templates",
    static_folder="/home/ynu/Rokey/project7/prj7_git/src/py_srvcli/py_srvcli/static",
)

# turtlebot3_multi_robot/map/map.yaml
image: /home/ynu/Rokey/project7/prj7_git/src/turtlebot3_multi_robot/map/map.pgm

# turtlebot3_multi_robot/models/turtlebot3_waffle/model.sdf
<mesh>
  <uri>/home/ynu/Rokey/project7/prj7_git/src/turtlebot3_multi_robot/models/turtlebot3_waffle/meshes/waffle_base_dog.dae</uri>
  <scale>0.001 0.001 0.001</scale>
</mesh>

# turtlebot3_multi_robot/worlds/mini_city_add_human.world <model: tb1, tb2, tb3>
<mesh>
  <uri>/home/ynu/Rokey/project7/prj7_git/src/turtlebot3_multi_robot/models/turtlebot3_waffle/meshes/waffle_base_dog.dae</uri>
  <scale>0.001 0.001 0.001</scale>
</mesh>
```

작업 환경에 맞게 수정 후, Flask 서버와 gazebo를 실행하세요.

### 기여

- 진민혁: 모델 mesh 수정, 드론 협력 시스템
- 서연우: World 제작, object detection, 터틀봇 협력 시스템
- 배민지: Flask 관제 시스템 제작
