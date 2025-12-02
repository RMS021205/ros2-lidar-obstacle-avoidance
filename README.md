# ROS2 LiDAR Obstacle Avoidance

LiDAR(/scan) 데이터를 기반으로 주변 환경을 감지하고  
상태머신(FORWARD → CHECK → BACKWARD → TURN)을 통해 경로를 결정하여  
`/cmd_vel` 토픽으로 로봇의 이동을 제어하는 ROS2 자율주행 노드입니다.

---

## 📡 ROS2 구조 개요

### Node  
ROS2에서 실행되는 하나의 독립 프로그램 단위이며,  
본 프로젝트에서는 아래 노드가 핵심 제어를 담당합니다:




## 상태머신 구조
1) FORWARD

   정면 거리 0.60m 이상 확보 시 전진

   속도: linear.x = 0.25

2) CHECK

   좌/우 공간 분석

   left_space < 0.50m and right_space < 0.50m → BACKWARD

   아니면 더 넓은 방향으로 TURN

3) BACKWARD

   1.0초간 후진

   속도: linear.x = -0.15

4) TURN

   방향: 좌(angular.z = 0.5), 우(angular.z = -0.5)

   최소 회전 시간: 0.8초

   정면 0.80m 이상 확보 시 → FORWARD 복귀

   3.0초 초과 + 앞이 막힘 → BACKWARD로 강제 탈출


