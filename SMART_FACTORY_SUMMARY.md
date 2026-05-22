# Smart Factory 프로젝트 정리

## 1. 프로젝트 개요

`smart_factory`는 Isaac Sim Nova Carter 시나리오를 기준으로 만든 스마트 물류/공장 자동화용 ROS 2 Python 패키지이다. 공장 내부의 로봇이 입고 지점에서 물건을 가져와 분류 지점으로 이동하거나, 대기 구역의 선반을 목적 슬롯으로 운반하는 상황을 모델링한다.

핵심 목표는 다음과 같다.

- 여러 로봇과 작업을 대상으로 우선순위 기반 배차 수행
- 격자 지도에서 A* 기반 최단 경로 계획
- 시간 예약 테이블을 이용한 로봇 간 충돌 회피
- 선반 크기와 점유 면적을 고려한 운반 경로 계획
- ArUco 마커 인식 결과를 이용한 픽업/드롭 정렬 명령 생성
- ROS 2 노드로 계획 결과와 속도 명령 발행

## 2. 전체 구조

```text
smart_factory/
├── config/
│   └── factory_map.yaml
├── launch/
│   └── task_manager.launch.py
├── smart_factory/
│   ├── aruco_alignment.py
│   ├── demo.py
│   ├── dispatcher.py
│   ├── footprint_reservation.py
│   ├── graph.py
│   ├── grid_planner.py
│   ├── models.py
│   ├── occupancy_grid.py
│   ├── reservation.py
│   ├── sample_world.py
│   ├── shelf_experiment.py
│   ├── shelf_geometry.py
│   ├── shelf_transport_planner.py
│   └── task_manager_node.py
├── test/
│   ├── test_aruco_alignment.py
│   ├── test_planning.py
│   └── test_shelf_transport.py
├── package.xml
└── setup.py
```

## 3. 주요 기능

### 3.1 작업 배차

관련 파일:

- `smart_factory/dispatcher.py`
- `smart_factory/graph.py`
- `smart_factory/reservation.py`
- `smart_factory/models.py`

`TaskDispatcher`는 대기 중인 작업과 유휴 로봇을 받아 각 로봇에 작업을 할당한다.

동작 방식:

1. `WAITING` 상태의 작업을 우선순위가 높은 순서로 정렬한다.
2. `IDLE` 상태의 로봇을 사용 가능 시간과 로봇 ID 기준으로 정렬한다.
3. 각 작업에 대해 픽업 지점까지 가장 빨리 도착할 수 있는 로봇을 선택한다.
4. `shortest_path()`로 현재 위치에서 픽업 지점, 픽업 지점에서 드롭 지점까지 경로를 계산한다.
5. `ReservationTable`로 시간별 노드/엣지 충돌을 확인하고 필요한 경우 대기 동작을 삽입한다.
6. `PlannedRoute`를 생성하고 작업과 로봇 상태를 갱신한다.

예약 테이블은 다음 충돌을 방지한다.

- 같은 시간에 같은 waypoint를 점유하는 경우
- 서로 반대 방향으로 같은 edge를 통과하는 경우
- 다음 시간의 목표 waypoint가 이미 예약된 경우

### 3.2 격자 기반 경로 계획

관련 파일:

- `smart_factory/grid_planner.py`
- `smart_factory/occupancy_grid.py`
- `smart_factory/shelf_geometry.py`

격자 경로 계획은 Manhattan distance를 휴리스틱으로 사용하는 A* 방식이다. 이동은 상하좌우 직교 이동만 허용한다.

특징:

- 시작점 또는 목표점이 막혀 있으면 예외 발생
- 장애물과 지도 경계를 고려한 neighbor 계산
- 일반 로봇 이동뿐 아니라 선반 운반 시 선반 전체 footprint도 함께 고려

### 3.3 선반 운반 계획

관련 파일:

- `smart_factory/shelf_transport_planner.py`
- `smart_factory/footprint_reservation.py`
- `smart_factory/occupancy_grid.py`
- `smart_factory/shelf_geometry.py`
- `smart_factory/sample_world.py`

`ShelfTransportPlanner`는 로봇이 대기 구역의 선반 아래로 접근한 뒤, 선반을 들고 목적 슬롯까지 이동하는 계획을 만든다.

계획 단계:

1. 목적 슬롯이 비어 있는지 확인한다.
2. source zone에 있는 대기 선반을 선택한다.
3. 로봇 단독 상태로 목표 선반 중심까지 접근 경로를 만든다.
4. 접근 경로를 시간 예약 테이블에 등록한다.
5. 선반을 든 상태로 목적 슬롯까지 이동할 경로를 만든다.
6. 운반 중에는 선반의 전체 footprint가 다른 선반이나 예약 경로와 겹치지 않도록 검사한다.

선반 geometry는 기본적으로 3x3 cell footprint를 사용하며, 선반 다리 위치와 전체 점유 영역을 분리해서 처리한다.

### 3.4 ArUco 정렬 제어

관련 파일:

- `smart_factory/aruco_alignment.py`
- `smart_factory/models.py`
- `test/test_aruco_alignment.py`

`ArucoAlignmentController`는 마커 관측값을 기반으로 로봇 정렬 명령을 생성한다.

지원하는 정렬:

- 선반 하부 ArUco 마커 기반 픽업 정렬
- 벽면 슬롯 ArUco 마커와 슬롯 offset 기반 드롭 정렬

정렬 오차가 허용 범위 안에 들어오면 다음 상태로 넘어갈 수 있는 reason을 반환한다.

- 픽업 완료 가능: `ready_to_lift_up`
- 드롭 완료 가능: `ready_to_lift_down`
- 정렬 중: `aligning`

기본 허용 오차:

- x 오차: `0.03 m`
- y 오차: `0.03 m`
- yaw 오차: `0.035 rad`

생성되는 명령:

- `linear_x`
- `linear_y`
- `angular_z`
- `aligned`
- `reason`

### 3.5 ROS 2 Task Manager 노드

관련 파일:

- `smart_factory/task_manager_node.py`
- `launch/task_manager.launch.py`

`TaskManagerNode`는 샘플 공장 맵, 로봇, 작업을 생성한 뒤 `TaskDispatcher`로 계획을 만든다.

ROS 2 인터페이스:

- 발행 토픽 `/iw_hub_01/cmd_vel`
  - 타입: `geometry_msgs/msg/Twist`
  - 샘플 active plan을 따라 단순 전진 명령 발행
- 발행 토픽 `/smart_factory/plan`
  - 타입: `std_msgs/msg/String`
  - 로봇별 작업 ID와 waypoint 경로 요약 발행

실행 파일:

- `task_manager`
- `factory_demo`
- `shelf_experiment`

## 4. 샘플 월드

### 4.1 물류 분류 맵

`make_sample_factory_map()`은 7x5 격자 맵을 생성한다.

주요 waypoint:

| 이름 | 위치 | 의미 |
| --- | --- | --- |
| `IN_A` | `(0, 0)` | 입고 지점 A |
| `IN_B` | `(0, 4)` | 입고 지점 B |
| `SORT_RED` | `(6, 0)` | 빨간 물품 분류 지점 |
| `SORT_BLUE` | `(6, 4)` | 파란 물품 분류 지점 |
| `STACK` | `(5, 2)` | 적재 지점 |
| `CHARGE` | `(1, 2)` | 충전 지점 |

장애물:

- `(3, 1)`
- `(3, 2)`
- `(3, 3)`

샘플 로봇:

- `iw_hub_01`: `CHARGE`에서 시작
- `iw_hub_02`: `N1_4`에서 시작

샘플 작업:

- `box_001`: `IN_A`에서 `SORT_RED`로 이동, priority 2
- `box_002`: `IN_B`에서 `SORT_BLUE`로 이동, priority 1
- `box_003`: `IN_A`에서 `STACK`으로 이동, priority 0

### 4.2 선반 운반 월드

`make_shelf_transport_world()`는 16x13 격자에서 선반과 슬롯을 생성한다.

초기 선반:

| 선반 ID | 중심 위치 | zone | slot | marker |
| --- | --- | --- | --- | --- |
| `shelf_a_1` | `(3, 2)` | A | A-1 | 201 |
| `shelf_b_1` | `(3, 6)` | B | B-1 | 301 |
| `shelf_c_1` | `(3, 10)` | C | C-1 | 401 |

목적 슬롯 예시:

- D zone: `D-1`, `D-2`, `D-3`
- E zone: `E-1`, `E-2`, `E-3`
- F zone: `F-1`, `F-2`, `F-3`

벽면 마커 offset 예시:

- `D-1`: marker 801
- `D-2`: marker 802
- `E-2`: marker 902
- `F-2`: marker 1002

## 5. 실행 방법

### 5.1 패키지 빌드

워크스페이스 루트에서 실행한다.

```bash
colcon build --packages-select smart_factory
source install/setup.bash
```

### 5.2 배차 및 선반 운반 데모

```bash
ros2 run smart_factory factory_demo
```

출력 내용:

- 로봇별 배정 작업
- 시작/픽업/완료 시간
- waypoint 경로
- 선반 접근 경로
- 선반 운반 경로

### 5.3 선반 운반 실험

기본값으로 D-1 슬롯까지 선반을 운반한다.

```bash
ros2 run smart_factory shelf_experiment
```

목적 슬롯과 source zone을 지정할 수 있다.

```bash
ros2 run smart_factory shelf_experiment --target E-2 --source-zone A
```

주요 옵션:

| 옵션 | 설명 |
| --- | --- |
| `--target` | 목적 슬롯, 예: `D-1`, `E-2` |
| `--source-zone` | 선반 출발 zone, 예: `A`, `B`, `C` |
| `--robot` | 로봇 ID |
| `--robot-x`, `--robot-y` | 로봇 시작 grid 좌표 |
| `--pickup-dx`, `--pickup-dy`, `--pickup-dyaw` | 픽업 ArUco 오차 |
| `--wall-x`, `--wall-y`, `--wall-yaw` | 벽면 마커 pose |
| `--robot-pose-x`, `--robot-pose-y`, `--robot-pose-yaw` | 드롭 위치 근처 로봇 pose |

### 5.4 Task Manager 노드 실행

```bash
ros2 launch smart_factory task_manager.launch.py
```

또는 직접 실행할 수 있다.

```bash
ros2 run smart_factory task_manager
```

## 6. 테스트

테스트는 ROS 2 환경 없이도 알고리즘 단위로 실행될 수 있도록 구성되어 있다.

```bash
pytest
```

또는 ROS 2 패키지 테스트로 실행한다.

```bash
colcon test --packages-select smart_factory
colcon test-result --verbose
```

테스트 범위:

| 테스트 파일 | 검증 내용 |
| --- | --- |
| `test/test_planning.py` | 작업 배차, 예약 충돌 회피, 대기 삽입 |
| `test/test_shelf_transport.py` | 선반 운반 경로, 직교 이동, footprint 충돌 회피, footprint 예약 |
| `test/test_aruco_alignment.py` | 픽업/드롭 ArUco 정렬 명령, 허용 오차, pose composition |

## 7. 현재 구현의 특징

- 공장 내 물류 이동을 waypoint 기반 모델과 grid 기반 모델로 나누어 다룬다.
- 작업 배차는 우선순위와 로봇 도착 예상 시간을 함께 고려한다.
- 단순 최단 경로뿐 아니라 시간 예약을 통해 로봇 간 충돌을 줄인다.
- 선반 운반에서는 로봇 중심점만 보지 않고 선반의 전체 점유 영역을 검사한다.
- ArUco 기반 정렬은 실제 lift up/down 판단에 필요한 aligned 상태와 reason을 함께 제공한다.
- ROS 2가 없는 환경에서도 알고리즘 테스트가 가능하도록 `rclpy` import 실패를 허용한다.

## 8. 향후 확장 아이디어

- 실제 Isaac Sim 월드의 좌표계와 grid/waypoint 좌표 변환 연결
- `/iw_hub_01/cmd_vel`, `/iw_hub_02/cmd_vel` 단순 전진 대신 waypoint 추종 제어기 적용
- 로봇별 상태 피드백 구독 후 실시간 재계획
- YAML 설정을 직접 읽어 샘플 월드 대신 외부 맵으로 초기화
- ArUco detection topic 구독과 alignment command 발행 노드 분리
- 선반 운반 완료 후 슬롯 점유 상태 갱신
- 다중 선반 운반 작업 큐와 우선순위 정책 추가

## 9. 핵심 파일 요약

| 파일 | 역할 |
| --- | --- |
| `models.py` | 로봇, 작업, 경로, 선반, 슬롯, ArUco 정렬 관련 데이터 모델 |
| `dispatcher.py` | 물류 작업 배차 및 waypoint 경로 계획 |
| `graph.py` | waypoint graph 생성 및 shortest path 계산 |
| `reservation.py` | waypoint/edge 기반 시간 예약 테이블 |
| `grid_planner.py` | grid A* 경로 계획 |
| `occupancy_grid.py` | 선반 점유를 반영한 grid 생성 |
| `shelf_geometry.py` | 선반 footprint, leg cell, 운반 footprint 계산 |
| `footprint_reservation.py` | 선반 footprint 기반 시간 예약 |
| `shelf_transport_planner.py` | 선반 접근 및 운반 경로 계획 |
| `aruco_alignment.py` | ArUco 마커 기반 픽업/드롭 정렬 제어 |
| `sample_world.py` | 샘플 공장 맵, 로봇, 작업, 선반, 슬롯 생성 |
| `task_manager_node.py` | ROS 2 task manager 노드 |
| `demo.py` | 배차와 선반 운반 데모 |
| `shelf_experiment.py` | CLI 기반 선반 운반 및 ArUco 정렬 실험 |
