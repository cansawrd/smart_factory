# iw_hub Action Graph 구현 가이드

> Isaac Sim 5.1 GUI에서 직접 구현할 때 참고  
> Graph 경로: `{robot_prim_path}/ActionGraph`  
> Evaluator: **Execution**
>
> 두 대 운용 기준 이름: `iw_hub_01`, `iw_hub_02`  
> 예: `robot_prim_path=/World/Robots/iw_hub_01/iw_hub_sensors`

---

## 노드 배치 & 연결 전체도

```
┌─────────────────────────────────────────────────────────────────────────────┐
│  [data]  ROS2Context                [data]  ReadSimulationTime              │
│             │ context                           │ simulationTime             │
│             ├──────────────────────┬────────────┼──────────────────────┐    │
│             │                      │            │                      │    │
│  [exec]  OnPlaybackTick ──tick──►──┼────────────┼──────────────────────┼──► │
│                │                   │            │                      │    │
│     ┌──────────┼──────────┐        │            │                      │    │
│     ▼          ▼          ▼        │            │                      │    │
│                                                                             │
│  ── DRIVE ────────────────────────────────────────────────────────────────  │
│  SubTwist──►DiffCtrl──►WheelArtCtrl                                        │
│                                                                             │
│  ── LIFT ─────────────────────────────────────────────────────────────────  │
│  SubLift──►MakeLiftArray──►LiftArtCtrl                                     │
│                                                                             │
│  ── ODOM ─────────────────────────────────────────────────────────────────  │
│  ComputeOdom──►PubOdom                                                     │
│                                                                             │
│  ── TF ───────────────────────────────────────────────────────────────────  │
│  PubTF                                                                      │
│                                                                             │
│  ── LIDAR ────────────────────────────────────────────────────────────────  │
│  PubFrontLidar    PubBackLidar                                              │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## 노드별 상세

### ① ROS2Context　　`isaacsim.ros2.bridge.ROS2Context`
- execIn 없음 (data 노드)
- 설정값 없음 (domain_id 기본 0)
- **output** `context` → SubTwist, SubLift, PubOdom, PubTF, PubFrontLidar, PubBackLidar

---

### ② ReadSimulationTime　　`isaacsim.core.nodes.IsaacReadSimulationTime`
- execIn 없음 (data 노드)
- **output** `simulationTime` → PubOdom, PubTF, PubFrontLidar, PubBackLidar

---

### ③ OnPlaybackTick　　`omni.graph.action.OnPlaybackTick`
- **output** `tick` → 아래 모든 exec 노드

---

### ④ SubTwist　　`isaacsim.ros2.bridge.ROS2SubscribeTwist`

| 입력 포트 | 연결 출처 |
|-----------|----------|
| `execIn` | OnPlaybackTick.tick |
| `context` | ROS2Context.context |

| 파라미터 | 값 |
|----------|----|
| `topicName` | `/{robot_name}/cmd_vel` (예: `/iw_hub_01/cmd_vel`) |

| 출력 포트 | 타입 | 연결 대상 |
|-----------|------|----------|
| `linearVelocity` | `double[3]` | BreakLinVel.tuple |
| `angularVelocity` | `double[3]` | BreakAngVel.tuple |

---

### ④-a BreakLinVel　　`omni.graph.nodes.BreakVector3d`

> `double[3]` → x, y, z 분해 (linear용)

| 입력 포트 | 연결 출처 |
|-----------|----------|
| `tuple` | SubTwist.linearVelocity |

| 출력 포트 | 연결 대상 |
|-----------|----------|
| `x` | DiffCtrl.linearVelocity ← **전진 속도** |

---

### ④-b BreakAngVel　　`omni.graph.nodes.BreakVector3d`

> `double[3]` → x, y, z 분해 (angular용)

| 입력 포트 | 연결 출처 |
|-----------|----------|
| `tuple` | SubTwist.angularVelocity |

| 출력 포트 | 연결 대상 |
|-----------|----------|
| `z` | DiffCtrl.angularVelocity ← **회전 속도 (yaw)** |

---

### ⑤ DiffCtrl　　`isaacsim.core.nodes.IsaacDifferentialController`

| 입력 포트 | 타입 | 연결 출처 |
|-----------|------|----------|
| `execIn` | exec | OnPlaybackTick.tick |
| `linearVelocity` | `double` | BreakLinVel.**x** |
| `angularVelocity` | `double` | BreakAngVel.**z** |

| 파라미터 | 값 |
|----------|----|
| `wheelDistance` | `0.580` |
| `wheelRadius` | `0.080` |
| `maxLinearSpeed` | `1.8` |

| 출력 포트 | 연결 대상 |
|-----------|----------|
| `velocityCommand` | WheelArtCtrl.velocityCommand |

---

### ⑥ WheelArtCtrl　　`isaacsim.core.nodes.IsaacArticulationController`

| 입력 포트 | 연결 출처 |
|-----------|----------|
| `execIn` | OnPlaybackTick.tick |
| `velocityCommand` | DiffCtrl.velocityCommand |

| 파라미터 | 값 |
|----------|----|
| `targetPrim` | `{robot_prim_path}` (예: `/World/iw_hub`) |
| `jointNames` | `["left_wheel_joint", "right_wheel_joint"]` |

---

### ⑦ SubLift　　`isaacsim.ros2.bridge.ROS2SubscribeJointState`

| 입력 포트 | 연결 출처 |
|-----------|----------|
| `execIn` | OnPlaybackTick.tick |
| `context` | ROS2Context.context |

| 파라미터 | 값 |
|----------|----|
| `topicName` | `/{robot_name}/lift_cmd` (예: `/iw_hub_01/lift_cmd`) |

| 출력 포트 | 타입 | 연결 대상 |
|-----------|------|----------|
| `positionCommand` | `double[]` | LiftArtCtrl.positionCommand |
| `jointNames` | `string[]` | LiftArtCtrl.jointNames |

---

### ⑧ LiftArtCtrl　　`isaacsim.core.nodes.IsaacArticulationController`

| 입력 포트 | 연결 출처 |
|-----------|----------|
| `execIn` | OnPlaybackTick.tick |
| `positionCommand` | SubLift.positionCommand |
| `jointNames` | SubLift.jointNames |

| 파라미터 | 값 |
|----------|----|
| `targetPrim` | `{robot_prim_path}` |

---

### ⑩ ComputeOdom　　`isaacsim.core.nodes.IsaacComputeOdometry`

| 입력 포트 | 연결 출처 |
|-----------|----------|
| `execIn` | OnPlaybackTick.tick |

| 파라미터 | 값 |
|----------|----|
| `chassisPrim` | `[{robot_prim_path}]` ← **배열** 형태로 입력 |

| 출력 포트 | 연결 대상 |
|-----------|----------|
| `execOut` | PubOdom.execIn ← **execOut으로 체이닝** |
| `position` | PubOdom.position |
| `orientation` | PubOdom.orientation |
| `linearVelocity` | PubOdom.linearVelocity |
| `angularVelocity` | PubOdom.angularVelocity |

---

### ⑪ PubOdom　　`isaacsim.ros2.bridge.ROS2PublishOdometry`

| 입력 포트 | 연결 출처 |
|-----------|----------|
| `execIn` | **ComputeOdom.execOut** ← OnTick 아님 |
| `context` | ROS2Context.context |
| `timeStamp` | ReadSimulationTime.simulationTime |
| `position` | ComputeOdom.position |
| `orientation` | ComputeOdom.orientation |
| `linearVelocity` | ComputeOdom.linearVelocity |
| `angularVelocity` | ComputeOdom.angularVelocity |

| 파라미터 | 값 |
|----------|----|
| `topicName` | `/{robot_name}/odom` (예: `/iw_hub_01/odom`) |
| `chassisFrameId` | `{robot_name}/base_link` |
| `odomFrameId` | `{robot_name}/odom` |
| `publishRawVelocities` | `False` |

---

### ⑫ PubTF　　`isaacsim.ros2.bridge.ROS2PublishTransformTree`

| 입력 포트 | 연결 출처 |
|-----------|----------|
| `execIn` | OnPlaybackTick.tick |
| `context` | ROS2Context.context |
| `timeStamp` | ReadSimulationTime.simulationTime |

| 파라미터 | 값 |
|----------|----|
| `targetPrims` | `[{robot_prim_path}]` |
| `parentPrim` | *(비워두기)* |

---

### ⑬ ReadFrontLidar　　`isaacsim.sensors.physx.IsaacReadLidarBeams`

| 입력 포트 | 타입 | 연결 출처 |
|-----------|------|----------|
| `execIn` | exec | OnPlaybackTick.tick |
| `lidarPrim` | target | Stage에서 `{robot_prim_path}/front_2d_lidar` 드래그 |

| 출력 포트 | 연결 대상 |
|-----------|----------|
| `execOut` | PubFrontLidar.execIn ← **체이닝** |
| `azimuthRange` | PubFrontLidar.azimuthRange |
| `depthRange` | PubFrontLidar.depthRange |
| `horizontalFov` | PubFrontLidar.horizontalFov |
| `horizontalResolution` | PubFrontLidar.horizontalResolution |
| `intensitiesData` | PubFrontLidar.intensitiesData |
| `linearDepthData` | PubFrontLidar.linearDepthData |
| `numCols` | PubFrontLidar.numCols |
| `numRows` | PubFrontLidar.numRows |
| `rotationRate` | PubFrontLidar.rotationRate |

---

### ⑭ PubFrontLidar　　`isaacsim.ros2.bridge.ROS2PublishLaserScan`

| 입력 포트 | 연결 출처 |
|-----------|----------|
| `execIn` | **ReadFrontLidar.execOut** ← OnTick 아님 |
| `context` | ROS2Context.context |
| `timeStamp` | ReadSimulationTime.simulationTime |
| `azimuthRange` | ReadFrontLidar.azimuthRange |
| `depthRange` | ReadFrontLidar.depthRange |
| `horizontalFov` | ReadFrontLidar.horizontalFov |
| `horizontalResolution` | ReadFrontLidar.horizontalResolution |
| `intensitiesData` | ReadFrontLidar.intensitiesData |
| `linearDepthData` | ReadFrontLidar.linearDepthData |
| `numCols` | ReadFrontLidar.numCols |
| `numRows` | ReadFrontLidar.numRows |
| `rotationRate` | ReadFrontLidar.rotationRate |

| 파라미터 | 값 |
|----------|----|
| `topicName` | `/{robot_name}/front_2d_lidar/scan` |
| `frameId` | `{robot_name}/front_2d_lidar` |

---

### ⑮ ReadBackLidar　　`isaacsim.sensors.physx.IsaacReadLidarBeams`

| 입력 포트 | 타입 | 연결 출처 |
|-----------|------|----------|
| `execIn` | exec | OnPlaybackTick.tick |
| `lidarPrim` | target | Stage에서 `{robot_prim_path}/back_2d_lidar` 드래그 |

| 출력 포트 | 연결 대상 |
|-----------|----------|
| `execOut` | PubBackLidar.execIn ← **체이닝** |
| `azimuthRange` | PubBackLidar.azimuthRange |
| `depthRange` | PubBackLidar.depthRange |
| `horizontalFov` | PubBackLidar.horizontalFov |
| `horizontalResolution` | PubBackLidar.horizontalResolution |
| `intensitiesData` | PubBackLidar.intensitiesData |
| `linearDepthData` | PubBackLidar.linearDepthData |
| `numCols` | PubBackLidar.numCols |
| `numRows` | PubBackLidar.numRows |
| `rotationRate` | PubBackLidar.rotationRate |

---

### ⑯ PubBackLidar　　`isaacsim.ros2.bridge.ROS2PublishLaserScan`

| 입력 포트 | 연결 출처 |
|-----------|----------|
| `execIn` | **ReadBackLidar.execOut** ← OnTick 아님 |
| `context` | ROS2Context.context |
| `timeStamp` | ReadSimulationTime.simulationTime |
| `azimuthRange` | ReadBackLidar.azimuthRange |
| `depthRange` | ReadBackLidar.depthRange |
| `horizontalFov` | ReadBackLidar.horizontalFov |
| `horizontalResolution` | ReadBackLidar.horizontalResolution |
| `intensitiesData` | ReadBackLidar.intensitiesData |
| `linearDepthData` | ReadBackLidar.linearDepthData |
| `numCols` | ReadBackLidar.numCols |
| `numRows` | ReadBackLidar.numRows |
| `rotationRate` | ReadBackLidar.rotationRate |

| 파라미터 | 값 |
|----------|----|
| `topicName` | `/{robot_name}/back_2d_lidar/scan` |
| `frameId` | `{robot_name}/back_2d_lidar` |

---

## ROS2 토픽 요약

| 방향 | 토픽 | 타입 | 값 범위 |
|------|------|------|---------|
| ← Sub | `/iw_hub_01/cmd_vel`, `/iw_hub_02/cmd_vel` | `geometry_msgs/Twist` | — | ex) ros2 topic pub /iw_hub_01/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
| ← Sub | `/iw_hub_01/lift_cmd`, `/iw_hub_02/lift_cmd` | `sensor_msgs/JointState` | position[0]: 0.000 ~ 0.040 m | ex) ros2 topic pub --once /iw_hub_01/lift_cmd sensor_msgs/msg/JointState "{name: ['lift_joint'], position: [0.04]}"
| → Pub | `/iw_hub_01/odom`, `/iw_hub_02/odom` | `nav_msgs/Odometry` | — |
| → Pub | `/tf` | `tf2_msgs/TFMessage` | — |
| → Pub | `/iw_hub_01/front_2d_lidar/scan`, `/iw_hub_02/front_2d_lidar/scan` | `sensor_msgs/LaserScan` | — |
| → Pub | `/iw_hub_01/back_2d_lidar/scan`, `/iw_hub_02/back_2d_lidar/scan` | `sensor_msgs/LaserScan` | — |
