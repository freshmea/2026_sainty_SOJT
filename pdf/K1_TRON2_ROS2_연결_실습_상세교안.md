# K1·TRON2 시뮬레이션을 ROS 2에 연결하기

실습의 구조

이번 실습에서는 시뮬레이터에서 얻은 관절각 하나가 ROS 메시지를 거쳐 rosbag 파일에 저장되는 과정을 추적한다. 두 로봇은 같은 관찰 코드를 쓰지만, 관절값을 얻는 지점과 명령이 전달되는 방향은 다르다.

### 두 로봇에서 데이터가 흐르는 방향

K1   관찰 노드가 /k1/cmd_vel로 목표 속도를 보낸다. verify_policy.py는 목표를 보행 정책에 입력하고 MuJoCo를 진행한 뒤, 관절 상태와 IMU를 ROS에 발행한다.

TRON2   SF_TRON2A 시뮬레이터가 루프백에서 LimX SDK 상태를 제공한다. sdk_bridge.py가 SDK 콜백으로 받은 상태를 ROS에 발행한다. 이 경로에서 관찰 노드는 로봇에 속도 명령을 보내지 않는다.

### 코드를 열어 둘 위치

```
cd ~/sainty/day1
ls ros_lab/{run.sh,telemetry.py,observe.py,replay.sh}
ls K1/verify_policy.py tron2_sdk/sdk_bridge.py
```

run.sh는 프로세스를 시작하고 결과 폴더를 만든다. telemetry.py는 로봇별 값을 공통 메시지로 변환한다. observe.py는 구독·검사·기록을 담당하고, replay.sh는 저장된 bag을 재생해 다시 받은 값을 대조한다.

### 실습 환경과 관찰 터미널

WSL2 Ubuntu 22.04, ROS 2 Humble, 시스템 Python 3.10, uv를 사용한다. K1과 TRON2는 서로 다른 잠금 파일로 환경을 구성한다. 실행 터미널과 별도로 아래 명령을 입력한 터미널을 준비한다. 실행이 짧으므로 관찰 명령을 먼저 대기시켜도 된다.

```
source /opt/ros/humble/setup.bash
export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=86
ros2 topic echo /k1/joint_states sensor_msgs/msg/JointState --once
```

**완료 기준**  각 로봇에서 두 토픽을 60개씩 기록하고 재생한 120개 메시지의 필드가 일치하는지 확인한다. K1에서는 명령 수신과 만료를 CSV에서도 확인한다.

## 0.15 m/s 명령을 보낸 뒤 어떤 값이 돌아오는가

K1 · 명령에서 피드백까지

### 실행하고 결과 위치 보관하기

```
source /opt/ros/humble/setup.bash
cd ~/sainty/day1
ROS_DOMAIN_ID=86 bash ros_lab/run.sh k1
read -r -p 'Evidence directory 경로를 붙여 넣으세요: ' K1_RUN
```

run.sh k1은 K1 프로젝트에서 uv sync --locked를 수행하고 results/ros-…에 이번 실행만의 결과 폴더를 만든다. verify_policy.py --no-render --ros2는 백그라운드에서 상태를 생산한다. observe.py k1 23은 관절 수 23을 기대하며 앞에서 두 토픽을 구독한다.

### 명령을 만드는 코드와 소비하는 코드

observe.py의 K1 분기는 Twist.linear.x를 0.15로 설정하고 0.1초 간격의 타이머로 발행한다. verify_policy.py의 accept_twist는 유한한 값만 받아 ±0.3 m/s 범위로 제한하고, 수신 시각을 단조 시계로 저장한다.

```
# K1/verify_policy.py 에서 제어 틱마다 실행되는 두 줄
rclpy.spin_once(telemetry, timeout_sec=0)
command[0] = latest_cmd[0] if time.monotonic() - latest_cmd[1] < 0.5 else 0.0
```

정책은 명령과 상태 이력을 받아 관절 목표 23개를 낸다. PD 제어로 토크를 계산하고 MuJoCo가 물리를 적분한다. 발행하는 관절각은 적분 뒤 읽은 data.qpos이며, 속도와 힘은 data.qvel과 data.actuator_force에서 온다.

### 명령이 끊긴 흔적 찾기

```
python3 - "$K1_RUN/sim/policy_rollout.csv" <<'PY'
import csv, sys
with open(sys.argv[1]) as f:
    rows = list(csv.DictReader(f))
cmd = [float(r["command_x"]) for r in rows]
print("received", 0.15 in cmd, "final", cmd[-1])
PY
```

**판독**  received True와 final 0.0이면 명령이 정책 입력까지 도달했고 오래된 입력이 해제됐다. 목표 속도를 실제로 추종했는지는 같은 CSV의 velocity_x를 별도로 비교한다.

## 시뮬레이터와 브리지를 분리해서 읽기

TRON2 · SDK에서 ROS로

```
cd ~/sainty/day1
ROS_DOMAIN_ID=86 bash ros_lab/run.sh tron2
read -r -p 'Evidence directory 경로를 붙여 넣으세요: ' TRON_RUN
```

run.sh tron2는 fetch_models.py로 모델 목록과 체크섬을 확인한다. ROBOT_TYPE=SF_TRON2A, ROBOT_IP=127.0.0.1 환경에서 headless 시뮬레이터와 sdk_bridge.py를 별도 프로세스로 시작한다. 관찰자는 이 족형 모델에서 관절 10개를 기대한다.

### sdk_bridge.py의 세 단계

① 연결과 구독. Robot(RobotType.Tron2)을 만든 뒤 robot.init으로 연결한다. subscribeRobotState(state)와 subscribeImuData(imu)가 서로 다른 종류의 샘플을 전달한다.

② 이름과 순서 검사. state는 SDK의 motor_names를 모델 정의의 LEG_SF 순서와 비교한다. 이름이 모두 비어 있을 때만 모델 순서를 대신 쓴다. 이름이 다르거나 q의 길이가 맞지 않으면 해당 샘플을 건너뛴다.

③ 발행. 콜백은 잠금 안에서 최신 샘플을 복사한다. 루프는 약 0.02초마다 각 종류의 stamp가 직전에 보낸 값과 다를 때만 발행한다. SDK가 더 자주 갱신되면 그사이의 샘플이 생략될 수 있다.

### 필드가 건너가는 경계

```
SDK RobotState    q / dq / tau      -> JointState.position / velocity / effort
SDK ImuData       quat / gyro / acc -> Imu.orientation / angular_velocity /
                                        linear_acceleration
SDK message.stamp                   -> ROS header.stamp
```

같은 결과 폴더의 simulator.log는 시뮬레이터, sdk.log는 브리지의 출력이다. 두 토픽이 모두 비면 브리지 초기화를, IMU만 들어오면 state 콜백의 이름·길이 조건을 먼저 본다.

**확인 문제**  TRON2에서 /tron2/cmd_vel의 구독자가 없어도 이 실습이 성공하는 이유는 무엇인가? 성공 조건은 SDK의 관절·IMU 상태가 ROS로 도착해 기록되는 것이다.

## telemetry.py와 observe.py를 한 쌍으로 읽기

공통 코드 · 메시지 계약

### 발행자가 약속하는 필드

Telemetry('k1')과 Telemetry('tron2')는 같은 클래스로 /<robot>/joint_states와 /<robot>/imu 발행자를 만든다. 큐 깊이는 10이다. joint_sample은 name, position, velocity, effort를 같은 인덱스로 정렬해 넣는다. name[i]가 바뀌면 같은 position[i]도 다른 관절을 가리킨다.

```
# ros_lab/telemetry.py
sec, nanosec = divmod(int(ns), 1_000_000_000)
# 예: 1_020_000_000 ns -> sec=1, nanosec=20_000_000
```

K1은 data.time을 나노초로 바꿔 넘기고 TRON2는 SDK의 message.stamp를 넘긴다. imu_sample은 원본 [w, x, y, z]를 ROS의 이름 있는 orientation.w, .x, .y, .z에 대입한다. 두 로봇의 기준 프레임은 각각 imu와 base_imu다.

### 수신자가 검사하는 조건

observe.py의 receive(topic, msg)는 토픽별 직전 stamp보다 새 stamp가 커야 통과시킨다. 관절은 첫 메시지의 이름 순서가 유지되는지, 이름이 고유한지, 네 배열의 길이가 각각 23 또는 10인지 확인한다.

IMU는 쿼터니언 노름이 1에 가까운지, 각속도와 가속도 성분이 유한한지 검사한다. 관절 위치·속도·힘에도 유한값 검사를 적용한다. 노름만으로 축의 순서나 센서 방향까지 판정할 수는 없다.

### 서로 다른 두 시각

```
# observe.py 의 receive 함수
raw = serialize_message(msg)
writer.write(topic, raw, node.get_clock().now().nanoseconds)
rows.append((stamp, hashlib.sha256(raw).hexdigest()))
```

**읽기 과제**  header.stamp는 원본 샘플의 시각이고 writer.write의 마지막 인자는 수신 노드가 기록한 시각이다. K1의 시뮬레이션 시간과 수신 시계를 곧바로 빼서 전송 지연을 계산하면 기준 시계가 서로 다르다.

## 두 종류의 일치 검사를 결과 파일에서 확인하기

기록 · 재생 · 판정

### 1. 실시간 수신 → SQLite bag

observe.py는 rclpy.spin_once로 콜백을 처리해 토픽당 60개를 채운다. 기록 종료 후 SequentialReader로 같은 bag을 다시 읽고, 각 토픽에서 직렬화 바이트의 SHA-256 목록을 수신 당시 목록과 순서대로 대조한다. 성공하면 receipt.json에 live, bag, 첫·마지막 stamp와 PASS가 남는다.

```
cat "$K1_RUN/receipt.json"
cat "$TRON_RUN/bag_info.txt"
ros2 bag info "$K1_RUN/telemetry"
```

### 2. 원본 bag → DDS 재생 → 두 번째 bag

```
cd ~/sainty/day1
ROS_DOMAIN_ID=88 bash ros_lab/replay.sh k1 "$K1_RUN"
ROS_DOMAIN_ID=88 bash ros_lab/replay.sh tron2 "$TRON_RUN"
```

replay.sh는 --passive 관찰자를 먼저 띄우고 ros2 bag play를 3초 지연해 구독 연결 시간을 준다. 이 관찰자는 K1 속도 명령을 발행하지 않는다. 재수신분은 원본 결과 아래 새 replay-… 폴더에 기록한다.

마지막에 원본·재생 SQLite에서 토픽별 메시지 순서대로 데이터를 읽어 역직렬화하고 모든 메시지 필드를 대조한다. 재생 시점에 새로 부여한 bag 기록 시각은 이 비교에 포함되지 않는다.

```
# 각 재생 명령이 출력한 replay-… 폴더의 comparison.json 확인
find "$K1_RUN" "$TRON_RUN" -name comparison.json -print -exec cat {} \;
```

**제출할 증거**  각 로봇별 receipt.json에서 두 토픽의 live=60, bag=60을 확인한다. comparison.json에서 all_fields_equal=true, messages=120을 기록한다. K1은 2쪽의 명령 만료 결과를 함께 제출한다.

교안 검증 결과   WSL2에서 현재 코드를 복사해 두 로봇의 수신·재생을 독립 실행했다. K1과 TRON2 모두 두 토픽에서 60개씩 기록했고, 재생 비교 결과는 각각 120개 전 필드 일치였다.

## 어느 경계에서 데이터가 멈췄는지 좁히기

문제 해결 · 선수 실습

토픽이 보이지 않는다   실행과 관찰 터미널에서 ROS_LOCALHOST_ONLY=1, ROS_DOMAIN_ID가 같은지 확인한다. run.sh의 기본 도메인은 86, replay.sh는 88이다. 이미 설정한 환경 변수 값이 우선한다. 실행이 끝나면 스크립트가 시작한 생산자도 종료되므로 그 뒤에는 bag으로 관찰한다.

50초 뒤 TimeoutError   예외의 토픽별 수신 개수를 비교한다. 둘 다 0이면 실행한 로봇, 도메인과 프로세스 로그를 확인한다. 한쪽만 부족하면 그 토픽의 발행자와 브리지의 입력을 추적한다.

K1에서 명령 확인 실패   receipt.json과 sim/policy_rollout.csv를 각각 본다. 관절·IMU 메시지를 받았어도 cmd_vel이 전달되고 만료됐다는 검사는 별개다. TRON2의 SDK init failed는 sdk.log에서 찾아 시뮬레이터의 구동과 루프백 주소부터 확인한다.

### 선수 교안 「ROS2 기초 관절상태와 제어」 코드 찾기

Windows에서는 그 PDF와 같은 상세교안 폴더의 ROS2_기초_실습코드.zip을 연다. WSL에서는 아래 위치에 같은 파일 세 개가 있다. leg.urdf는 무릎 한 관절과 모의 하드웨어, controllers.yaml은 상태 방송기와 무릎 위치 컨트롤러, lesson.launch.py는 두 모드의 시작 구성을 정의한다.

```
cd ~/sainty/day1/ros_lab/ros2_basics
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=81 ROS_LOCALHOST_ONLY=1
ros2 launch ./lesson.launch.py mode:=model
# Ctrl+C로 종료한 뒤 같은 터미널에서 실행
ros2 launch ./lesson.launch.py mode:=control
```

model에서는 joint_state_publisher가 상태값을 공급한다. control에서는 mock_components/GenericSystem의 상태 인터페이스를 joint_state_broadcaster가 읽는다. 두 경우 모두 robot_state_publisher가 관절 상태와 URDF로 링크 자세를 계산한다.

**다음 읽기**  같은 상세교안 폴더의 K1_ROS2.pdf는 K1 정책과 ROBOTIS 구성을, TRON2_SDK.pdf는 LimX SDK와 형상별 동작을 더 깊게 다룬다.
