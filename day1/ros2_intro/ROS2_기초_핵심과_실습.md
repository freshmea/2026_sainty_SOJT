# ROS 2 기초 실습

시작하기 · 무릎 관절 하나로 배우는 노드 통신과 상태 관찰

무릎 목표각을 0.5 rad로 바꾸었다. 명령은 받아들여졌는데 발끝도 정말 움직였을까? 이 실습에서는 목표값, 노드가 발행한 관절각, 관절각으로 계산한 발끝 위치를 각각 읽어 답한다. 실물 장비 없이 두 개의 WSL 터미널에서 진행한다.

목표각 설정 → 관절각 발행 → 링크 자세 계산

target_rad / JointState / TF

### 수업 순서

| 구간 | 할 일 | 완료 기준 |
|---|---|---|
| 2~4쪽 | 환경 준비와 그래프 관찰 | 발행자와 구독자를 이름으로 찾는다. |
| 5~8쪽 | 메시지·QoS·파라미터·서비스·액션 | 목표를 바꾸고 수신 여부를 설명한다. |
| 9~10쪽 | launch·URDF·TF·rosbag | 발끝 좌표를 예측하고 기록을 재생한다. |
| 11~12쪽 | 종합 실습과 오류 진단 | 다른 사람이 확인할 증거를 남긴다. |

### 예제와 터미널

실습 코드는 ~/sainty/day1/ros2_intro/src/sainty_knee에 있다. A에서는 주로 노드를 계속 실행하고 B에서는 명령을 보낸다. C가 필요할 때는 터미널을 하나 더 열어 3쪽의 공통 준비를 적용한다. Ctrl+C는 해당 터미널의 실행을 끝낸다.

**관찰할 값**  회전 관절의 단위는 rad이다. 교안의 무릎은 일정 간격으로 값을 갱신하는 소프트웨어 모형이다. 여기서 확인할 대상은 ROS 통신과 좌표 계산이다.

## ROS 2 그래프를 읽는 네 단어

개념 지도 · 같은 값을 여러 프로그램이 함께 관찰하려면

패키지는 프로그램·설정·설치 정보를 묶는 배포 단위다. 노드는 실행 시 그래프에 이름을 등록하고 한 역할을 수행한다. 토픽은 이름을 가진 메시지 통로이고 메시지 타입은 그 통로에 실리는 필드와 자료형을 정한다.

| 실습 속 이름 | 종류 | 맡은 일 |
|---|---|---|
| sainty_knee | 패키지 | 발행·관찰 프로그램과 launch·URDF를 설치 |
| /classroom/knee_source | 노드 | 모의 무릎 각도를 계산 |
| /classroom/joint_states | 토픽 | 각도와 시각을 여러 구독자에게 전달 |
| sensor_msgs/msg/JointState | 타입 | 관절 이름과 위치 등을 배열로 표현 |

### 메시지를 보낼 때 거치는 층

Python 코드는 rclpy의 publish를 호출한다. ROS 클라이언트 계층과 미들웨어 추상화 계층(RMW)을 거쳐 이 환경의 DDS 구현이 엔드포인트를 발견하고 데이터를 전달한다. 발행자는 관찰 프로그램의 PID나 주소를 코드에 적지 않는다. 이름·타입·통신 조건이 맞는 구독자를 나중에 추가할 수 있다.

### 왜 네임스페이스를 쓰는가

source.py는 상대 이름 joint_states로 발행한다. launch가 노드를 classroom 아래에서 시작하면 전체 이름은 /classroom/joint_states가 된다. 같은 코드를 다른 로봇 이름 아래에서도 실행할 수 있다.

**생각해 보기**  같은 토픽을 knee_watch와 robot_state_publisher가 동시에 구독하면 메시지가 반씩 나뉠까? 각 구독자는 독립적으로 받을 수 있다. 실제 수신 건수는 QoS와 실행 상태에 따라 달라질 수 있다.

## 패키지를 빌드하고 현재 셸에 연결하기

실습 01 · 준비 · Ubuntu 22.04 · ROS 2 Humble · 시스템 Python 3.10

### 최초 1회 또는 패키지 설치 구성이 바뀐 뒤

```
cd ~/sainty/day1/ros2_intro
source /opt/ros/humble/setup.bash
colcon list
colcon build --symlink-install --packages-select sainty_knee
```

src는 편집할 코드, build는 빌드 작업 공간, install은 실행 파일과 리소스를 찾는 설치 공간, log는 빌드 기록이다. --symlink-install을 쓰면 개발 중 Python 소스의 변경을 반영하기 쉽다. 실행 중인 노드는 다시 시작한다.

### 새 터미널마다 A·B·C 공통으로 실행

```
cd ~/sainty/day1/ros2_intro
source /opt/ros/humble/setup.bash
source install/local_setup.bash
export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=93
ros2 pkg executables sainty_knee
```

첫 source는 Humble 기본 환경을, 두 번째는 이 패키지를 추가한다. 마지막 출력에서 sainty_knee knee_source와 sainty_knee knee_watch를 찾는다. 도메인 93은 같은 PC의 이 실습 노드를 묶는 번호다. 강의실에서 번호를 바꾸면 모든 터미널에 같은 값을 쓴다.

### 코드 위치

```
src/sainty_knee/
  package.xml          setup.py          setup.cfg
  sainty_knee/source.py                  # 관절 상태와 서비스
  sainty_knee/watch.py                   # 상태 구독
  launch/knee.launch.py                  # 세 노드 시작
  urdf/leg.urdf                          # 링크와 관절 치수
```

**빌드 점검**  colcon list에 패키지가 없으면 현재 디렉터리를 확인한다. 빌드가 성공했는데 Package not found가 나오면 명령을 실행한 터미널에서 install/local_setup.bash를 읽었는지 확인한다.

## 발행자를 띄우고 구독자를 찾아보기

실습 02 · 노드와 토픽 · launch를 쓰기 전에 두 프로그램을 직접 연결한다

### A에서 시작하고 B에서 관찰

```
# A
ros2 run sainty_knee knee_source --ros-args -r __ns:=/classroom
# B
ros2 node list
ros2 node info /classroom/knee_source
ros2 topic list -t
ros2 topic info /classroom/joint_states --verbose
```

node info의 Publishers에서 /classroom/joint_states를 찾는다. topic info의 Type은 sensor_msgs/msg/JointState다. /rosout이나 /parameter_events 같은 시스템 토픽도 함께 보일 수 있다. 관심 있는 토픽 이름과 타입을 함께 기록한다.

### B에서 구독하고 C에서 연결 수 확인

```
# B
ros2 run sainty_knee knee_watch --ros-args -r __ns:=/classroom
# C
ros2 topic info /classroom/joint_states
ros2 interface show sensor_msgs/msg/JointState
```

watch는 첫 수신 뒤 20개마다 sample과 angle을 출력한다. 잠시 기다리면 angle=+0.300 rad가 보인다. C에서 다른 관찰자가 없다면 발행자 1개와 구독자 1개가 나타난다. B의 구독자를 Ctrl+C로 종료한 뒤 구독자 수가 줄어드는지 확인한다.

**다음 단계**  A의 단독 발행자도 Ctrl+C로 종료한다. 5쪽부터는 launch로 같은 발행자와 구독자, 링크 자세 계산 노드를 함께 실행한다. 같은 이름의 발행자를 두 번 띄우면 뒤의 측정값이 섞인다.

## JointState 한 건이 만들어지는 과정

실습 03 · Python 코드 · 타이머 주기와 실제 수신 빈도를 구분한다

```
# A · 9쪽까지 유지 (액션 서버는 별도 터미널)
ros2 launch sainty_knee knee.launch.py
# B
ros2 topic echo /classroom/joint_states sensor_msgs/msg/JointState \
  --qos-reliability best_effort --once
```

source.py의 create_timer(0.05, self.sample)은 50 ms마다 실행할 콜백을 등록한다. spin은 타이머와 서비스 요청 등 준비된 작업을 처리한다. sample은 현재각을 목표 쪽으로 한 번에 최대 0.025 rad 이동시키고 다음 메시지를 발행한다.

```
msg = JointState()
msg.header.stamp = self.get_clock().now().to_msg()
msg.name = ["knee_joint"]
msg.position = [self.angle]
self.publisher.publish(msg)
```

name[0]과 position[0]이 한 쌍이다. velocity와 effort는 이 예제가 제공하지 않아 빈 배열이다. watch.py는 knee_joint의 이름을 찾아 인덱스를 결정하고 position에서 각도를 읽는다. 여러 관절에서 순서를 하드코딩하면 다른 관절값을 읽을 수 있다.

### 예측하고 관찰

0에서 0.30까지 한 틱당 최대 0.025씩 움직이면 목표까지 12틱, 명목상 0.6초가 걸린다. 첫 메시지를 늦게 구독했다면 이미 0.30일 수 있다. header.stamp는 노드가 시각을 찍은 때다. 도착 간격은 스케줄링과 전송 상태에도 영향을 받는다.

**코드 읽기**  목표가 이미 0.30일 때에도 stamp는 계속 바뀌는가? sample이 매번 새 메시지를 만들고 발행하므로 각도가 같아도 시각은 갱신된다. echo에서 --once를 빼고 두 건을 비교한 뒤 Ctrl+C를 누른다.

## 토픽이 보이는데 값이 오지 않을 때

실습 04 · QoS · 발행자가 제공하는 신뢰성과 구독자가 요청하는 신뢰성을 비교한다

센서 스트림에서는 일부 샘플을 놓쳐도 다음 최신 상태가 유용하다. 이 예제의 발행자와 관찰자는 qos_profile_sensor_data를 사용한다. 실습에서 확인할 정책은 best_effort 신뢰성이다. CLI로 구독자 하나의 요청만 바꿔 본다.

### B에서 같은 토픽에 두 번 접속

```
T=/classroom/joint_states
ros2 topic info "$T" -v
timeout 5 ros2 topic echo "$T" sensor_msgs/msg/JointState \
  --qos-reliability reliable --once
echo $?
ros2 topic echo "$T" sensor_msgs/msg/JointState \
  --qos-reliability best_effort --once --field position
```

| 발행자가 제공 | 구독자가 요청 | 이 설정에서의 결과 |
|---|---|---|
| best_effort | reliable | 조건 불일치. 5초 안에 샘플 없이 종료. |
| best_effort | best_effort | 조건 일치. 현재각 한 건 수신. |

첫 명령의 종료 코드 124는 timeout이 기다림을 끝냈다는 뜻이다. 그래프 발견과 데이터 전달은 별개다. 반드시 둘째 명령으로 실제 발행 중인지 확인한 후 두 결과를 함께 해석한다.

응용   신뢰성 설정이 맞으면 타입을 바꾸는 대신 --field name과 --field position을 번갈아 조회한다. 이름과 값의 대응을 확인한 뒤, 관절이 두 개라면 수신 코드를 어디서 바꿀지 watch.py에서 찾아본다.

**큐에 관한 질문**  depth는 대기시킬 최근 샘플 수와 관련된 이력 설정이다. 큐를 늘려도 best_effort 발행자가 reliable 요청을 충족하지는 않는다. topic info -v에서 Reliability부터 비교한다.

## 목표는 바뀌었는데 무릎이 멈춰 있는 이유

실습 05 · 서비스와 파라미터 · 현재각을 유지한 채 새 목표를 지정하고 다시 추종시킨다

target_rad는 knee_source 노드가 가진 double 파라미터다. enable은 요청의 bool을 받아 적용 결과를 돌려주는 SetBool 서비스다. target_rad는 원하는 값, joint_states.position은 현재 모형값이므로 동시에 다를 수 있다.

```
# B · 기본값 0.30에 도달한 상태에서 시작
ros2 interface show std_srvs/srv/SetBool
ros2 service call /classroom/enable std_srvs/srv/SetBool '{data: false}'
ros2 param set /classroom/knee_source target_rad -0.5
ros2 param get /classroom/knee_source target_rad
ros2 topic echo /classroom/joint_states --field position \
  --qos-reliability best_effort --once
```

서비스 응답은 success=True, message='holding'이다. 파라미터는 -0.5로 바뀌어도 관절각은 약 +0.30에 머문다. set_enabled가 타이머를 끄는 대신 각도 갱신만 멈추기 때문에 계속 상태 메시지를 받을 수 있다.

```
ros2 service call /classroom/enable std_srvs/srv/SetBool '{data: true}'
# 약 2초 뒤 위의 echo 명령을 다시 실행한다.
ros2 param set /classroom/knee_source target_rad 1.5
ros2 param get /classroom/knee_source target_rad
```

재개하면 응답은 tracking이고 각도는 -0.5에 도달한다. 1.5를 요청하면 validate 콜백이 허용 범위 [-1, 1] 밖의 값을 거부한다. 마지막 get 결과는 여전히 -0.5다. 정수 1과 실수 1.0은 ROS 파라미터 타입이 다르다.

**제출**  정지 직후, 정지 중 목표 변경 후, 재개 후의 세 행에 ‘요청한 목표 / 발행 각도 / 서비스 응답’을 적는다. 서비스 응답이 성공한 시점과 각도가 목표에 도달한 시점을 구분한다.

## 오래 걸리는 요청의 진행 상태를 받기

실습 06 · 액션 · Humble에 설치된 공개 액션 예제로 세 가지 통신 방식을 비교한다

| 인터페이스 | 대화 방식 | 이 수업에서 관찰하는 예 |
|---|---|---|
| 토픽 | 연속 샘플을 발행하고 구독 | 현재 관절각 |
| 서비스 | 한 요청에 한 응답 | 각도 추종 정지·재개 |
| 액션 | 목표, 중간 피드백, 최종 결과 | 피보나치 수열 계산 |

### C에서 예제 서버, B에서 목표 전송

```
# C · 3쪽 공통 환경을 적용한 터미널
ros2 run action_tutorials_py fibonacci_action_server
# B
ros2 action list -t
ros2 interface show action_tutorials_interfaces/action/Fibonacci
ros2 action send_goal /fibonacci \
  action_tutorials_interfaces/action/Fibonacci '{order: 5}' --feedback
```

인터페이스 출력에서 ---가 목표·결과·피드백을 나눈다. 목표 수락은 계산 시작을 의미한다. CLI의 Feedback과 마지막 Result, 종료 상태 SUCCEEDED를 구별해 읽는다. 완료된 뒤 C의 서버는 Ctrl+C로 닫아도 무릎 실습은 계속된다. 검증 시 order 5의 최종 수열은 [0, 1, 1, 2, 3, 5]였다.

### 무릎 모형에 적용해 보기

측정 각도를 여러 관찰자에게 계속 보내려면 토픽을 쓴다. 현재 추종을 잠시 멈추고 적용 여부를 받으려면 서비스를 쓴다. 몇 초짜리 자세 전환에 진행률과 종료 결과, 취소 요청이 필요하다면 액션이 적합하다. 취소 수락과 처리는 서버 구현에 달려 있다.

**기록**  요청한 order, 실제로 받은 중간 수열 한 개, 최종 수열을 적는다. 같은 요청을 서비스만으로 만들면 진행 과정을 어디에서 얻어야 하는지 설명한다.

## 관절각에서 발끝 위치 계산하기

실습 07 · 모델과 좌표 · 같은 상태 메시지를 이번에는 robot_state_publisher가 읽는다

knee.launch.py는 knee_source, knee_watch, robot_state_publisher를 같은 네임스페이스로 시작한다. launch는 URDF 텍스트를 robot_description 파라미터로 전달한다. URDF가 기하 관계를, JointState가 움직이는 관절의 현재각을 제공한다.

thigh → shin → foot

기준 링크 / Y축 무릎 회전 / 정강이에서 z 방향 −0.30 m

다리가 곧을 때 발끝은 thigh 좌표계에서 (0, 0, -0.30) m다. 무릎을 +Y축으로 θ만큼 회전하면 x = -0.30 × sin θ, z = -0.30 × cos θ가 된다. tf2_echo의 기준 프레임을 바꾸면 같은 발끝이라도 좌표 숫자가 달라진다.

```
# B · A의 launch 유지, 추종이 켜져 있는 상태
ros2 param set /classroom/knee_source target_rad 0.5
ros2 run tf2_ros tf2_echo thigh foot
# Translation이 안정되면 Ctrl+C
```

| 관절각 | x 예측 (m) | z 예측 (m) |
|---|---|---|
| 0.0 rad | 0.000 | -0.300 |
| +0.5 rad | -0.144 | -0.263 |
| -0.5 rad | +0.144 | -0.263 |

**변형 실습**  목표를 -0.5로 바꿔 어느 성분만 부호가 바뀌는지 확인한다. 첫 조회에서 프레임이 없다고 표시되면 몇 초 기다린다. 계속 실패하면 A의 launch와 /classroom/joint_states 발행을 확인한다.

## 발행자를 종료하고도 같은 각도 읽기

실습 08 · 기록과 재생 · 관절 상태 한 토픽만 기록하여 재생의 증거를 남긴다

### B에서 5초 이상 기록

```
cd ~/sainty/day1/ros2_intro
mkdir -p bags
BAG="bags/knee-$(date +%Y%m%d-%H%M%S)"
ros2 bag record -o "$BAG" /classroom/joint_states
# 메시지가 기록되면 Ctrl+C. 아래 명령은 같은 B에서 실행한다.
ros2 bag info "$BAG"
```

info에서 토픽 이름, 타입, 메시지 수가 0보다 큰지 확인한다. 20 Hz로 5초 기록하면 대략 100건을 예상할 수 있지만 실제 수는 발견 지연과 기록 구간에 따라 달라진다. 서로 다른 목표각을 기록하려면 기록 중 C에서 target_rad를 바꾼다.

### A의 launch를 Ctrl+C로 종료한 다음

```
# C · 먼저 구독을 대기시킨다
ros2 topic echo /classroom/joint_states sensor_msgs/msg/JointState \
  --qos-reliability best_effort --once --field position
# B · BAG 변수가 있는 터미널
ros2 bag play "$BAG" --delay 2
```

C가 받은 position은 파일에 기록되었던 샘플이다. 이때 발행 역할은 rosbag 플레이어가 맡는다. bag에 /tf와 /tf_static은 넣지 않았으므로 이 파일만 재생해 발끝 변환까지 다시 방송되리라고 기대할 수 없다.

**증거 남기기**  bag info의 메시지 수와 토픽 타입, launch 종료 여부, 재생 후 C에서 받은 position을 적는다. 측정치를 다시 관찰하는 것과 모형을 다시 계산하는 것의 차이를 설명한다.

실행 예   제작 시 Humble 환경에서 88건을 기록했다. 원래 노드를 종료하고 재생했을 때 대기 중인 구독자가 0.50 rad 샘플을 받았다. 이 수치는 제출 목표량이 아니라 한 번의 관찰 기록이다.

## 목표값·관절값·발끝 좌표로 원인 찾기

종합 실습 · 제시된 순서만 보고 새 세션에서 한 번 더 수행한다

10쪽에서 launch를 종료했으므로 A에서 5쪽의 launch를 다시 실행한다. B에서 /classroom/knee_source의 target_rad가 0.3인지 확인한다. 이번에는 명령을 실행하기 전에 각 단계의 예상값부터 적는다.

| 순서 | 직접 할 일 | 비교할 증거 |
|---|---|---|
| 1 | 초기 상태에서 두 구독자 확인 | node list와 topic info -v |
| 2 | 서비스로 추종을 멈추고 목표를 -0.5로 변경 | param get과 position을 나란히 기록 |
| 3 | 추종 재개 후 2초 대기 | position과 thigh → foot의 x, z |
| 4 | 현재각 유지 중 reliable 구독자 추가 | graph에 보이는 연결과 실제 수신 |
| 5 | best_effort로 복원하고 5초 이상 기록 | bag info와 한 건의 재생 출력 |

### 판독 예시

2단계에서 목표 -0.5와 관절각 +0.30이 동시에 보이면 서비스와 파라미터가 각각 의도한 역할을 한 것이다. 3단계에서 각도가 -0.5에 수렴하면 발끝 x는 약 +0.144 m여야 한다. 각도가 맞는데 좌표가 다르면 기준 프레임과 URDF의 길이·축부터 본다.

### 다음 실습과 연결

~/sainty/day5/ros_lab/run.sh k1은 K1 시뮬레이터에서 온 관절 상태와 IMU를, run.sh tron2는 루프백 SDK의 상태를 관찰한다. 이번 예제의 /classroom/joint_states와 같은 JointState 타입이라도 관절 수·발행자·시간 출처가 바뀐다. 기존 K1에서 name과 position의 길이, header.stamp의 출처를 먼저 찾아본다.

**수료 확인**  (1) 실행 명령 하나를 재현한다. (2) 예상과 실제 값을 단위와 함께 두 쌍 적는다. (3) 메시지가 안 올 때 이름·타입·QoS를 어떤 순서로 볼지 근거를 붙여 설명한다.

## 오류를 좁히는 순서와 원문 읽기

현장 참조 · 페이지 번호는 제공받은 PDF 뷰어의 1부터 시작하는 페이지 기준이다

| 증상 | 먼저 볼 곳 | 판독과 조치 |
|---|---|---|
| 패키지를 못 찾음 | ros2 pkg prefix sainty_knee | 3쪽 환경과 build 결과 확인 |
| 노드 목록이 비어 있음 | echo $ROS_DOMAIN_ID | 모든 터미널의 도메인·localhost 설정 확인 |
| 토픽은 있는데 값이 없음 | ros2 topic info <토픽> -v | 발행자 수, 타입, QoS 비교 |
| 각도가 계속 같은 값 | param get과 enable 응답 | 목표에 이미 도달했는지, holding인지 확인 |
| TF가 처음에 없음 | tf2_echo thigh foot | launch·URDF·관절 이름·수신 확인 |
| bag 출력을 만들 수 없음 | 출력 디렉터리 이름 | 새 BAG 이름으로 기록 후 정상 종료 |

### 참고한 두 교재에서 다시 읽을 곳

① 바인드소프트 「ROS2 Open Seminar 교재」. PDF 28~38쪽은 CLI와 Python 발행·구독, 42~54쪽은 통신 방식 비교와 토픽 관찰, 56~69쪽은 서비스·액션, 70~77쪽은 파라미터와 launch를 다룬다. 이 교안에서는 turtlesim 중심 활동을 무릎 모형의 관찰·정지·재개로 재구성했다.

② Lentin Joseph·Jonathan Cacace, 『Mastering ROS 2』 한국어 PDF. 44~57쪽의 DDS·RMW, 88~101쪽의 그래프·메시지·서비스, 107~121쪽의 패키지·액션·QoS, 144~145쪽의 launch·도메인, 151~162쪽의 모델링·URDF를 참조했다. 원서 예제의 Jazzy 환경을 세인티 WSL2의 Humble과 Python으로 조정했다.

실습 코드의 출처   sainty_knee는 이번 수업용으로 작성했다. 액션 절에서 실행하는 fibonacci_action_server는 Humble의 action_tutorials_py 예제이고, 좌표 변환과 기록에는 배포판의 robot_state_publisher, tf2_ros, rosbag2를 사용한다.

**파일 안내**  편집본과 실습 소스는 ~/sainty/day1/ros2_intro, 인쇄용 교안은 ~/sainty/pdf/ROS2_기초_핵심과_실습.pdf에 있다.
