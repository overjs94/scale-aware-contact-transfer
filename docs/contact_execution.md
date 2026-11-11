# 계획과 접촉 실행 검증

CasADi + IPOPT로 접촉 계획을 풀고, MuJoCo에서 제어기와 물체의 동역학을
검증한 뒤, 공차가 작은 삽입 구간을 CRISP에서 교차 검증합니다. 접촉 후보와
로봇 자세를 고정할 수 있는 단계에는 Clarabel SOCP를 독립 검산기로 사용합니다.

## 솔버 비교

`SolverSettings.backend`는 `ipopt`와 `snopt`를 선택합니다. 공통 tolerance는
IPOPT의 `tol` 및 SNOPT의 major feasibility/optimality tolerance에 전달하고,
max_iterations는 각각 IPOPT iteration 및 SNOPT major iteration 한도에 전달합니다.
두 알고리즘의 iteration이 동일한 연산량을 뜻하지는 않습니다. 명시적
`settings.options`는 이 기본값을 덮어쓸 수 있습니다.

~~~bash
python tools/benchmark_cito_solvers.py --backends ipopt snopt --repeats 3
~~~

이 도구는 1 kg 자유 물체의 정지 지지 CITO를 매번 새로 구성합니다. 초기값,
8개 interval, 접촉 후보, normal-force bound, continuation과 허용오차를 고정하고
primal·unrelaxed complementarity·friction·gap·gripper velocity residual을 검사합니다.
JSON에는 실패와 backend 부재를 구분하고, residual을 통과한 실행의 성공률과
중앙 시간을 기록합니다. 시간에는 NLP 구성과 continuation이 포함됩니다.
이 작은 문제의 시간은 가구 운반·삽입 성능을 대표하지 않습니다.

실제 과제를 비교할 때도 동일한 URDF·물체 관성·접촉 후보·attachment phase,
같은 초기값 묶음과 multistart budget을 사용합니다. 모든 실행의 residual과
실패 이유를 보관하고, 성공 실행의 시간 분포를 비교합니다. SNOPT는 작동하는
CasADi plugin 및 해당 solver 설치·라이선스가 있어야 합니다.

## 고정 형상의 SOCP 검산

`fixed_wrench_margin`은 접촉 위치와 회전 행렬을 고정하고 `rho`와 vertex별
reaction만 최적화합니다. 평형은 affine, point contact의 Coulomb cone은
Lorentz cone, calibrated gripper capability와 관절 토크 한계는 선형 부등식입니다.
접촉 법선은 rotation의 세 번째 열이며, reaction의 부호는 object-on-agent입니다.
점 접촉의 moment는 0이고 모든 wrench는 moment-first입니다.

~~~python
from contact_transfer.conic import fixed_wrench_margin

# source_result는 refine_contacts의 반환값입니다.
check = fixed_wrench_margin(source_result.positions, source_result.rotations,
                           task, capabilities)

# target_result는 realize_keyframe의 반환값입니다.
target_check = fixed_wrench_margin(
    target_result.positions, target_result.rotations, task, capabilities,
    arms=arms, q=target_result.q, assignment=target_result.assignment,
    object_pose=object_pose)
~~~

각 vertex의 평형, cone, force/gripper bound와 토크 한계를 해를 받은 뒤 다시
계산합니다. geometry·contact coincidence·normal alignment·collision은 이 함수의
검사 범위 밖이며, III-C에서 이미 실현된 자세를 입력합니다. `rho`는 기존과
같은 conservative box margin이고, 정확한 ellipsoidal margin이 아닙니다.
고정 형상에서 force를 다시 분배하므로 geometry/posture regularization이 있는
NLP의 반환 `rho`와 항상 같아야 하는 것은 아닙니다.

## MuJoCo에서 계획 실행

`MujocoContactScene`은 collision-enabled MJCF를 읽습니다. 로봇 링크·그리퍼·패널·
fixture의 형상과 충돌 제외 쌍, 마찰·질량·관성은 scene asset에서 지정합니다.
기존 dynamics-only URDF의 marker sphere를 충돌 형상으로 바꾸지 않습니다.
각 로봇의 mechanics는 URDF를 기준으로 맞추고, mesh의 원점과 단위를 확인합니다.

~~~python
from contact_transfer.contact_simulation import MujocoContactScene, trajectory_reference
from contact_transfer.control import ComputedTorque

# 단일 fixed-base robot 예시입니다. 여러 팔에는 동일 순서로 묶은 joint 이름과
# 팔별 ComputedTorque.command를 조합한 controller를 전달합니다.
reference = trajectory_reference(plan, q_indices=robot_q_indices,
                                  v_indices=robot_v_indices)
scene = MujocoContactScene("assembly.xml", robot_joint_names,
                           torque_motor_names, ["panel"])
log = scene.run(ComputedTorque(robot), reference, float(plan.dt.sum()),
                case_id=case_hash, plan_id=plan_hash,
                attachments=[(0., "left_hold", True),
                             (release_time, "left_hold", False)])
log.save("mujoco_execution.npz")
~~~

Scene의 initial state를 계획의 시작 상태에 맞춥니다. 실행 중 물체 pose를
최적화 궤적에 맞춰 덮어쓰지 않습니다. 제어 대상은 scalar hinge/slide joint와
unit-gear direct torque motor이며, scene의 모든 actuator를 명시해야 합니다.
floating-base whole-body 제어와 position actuator는 이 runner의 대상이 아닙니다.
관절 reference는 CITO의 q/v node를 cubic Hermite 보간합니다. 보간 이후의
관절·속도·토크·충돌 여유는 실행 trace로 별도 평가합니다.

Scene timestep의 정수배 duration과 step에 맞는 부착 이벤트를 사용합니다.
`attachments`는 scene에 정의된 equality constraint를 켜고 끕니다. 이 모델은
이상적인 holding이므로 finger closure, aperture, slip과 파지 유지 실험에는
실제 gripper 접촉 모델을 사용해야 합니다. 제어 torque의 scene bound 위반과
MuJoCo warning은 실행 실패로 처리합니다.

## 공통 실행 trace와 CRISP 연동

`ExecutionTrace`는 pickle 없이 NPZ로 저장하며 다음 convention을 강제합니다.

| 항목 | 형식 |
|---|---|
| metadata | `TRACE_CONVENTIONS`의 format_version, SI units, pose/wrench convention |
| 실행 식별 | engine/version, case_id, plan_id, joint_names, object_names |
| 설정 | timestep, contact_settings; MuJoCo는 원본 XML 및 compiled model SHA256도 저장 |
| time, q, q_reference | N개 시간과 관절 상태/reference |
| torque | N-1개 interval의 commanded joint torque |
| object_pose | N × object 개수 × 7, world position과 xyzw quaternion |
| contact_time, contact_pair | 접촉 sample 시간과 순서가 있는 두 geom 이름 |
| contact_position, contact_normal | world 좌표, normal은 첫 geom에서 두 번째 geom 방향 |
| contact_gap | 양수는 분리, 음수는 침투인 signed distance |
| contact_wrench | 접촉점 기준 두 번째 geom에 작용하는 world `[moment; force]` |

CRISP는 이 패키지의 Python dependency가 아닙니다. 별도 C++ 실행기에서 같은
scene asset·관성·초기 상태·제어 reference·release schedule을 사용하고, 결과를
위 형식으로 내보내는 adapter가 필요합니다. 선택한 배포판의 라이선스와 API를
확인하고 버전을 고정합니다. CRISP C++ 실행 및 adapter는 아직 구현·검증하지
않았으며, 여기서 CRISP 성능이나 삽입 성공률을 보고하지 않습니다.

Adapter에서는 접촉 wrench의 로컬/전역 좌표, 작용/반작용, moment 기준점을
명시적으로 변환합니다. CRISP가 impulse를 반환하면 interval로 나누어 평균
force로 옮깁니다. 단순 block rotation으로 moment origin을 바꾸지 않습니다.
좌표계는 바닥 지지력과 단일 접촉 실험으로 먼저 검증합니다. Python에서
배열을 구성한 뒤 `ExecutionTrace`에 `TRACE_CONVENTIONS`와 metadata를 전달하고
`save`하면 동일한 shape·유한성·quaternion·timestamp 검사를 적용할 수 있습니다.

~~~python
from contact_transfer.execution import ExecutionTrace, compare_executions

comparison = compare_executions(ExecutionTrace.load("mujoco_execution.npz"),
                                ExecutionTrace.load("crisp_execution.npz"))
~~~

`case_id`에는 공통 물체·fixture·초기 상태·제어 설정 bundle의 hash를,
`plan_id`에는 reference와 phase schedule의 hash를 사용합니다. 엔진별
scene의 compiled-model hash는 서로 같을 필요가 없습니다. 비교 함수는 동일
case/plan·joint/object 이름·실행 구간·초기 자세를 확인한 뒤 최종 물체 위치/회전
차이, 관절 추종 오차와 접촉 sample 수를 반환합니다. 서로 다른 timestep의
contact sample 수는 접촉 빈도나 성공률로 해석하지 않습니다.

## 실제 가구 조립 실험의 완료 기준

먼저 dual-arm 패널 운반·지지·release를 실제 collision asset으로 실행합니다.
이후 같은 계획의 최소 공차 삽입 구간을 두 엔진에서 재현합니다. Fixture 축의
삽입 깊이·횡방향 오차·각도·settling 시간을 과제 설정에서 정하고, 삽입 성공,
지속적인 걸림, 미끄럼, 의도하지 않은 충돌과 파지 유지를 판정합니다. Contact
wrench는 동일 기준점/좌표로 합산한 뒤 calibrated grasp·joint bound와 비교합니다.

각 엔진의 timestep과 접촉 설정을 기록하고 더 작은 timestep에서도 판정을
비교합니다. MuJoCo soft contact의 침투량과 마찰 설정 민감도도 함께 보고합니다.
Trace 비교 함수의 endpoint 차이만으로 삽입 성공이나 걸림을 판정하지 않습니다.
현재 자동 검증은 합성 물체 낙하·바닥 지지력·이상적 hold/release·관절 추종까지이며,
실제 가구 collision asset과 CRISP replay가 준비되어야 위 실험을 완료할 수 있습니다.

CRISP 실행 후 contact position/normal/gap은 실패 구간의 추가 후보를 만드는
관측 자료로 사용할 수 있습니다. 임의 최적화 상태의 거리·미분 oracle로 쓰는
것은 별도의 API·미분 검증 이후에 진행합니다. 완전한 STOCS 탐색은 계속
`docs/implementation_status.md`의 미구현 범위로 남습니다.
