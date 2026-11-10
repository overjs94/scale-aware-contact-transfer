# Scale-aware contact transfer

URDF 기반 동역학·기구학과 논문의 Sec. III-B–IV를 위한 연구용 Python 패키지입니다.
로봇별 System_Param의 질량·관성·축·링크 길이를 재작성하지 않고 URDF에서 읽습니다.

## 설치

검증은 Python 3.12, Linux에서 수행했습니다. Windows에서는 WSL2 사용을
추천하며 dev container 설정도 제공합니다. WSL2와 Docker 실행 자체는 이번
환경에서 별도로 검증하지 않았습니다.

~~~bash
python -m venv .venv
source .venv/bin/activate
python -m pip install -e ".[dev,simulation]"
python -m pytest -q
contact-transfer models
contact-transfer demo-refinement
contact-transfer demo-control franka_panda
~~~

Windows PowerShell에서 가상환경 활성화는 .venv\Scripts\Activate.ps1입니다.
Windows native의 전체 조합은 이 버전에서 검증하지 않았습니다.
ROS 설치와 압축파일에 포함된 과거 third_party 빌드는 필요하지 않습니다.

## URDF → 수치 함수와 최적화 함수

~~~python
import numpy as np
from contact_transfer import Robot
from contact_transfer.embodiments import load

robot = load("franka_panda")
# 또는 Robot("/path/to/expanded_robot.urdf", locked_joints={"finger_joint": 0.0})
q = robot.neutral()
v = np.zeros(robot.nv)

M = robot.mass(q)
C = robot.coriolis(q, v)
G = robot.gravity(q)
T = robot.fk(q, "panda_link8")
J = robot.jacobian(q, "panda_link8")  # [omega_body; pdot_world]

symbolic = robot.symbolic()
# CasADi SX/MX 입력도 허용하므로 NLP에 직접 포함할 수 있습니다.
symbolic.export("outputs/panda_functions", frames=["panda_link8"])
~~~

~~~bash
contact-transfer inspect rb3
contact-transfer inspect unitree_g1
contact-transfer export rb5 --frame link6 --output outputs/rb5
~~~

고정/회전/연속회전/직선 관절, 고정 링크의 관성, inertial-origin 회전,
branched tree, mimic 및 locked joint를 처리합니다. Floating base는 별도로 선택합니다.
닫힌 기구 루프는 URDF tree 밖의 제약으로 지정해야 합니다.
Xacro는 먼저 일반 URDF로 확장합니다.

## 포함된 8종

| ID | 모델 | 기본 base | 기본 tool frame |
|---|---|---|---|
| kuka_iiwa7 | KUKA LBR iiwa 7 | Fixed | iiwa_link_ee |
| franka_panda | Franka Panda, fingers locked | Fixed | panda_link8 |
| ur5e | Universal Robots UR5e | Fixed | tool0 |
| kinova_gen3 | Kinova Gen3 7 DoF | Fixed | end_effector_link |
| xarm7 | UFACTORY xArm7 | Fixed | link_eef |
| unitree_g1 | Unitree G1 29 DoF | Floating | right_wrist_yaw_link |
| rb3 | Rainbow RB3-730ES U | Fixed | link6 |
| rb5 | Rainbow RB5-850E | Fixed | link6 |

각 모델의 model.json에 upstream repository, commit, source path와 변경 내용이 있습니다.
모델마다 원래 라이선스를 함께 보관합니다.
이 URDF들은 **동역학·기구학 계산용 subset**입니다. Mesh와 충돌 형상,
vendor controller 태그를 제거했으며 질량·관성·관절 기하는 유지했습니다.
이 파일만으로 실제 로봇의 full-geometry collision clearance를 인증하지 않습니다.
자세한 정보는 [docs/models.md](docs/models.md)를 참조하세요.

## 최적화

- refinement.py: III-B의 surface patch, friction cone, box vertex별 force,
  dimensionless robustness margin.
- realization.py: III-C의 URDF arm posture, contact coincidence, normal
  alignment, point/gripper capability, vertex별 torque limit.
- sequence.py: 공통 arm-level contact type과 injective assignment 후보를
  전부 평가하고 keyframe/transition이 모두 성공한 후보를 비교.
- trajectory.py: IV의 semi-implicit momentum balance, body SE(3) integration,
  normal complementarity continuation, Coulomb maximum dissipation,
  configured gripper attachment/release, torque/velocity/time bounds.

각 NLP의 primal residual을 확인하고 CITO는 unrelaxed contact residual을 별도로 검사합니다.
솔버는 **CasADi + IPOPT**입니다. Pinocchio는 빠른 수치 계산과 독립 검증에 사용합니다.
PyPI의 Pinocchio에 CasADi 바인딩이 없어도 symbolic Newton–Euler graph를 생성합니다.

~~~bash
python examples/dual_arm_keyframe.py
~~~

현재 지원 범위와 아직 일반화하지 않은 부분은
[docs/implementation_status.md](docs/implementation_status.md)에 구분했습니다.
전체 mesh에 대한 STOCS oracle, III-D의 shared-variable joint NLP,
실제 로봇의 통신/주기 보장, G1의 whole-body support controller는 완성된 것으로
표시하지 않습니다.

## 제어와 시뮬레이션

- control.ComputedTorque: URDF inverse dynamics + joint feedback.
- control.CartesianAdmittance: mixed-frame pose/wrench admittance.
- control.position_servo_step: bounded differential IK → position command.
- simulation.MujocoRobot: 동일 URDF 관성의 MJCF 생성과 headless 제어 검증.

MuJoCo exporter의 sphere는 시각화 marker이고 충돌은 꺼져 있습니다.
실제 접촉 실행 검증에는 검증된 물체/로봇 collision geometry가 있는 scene을 사용해야 합니다.
G1의 floating-base 모델을 제공하지만 고정식 팔용 computed-torque controller를 G1에
그대로 적용하지 않습니다.

RB3/RB5는 position/admittance 경로를 기본으로 선택했습니다. 확인한 최신 공식
rbpodo_ros2 소스에는 position, velocity, effort 및 move_servo_t 경로가
존재합니다. 실제 사용 가능 범위와 compensation mode는 장비/펌웨어에서 확인해야 합니다.
이 패키지는 로봇에 자동 접속하거나 물리 명령을 전송하지 않습니다.

## 좌표계와 검증

[omega_tool_body; pdot_world]와 그 power-dual wrench
[moment_tool_body; force_world]를 사용합니다. 순서, 기준점, 부호는
[docs/conventions.md](docs/conventions.md)에 정의했습니다.

검증은 8종 모델의 Pinocchio–CasADi 비교, finite-difference Jacobian/Jdot,
fixed-base Coriolis energy identity, mimic reduction, analytic robustness,
target torque balance, CITO impulse balance, MuJoCo 제어 추종을 포함합니다.

## 개발 환경과 GitHub

.devcontainer 또는 Dockerfile로 같은 Linux 환경을 구성할 수 있습니다.
GitHub Actions는 설치·테스트·CLI 예제를 실행합니다.

저장소: [overjs94/scale-aware-contact-transfer](https://github.com/overjs94/scale-aware-contact-transfer) (private).

~~~bash
git clone https://github.com/overjs94/scale-aware-contact-transfer.git
cd scale-aware-contact-transfer
~~~

GitHub의 Code → Codespaces에서 컨테이너를 생성해 같은 환경을 구성할 수 있습니다.
원본 reference와 대용량 third_party/build 결과물은 저장소에 포함하지 않습니다.

## 브라우저에서 보기

[웹 시뮬레이션 뷰어 열기](https://overjs94-contact-transfer-lab.engagingchina4.chatgpt.site)
(계산 결과 재생용, 본인 계정 접근).

web/에는 3D 결과 뷰어와 Python 계산 서버가 있습니다.

~~~bash
npm --prefix web ci
npm --prefix web run build
python web/server.py --host 127.0.0.1 --port 8000
~~~

브라우저에서 http://127.0.0.1:8000 을 열면 녹화 결과를 확인하고,
진폭·시간을 바꾸어 새 MuJoCo 시뮬레이션을 계산할 수 있습니다.
Codespaces에서는 같은 서버의 8000번 포트를 전달해 브라우저로 사용합니다.
단순 GitHub 저장소 페이지가 Python을 실행하는 것은 아닙니다.

별도로 배포한 정적 웹 링크는 **사전에 계산한 결과의 3D 재생용**입니다.
Python 서버가 연결되지 않은 화면에서는 새 계산 버튼이 비활성화됩니다.
7종 fixed-base arm은 실제 MuJoCo 폐루프 시뮬레이션 결과입니다.
G1 화면은 기구학 미리보기이며 동적 보행/균형 제어 시뮬레이션으로 표시하지 않습니다.
CITO 화면은 두 Cartesian arm의 3 cm 물체 이동 제한 모델입니다.
WebGL이 제공되지 않는 브라우저에서는 같은 좌표를 CPU로 3D 투영하며,
카메라 회전·확대와 재생 조작을 유지합니다. 전체 형상의 mesh renderer는 아닙니다.

## 최신 draft 대조

2026-11-10 갱신된 TeX/PDF v12과 대조한 결과는
[docs/draft_v12_code_check.md](docs/draft_v12_code_check.md)에 있습니다.
특히 draft의 선택 솔버는 SNOPT/SQP이며 검증된 기본 구현은 IPOPT입니다.
SolverSettings.backend="snopt" 선택 경로를 제공하지만 SNOPT plugin/설치/라이선스가
있는 환경에서의 실행 검증은 아직 수행하지 않았습니다.
