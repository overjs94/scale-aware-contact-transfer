# Scale-aware contact transfer

URDF 기반 동역학·기구학과 논문의 Sec. III-B–IV를 위한 연구용 Python 패키지입니다.
로봇별 System_Param의 질량·관성·축·링크 길이를 재작성하지 않고 URDF에서 읽습니다.


## 자유 articulated object · 3-arm 명목 진단 baseline

**새 기록은 시나리오·동역학 검증용 baseline이며 Sec. III-B–IV 전체 계획 및 hybrid 제어의 완료 결과가 아닙니다.**

[자유 박스 + 뚜껑](https://overjs94.github.io/Contact_contraction/?model=free_box_lid),
[자유 접이식 패널](https://overjs94.github.io/Contact_contraction/?model=free_folding_object),
[자유 수납함 + 서랍](https://overjs94.github.io/Contact_contraction/?model=free_portable_drawer)는
Panda G/P/G 세 팔이 본체 파지·본체 지지·내부 링크 조작을 담당합니다. 물체 본체는 자유 6 DoF,
힌지/레일은 물체 내부 관절이며 world에 고정하지 않습니다. 125개 소형 dense contact를
링크/기능별 대표 접촉 3개로 contraction하고 위치만 4배 전달한 뒤, 실제 target 동역학으로
세 팔의 IK·접촉력/토크 realization을 계산합니다. Group ID와 assignment/path는 synthetic 입력이며
학습 inference는 실행하지 않습니다. 기존 단일 팔 고정 힌지 과제는 component test로 구분합니다.

[5 N 외력 + 화살표](https://overjs94.github.io/Contact_contraction/?model=free_box_push)의
**외력 최대 · 화살표 · 8.5 s**를 누르면 실제 작용점의 화살표와 크기를 확인할 수 있습니다.
[수식·실행 검증·한계](docs/free-articulated-realization.md)에 세 과제와 contact-disabled 대조를 정리했습니다.

## 본 실험: 단일 패널 → 고정 지그

[공개 URDF/mesh 뷰어](https://overjs94.github.io/Contact_contraction/?model=panel_fixture)
에는 단일 패널의 lift → 12° 재배향 → 지그 안착 → 하중 전달 →
전체 공구 릴리스 과제를 구성했습니다. 기존 상판·다리 조립은 보조 데모입니다.

기본 30초 / 8 keyframe / Panda G·P·P 실행은 작은 synthetic contracted 접촉점의
위치 refinement를 실제 수행한 뒤, 위치만 4배 전달하고 2 kg target 패널의
힘·토크를 다시 계산합니다. 네트워크 추론이나 손 시연 학습 결과는 아닙니다.
[실험 비교 및 로봇별 저수준 제어 수식](docs/panel-experiments-and-control.md)에
검증 범위와 G1 이동형 듀얼암의 미구현 조건을 정리했습니다.

추가한 [RBY1 + Panda 작업영역 확장 시나리오](https://overjs94.github.io/Contact_contraction/?model=mobile_panel)는
36초·9 keyframe입니다. 고정 Panda가 지지를 넘기고 물러난 뒤, RBY1-A의 바퀴를
구동해 1.4 m 이동하고 고정 베이스로는 닿지 않는 지그에 패널을 안착·해제합니다.
베이스와 패널은 모두 자유 강체이며 실행 중 위치를 강제하지 않습니다. 공식
URDF·mesh·라이선스를 보존하고, 바퀴 토크 한계와 수동 caster의 skid 근사는
시뮬레이션 가정으로 구분합니다. 직선 경로·초기 파지·모드와 torso 자세는 주어집니다.

~~~bash
python tools/fetch_panda_meshes.py
python examples/panel_fixture.py
python tools/fetch_rby1_assets.py
python examples/mobile_panel.py
# 선택한 RB5-850E의 독립 위치제어 검증 (패널 운반 시험과 구별)
python tools/fetch_rb5_meshes.py
python examples/rb5_position_servo.py
~~~

Python 서버는 새 최적화·MuJoCo 실행을 수행하는 로컬 계산 백엔드입니다.
GitHub Pages는 무료 정적 기록 재생이며 Python 서버를 실행하지 않습니다.
`npm --prefix web run build` 후 `.venv/bin/python web/server.py --port 8010`으로
로컬 UI에서 contracted source JSON을 입력·업로드하고 같은 과제를 재계산하며
source refinement on/off를 비교할 수 있습니다.

## 수동 관절 물체 · 외력 시험 · 렌더링

[문 열기](https://overjs94.github.io/Contact_contraction/?model=hinged_door),
[접이식 상판](https://overjs94.github.io/Contact_contraction/?model=folding_panel),
[서랍 당기기](https://overjs94.github.io/Contact_contraction/?model=drawer)를 각각 14초/5 keyframe으로
추가했습니다. 물체 actuator 없이 실제 로봇 접촉으로 수동 hinge/rail을 움직이며, 접촉을 끈
대조 실행은 세 과제 모두 실패합니다. 주어진 contact/관절 경로의 일반화된 힘·토크 최적화이며
articulated contraction 학습이나 일반 다관절 CITO를 수행한 것은 아닙니다.

Studio PBR/그림자와 Analysis/collision 표시를 선택할 수 있습니다.
[2 N](https://overjs94.github.io/Contact_contraction/?model=mobile_push_2),
[8 N](https://overjs94.github.io/Contact_contraction/?model=mobile_push_8),
[25 N](https://overjs94.github.io/Contact_contraction/?model=mobile_push_25) 외력 실행 기록은
같은 mobile 계획·제어 이득을 사용합니다. 25 N은 위치 회복 후에도 계획 접촉 허용영역을 벗어나
전체 FAIL로 보존했습니다. [물리 검증·제어 수식·오픈 데이터셋 및 조건](docs/contact-validation-and-datasets.md)을
함께 확인하세요. 가정된 joint/contact 물성은 실물에서 보정하지 않았습니다.

## 설치

검증은 Python 3.12, Linux에서 수행했습니다. Windows에서는 WSL2 사용을
추천하며 dev container 설정도 제공합니다. WSL2와 Docker 실행 자체는 이번
환경에서 별도로 검증하지 않았습니다.

~~~bash
python -m venv .venv
source .venv/bin/activate
python -m pip install -e ".[dev,simulation,conic]"
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

[무료 공개 웹 뷰어 열기](https://overjs94.github.io/Contact_contraction/)
(계산 결과 재생용, GitHub Pages).

실험 라이브러리는 목적에 따라 나눕니다.

- **논문 목표:** 단일 패널 → 지그를 핵심으로, 모바일 협력·자유 articulated
  물체를 확장 과제, 두 부품 조립을 보조 과제로 표시합니다. 현재 기록은
  기준 동작이며 Sec. III-B–IV 전체 최적화 완료를 뜻하지 않습니다.
- **단일 embodiment:** 7종 로봇팔의 관절 추종과 G1 기구학 미리보기입니다.
- **요소·강건성:** 외력 시험, 접촉 도구·제한 CITO, 고정 힌지/레일 단일 팔
  시험입니다. 외란 시험에서 발생한 허용영역 위반도 그대로 표시합니다.

각 화면은 목적·검증 범위를 표시하며, 실험 설정은 선택한 시나리오와 함께
전환됩니다. 단일 로봇 화면에는 협력 조작 설정을 표시하지 않습니다.

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
8종 로봇은 원본 URDF의 visual mesh를 로드하고 기록된 관절 자세로 재생합니다.
표시 형상에서 URDF visual, URDF collision, 링크 좌표를 선택할 수 있습니다.
URDF origin·mesh scale·material과 floating-base pose를 적용하며, mesh 로딩 실패를
화면에 표시합니다. Mesh는 모델과 동일한 upstream commit의 HTTPS 경로에서
불러오므로 네트워크 연결이 필요합니다. WebGL이 없는 브라우저는 명시적으로
표시한 CPU 링크 좌표 미리보기를 사용합니다. 원본 형상을 보여도 기존 추종
시뮬레이션의 충돌이 켜지는 것은 아니며, CITO 제한 모델은 좌표 재생을 유지합니다.
표시용 URDF·라이선스·재생성 절차는 [docs/models.md](docs/models.md)에 있습니다.

## 최신 draft 대조

첨부된 `Contact_contraction_MS_TRO (2).pdf`의 Sec. III-B–IV 대조와 현재 실행
경계는 [docs/manuscript-equation-audit.md](docs/manuscript-equation-audit.md)에
있습니다. 이 원고의 수치 절차는 CasADi + IPOPT입니다. 이전 v12 대조 문서는
과거 기록으로 유지합니다. SNOPT 비교 경로는 제공하지만 plugin/설치/라이선스가
있는 환경에서의 실행 검증은 아직 수행하지 않았습니다.

## 계획과 접촉 실행 검증

기본 계획기는 CasADi + IPOPT로 유지합니다. 같은 CITO 문제를 SNOPT와 비교하는
도구는 동일한 초기화·접촉 후보·continuation·residual 기준을 사용하며,
통과한 실행의 시간만 집계합니다. SNOPT가 없으면 `unavailable`로 기록합니다.

~~~bash
python examples/fixed_wrench_margin.py
python tools/benchmark_cito_solvers.py --backends ipopt snopt --repeats 3
~~~

`conic.fixed_wrench_margin`은 고정 접촉 위치·법선의 wrench margin을 Clarabel의
SOCP로 독립 계산합니다. 고정 arm 자세와 assignment를 함께 주면 III-C의
중력·관절 토크 제한도 검사합니다. 접촉 형상·자세·모드 탐색은 기존 NLP에 둡니다.

`contact_simulation.MujocoContactScene`은 별도 MJCF scene의 충돌 형상과 자유 물체를
사용합니다. CITO 관절 궤적은 `trajectory_reference`로 제어기 reference에 연결하고,
물체 자세는 MuJoCo 동역학으로 계산합니다. 접촉 위치·법선·signed gap·wrench와
엔진 버전·시간 간격·접촉 설정을 `execution.ExecutionTrace`에 기록합니다.
`simulation.MujocoRobot`의 기존 marker 모델은 관절 추종 검증용으로 유지합니다.

CRISP C++ 실행기는 아직 연결하지 않았습니다. 같은 부품·fixture·제어 reference의
실행 결과를 공통 trace 형식으로 옮겨 비교하는 경계와, 실제 dual-arm 패널 운반 및
정밀 삽입 실험의 완료 조건은 [docs/contact_execution.md](docs/contact_execution.md)에
정리했습니다. 현재의 합성 접촉 테스트가 실제 가구 조립 성공을 의미하지는 않습니다.

## Contracted contact에서 세 팔 실행까지

네트워크가 준 contact group을 가정한 3-keyframe·3-arm 예제를 실행할 수 있습니다.
URDF Cartesian arm 세 개에 구형 point tool, 두 손가락 gripper, 흡착 cup을 붙이고,
keyframe 최적화와 두 CITO 전이를 거쳐 자유 패널의 실제 MuJoCo 동역학을 실행합니다.
세 keyframe 뒤에는 손가락을 열고 흡착을 vent하며 도구를 물립니다.

~~~bash
contact-transfer demo-three-arm --output outputs/three_arm --viewer web
npm --prefix web run build
python web/server.py --host 127.0.0.1 --port 8000
~~~

뷰어에서 **Three-arm contracted contacts**를 선택하면 실제 도구·물체 상태를
재생하고 keyframe와 release 시점으로 이동할 수 있습니다. 입력 JSON, 도구 모델,
힘 한계 및 검증 범위는 [docs/three_arm_experiment.md](docs/three_arm_experiment.md)에
있습니다. 현재 팔은 접촉 검증용 Cartesian 모델이며 실제 vendor arm 실험은 아닙니다.

Three real Panda arms and the experiment web UI are available:

```sh
python tools/fetch_panda_meshes.py
contact-transfer demo-three-arm --scenario franka --output outputs/three_franka --viewer web
npm --prefix web ci && npm --prefix web run build
.venv/bin/python web/server.py
```

Open `http://localhost:8000` to edit three keyframes, panel mass and contracted
contact JSON, run optimization and MuJoCo, cancel a job, and inspect measured
results using actual URDF/mesh playback. The Panda baseline optimizes forces and
torques on an IK-prescribed path; the original Cartesian example uses CITO.
See [experiment scope and validation](docs/three_arm_experiment.md).

The free public viewer is live at
[Contact Transfer Lab](https://overjs94.github.io/Contact_contraction/), published
from [overjs94/Contact_contraction](https://github.com/overjs94/Contact_contraction).
This source repository stays private. Only the built web UI, recorded examples,
URDF manifests and required licenses are exported:

```sh
npm --prefix web run build
python tools/export_public_viewer.py /path/to/Contact_contraction
```

The source workflow packages a `public-viewer` artifact without requiring Pages
permissions here. The public repository verifies file hashes and deploys its
`site/` directory using GitHub Pages. Initial activation may require an owner to
select **Settings → Pages → Source → GitHub Actions** in the public repository;
the integration token cannot create a Pages site. Static hosting replays recorded
runs; new CasADi/IPOPT and MuJoCo computations require the Python backend.

The viewer opens the free box/lid three-arm baseline. The
[30-second tabletop/leg assembly](docs/table_assembly.md) is an auxiliary
multi-body demonstration under the paper-task category: three real Panda arms,
two native grippers and one spherical support, tabletop reorientation,
peg/socket insertion, and leg-hand release.

```sh
contact-transfer demo-table-assembly --output outputs/table_assembly --viewer web
npm --prefix web run build
.venv/bin/python web/server.py --port 8010
```

For new computations, open the Python-served UI and choose **책상 조립 · G / P / G**.
The existing backend now implements this scenario, duration retiming, isolated
jobs, progress and cancellation. See [what the Python server does](docs/python_server.md).
GitHub Pages continues to serve the measured recording without a compute backend.

The coupled Panda lift now has an optional CRISP execution backend and explicit
hybrid-feedback versus pose/force-feedforward controller comparisons. See
[the reproducible commands and PMI scope](docs/crisp-and-midpoint.md).
The CRISP SDK is installed separately under its academic/non-commercial license;
it is not bundled with this package or the public viewer.
