# 접촉 형상·실패·조합 선택 점검

2026-11-12 사용자 지적에 따라 원본 URDF, native 접촉 기록과 웹 좌표계를 대조했습니다.

## 장착 길이

공통 P의 EE→leading contact는 165 mm입니다. 140 mm stem 끝에 반경 25 mm 구를
장착한 물리 공구의 길이이며 수치적 위치 오차를 숨기는 bias가 아닙니다. Native G의
EE→tip은 약 152.9 mm여서 P가 약 12.1 mm 더 깁니다. 최소 장착 길이를 최적화하거나
165 mm의 필요성을 입증하지 않았습니다. 짧게 변경하려면 공통 URDF의 stem·질량·관성과
TCP를 함께 바꾸고 모든 과제의 도달성·충돌·계획·접촉 실행을 다시 검사해야 합니다.
이번 수정에서 공구 길이는 유지했습니다.

## 박스의 실제 모델 결함

구형 base handle은 x=-0.32 m의 40 mm 블록입니다. 벽 외면 x=-0.25 m와 손잡이 내면
x=-0.30 m 사이 **50 mm에 연결 형상이 없었습니다**. 같은 rigid link라서 동역학적으로
힘은 전달됐지만, 이것만으로 물리적으로 연결된 손잡이를 모델링했다고 할 수 없습니다.
기존 힘/토크 검사는 이 누락을 검출하지 못했습니다.

원본 `scenarios/free_articulated/box_lid.urdf`에 62×16×18 mm 연결부를 visual과
collision 모두 추가했습니다. 연결부는 손잡이와 벽에 겹쳐 연결되며 기존 collision
index는 보존합니다. Base의 2 kg, COM과 관성은 하드웨어를 포함한 **선언된 집합 물성**으로
유지합니다. 부품별 실측 질량 합이나 제조사 손잡이 강성·강도 검증을 주장하지 않습니다.
회귀 검사는 base와 lid 각각의 모든 box primitive가 연결되고 visual/collision이
일치하는지 확인합니다.

새 세 과제의 authored box collision을 점검했을 때 상판 5개 primitive, 다리 2개,
패널 1개, 수정 박스 본체 7개와 뚜껑 2개는 각 rigid link 안에서 모두 연결됩니다.
이 검사는 연결성 확인이며 손잡이의 재료 강도나 탄성 변형을 검증하지 않습니다.

Lid의 돌출 손잡이는 lid 윗면과 맞닿아 있었습니다. 저장된 양쪽 pad 접촉은 이 손잡이의
collision에 발생하며, native object pose와 웹 URDF FK의 오차는 6e-16 m 이하였습니다.
즉 lid의 큰 좌표계 오프셋은 확인되지 않았고 base에는 실제 누락이 있었습니다.

수정된 모델의 실행은 `outputs/box_connected_v1/box/`에 기록합니다. 기존 keyframe과
interval certificate는 현 모델의 독립 mechanics로 재검산하고, 새 연결부를 포함한
reference collision gate와 **새 native 실행**을 통과시킵니다. Target NLP를 새로 푼
실험으로 집계하지 않습니다. 기존 박스 원방법/비교 기록을 이 결과로 교체하며,
구형 `free_box_lid`와 `free_box_push`는 모델 결함을 표시한 역사 기록으로 보존합니다.

## 웹에 노출한 증거

- Viewer 아래에서 7조건 ablation과 조합 탐색 표로 바로 이동합니다. 각 비교는 재생 링크,
  PASS/FAIL, 위치 오차, 하중 접촉 유지율과 실제 실패 이유를 제공합니다.
- 접촉 확대는 일시 정지 후 기록된 native step으로 맞춥니다. 흰 점은 실제 양 pad 또는
  구형 tool의 접촉 patch 중심입니다. 원시 접촉력 norm으로 가중 평균하며 world 좌표와
  robot에 작용하는 힘의 부호, 최소 signed gap, native geom 쌍을 함께 내보냅니다.
  접촉 없는 표본에는 점을 만들지 않습니다. 색 점은 계획된 contracted anchor입니다.
- 접촉 시각화 표본은 601프레임이며 시각 사이의 접촉을 보간하지 않습니다. 물리 실행은
  1 ms입니다. 샘플 접촉 표시는 연속 접촉 certificate가 아닙니다.

## FAIL과 ablation의 해석

`execution_passed`는 task, 위치/자세 추종, 계획 하중 접촉 유지, jaw force, actuator
여유, 측정 wrench capability, reference 형상과 의도하지 않은 loaded contact를 모두
검사합니다. 동작을 끝냈거나 NLP가 수렴했다는 사실만으로 PASS가 아닙니다. 허용 위치
오차는 8 mm, 회전 오차는 0.06 rad, 필요한 접촉 유지율은 99%, authored jaw 운영
한계는 40 N/jaw입니다. 실제 하드웨어의 정격 인증으로 해석하면 안 됩니다.

각 과제는 원방법 + 6조건, 총 21회입니다. 접촉 비활성화는 음성 대조군이며 외란은
스트레스 테스트입니다. 1회 결과이므로 성공률·통계적 우위를 주장하지 않습니다.
박스/조립 source fallback에서는 refinement 제거와 같은 prior를 사용하므로 source
refinement의 효과를 보인 실험으로 해석할 수 없습니다.

## 조합이 바뀌지 않은 이유

패널 12, 박스/조립 각 24개의 허용 유형×injective 팔 배정을 조사했습니다. 두 번째
아래 지지는 P로 제한하며 현재 목적함수는 keyframe geometry/posture−rho + 비음수
전이 비용입니다. 공구 종류·개수의 비용은 없습니다. 현재 조건에서는 G의 wrench 여유가
유리하고 다른 배정은 주어진 초기값의 IK에서 실패하는 경우가 많습니다. 선택 결과
G/P/P 또는 G/P/G가 수동 고정 배치와 같으므로 화면이 바뀌지 않는 관찰이 맞습니다.
`fixed_contact_types`는 동일 연속 계획을 재사용하고 물리 실행만 새로 수행합니다.
따라서 현재 결과로 유형 선택의 성능 향상을 입증했다고 하면 안 됩니다.

웹 후보 표에는 실제 목적값, 조건부 비용 하한 가지치기와 로컬 solver/IK 실패를
구분합니다. 조건부 하한을 전역 IK 최적성으로, 로컬 실패를 물리적 불가능으로
해석하지 않습니다. 실행 중 온라인 공구 교환은 구현하지 않았습니다.

## 이번 수정의 검증

수정 박스의 7개 native 실행을 완료했습니다. 원방법·source 제거·고정 유형·팔 FF 제거·
5 N 외란은 통과했고, 고정 경로는 jaw 41.3469 N / 40 N으로 실패했습니다.
접촉 비활성화도 기대한 대로 실패했습니다. 원방법 최대 body 위치 오차는 3.2941 mm,
최대 jaw 힘은 35.9431 N입니다. 세 과제의 21개 웹 기록에 접촉 표본을 추가했고
각각의 원래 판정과 7조건 행렬을 보존했습니다.

변경부 Python 검사 9개와 free-body mechanics/API·공통 공구 검사 6개,
최종 브라우저 검사 5개가 통과했습니다. 실제 Panda/Robotiq mesh 16개를 읽은
desktop/mobile의 뚜껑·본체 확대에서도 page error와 문서 overflow가 없었습니다.
로그는 [validation/full_tasks](validation/full_tasks)에 보존합니다.
