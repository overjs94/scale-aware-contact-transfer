# Python 계산 서버와 GitHub Pages

GitHub Pages는 HTML/JavaScript와 저장된 결과 파일을 배포합니다. 로봇 URDF와
mesh, 실제 시뮬레이션 기록은 브라우저에서 재생하지만 Python·CasADi·IPOPT·
MuJoCo 프로세스를 실행하지 않습니다. 페이지에서 조건을 바꾼 것만으로 새
계산이 실행되지는 않습니다.

`web/server.py`가 계산 서버입니다. 웹 UI와 같은 주소에서 API를 제공하고,
계산 요청마다 별도 Python 프로세스를 실행합니다. 새 `table_assembly` 시나리오도
구현되어 있습니다. 시간 변경 시 두 부품의 8개 keyframe, 파지·삽입·해제 시점과
계획 표본 간격을 함께 바꾸고, IK·접촉력/토크 최적화·동역학을 다시 실행합니다.
실제 계산 시간과 시뮬레이션 내 30초는 서로 다릅니다. API는 현재 계산 단계와
시뮬레이션 진행 시간을 보고하며, 실행 취소를 지원합니다.

`hinged_door`, `folding_panel`, `drawer`도 같은 API/작업 프로세스를 사용합니다.
예: `{"preset":"drawer","duration":14}`. 세 과제의 duration 범위는 12–60초이며
관절 경로·계획 표본 시간도 함께 조절합니다. 물체 actuator 없이 수동 hinge/rail을
로봇 접촉으로 움직입니다. 이 preset의 contact와 물성은 고정된 authored 입력이고,
네트워크 추론이나 소형 articulated source refinement는 실행하지 않습니다.
기본 14초 실행과 [물리 검증 범위](contact-validation-and-datasets.md)를 함께 확인하세요.

```sh
# 저장소 루트에서, 패키지와 mesh가 아직 없을 때:
python -m pip install -e ".[simulation,conic]"
python tools/fetch_panda_meshes.py
npm --prefix web ci
npm --prefix web run build

# 현재 작업 환경의 가상환경 사용:
.venv/bin/python web/server.py
```

`http://localhost:8000`에 접속해 **책상 조립 · G / P / G**를 선택하고
**Optimize & simulate**를 누릅니다. 기본값은 30초이며 24–60초 범위에서
전체 동작 시간을 변경할 수 있습니다. 모델 형상·접촉·solver 설정은
`src/contact_transfer/scenarios/table_assembly/`에 있습니다.

## API

- `GET /api/health`: 계산 서버와 Panda mesh 준비 상태.
- `GET /api/experiments/preset?preset=table_assembly`: 시나리오 입력.
- `POST /api/experiments`에 `{"preset":"table_assembly","duration":30}` 전송:
  작업 ID를 반환하고 별도 프로세스에서 계산 시작.
- `GET /api/experiments/{id}`: 단계, 진행 시간, 실패 이유 또는 완료 보고서.
- `POST /api/experiments/{id}/cancel`: 실행 중인 프로세스 종료.
- 완료 응답의 `recording` 경로: 실제 결과 재생 데이터. URDF manifest들도
  해당 작업 폴더에서 제공됩니다.

동시에 한 작업을 실행하며, 클라이언트가 임의의 파일 경로나 코드를 넘길 수
없습니다. 결과는 `outputs/web_jobs/{id}/`에 저장됩니다. 현재 서버는 기본적으로
loopback에 바인딩하는 로컬 연구용 서버입니다. 공개 GitHub Pages에서 새 계산을
직접 실행하는 상시 HTTPS 계산 서비스는 별도의 배포가 필요합니다.

## 서버 없이 새 기록 만들기

```sh
contact-transfer demo-table-assembly --output outputs/table_assembly --viewer web
npm --prefix web run build
python tools/export_public_viewer.py /path/to/Contact_contraction
```

공개 저장소에 갱신된 `site/`를 반영하면 새 기록을 Pages에서 재생할 수 있습니다.
Python 백엔드와 최적화 원본 코드는 공개 배포물에 포함하지 않습니다.
