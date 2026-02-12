# R1Pro 커스터마이징 실습 가이드

> OmniGibson의 R1Pro 로봇을 분석하고, 컨트롤러를 교체하고, 나만의 로봇 정의를 만들어 태스크에 투입하는 4단계 실습입니다.

## 사전 준비

```bash
conda activate behavior
python -c "import omnigibson; print('OK')"
```

모든 실습은 프로젝트 루트(`~/Documents/BEHAVIOR-1K`)에서 실행합니다.

---

## Lab 1: R1Pro 구조 분석

> Isaac Sim 없이 실행 가능 (YAML 파싱만 수행)

### 목표

- R1Pro의 YAML 정의가 `Robot` 클래스에 로딩되는 과정 이해
- 컨트롤러 구조 (base, trunk, arm, gripper) 파악
- 조인트/링크 이름 매핑 확인
- R1 vs R1Pro 차이점 비교

### 실행

```bash
python custom/lab1_anatomy/inspect_r1pro.py
```

### 출력 내용

스크립트는 8개 섹션을 순서대로 출력합니다:

| 섹션 | 내용 |
|------|------|
| 등록된 로봇 목록 | `definitions/` 디렉토리의 모든 YAML (15종) |
| 1. 컨트롤러 구조 | action 벡터 내 배치 순서와 기본 컨트롤러 매핑 |
| 2. 이동 (Locomotion) | 베이스 조인트 3개 (x, y, rz) |
| 3. Holonomic Base | 전방향 이동 설정, 구형 휠 근사 여부 |
| 4. Trunk (몸통) | 토르소 조인트 4개 |
| 5. 매니퓰레이션 | 양팔 각 7 DOF, EEF/핑거/그리퍼 링크 |
| 6. Tuck/Untuck | 기본 포즈 28개 조인트값, 차이나는 인덱스 표시 |
| 7. 충돌 비활성화 쌍 | 자기 충돌 무시 링크 페어 14개 |
| 8. 속도 게인 | primitives용 선속도/각속도 게인 |

마지막에 **R1 vs R1Pro 비교**가 출력됩니다:

```
R1 vs R1Pro 비교 핵심:
  - R1:  6 DOF 팔, 26개 조인트, 그리퍼 축(axis) 제어
  - R1Pro: 7 DOF 팔, 28개 조인트, 핑거 조인트 제어, RealSense 탑재
```

### 핵심 개념: 로봇 정의 로딩 과정

```
r1pro.yaml
    ↓ OmegaConf.load()
YAML dict
    ↓ OmegaConf.merge(RobotDefinition schema, yaml)
검증된 정의
    ↓ Robot.__init__() 에서 사용
로봇 객체 (컨트롤러, 조인트, 센서 초기화)
```

`RobotDefinition` 스키마(`definition_schema.py`)가 YAML에 누락된 필드를 기본값으로 채우고, 잘못된 필드는 에러를 발생시킵니다.

### 직접 해보기

`inspect_r1pro.py`에서 `load_robot_definition("fetch")`나 `load_robot_definition("tiago")` 등 다른 로봇도 분석해보세요.

---

## Lab 2: 컨트롤러 교체 실험

> `--analyze-only` 모드는 Isaac Sim 불필요, `--experiment N`은 시뮬레이션 필요

### 목표

- InverseKinematicsController vs JointController 차이 이해
- velocity vs position 모터 타입 비교
- MultiFingerGripperController의 binary/smooth 모드 비교
- `controller_config` 오버라이드 방법 습득

### 실행

```bash
# 분석만 (시뮬레이션 없음)
python custom/lab2_controller_swap/controller_experiment.py --analyze-only

# 특정 실험 시뮬레이션 (1~4 중 선택)
python custom/lab2_controller_swap/controller_experiment.py --experiment 1
python custom/lab2_controller_swap/controller_experiment.py --experiment 2
```

### 4가지 실험 설정

#### Exp1: IK Controller (R1Pro 기본값)

```
arm_left/right -> InverseKinematicsController
```
- 입력: EEF 목표 pose `(x, y, z, rx, ry, rz)` = **6차원**
- 내부에서 역기구학 계산 후 조인트 각도로 변환
- 직관적이지만 singularity 근처에서 불안정

#### Exp2: Joint Controller (직접 조인트 제어)

```
arm_left/right -> JointController (position)
```
- 입력: 각 조인트의 목표 각도 = **7차원** (R1Pro)
- PD 제어로 목표 위치 추종
- 정밀하지만 작업 공간 직관성 떨어짐

#### Exp3: Velocity Motor + Binary Gripper

```
arm/trunk -> JointController (velocity)
gripper   -> MultiFingerGripperController (binary)
```
- 모든 조인트를 속도로 제어 (정지 = 0)
- 그리퍼는 열림/닫힘 이진 명령만 가능

#### Exp4: Delta Joint Commands

```
arm/trunk -> JointController (position, use_delta_commands=True)
```
- 입력: 현재 위치 기준 **변화량**
- `action=0`이면 현재 위치 유지
- RL 학습에서 가장 자주 사용되는 설정

### 핵심 개념: Action Space 크기 변화

컨트롤러 종류에 따라 로봇의 총 action 차원이 달라집니다:

| 컨트롤러 구성 요소 | IK 사용 시 | Joint 사용 시 |
|---------------------|-----------|--------------|
| base                | 3         | 3            |
| trunk               | 4         | 4            |
| arm_left            | **6**     | **7**        |
| gripper_left        | 1         | 1            |
| arm_right           | **6**     | **7**        |
| gripper_right       | 1         | 1            |
| **합계**            | **21**    | **23**       |

### 직접 해보기

`controller_experiment.py`의 `EXPERIMENT_*` dict를 수정해서 자신만의 조합을 만들어보세요. 예를 들어 왼팔은 IK, 오른팔은 Joint로 설정하는 비대칭 구성도 가능합니다.

---

## Lab 3: 커스텀 로봇 YAML 만들기

> `--analyze-only`는 Isaac Sim 불필요, `--register-and-run`은 시뮬레이션 필요

### 목표

- 로봇 YAML 정의 파일의 자동 등록 메커니즘 이해
- 커스텀 YAML 작성 및 원본과의 차이 확인
- 커스텀 로봇 시뮬레이션 스폰

### 등록 메커니즘

OmniGibson은 `robots/definitions/*.yaml` 파일명을 로봇 이름으로 자동 등록합니다:

```python
# omnigibson/robots/__init__.py
robot_config_dir = Path(__file__).parent / "definitions"
for yaml_file in sorted(robot_config_dir.glob("*.yaml")):
    REGISTERED_ROBOTS.append(yaml_file.stem)
```

따라서 `r1pro_custom.yaml`을 `definitions/`에 넣으면 `"r1pro_custom"`으로 사용할 수 있습니다.

### 주의사항

커스텀 로봇을 만들 때 흔히 부딪히는 두 가지 문제가 있습니다:

#### 1. Import 순서와 등록 타이밍

`REGISTERED_ROBOTS`는 `omnigibson.robots` 모듈이 **최초 import될 때 한 번만** 구축됩니다. YAML 파일을 `definitions/`에 복사하기 **전에** omnigibson을 import하면, 이미 구축된 리스트에 새 로봇이 포함되지 않습니다. Python의 모듈 캐싱으로 인해 이후 import에서도 리스트가 갱신되지 않습니다.

```
❌ analyze_diff() → omnigibson import (REGISTERED_ROBOTS 확정) → register_robot() → 등록 실패
✅ register_robot() → analyze_diff() → omnigibson import (r1pro_custom 포함) → 등록 성공
```

#### 2. USD 에셋 경로 (`usd_path` 필드)

커스텀 로봇이 기존 로봇과 동일한 하드웨어(USD 모델)를 사용하는 경우, YAML에 `usd_path`를 명시해야 합니다. 지정하지 않으면 `robot.py`가 모델 이름으로 경로를 자동 생성하여 존재하지 않는 파일을 참조합니다:

```
기본 규칙: models/{model}/usd/{model}.usda → models/r1pro_custom/usd/r1pro_custom.usda (존재하지 않음!)
```

YAML에 아래 한 줄을 추가하면 원본 R1Pro의 USD 에셋을 공유합니다:

```yaml
usd_path: models/r1pro/usd/r1pro.usda
```

### r1pro_custom.yaml 변경사항

`custom/lab3_custom_robot/r1pro_custom.yaml`은 원본 R1Pro를 기반으로 5가지를 변경합니다:

| 항목 | 원본 R1Pro | 커스텀 |
|------|-----------|--------|
| USD 에셋 경로 | (자동: model명 기반) | `models/r1pro/usd/r1pro.usda` (원본 공유) |
| 기본 arm 컨트롤러 | `InverseKinematicsController` | `JointController` |
| 선속도 게인 | 0.3 | **0.5** |
| 각속도 게인 | 0.2 | **0.3** |
| 작업 공간 범위 | [-45, 45]도 | **[-60, 60]도** |
| untucked 포즈 | 원본 | 팔을 더 벌린 **준비 자세** |

### 실행

```bash
# 1단계: 원본과 차이 분석
python custom/lab3_custom_robot/spawn_custom_robot.py --analyze-only

# 2단계: 등록 + 시뮬레이션
python custom/lab3_custom_robot/spawn_custom_robot.py --register-and-run

# (정리) 등록 해제
python custom/lab3_custom_robot/spawn_custom_robot.py --unregister
```

### --analyze-only 출력 예시

```
[1] 기본 컨트롤러 변경:
    arm_left:  InverseKinematicsController -> JointController          <-- 변경
    arm_right: InverseKinematicsController -> JointController          <-- 변경

[2] Primitives 속도 게인:
    linear:  0.3 -> 0.5  <-- 변경
    angular: 0.2 -> 0.3  <-- 변경

[3] 팔 작업 공간 범위:
    left:  [-45, 45] -> [-60, 60]  <-- 변경

[4] Untucked 포즈 변경된 조인트:
    조인트[12]:  +1.5700 ->  +1.2000
    조인트[13]:  -1.5700 ->  -1.2000
    ...
```

### --register-and-run 동작

1. `r1pro_custom.yaml`을 `OmniGibson/omnigibson/robots/definitions/`에 복사
2. `REGISTERED_ROBOTS`에 `"r1pro_custom"` 자동 추가
3. 빈 장면에 커스텀 로봇 스폰
4. 로봇 정보 (action dim, 컨트롤러, 팔 DOF) 출력
5. 100스텝 랜덤 액션 시뮬레이션

### 직접 해보기

`r1pro_custom.yaml`을 열어서 직접 수정해보세요:

- `arm_workspace_range`를 `[-90, 90]`으로 확장
- `untucked_default_joint_pos`의 팔 조인트 값을 바꿔서 다른 초기 자세 만들기
- `disabled_collision_pairs`에 새 쌍을 추가/제거하고 시뮬레이션에서 충돌 확인
- 단, `raw_controller_order`, `arm_joint_names`, `finger_joint_names` 등은 실제 USD 모델의 조인트와 일치해야 하므로 함부로 변경하면 안됩니다

---

## Lab 4: 커스텀 로봇으로 태스크 수행

> 시뮬레이션 필요. `--simple`은 데이터셋 불필요, 기본 실행은 데이터셋 필요.

### 사전 조건

- Lab 3에서 `r1pro_custom.yaml`이 등록되어 있어야 합니다
- 기본 실행(`picking_up_trash` 태스크)은 데이터셋 다운로드 필요

### 목표

- YAML config 하나로 환경 전체(장면 + 로봇 + 태스크)를 구성하는 방법
- Gymnasium 인터페이스의 `obs, reward, terminated, truncated, info` 흐름
- `grasping_mode`에 따른 물체 조작 차이
- 원본 vs 커스텀 로봇 성능 비교

### 실행

```bash
# 빈 장면에서 간단 테스트 (데이터셋 불필요)
python custom/lab4_task_integration/run_task.py --simple

# picking_up_trash 태스크 전체 실행 (데이터셋 필요)
python custom/lab4_task_integration/run_task.py

# 원본 R1Pro vs 커스텀 비교
python custom/lab4_task_integration/run_task.py --compare
```

### custom_behavior.yaml 구조

`custom/lab4_task_integration/custom_behavior.yaml`은 4개 블록으로 구성됩니다:

```yaml
env:        # 시뮬레이션 환경 설정 (외부 센서, 해상도 등)
scene:      # 장면 선택 (house_double_floor_lower)
robots:     # 로봇 설정 (커스텀 로봇 + 컨트롤러 + 센서)
task:       # 태스크 정의 (BehaviorTask + 보상/종료 조건)
```

원본(`r1pro_behavior.yaml`) 대비 커스텀 변경 3가지 (`# CUSTOM:` 주석으로 표시):

| 항목 | 원본 | 커스텀 |
|------|------|--------|
| 로봇 모델 | `R1Pro` | `r1pro_custom` |
| grasping_mode | `physical` | `assisted` (물체 잡기 쉬움) |
| max_steps | 500 | `1000` (탐색 시간 확보) |

### grasping_mode 비교

| 모드 | 동작 | 용도 |
|------|------|------|
| `physical` | 접촉 마찰 + 핑거 힘으로만 파지 | 현실적 시뮬레이션 |
| `assisted` | 두 핑거가 물체에 닿으면 자석처럼 부착 | 학습/디버깅 |
| `sticky` | 한 핑거만 닿아도 부착 | 빠른 프로토타이핑 |

### Gymnasium 스텝 루프

```python
obs, reward, terminated, truncated, info = env.step(action)
```

| 반환값 | 의미 |
|--------|------|
| `obs` | RGB/depth 이미지 + 고유 수용 감각 (proprio) |
| `reward` | `r_potential` 기반 보상 (태스크 목표에 가까울수록 높음) |
| `terminated` | 태스크 성공 완료 |
| `truncated` | `max_steps` 도달로 종료 |
| `info` | 태스크별 추가 정보 |

### --compare 출력 예시

```
R1Pro (원본)    평균 보상: -0.0023
r1pro_custom    평균 보상: -0.0019
차이:                       +0.0004
```

> 랜덤 액션이므로 보상 자체보다는 **로봇 구조/컨트롤러 차이가 동작에 미치는 영향**을 관찰하는 것이 목적입니다.

### 직접 해보기

- `custom_behavior.yaml`에서 `activity_name`을 다른 태스크로 변경 (예: `cleaning_up_the_kitchen_only`)
- `grasping_mode`를 `physical` → `sticky`로 바꿔서 파지 성공률 차이 관찰
- `max_steps`를 줄여서 truncation 빈도 확인
- `run_task.py`에 랜덤 대신 간단한 규칙 기반 액션 로직 작성

---

## 실습 요약 / 빠른 참조

### 전체 실행 순서

```bash
# Lab 1: 구조 분석 (시뮬 불필요)
python custom/lab1_anatomy/inspect_r1pro.py

# Lab 2: 컨트롤러 분석 (시뮬 불필요)
python custom/lab2_controller_swap/controller_experiment.py --analyze-only

# Lab 2: 컨트롤러 시뮬레이션 (시뮬 필요)
python custom/lab2_controller_swap/controller_experiment.py --experiment 1

# Lab 3: 커스텀 분석 (시뮬 불필요)
python custom/lab3_custom_robot/spawn_custom_robot.py --analyze-only

# Lab 3: 커스텀 등록 + 스폰 (시뮬 필요)
python custom/lab3_custom_robot/spawn_custom_robot.py --register-and-run

# Lab 4: 간단 테스트 (시뮬 필요, 데이터셋 불필요)
python custom/lab4_task_integration/run_task.py --simple

# Lab 4: 전체 태스크 (시뮬 + 데이터셋 필요)
python custom/lab4_task_integration/run_task.py

# (정리) 커스텀 로봇 등록 해제
python custom/lab3_custom_robot/spawn_custom_robot.py --unregister
```

### 파일 맵

```
custom/
├── INSTALL_GUIDE.md                          # 설치 가이드
├── LAB_GUIDE.md                              # 이 문서
├── README.md                                 # 폴더 개요
│
├── r1pro_original.yaml                       # 참고: 원본 R1Pro 정의
├── r1_original.yaml                          # 참고: 원본 R1 정의
├── r1pro_behavior_original.yaml              # 참고: 원본 환경 config
├── definition_schema_reference.py            # 참고: 로봇 정의 스키마
│
├── lab1_anatomy/
│   └── inspect_r1pro.py                      # YAML 파싱 + 구조 출력
│
├── lab2_controller_swap/
│   └── controller_experiment.py              # 4가지 컨트롤러 실험
│
├── lab3_custom_robot/
│   ├── r1pro_custom.yaml                     # 커스텀 로봇 정의 (편집 대상)
│   └── spawn_custom_robot.py                 # 등록/스폰/해제
│
└── lab4_task_integration/
    ├── custom_behavior.yaml                  # 커스텀 환경 config (편집 대상)
    └── run_task.py                           # 태스크 실행 + 비교
```

### 원본 코드 참조 경로

| 파일 | 역할 |
|------|------|
| `OmniGibson/omnigibson/robots/robot.py` | Robot 클래스 (모든 로봇의 기반) |
| `OmniGibson/omnigibson/robots/__init__.py` | YAML 자동 등록 로직 |
| `OmniGibson/omnigibson/robots/definitions/` | 15종 로봇 YAML 정의 |
| `OmniGibson/omnigibson/robots/definition_schema.py` | YAML 스키마 (dataclass) |
| `OmniGibson/omnigibson/controllers/` | 컨트롤러 구현체 |
| `OmniGibson/omnigibson/configs/` | 환경 config 예시 |
| `OmniGibson/omnigibson/examples/robots/` | 공식 로봇 예제 스크립트 |
