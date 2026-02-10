# R1Pro 커스터마이징 실습

OmniGibson의 R1Pro 로봇을 이해하고 커스터마이징하는 실습 자료입니다.
원본 코드를 건드리지 않고 custom/ 폴더 안에서 작업합니다.

## 폴더 구조

```
custom/
├── README.md                        # 이 파일
├── r1pro_original.yaml              # 원본 R1Pro 로봇 정의 (읽기 참고용)
├── r1_original.yaml                 # 원본 R1 로봇 정의 (읽기 참고용)
├── r1pro_behavior_original.yaml     # 원본 환경 설정 (읽기 참고용)
├── definition_schema_reference.py   # 로봇 정의 스키마 (읽기 참고용)
│
├── lab1_anatomy/                    # 실습 1: R1Pro 구조 분석
│   └── inspect_r1pro.py
│
├── lab2_controller_swap/            # 실습 2: 컨트롤러 교체 실험
│   └── controller_experiment.py
│
├── lab3_custom_robot/               # 실습 3: 나만의 로봇 정의 만들기
│   ├── r1pro_custom.yaml
│   └── spawn_custom_robot.py
│
└── lab4_task_integration/           # 실습 4: 커스텀 로봇으로 태스크 수행
    ├── custom_behavior.yaml
    └── run_task.py
```

## 실습 순서

| 실습 | 내용 | 핵심 개념 |
|------|------|-----------|
| Lab 1 | R1Pro 내부 구조 출력/분석 | YAML 정의, 컨트롤러 구조, 조인트 매핑 |
| Lab 2 | 컨트롤러 종류 바꿔보기 | IK vs Joint, velocity vs position, gripper mode |
| Lab 3 | 커스텀 로봇 YAML 작성 | 로봇 등록, 조인트 수정, collision pair |
| Lab 4 | 커스텀 로봇으로 태스크 | 환경 config, BehaviorTask 연동 |

## 사전 요구사항

```bash
conda activate behavior
# OmniGibson + BDDL이 설치되어 있어야 합니다
python -c "import omnigibson; print('OK')"
```
