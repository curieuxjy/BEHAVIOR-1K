"""
Lab 2: 컨트롤러 교체 실험
=========================
R1Pro의 컨트롤러 설정을 다양하게 바꿔보고, 각 설정이 action space에
미치는 영향을 분석합니다. 시뮬레이션을 실행하여 실제 동작 차이를 확인합니다.

실행:
    python custom/lab2_controller_swap/controller_experiment.py

    # 시뮬레이션 없이 분석만 (Isaac Sim 불필요)
    python custom/lab2_controller_swap/controller_experiment.py --analyze-only

학습 목표:
    - JointController vs InverseKinematicsController 차이
    - velocity vs position 모터 타입의 차이
    - MultiFingerGripperController의 mode (binary/smooth) 차이
    - controller_config 오버라이드 방법
"""

import argparse
import sys
from pathlib import Path
from copy import deepcopy

PROJECT_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT_ROOT / "OmniGibson"))
sys.path.insert(0, str(PROJECT_ROOT / "bddl3"))


# ============================================================
# 실험 설정 정의
# ============================================================

# 기본 로봇 설정 (공통)
BASE_ROBOT_CFG = {
    "model": "R1Pro",
    "obs_modalities": ["rgb"],
    "action_type": "continuous",
    "action_normalize": True,
    "grasping_mode": "physical",
    "default_reset_mode": "untuck",
    "sensor_config": {
        "VisionSensor": {
            "sensor_kwargs": {
                "image_height": 128,
                "image_width": 128,
            }
        }
    },
}

# 실험 1: 기본 설정 (IK 컨트롤러) — R1Pro 기본값
EXPERIMENT_1_IK = {
    "name": "Exp1: IK Controller (기본값)",
    "description": "InverseKinematicsController로 팔을 EEF 좌표로 제어",
    "controller_config": {
        "base": {
            "name": "HolonomicBaseJointController",
            "motor_type": "velocity",
            "vel_kp": 150,
        },
        "trunk": {
            "name": "JointController",
            "motor_type": "position",
            "pos_kp": 150,
            "use_delta_commands": False,
        },
        "arm_left": {
            "name": "InverseKinematicsController",
            "motor_type": "position",
            "pos_kp": 150,
        },
        "arm_right": {
            "name": "InverseKinematicsController",
            "motor_type": "position",
            "pos_kp": 150,
        },
        "gripper_left": {
            "name": "MultiFingerGripperController",
            "mode": "smooth",
        },
        "gripper_right": {
            "name": "MultiFingerGripperController",
            "mode": "smooth",
        },
    },
}

# 실험 2: Joint 직접 제어
EXPERIMENT_2_JOINT = {
    "name": "Exp2: Joint Controller (직접 조인트 제어)",
    "description": "JointController로 각 조인트 각도를 직접 제어",
    "controller_config": {
        "base": {
            "name": "HolonomicBaseJointController",
            "motor_type": "velocity",
            "vel_kp": 150,
        },
        "trunk": {
            "name": "JointController",
            "motor_type": "position",
            "pos_kp": 150,
            "use_delta_commands": False,
        },
        "arm_left": {
            "name": "JointController",
            "motor_type": "position",
            "pos_kp": 150,
            "use_delta_commands": False,
        },
        "arm_right": {
            "name": "JointController",
            "motor_type": "position",
            "pos_kp": 150,
            "use_delta_commands": False,
        },
        "gripper_left": {
            "name": "MultiFingerGripperController",
            "mode": "smooth",
        },
        "gripper_right": {
            "name": "MultiFingerGripperController",
            "mode": "smooth",
        },
    },
}

# 실험 3: Velocity 모터 + Binary 그리퍼
EXPERIMENT_3_VELOCITY = {
    "name": "Exp3: Velocity Motor + Binary Gripper",
    "description": "속도 기반 제어 + 그리퍼 열림/닫힘 이진 제어",
    "controller_config": {
        "base": {
            "name": "HolonomicBaseJointController",
            "motor_type": "velocity",
            "vel_kp": 150,
        },
        "trunk": {
            "name": "JointController",
            "motor_type": "velocity",
            "vel_kp": 150,
        },
        "arm_left": {
            "name": "JointController",
            "motor_type": "velocity",
            "vel_kp": 150,
        },
        "arm_right": {
            "name": "JointController",
            "motor_type": "velocity",
            "vel_kp": 150,
        },
        "gripper_left": {
            "name": "MultiFingerGripperController",
            "mode": "binary",
        },
        "gripper_right": {
            "name": "MultiFingerGripperController",
            "mode": "binary",
        },
    },
}

# 실험 4: Delta 명령 + OSC (Operational Space Controller)
EXPERIMENT_4_DELTA = {
    "name": "Exp4: Delta Joint Commands",
    "description": "현재 위치 기준 상대적 이동 명령 (use_delta_commands=True)",
    "controller_config": {
        "base": {
            "name": "HolonomicBaseJointController",
            "motor_type": "velocity",
            "vel_kp": 150,
        },
        "trunk": {
            "name": "JointController",
            "motor_type": "position",
            "pos_kp": 150,
            "use_delta_commands": True,
        },
        "arm_left": {
            "name": "JointController",
            "motor_type": "position",
            "pos_kp": 150,
            "use_delta_commands": True,
        },
        "arm_right": {
            "name": "JointController",
            "motor_type": "position",
            "pos_kp": 150,
            "use_delta_commands": True,
        },
        "gripper_left": {
            "name": "MultiFingerGripperController",
            "mode": "smooth",
        },
        "gripper_right": {
            "name": "MultiFingerGripperController",
            "mode": "smooth",
        },
    },
}

ALL_EXPERIMENTS = [
    EXPERIMENT_1_IK,
    EXPERIMENT_2_JOINT,
    EXPERIMENT_3_VELOCITY,
    EXPERIMENT_4_DELTA,
]


# ============================================================
# 분석 전용 모드 (시뮬레이션 불필요)
# ============================================================

def analyze_experiments():
    """각 실험의 컨트롤러 설정을 비교 분석합니다."""
    print("=" * 70)
    print("  컨트롤러 설정 비교 분석 (시뮬레이션 없음)")
    print("=" * 70)

    for exp in ALL_EXPERIMENTS:
        print(f"\n{'─'*70}")
        print(f"  {exp['name']}")
        print(f"  {exp['description']}")
        print(f"{'─'*70}")

        cfg = exp["controller_config"]
        for ctrl_name, ctrl_cfg in cfg.items():
            ctrl_type = ctrl_cfg["name"]
            motor = ctrl_cfg.get("motor_type", "-")
            mode = ctrl_cfg.get("mode", "-")
            delta = ctrl_cfg.get("use_delta_commands", False)

            # Action dimension 추정
            if ctrl_type == "HolonomicBaseJointController":
                dim_note = "3 (x, y, rz)"
            elif ctrl_type == "InverseKinematicsController":
                dim_note = "6 (x,y,z, rx,ry,rz) EEF pose"
            elif ctrl_type == "JointController" and "arm" in ctrl_name:
                dim_note = "7 (각 조인트 각도)" if "Pro" in BASE_ROBOT_CFG["model"] else "6"
            elif ctrl_type == "JointController" and "trunk" in ctrl_name:
                dim_note = "4 (토르소 조인트)"
            elif ctrl_type == "MultiFingerGripperController":
                dim_note = "1 (binary)" if mode == "binary" else "1 (smooth 0~1)"
            else:
                dim_note = "?"

            print(f"    {ctrl_name:16s} | {ctrl_type:38s} | motor={motor:10s} | "
                  f"delta={str(delta):5s} | mode={mode:6s} | dim~{dim_note}")

    # 요약 표
    print(f"\n{'='*70}")
    print("  핵심 차이 요약")
    print(f"{'='*70}")
    print("""
    IK Controller:
      - 입력: EEF 목표 pose (x,y,z,rx,ry,rz) = 6차원
      - 내부에서 역기구학 계산 -> 조인트 각도 변환
      - 장점: 직관적인 end-effector 위치 제어
      - 단점: 특이점(singularity) 근처에서 불안정

    Joint Controller (position):
      - 입력: 각 조인트의 목표 각도 = N차원 (R1Pro: 7)
      - 직접 조인트 PD 제어
      - 장점: 정밀한 조인트 레벨 제어
      - 단점: 작업 공간에서의 직관성 떨어짐

    Joint Controller (velocity):
      - 입력: 각 조인트의 목표 속도 = N차원
      - 연속적 움직임에 적합
      - delta_commands=True와 비슷하지만 속도 레벨

    Delta Commands:
      - 입력: 현재 위치 대비 변화량
      - action=0이면 현재 위치 유지
      - RL 학습에 자주 사용됨

    Gripper Binary vs Smooth:
      - binary: 0 또는 1 (완전 열림/닫힘)
      - smooth: 0~1 사이 연속값 (부분 파지 가능)
    """)


# ============================================================
# 시뮬레이션 모드
# ============================================================

def run_simulation_experiment(exp_config: dict, steps: int = 200):
    """하나의 실험 설정으로 시뮬레이션을 실행합니다."""
    import torch as th
    import omnigibson as og
    from omnigibson.macros import gm

    gm.USE_GPU_DYNAMICS = False
    gm.ENABLE_FLATCACHE = True

    print(f"\n{'='*70}")
    print(f"  시뮬레이션: {exp_config['name']}")
    print(f"{'='*70}")

    # 환경 설정
    robot_cfg = deepcopy(BASE_ROBOT_CFG)
    robot_cfg["controller_config"] = exp_config["controller_config"]

    cfg = {
        "scene": {"type": "Scene"},  # 빈 장면 (빠른 로딩)
        "robots": [robot_cfg],
    }

    env = og.Environment(configs=cfg)
    robot = env.robots[0]

    # 로봇 정보 출력
    print(f"\n  로봇 모델:       {robot.model}")
    print(f"  Action 차원:     {robot.action_dim}")
    print(f"  Action space:    {robot.action_space}")
    print(f"  컨트롤러 순서:   {robot.controller_order}")

    print(f"\n  각 컨트롤러 상세:")
    for name in robot.controller_order:
        ctrl = robot.controllers[name]
        print(f"    {name:16s} -> {ctrl.__class__.__name__:38s} | "
              f"command_dim={ctrl.command_dim}")

    # 환경 리셋
    env.reset()
    robot.reset()

    # 랜덤 액션으로 시뮬레이션
    print(f"\n  {steps} 스텝 랜덤 액션 실행 중...")
    for step in range(steps):
        if step % 30 == 0:
            action = robot.action_space.sample() * 0.05  # 작은 액션
        env.step(action=action)

        if step % 50 == 0:
            joint_pos = robot.get_joint_positions()
            print(f"    Step {step:4d} | 조인트 위치 (처음 6개): "
                  f"{joint_pos[:6].tolist()}")

    print(f"  시뮬레이션 완료!")
    og.shutdown()


def main():
    parser = argparse.ArgumentParser(description="Lab 2: 컨트롤러 교체 실험")
    parser.add_argument(
        "--analyze-only",
        action="store_true",
        help="시뮬레이션 없이 컨트롤러 설정 분석만 수행",
    )
    parser.add_argument(
        "--experiment",
        type=int,
        choices=[1, 2, 3, 4],
        default=None,
        help="특정 실험만 실행 (1~4). 미지정시 전체 분석.",
    )
    args = parser.parse_args()

    if args.analyze_only:
        analyze_experiments()
        return

    if args.experiment:
        exp = ALL_EXPERIMENTS[args.experiment - 1]
        run_simulation_experiment(exp)
    else:
        # 분석 먼저 출력 후, 실험 1만 시뮬레이션
        analyze_experiments()
        print("\n\n시뮬레이션은 --experiment N 으로 개별 실행하세요.")
        print("예: python custom/lab2_controller_swap/controller_experiment.py --experiment 1")


if __name__ == "__main__":
    main()
