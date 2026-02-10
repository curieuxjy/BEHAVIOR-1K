"""
Lab 4: 커스텀 로봇으로 BehaviorTask 수행
=========================================
Lab 3에서 등록한 r1pro_custom 로봇으로 실제 태스크(picking_up_trash)를
수행하고, 보상/종료 조건을 모니터링합니다.

사전 조건:
    - Lab 3에서 r1pro_custom.yaml이 등록되어 있어야 합니다
    - 데이터셋이 다운로드되어 있어야 합니다 (setup.sh --dataset)

실행:
    # YAML config 파일로 실행
    python custom/lab4_task_integration/run_task.py

    # 빈 장면에서 간단 테스트 (데이터셋 불필요)
    python custom/lab4_task_integration/run_task.py --simple

    # 원본 R1Pro와 비교 실행
    python custom/lab4_task_integration/run_task.py --compare

학습 목표:
    - YAML config로 전체 환경 구성하는 방법
    - BehaviorTask의 보상/종료 메커니즘
    - grasping_mode에 따른 물체 조작 차이
    - 커스텀 로봇과 원본 로봇의 성능 비교 방법
"""

import argparse
import sys
from pathlib import Path
from copy import deepcopy

import yaml

PROJECT_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT_ROOT / "OmniGibson"))
sys.path.insert(0, str(PROJECT_ROOT / "bddl3"))

CONFIG_PATH = Path(__file__).parent / "custom_behavior.yaml"


def run_simple_test(model: str = "r1pro_custom", steps: int = 200):
    """빈 장면에서 로봇의 기본 동작을 테스트합니다."""
    import torch as th
    import omnigibson as og
    from omnigibson.macros import gm

    gm.USE_GPU_DYNAMICS = False
    gm.ENABLE_FLATCACHE = True

    print(f"\n{'='*70}")
    print(f"  Simple Test: {model}")
    print(f"{'='*70}")

    cfg = {
        "scene": {"type": "Scene"},
        "robots": [
            {
                "model": model,
                "obs_modalities": ["rgb", "proprio"],
                "action_type": "continuous",
                "action_normalize": True,
                "grasping_mode": "assisted",
                "default_reset_mode": "untuck",
            }
        ],
    }

    env = og.Environment(configs=cfg)
    robot = env.robots[0]

    print(f"  모델:        {robot.model}")
    print(f"  Action dim:  {robot.action_dim}")

    env.reset()
    robot.reset()

    rewards = []
    for step in range(steps):
        if step % 30 == 0:
            action = robot.action_space.sample() * 0.03
        obs, reward, terminated, truncated, info = env.step(action=action)

        rewards.append(reward)
        if step % 50 == 0:
            print(f"    Step {step:4d} | reward={reward:.4f} | "
                  f"terminated={terminated} | truncated={truncated}")

    avg_reward = sum(rewards) / len(rewards)
    print(f"\n  평균 보상: {avg_reward:.4f}")
    print(f"  시뮬레이션 완료!")
    og.shutdown()
    return avg_reward


def run_from_config():
    """custom_behavior.yaml 설정으로 전체 태스크를 실행합니다."""
    import torch as th
    import omnigibson as og
    from omnigibson.macros import gm

    gm.USE_GPU_DYNAMICS = False
    gm.ENABLE_FLATCACHE = True

    print(f"\n{'='*70}")
    print(f"  BehaviorTask 실행: {CONFIG_PATH.name}")
    print(f"{'='*70}")

    # YAML 로드
    with open(CONFIG_PATH, "r") as f:
        cfg = yaml.safe_load(f)

    print(f"  태스크: {cfg['task']['activity_name']}")
    print(f"  로봇:  {cfg['robots'][0]['type']}")
    print(f"  장면:  {cfg['scene']['scene_model']}")

    env = og.Environment(configs=cfg)
    robot = env.robots[0]

    print(f"\n  로봇 스폰 완료")
    print(f"  Action dim:  {robot.action_dim}")
    print(f"  컨트롤러:")
    for name in robot.controller_order:
        ctrl = robot.controllers[name]
        print(f"    {name:16s} -> {ctrl.__class__.__name__} (dim={ctrl.command_dim})")

    env.reset()
    robot.reset()

    # 태스크 실행 루프
    max_steps = cfg["task"]["termination_config"]["max_steps"]
    print(f"\n  최대 스텝: {max_steps}")
    print(f"  실행 시작...\n")

    total_reward = 0.0
    for step in range(max_steps):
        if step % 30 == 0:
            action = robot.action_space.sample() * 0.05
        obs, reward, terminated, truncated, info = env.step(action=action)
        total_reward += reward

        if step % 100 == 0:
            print(f"    Step {step:4d}/{max_steps} | reward={reward:.4f} | "
                  f"cumulative={total_reward:.4f} | "
                  f"terminated={terminated} | truncated={truncated}")

        if terminated or truncated:
            print(f"\n  에피소드 종료! (step={step})")
            print(f"    terminated={terminated}, truncated={truncated}")
            print(f"    info: {info}")
            break

    print(f"\n  총 누적 보상: {total_reward:.4f}")
    print(f"  태스크 실행 완료!")
    og.shutdown()


def run_comparison():
    """원본 R1Pro와 커스텀 R1Pro의 성능을 비교합니다."""
    print("=" * 70)
    print("  R1Pro vs r1pro_custom 비교 실험")
    print("=" * 70)
    print("\n  동일한 랜덤 시드로 두 로봇을 각각 실행합니다.\n")

    # 원본
    print("--- 원본 R1Pro ---")
    reward_original = run_simple_test(model="R1Pro", steps=200)

    # 커스텀
    print("\n--- 커스텀 r1pro_custom ---")
    reward_custom = run_simple_test(model="r1pro_custom", steps=200)

    print(f"\n{'='*70}")
    print(f"  비교 결과")
    print(f"{'='*70}")
    print(f"  R1Pro (원본)    평균 보상: {reward_original:.4f}")
    print(f"  r1pro_custom    평균 보상: {reward_custom:.4f}")
    print(f"  차이:                       {reward_custom - reward_original:+.4f}")


def main():
    parser = argparse.ArgumentParser(description="Lab 4: 커스텀 로봇으로 태스크 수행")
    group = parser.add_mutually_exclusive_group()
    group.add_argument("--simple", action="store_true",
                       help="빈 장면에서 간단 테스트 (데이터셋 불필요)")
    group.add_argument("--compare", action="store_true",
                       help="원본 vs 커스텀 비교 실행")
    args = parser.parse_args()

    if args.simple:
        run_simple_test()
    elif args.compare:
        run_comparison()
    else:
        run_from_config()


if __name__ == "__main__":
    main()
