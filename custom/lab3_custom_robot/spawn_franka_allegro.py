"""
Franka + Allegro Hand 시뮬레이션
================================
R1Pro에 Allegro hand를 마운트하기 전에,
Franka 로봇에서 gripper vs Allegro hand 차이를 먼저 확인합니다.

실행:
    # 1단계: YAML 분석만 (시뮬레이션 불필요)
    python custom/lab3_custom_robot/spawn_franka_allegro.py --analyze-only

    # 2단계: Franka + 기본 gripper 시뮬레이션
    python custom/lab3_custom_robot/spawn_franka_allegro.py --run-gripper

    # 3단계: Franka + Allegro hand 시뮬레이션
    python custom/lab3_custom_robot/spawn_franka_allegro.py --run-allegro

    # 4단계: 둘 다 나란히 비교 (동일 환경에서 동시 실행)
    python custom/lab3_custom_robot/spawn_franka_allegro.py --compare

학습 목표:
    - end_effector variant 시스템의 동작 원리 이해
    - gripper(2 DOF) vs Allegro hand(16 DOF)의 컨트롤러 차이 확인
    - R1Pro에 Allegro hand를 마운트할 때 필요한 요소 파악
"""

import argparse
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT_ROOT / "OmniGibson"))
sys.path.insert(0, str(PROJECT_ROOT / "bddl3"))

DEFINITIONS_DIR = PROJECT_ROOT / "OmniGibson" / "omnigibson" / "robots" / "definitions"


def analyze_end_effectors():
    """Franka의 gripper vs Allegro 정의를 비교 분석합니다."""
    from omegaconf import OmegaConf
    from omnigibson.robots.definition_schema import RobotDefinition

    schema = OmegaConf.structured(RobotDefinition)
    franka_def = OmegaConf.to_container(
        OmegaConf.merge(schema, OmegaConf.load(DEFINITIONS_DIR / "franka.yaml")),
        resolve=True,
    )

    eefs = franka_def["manipulation"]["end_effectors"]
    supported = franka_def["manipulation"]["supported_end_effector"]

    print("=" * 70)
    print("  Franka End Effector Variant 분석")
    print("=" * 70)
    print(f"\n  지원 end effector: {supported}")

    # gripper vs allegro 상세 비교
    gripper = eefs["gripper"]
    allegro = eefs["allegro"]

    print(f"\n{'─'*70}")
    print(f"  {'항목':30s} {'Gripper':20s} {'Allegro':20s}")
    print(f"{'─'*70}")
    print(f"  {'model':30s} {gripper['model']:20s} {allegro['model']:20s}")
    print(f"  {'USD 경로':30s} {gripper['usd_path']}")
    print(f"  {'':30s} {allegro['usd_path']}")
    print(f"  {'EEF 링크':30s} {gripper['eef_link_names']['0']:20s} {allegro['eef_link_names']['0']:20s}")

    g_fingers = gripper["finger_link_names"]["0"]
    a_fingers = allegro["finger_link_names"]["0"]
    print(f"  {'Finger 링크 수':30s} {len(g_fingers):20d} {len(a_fingers):20d}")

    g_joints = gripper["finger_joint_names"]["0"]
    a_joints = allegro["finger_joint_names"]["0"]
    print(f"  {'Finger 조인트 수':30s} {len(g_joints):20d} {len(a_joints):20d}")

    g_dof = len(gripper["default_joint_pos"])
    a_dof = len(allegro["default_joint_pos"])
    print(f"  {'전체 DOF (arm+hand)':30s} {g_dof:20d} {a_dof:20d}")
    print(f"  {'  - Arm DOF':30s} {7:20d} {7:20d}")
    print(f"  {'  - Hand DOF':30s} {g_dof - 7:20d} {a_dof - 7:20d}")
    print(f"  {'URDF 지원':30s} {'O':20s} {'X (not_support_urdf)':20s}")

    print(f"\n[Gripper] finger_joint_names:")
    for j in g_joints:
        print(f"    - {j}")

    print(f"\n[Allegro] finger_joint_names (16 DOF):")
    for i, j in enumerate(a_joints):
        finger_map = {0: "thumb", 4: "index", 8: "middle", 12: "ring"}
        label = finger_map.get(i, "")
        prefix = f"  ← {label}" if label else ""
        print(f"    [{i:2d}] {j:20s}{prefix}")

    print(f"\n[Allegro] default_joint_pos:")
    pos = allegro["default_joint_pos"]
    print(f"    Arm (7):  {pos[:7]}")
    print(f"    Hand(16): {pos[7:]}")

    print(f"\n[Allegro] Assisted Grasping 포인트:")
    print(f"    start: {allegro['ag_start_points']}")
    print(f"    end:   {allegro['ag_end_points']}")

    print(f"\n{'='*70}")
    print(f"  핵심 차이: Gripper는 열고/닫기(2 DOF), Allegro는 개별 관절 제어(16 DOF)")
    print(f"  R1Pro 적용 시: 양쪽 팔 각각 16 DOF → 총 32 DOF의 hand 제어 필요")
    print(f"{'='*70}")


def setup_viewport(distance=1.2):
    """Viewport를 복원하고 카메라를 Franka에 맞게 설정합니다."""
    from pxr import UsdGeom, Gf
    import carb
    import omni.usd
    import omni.ui
    import omni.kit.viewport.window
    import omni.kit.app

    vp_win = omni.ui.Workspace.get_window("Viewport")
    if vp_win is not None:
        vp_win.visible = True
        omni.kit.app.get_app().update()

    s = carb.settings.get_settings()
    s.set_bool("/rtx/reflections/enabled", True)
    s.set_bool("/rtx/indirectDiffuse/enabled", True)
    s.set_bool("/rtx/ambientOcclusion/enabled", True)
    s.set_bool("/rtx/directLighting/sampledLighting/enabled", True)
    s.set_bool("/rtx/shadows/enabled", True)
    s.set_bool("/rtx/flow/enabled", True)
    s.set_bool("/rtx/translucency/enabled", True)
    s.set_bool("/rtx/post/aa/enabled", True)
    s.set_bool("/rtx/post/tonemap/enabled", True)
    s.set_bool("/rtx/post/denoiser/enabled", True)
    s.set_bool("/app/renderer/skipMaterialLoading", False)

    s.set_string("/rtx/rendermode", "PathTracing")
    s.set_int("/rtx/pathtracing/spp", 1)
    s.set_int("/rtx/pathtracing/totalSpp", 64)
    s.set_int("/rtx/pathtracing/maxBounces", 4)
    s.set_int("/rtx/pathtracing/maxSpecularAndTransmissionBounces", 4)

    stage = omni.usd.get_context().get_stage()
    cam_path = "/World/viewport_camera"
    if not stage.GetPrimAtPath(cam_path).IsValid():
        UsdGeom.Camera.Define(stage, cam_path)
    prim = stage.GetPrimAtPath(cam_path)
    xf = UsdGeom.Xformable(prim)
    xf.ClearXformOpOrder()
    # Franka는 테이블 위 로봇이므로 더 가까이 + 약간 위에서 바라봄
    xf.AddTranslateOp().Set(Gf.Vec3d(distance, -distance, 1.0))
    xf.AddOrientOp().Set(Gf.Quatf(0.8446, 0.4618, 0.1160, 0.2421))
    UsdGeom.Camera(prim).GetClippingRangeAttr().Set(Gf.Vec2f(0.01, 10000.0))
    UsdGeom.Camera(prim).GetFocalLengthAttr().Set(24.0)

    vps = list(omni.kit.viewport.window.get_viewport_window_instances())
    if vps:
        vps[0].viewport_api.set_active_camera(cam_path)
    print("  뷰포트 카메라 설정 완료")


def print_robot_info(robot, eef_type):
    """로봇 정보를 출력합니다."""
    print(f"\n{'='*70}")
    print(f"  Franka + {eef_type} 스폰 완료!")
    print(f"{'='*70}")
    print(f"  모델 (model):               {robot.model}")
    print(f"  kinematic_tree_identifier:   {robot.kinematic_tree_identifier}")
    print(f"  end_effector:                {robot.end_effector}")
    print(f"  Action 차원:                 {robot.action_dim}")
    print(f"  전체 조인트 수:              {len(robot.joints)}")
    print(f"  컨트롤러 순서:               {robot.controller_order}")

    print(f"\n  [컨트롤러 상세]")
    for name in robot.controller_order:
        ctrl = robot.controllers[name]
        print(f"    {name:16s} -> {ctrl.__class__.__name__} (command_dim={ctrl.command_dim})")

    print(f"\n  [EEF 정보]")
    print(f"    eef_link_names:    {robot.eef_link_names}")
    print(f"    finger_link_names: {robot.finger_link_names}")
    print(f"    finger_joint_names:{robot.finger_joint_names}")

    print(f"\n  [조인트 목록]")
    for i, (jname, joint) in enumerate(robot.joints.items()):
        print(f"    [{i:2d}] {jname}")


def run_simulation(eef_type):
    """Franka 로봇을 지정된 end effector로 스폰하고 시뮬레이션합니다."""
    from omnigibson.macros import gm

    gm.USE_GPU_DYNAMICS = False
    gm.ENABLE_FLATCACHE = True
    gm.RENDER_VIEWER_CAMERA = False
    import omnigibson as og

    print(f"\n{'='*70}")
    print(f"  Franka + {eef_type} 시뮬레이션 시작")
    print(f"{'='*70}")

    cfg = {
        "scene": {"type": "Scene"},
        "robots": [
            {
                "model": "franka",
                "end_effector": eef_type,
                "obs_modalities": [],
                "action_type": "continuous",
                "action_normalize": True,
                "grasping_mode": "physical",
                "sensor_config": {
                    "VisionSensor": {
                        "sensor_kwargs": {
                            "image_height": 128,
                            "image_width": 128,
                        }
                    }
                },
            }
        ],
    }

    env = og.Environment(configs=cfg)
    robot = env.robots[0]

    setup_viewport()
    print_robot_info(robot, eef_type)

    # 리셋
    env.reset()

    joint_pos = robot.get_joint_positions()
    print(f"\n  리셋 후 조인트 위치: {[f'{v:.4f}' for v in joint_pos.tolist()]}")

    # 시뮬레이션 루프
    n_steps = 3000
    print(f"\n  {n_steps} 스텝 랜덤 액션 실행 중...")
    print(f"  Action space shape: {robot.action_space.shape}")
    print(f"  Action space low:   {robot.action_space.low}")
    print(f"  Action space high:  {robot.action_space.high}")

    for step in range(n_steps):
        if step % 30 == 0:
            # 작은 액션으로 부드럽게 움직임
            action = robot.action_space.sample() * 0.03
        env.step(action=action)

        if step % 500 == 0:
            joint_pos = robot.get_joint_positions()
            arm_pos = joint_pos[:7].tolist()
            hand_pos = joint_pos[7:].tolist()
            print(f"    Step {step:4d} | arm={[f'{v:.3f}' for v in arm_pos]}")
            print(f"    {'':9s} | hand={[f'{v:.3f}' for v in hand_pos]}")

    print(f"\n  시뮬레이션 완료! ({eef_type})")
    og.shutdown()


def run_compare():
    """gripper와 allegro를 같은 환경에 나란히 스폰하여 비교합니다."""
    from omnigibson.macros import gm

    gm.USE_GPU_DYNAMICS = False
    gm.ENABLE_FLATCACHE = True
    gm.RENDER_VIEWER_CAMERA = False
    import omnigibson as og

    print(f"\n{'='*70}")
    print("  [비교 모드] Gripper & Allegro 나란히 스폰")
    print(f"{'='*70}")

    # 두 로봇을 X축 기준 1m 간격으로 배치
    spacing = 1.0
    robot_common = {
        "model": "franka",
        "obs_modalities": [],
        "action_type": "continuous",
        "action_normalize": True,
        "grasping_mode": "physical",
        "sensor_config": {
            "VisionSensor": {
                "sensor_kwargs": {
                    "image_height": 128,
                    "image_width": 128,
                }
            }
        },
    }

    cfg = {
        "scene": {"type": "Scene"},
        "robots": [
            {
                **robot_common,
                "name": "franka_gripper",
                "end_effector": "gripper",
                "position": [-spacing / 2, 0.0, 0.0],
            },
            {
                **robot_common,
                "name": "franka_allegro",
                "end_effector": "allegro",
                "position": [spacing / 2, 0.0, 0.0],
            },
        ],
    }

    env = og.Environment(configs=cfg)
    r_grip = env.robots[0]  # franka_gripper
    r_alle = env.robots[1]  # franka_allegro

    # 카메라를 두 로봇 중앙에서 바라보도록 설정
    setup_viewport(distance=1.8)

    # 로봇 정보 비교 출력
    print(f"\n{'='*70}")
    print(f"  두 로봇 나란히 스폰 완료!")
    print(f"{'='*70}")
    print(f"  {'':30s} {'Gripper':20s} {'Allegro':20s}")
    print(f"  {'─'*70}")
    print(f"  {'name':30s} {r_grip.name:20s} {r_alle.name:20s}")
    print(f"  {'kinematic_tree_identifier':30s} {r_grip.kinematic_tree_identifier:20s} {r_alle.kinematic_tree_identifier:20s}")
    print(f"  {'end_effector':30s} {r_grip.end_effector:20s} {r_alle.end_effector:20s}")
    print(f"  {'action_dim':30s} {r_grip.action_dim:<20d} {r_alle.action_dim:<20d}")
    print(f"  {'전체 조인트 수':30s} {len(r_grip.joints):<20d} {len(r_alle.joints):<20d}")
    print(f"  {'eef_link':30s} {r_grip.eef_link_names['0']:20s} {r_alle.eef_link_names['0']:20s}")
    print(f"  {'finger 조인트 수':30s} {len(r_grip.finger_joint_names['0']):<20d} {len(r_alle.finger_joint_names['0']):<20d}")

    print(f"\n  [컨트롤러 비교]")
    for name in r_grip.controller_order:
        g_ctrl = r_grip.controllers[name]
        a_ctrl = r_alle.controllers[name]
        g_info = f"{g_ctrl.__class__.__name__}(dim={g_ctrl.command_dim})"
        a_info = f"{a_ctrl.__class__.__name__}(dim={a_ctrl.command_dim})"
        marker = " <-- 차이!" if g_ctrl.command_dim != a_ctrl.command_dim else ""
        print(f"    {name:12s} | {g_info:40s} | {a_info:40s}{marker}")

    # 리셋
    env.reset()

    print(f"\n  리셋 후 조인트 위치:")
    g_pos = r_grip.get_joint_positions()
    a_pos = r_alle.get_joint_positions()
    print(f"    Gripper arm:  {[f'{v:.3f}' for v in g_pos[:7].tolist()]}")
    print(f"    Gripper hand: {[f'{v:.3f}' for v in g_pos[7:].tolist()]}")
    print(f"    Allegro arm:  {[f'{v:.3f}' for v in a_pos[:7].tolist()]}")
    print(f"    Allegro hand: {[f'{v:.3f}' for v in a_pos[7:].tolist()]}")

    # 동일한 arm 액션을 양쪽에 주고, hand 액션은 각각 랜덤
    n_steps = 3000
    print(f"\n  {n_steps} 스텝 실행 중 (동일 arm 액션, 각각 hand 랜덤)...")

    import numpy as np

    arm_dof = 7
    for step in range(n_steps):
        if step % 30 == 0:
            # 동일한 arm 액션 생성
            shared_arm_action = r_grip.action_space.sample()[:arm_dof] * 0.03
            # 각 로봇의 hand 액션은 독립 랜덤
            grip_hand = r_grip.action_space.sample()[arm_dof:] * 0.03
            alle_hand = r_alle.action_space.sample()[arm_dof:] * 0.03
            grip_action = np.concatenate([shared_arm_action, grip_hand])
            alle_action = np.concatenate([shared_arm_action, alle_hand])

        action = {
            r_grip.name: grip_action,
            r_alle.name: alle_action,
        }
        env.step(action=action)

        if step % 500 == 0:
            g_pos = r_grip.get_joint_positions()
            a_pos = r_alle.get_joint_positions()
            print(f"    Step {step:4d}")
            print(f"      Gripper | arm={[f'{v:.3f}' for v in g_pos[:7].tolist()]} hand={[f'{v:.3f}' for v in g_pos[7:].tolist()]}")
            print(f"      Allegro | arm={[f'{v:.3f}' for v in a_pos[:7].tolist()]} hand={[f'{v:.2f}' for v in a_pos[7:].tolist()]}")

    print(f"\n{'='*70}")
    print("  비교 시뮬레이션 완료!")
    print(f"  - Gripper (왼쪽): arm 7 DOF + finger  2 DOF = action_dim {r_grip.action_dim}")
    print(f"  - Allegro (오른쪽): arm 7 DOF + finger 16 DOF = action_dim {r_alle.action_dim}")
    print(f"  R1Pro 적용 시: 양쪽 팔이므로 hand DOF가 2x2=4 -> 2x16=32 로 증가")
    print(f"{'='*70}")
    og.shutdown()


def main():
    parser = argparse.ArgumentParser(description="Franka + Allegro Hand 시뮬레이션")
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--analyze-only", action="store_true", help="YAML 정의 분석만 수행 (시뮬레이션 불필요)")
    group.add_argument("--run-gripper", action="store_true", help="Franka + 기본 gripper 시뮬레이션")
    group.add_argument("--run-allegro", action="store_true", help="Franka + Allegro hand 시뮬레이션")
    group.add_argument("--compare", action="store_true", help="gripper와 allegro를 나란히 스폰하여 비교")
    args = parser.parse_args()

    if args.analyze_only:
        analyze_end_effectors()
    elif args.run_gripper:
        run_simulation("gripper")
    elif args.run_allegro:
        run_simulation("allegro")
    elif args.compare:
        run_compare()


if __name__ == "__main__":
    main()
