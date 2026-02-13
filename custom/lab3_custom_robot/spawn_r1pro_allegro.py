"""
R1Pro + Allegro Hand 스폰 및 테스트
====================================
R1Pro 로봇의 gripper vs Allegro hand variant를 스폰하고 비교합니다.

사전 조건:
    1. build_r1pro_allegro_usd.py로 USD 에셋을 먼저 조립해야 합니다.
    2. r1pro_allegro.yaml이 definitions 디렉토리에 등록되어야 합니다.
       (이 스크립트가 자동으로 처리합니다)

실행:
    # 1단계: YAML 분석 (시뮬레이션 불필요)
    python custom/lab3_custom_robot/spawn_r1pro_allegro.py --analyze-only

    # 2단계: R1Pro + 기본 gripper 시뮬레이션
    python custom/lab3_custom_robot/spawn_r1pro_allegro.py --run-gripper

    # 3단계: R1Pro + Allegro hand 시뮬레이션
    python custom/lab3_custom_robot/spawn_r1pro_allegro.py --run-allegro

    # 4단계: 둘 다 나란히 비교
    python custom/lab3_custom_robot/spawn_r1pro_allegro.py --compare
"""

import argparse
import shutil
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT_ROOT / "OmniGibson"))
sys.path.insert(0, str(PROJECT_ROOT / "bddl3"))

DEFINITIONS_DIR = PROJECT_ROOT / "OmniGibson" / "omnigibson" / "robots" / "definitions"
YAML_SRC = Path(__file__).parent / "r1pro_allegro.yaml"


def register_yaml():
    """r1pro_allegro.yaml을 OmniGibson definitions 디렉토리에 복사합니다."""
    dst = DEFINITIONS_DIR / "r1pro_allegro.yaml"
    if not dst.exists() or dst.read_text() != YAML_SRC.read_text():
        shutil.copy2(str(YAML_SRC), str(dst))
        print(f"  YAML 등록: {YAML_SRC.name} → {dst}")
    else:
        print(f"  YAML 이미 등록됨: {dst}")


def analyze_end_effectors():
    """r1pro_allegro.yaml의 gripper vs allegro 정의를 비교 분석합니다."""
    from omegaconf import OmegaConf
    from omnigibson.robots.definition_schema import RobotDefinition

    register_yaml()

    schema = OmegaConf.structured(RobotDefinition)
    definition = OmegaConf.to_container(
        OmegaConf.merge(schema, OmegaConf.load(DEFINITIONS_DIR / "r1pro_allegro.yaml")),
        resolve=True,
    )

    manip = definition["manipulation"]
    eefs = manip["end_effectors"]
    supported = manip["supported_end_effector"]

    print("=" * 70)
    print("  R1Pro Allegro End Effector Variant 분석")
    print("=" * 70)
    print(f"\n  지원 end effector: {supported}")

    gripper = eefs["gripper"]
    allegro = eefs["allegro"]

    # --- 기본 비교 ---
    print(f"\n{'─'*70}")
    print(f"  {'항목':30s} {'Gripper':20s} {'Allegro':20s}")
    print(f"{'─'*70}")
    print(f"  {'model':30s} {gripper['model']:20s} {allegro['model']:20s}")
    print(f"  {'USD 경로':30s}")
    print(f"    Gripper: {gripper['usd_path']}")
    print(f"    Allegro: {allegro['usd_path']}")

    # EEF links
    print(f"\n  {'EEF 링크':30s}")
    for side in ["left", "right"]:
        g_eef = gripper["eef_link_names"][side]
        a_eef = allegro["eef_link_names"][side]
        print(f"    {side:10s} {g_eef:20s} {a_eef:20s}")

    # Finger links
    for side in ["left", "right"]:
        g_fingers = gripper["finger_link_names"][side]
        a_fingers = allegro["finger_link_names"][side]
        print(f"\n  [{side}] Finger 링크:")
        print(f"    Gripper ({len(g_fingers)}개): {g_fingers}")
        print(f"    Allegro ({len(a_fingers)}개): {a_fingers[:4]}...{a_fingers[-2:]}")

    # Finger joints
    for side in ["left", "right"]:
        g_joints = gripper["finger_joint_names"][side]
        a_joints = allegro["finger_joint_names"][side]
        print(f"\n  [{side}] Finger 조인트:")
        print(f"    Gripper ({len(g_joints)}개): {g_joints}")
        print(f"    Allegro ({len(a_joints)}개):")
        finger_map = {0: "ring", 4: "middle", 8: "index", 12: "thumb"}
        for i, j in enumerate(a_joints):
            label = finger_map.get(i, "")
            prefix = f"  <- {label}" if label else ""
            print(f"      [{i:2d}] {j}{prefix}")

    # DOF 비교
    g_dof = len(gripper["default_joint_pos"])
    a_dof = len(allegro["default_joint_pos"])
    print(f"\n{'─'*70}")
    print(f"  {'전체 DOF':30s} {g_dof:20d} {a_dof:20d}")
    print(f"  {'  - Base':30s} {6:20d} {6:20d}")
    print(f"  {'  - Trunk':30s} {4:20d} {4:20d}")
    print(f"  {'  - Left arm':30s} {7:20d} {7:20d}")
    print(f"  {'  - Right arm':30s} {7:20d} {7:20d}")
    print(f"  {'  - Left hand':30s} {2:20d} {16:20d}")
    print(f"  {'  - Right hand':30s} {2:20d} {16:20d}")
    print(f"  {'URDF 지원':30s} {'O':20s} {'X (not_support_urdf)':20s}")

    # Collision pairs
    print(f"\n  충돌 비활성화 쌍:")
    g_coll = gripper.get("disabled_collision_pairs", [])
    a_coll = allegro.get("disabled_collision_pairs", [])
    print(f"    Gripper: {len(g_coll)}쌍 - {g_coll}")
    print(f"    Allegro: {len(a_coll)}쌍 - {a_coll}")

    # AG points
    a_ag_start = allegro.get("ag_start_points", [])
    a_ag_end = allegro.get("ag_end_points", [])
    print(f"\n  [Allegro] Assisted Grasping 포인트:")
    print(f"    start ({len(a_ag_start)}개): {a_ag_start}")
    print(f"    end ({len(a_ag_end)}개): {a_ag_end}")

    print(f"\n{'='*70}")
    print(f"  핵심 차이:")
    print(f"    Gripper: 각 팔 2 DOF (열기/닫기) = 총 4 DOF")
    print(f"    Allegro: 각 팔 16 DOF (개별 관절 제어) = 총 32 DOF")
    print(f"    DOF 증가량: {a_dof - g_dof} (= 32 - 4)")
    print(f"{'='*70}")


def setup_viewport(distance=2.5):
    """Viewport를 설정합니다. R1Pro는 모바일 로봇이므로 더 멀리서 바라봅니다."""
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
    # R1Pro 전체가 보이도록 멀리서 + 위에서 바라봄
    xf.AddTranslateOp().Set(Gf.Vec3d(distance, -distance, 1.8))
    xf.AddOrientOp().Set(Gf.Quatf(0.8446, 0.4618, 0.1160, 0.2421))
    UsdGeom.Camera(prim).GetClippingRangeAttr().Set(Gf.Vec2f(0.01, 10000.0))
    UsdGeom.Camera(prim).GetFocalLengthAttr().Set(18.0)

    vps = list(omni.kit.viewport.window.get_viewport_window_instances())
    if vps:
        vps[0].viewport_api.set_active_camera(cam_path)
    print("  뷰포트 카메라 설정 완료")


def print_robot_info(robot, eef_type):
    """로봇 정보를 출력합니다."""
    print(f"\n{'='*70}")
    print(f"  R1Pro + {eef_type} 스폰 완료!")
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
        print(f"    {name:20s} -> {ctrl.__class__.__name__} (command_dim={ctrl.command_dim})")

    print(f"\n  [EEF 정보]")
    print(f"    eef_link_names:    {robot.eef_link_names}")
    print(f"    finger_link_names: (left={len(robot.finger_link_names.get('left', []))}, right={len(robot.finger_link_names.get('right', []))})")
    for side in ["left", "right"]:
        names = robot.finger_link_names.get(side, [])
        if len(names) <= 4:
            print(f"      {side}: {names}")
        else:
            print(f"      {side}: [{names[0]}, ..., {names[-1]}] ({len(names)}개)")

    print(f"    finger_joint_names: (left={len(robot.finger_joint_names.get('left', []))}, right={len(robot.finger_joint_names.get('right', []))})")
    for side in ["left", "right"]:
        names = robot.finger_joint_names.get(side, [])
        if len(names) <= 4:
            print(f"      {side}: {names}")
        else:
            print(f"      {side}: [{names[0]}, ..., {names[-1]}] ({len(names)}개)")

    print(f"\n  [조인트 목록]")
    for i, (jname, joint) in enumerate(robot.joints.items()):
        marker = ""
        if "allegro" in jname or "palm" in jname:
            marker = "  <-- ALLEGRO"
        elif "gripper" in jname:
            marker = "  <-- GRIPPER"
        elif "finger" in jname:
            marker = "  <-- FINGER"
        print(f"    [{i:2d}] {jname}{marker}")


def run_simulation(eef_type):
    """R1Pro 로봇을 지정된 end effector로 스폰하고 시뮬레이션합니다."""
    from omnigibson.macros import gm

    gm.USE_GPU_DYNAMICS = False
    gm.ENABLE_FLATCACHE = True
    gm.RENDER_VIEWER_CAMERA = False
    import omnigibson as og

    register_yaml()

    print(f"\n{'='*70}")
    print(f"  R1Pro + {eef_type} 시뮬레이션 시작")
    print(f"{'='*70}")

    cfg = {
        "scene": {"type": "Scene"},
        "robots": [
            {
                "model": "r1pro_allegro",
                "end_effector": eef_type,
                "obs_modalities": [],
                "action_type": "continuous",
                "action_normalize": True,
                "grasping_mode": "physical" if eef_type == "allegro" else "assisted",
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
    print(f"\n  리셋 후 조인트 위치 ({len(joint_pos)}개):")
    # Base + trunk
    base_trunk = joint_pos[:10].tolist()
    print(f"    Base+Trunk (10): {[f'{v:.4f}' for v in base_trunk]}")
    # Arms
    left_arm = joint_pos[10:17].tolist()
    right_arm = joint_pos[17:24].tolist()
    print(f"    Left arm (7):    {[f'{v:.4f}' for v in left_arm]}")
    print(f"    Right arm (7):   {[f'{v:.4f}' for v in right_arm]}")
    # Hands
    remaining = joint_pos[24:].tolist()
    print(f"    Hands ({len(remaining)}):      {[f'{v:.3f}' for v in remaining]}")

    # 시뮬레이션 루프
    n_steps = 3000
    print(f"\n  {n_steps} 스텝 랜덤 액션 실행 중...")
    print(f"  Action space shape: {robot.action_space.shape}")

    for step in range(n_steps):
        if step % 30 == 0:
            action = robot.action_space.sample() * 0.03
        env.step(action=action)

        if step % 500 == 0:
            joint_pos = robot.get_joint_positions()
            left_arm = joint_pos[10:17].tolist()
            right_arm = joint_pos[17:24].tolist()
            hands = joint_pos[24:].tolist()
            print(f"    Step {step:4d} | L_arm={[f'{v:.3f}' for v in left_arm]}")
            print(f"    {'':9s} | R_arm={[f'{v:.3f}' for v in right_arm]}")
            if len(hands) <= 8:
                print(f"    {'':9s} | hands={[f'{v:.3f}' for v in hands]}")
            else:
                left_hand = hands[:len(hands)//2]
                right_hand = hands[len(hands)//2:]
                print(f"    {'':9s} | L_hand({len(left_hand)})={[f'{v:.2f}' for v in left_hand]}")
                print(f"    {'':9s} | R_hand({len(right_hand)})={[f'{v:.2f}' for v in right_hand]}")

    print(f"\n  시뮬레이션 완료! ({eef_type})")
    og.shutdown()


def run_compare():
    """gripper와 allegro를 같은 환경에 나란히 스폰하여 비교합니다."""
    from omnigibson.macros import gm

    gm.USE_GPU_DYNAMICS = False
    gm.ENABLE_FLATCACHE = True
    gm.RENDER_VIEWER_CAMERA = False
    import omnigibson as og

    register_yaml()

    print(f"\n{'='*70}")
    print("  [비교 모드] R1Pro Gripper & Allegro 나란히 스폰")
    print(f"{'='*70}")

    spacing = 2.0
    robot_common = {
        "model": "r1pro_allegro",
        "obs_modalities": [],
        "action_type": "continuous",
        "action_normalize": True,
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
                "name": "r1pro_gripper",
                "end_effector": "gripper",
                "grasping_mode": "assisted",
                "position": [-spacing / 2, 0.0, 0.0],
            },
            {
                **robot_common,
                "name": "r1pro_allegro",
                "end_effector": "allegro",
                "grasping_mode": "physical",
                "position": [spacing / 2, 0.0, 0.0],
            },
        ],
    }

    env = og.Environment(configs=cfg)
    r_grip = env.robots[0]
    r_alle = env.robots[1]

    setup_viewport(distance=3.5)

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

    # EEF link names
    for side in ["left", "right"]:
        g_eef = r_grip.eef_link_names.get(side, "N/A")
        a_eef = r_alle.eef_link_names.get(side, "N/A")
        print(f"  {f'eef_link ({side})':30s} {g_eef:20s} {a_eef:20s}")

    # Finger counts
    for side in ["left", "right"]:
        g_cnt = len(r_grip.finger_joint_names.get(side, []))
        a_cnt = len(r_alle.finger_joint_names.get(side, []))
        print(f"  {f'finger joints ({side})':30s} {g_cnt:<20d} {a_cnt:<20d}")

    print(f"\n  [컨트롤러 비교]")
    for name in r_grip.controller_order:
        g_ctrl = r_grip.controllers[name]
        a_ctrl = r_alle.controllers[name]
        g_info = f"{g_ctrl.__class__.__name__}(dim={g_ctrl.command_dim})"
        a_info = f"{a_ctrl.__class__.__name__}(dim={a_ctrl.command_dim})"
        marker = " <-- 차이!" if g_ctrl.command_dim != a_ctrl.command_dim else ""
        print(f"    {name:20s} | {g_info:40s} | {a_info:40s}{marker}")

    # 리셋
    env.reset()

    print(f"\n  리셋 후 조인트:")
    g_pos = r_grip.get_joint_positions()
    a_pos = r_alle.get_joint_positions()
    print(f"    Gripper: {len(g_pos)} DOF | arm=[{', '.join(f'{v:.3f}' for v in g_pos[10:24].tolist())}]")
    print(f"    Allegro: {len(a_pos)} DOF | arm=[{', '.join(f'{v:.3f}' for v in a_pos[10:24].tolist())}]")
    print(f"    Gripper hands: {[f'{v:.3f}' for v in g_pos[24:].tolist()]}")
    a_hands = a_pos[24:].tolist()
    if len(a_hands) > 8:
        print(f"    Allegro L_hand({len(a_hands)//2}): {[f'{v:.2f}' for v in a_hands[:len(a_hands)//2]]}")
        print(f"    Allegro R_hand({len(a_hands)//2}): {[f'{v:.2f}' for v in a_hands[len(a_hands)//2:]]}")
    else:
        print(f"    Allegro hands: {[f'{v:.3f}' for v in a_hands]}")

    # 동일한 arm 액션으로 비교
    n_steps = 3000
    print(f"\n  {n_steps} 스텝 실행 중 (동일 base+arm 액션, 각각 hand 랜덤)...")

    import numpy as np

    # base(3) + trunk(4) + arm_left(7) + arm_right(7) = 21 공유 DOF (action space 기준)
    # gripper_left + gripper_right = 나머지
    shared_dof = 21  # base(3 controllable) + trunk(4) + left_arm(7) + right_arm(7)

    for step in range(n_steps):
        if step % 30 == 0:
            shared_action = r_grip.action_space.sample()[:shared_dof] * 0.03

            grip_hand = r_grip.action_space.sample()[shared_dof:] * 0.03
            alle_hand = r_alle.action_space.sample()[shared_dof:] * 0.03

            grip_action = np.concatenate([shared_action, grip_hand])
            alle_action = np.concatenate([shared_action, alle_hand])

        action = {
            r_grip.name: grip_action,
            r_alle.name: alle_action,
        }
        env.step(action=action)

        if step % 500 == 0:
            g_pos = r_grip.get_joint_positions()
            a_pos = r_alle.get_joint_positions()
            g_arm = g_pos[10:24].tolist()
            a_arm = a_pos[10:24].tolist()
            g_hand = g_pos[24:].tolist()
            a_hand = a_pos[24:].tolist()
            print(f"    Step {step:4d}")
            print(f"      Gripper | arm={[f'{v:.3f}' for v in g_arm]} hand={[f'{v:.3f}' for v in g_hand]}")
            if len(a_hand) > 8:
                print(f"      Allegro | arm={[f'{v:.3f}' for v in a_arm]}")
                print(f"              | L_hand={[f'{v:.2f}' for v in a_hand[:len(a_hand)//2]]}")
                print(f"              | R_hand={[f'{v:.2f}' for v in a_hand[len(a_hand)//2:]]}")
            else:
                print(f"      Allegro | arm={[f'{v:.3f}' for v in a_arm]} hand={[f'{v:.3f}' for v in a_hand]}")

    print(f"\n{'='*70}")
    print("  비교 시뮬레이션 완료!")
    print(f"  - Gripper (왼쪽): action_dim={r_grip.action_dim} (base+trunk+arm 21 + gripper 4 = 25)")
    print(f"  - Allegro (오른쪽): action_dim={r_alle.action_dim} (base+trunk+arm 21 + allegro 32 = 53)")
    print(f"  DOF 증가: {r_alle.action_dim - r_grip.action_dim} (= 32 - 4)")
    print(f"{'='*70}")
    og.shutdown()


def main():
    parser = argparse.ArgumentParser(description="R1Pro + Allegro Hand 스폰 및 테스트")
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--analyze-only", action="store_true", help="YAML 정의 분석만 수행 (시뮬레이션 불필요)")
    group.add_argument("--run-gripper", action="store_true", help="R1Pro + 기본 gripper 시뮬레이션")
    group.add_argument("--run-allegro", action="store_true", help="R1Pro + Allegro hand 시뮬레이션")
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
