#!/usr/bin/env python3
"""
R1Pro + Allegro Hand 통합 로봇 관절 테스트
==========================================
build_r1pro_allegro_usd.py로 생성한 통합 USD를 로드하여
각 관절을 카테고리별로 순차 테스트합니다.

카테고리: trunk → left_arm → right_arm → left_hand → right_hand

사용법:
    # 전체 관절 테스트
    python custom/lab3_custom_robot/test_r1pro_allegro_joints.py

    # 특정 카테고리만
    python custom/lab3_custom_robot/test_r1pro_allegro_joints.py --category hand_left
    python custom/lab3_custom_robot/test_r1pro_allegro_joints.py --category arm_right

    # 파라미터 조정
    python custom/lab3_custom_robot/test_r1pro_allegro_joints.py --steps-per-joint 120 --stiffness 500 --damping 50
"""

import argparse
import math
import os
from pathlib import Path

import torch as th

from omnigibson.macros import gm

gm.USE_GPU_DYNAMICS = False
gm.ENABLE_FLATCACHE = False
gm.RENDER_VIEWER_CAMERA = False

import omnigibson as og

PROJECT_ROOT = Path(__file__).resolve().parents[2]
DATASETS_DIR = PROJECT_ROOT / "datasets" / "omnigibson-robot-assets"

BASE_USD = DATASETS_DIR / "models" / "r1pro" / "r1pro_allegro" / "usd" / "configuration" / "r1pro_allegro_base.usd"
PHYSICS_USD = DATASETS_DIR / "models" / "r1pro" / "r1pro_allegro" / "usd" / "configuration" / "r1pro_allegro_physics.usd"

# Joint category classification by name prefix
CATEGORIES = ["trunk", "arm_left", "arm_right", "hand_left", "hand_right"]

SKIP_JOINTS = {
    "base_footprint_x_joint", "base_footprint_y_joint", "base_footprint_rz_joint",
}


def classify_joint(name):
    """Classify a joint name into a category."""
    if name in SKIP_JOINTS or "wheel" in name:
        return None
    if "torso" in name:
        return "trunk"
    if "left_arm" in name:
        return "arm_left"
    if "right_arm" in name:
        return "arm_right"
    # Allegro hand joints: left_joint_0_0 etc.
    if name.startswith("left_") and ("joint_" in name or "allegro" in name):
        return "hand_left"
    if name.startswith("right_") and ("joint_" in name or "allegro" in name):
        return "hand_right"
    return None


def setup_viewport(position=(1.5, -2.5, 1.5), orientation_wxyz=(0.82, 0.35, 0.1, 0.44)):
    """Viewport restore + camera/render setup (lab1~4 common pattern)."""
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
    s.set_int("/rtx/raytracing/showLights", 1)
    s.set_float("/rtx/sceneDb/ambientLightIntensity", 1.0)
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
    xf.AddTranslateOp().Set(Gf.Vec3d(*position))
    xf.AddOrientOp().Set(Gf.Quatf(*orientation_wxyz))
    UsdGeom.Camera(prim).GetClippingRangeAttr().Set(Gf.Vec2f(0.01, 10000.0))
    UsdGeom.Camera(prim).GetFocalLengthAttr().Set(18.0)

    vps = list(omni.kit.viewport.window.get_viewport_window_instances())
    if vps:
        vps[0].viewport_api.set_active_camera(cam_path)
    print("  viewport camera/render setup done")


def main():
    parser = argparse.ArgumentParser(
        description="R1Pro + Allegro 통합 로봇 관절 테스트",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
카테고리:
  trunk       : torso 관절 (4 DOF)
  arm_left    : 왼쪽 팔 관절 (7 DOF)
  arm_right   : 오른쪽 팔 관절 (7 DOF)
  hand_left   : 왼쪽 Allegro hand (16 DOF)
  hand_right  : 오른쪽 Allegro hand (16 DOF)
  all         : 전체 (기본값)

예시:
  python custom/lab3_custom_robot/test_r1pro_allegro_joints.py --category hand_left
  python custom/lab3_custom_robot/test_r1pro_allegro_joints.py --stiffness 500 --damping 50
""")
    parser.add_argument("--category", default="all",
                        choices=CATEGORIES + ["all"],
                        help="테스트할 관절 카테고리 (기본: all)")
    parser.add_argument("--steps-per-joint", type=int, default=60,
                        help="관절당 스텝 수 (편도, 기본: 60)")
    parser.add_argument("--stiffness", type=float, default=500,
                        help="드라이브 강성 (P gain, 기본: 500)")
    parser.add_argument("--damping", type=float, default=50,
                        help="드라이브 감쇠 (D gain, 기본: 50)")
    parser.add_argument("--max-force", type=float, default=1e3,
                        help="최대 토크 (기본: 1000)")
    parser.add_argument("--drive-type", choices=["force", "acceleration"], default="acceleration",
                        help="드라이브 모드 (기본: acceleration)")
    args = parser.parse_args()

    # --- Check USD files ---
    if not BASE_USD.exists():
        print(f"ERROR: USD not found: {BASE_USD}")
        print("Run build_r1pro_allegro_usd.py first.")
        return

    print(f"Loading: {BASE_USD}")

    # --- OG Environment (empty scene + lights) ---
    cfg = {
        "scene": {"type": "Scene"},
        "objects": [
            dict(type="LightObject", light_type="Sphere", name="light0",
                 radius=0.01, intensity=1e5, position=[-1.0, -1.0, 2.0]),
            dict(type="LightObject", light_type="Sphere", name="light1",
                 radius=0.01, intensity=1e5, position=[1.0, 1.0, 2.0]),
        ],
    }
    env = og.Environment(configs=cfg)

    # --- Load USD onto stage ---
    from pxr import UsdGeom, Gf, UsdPhysics

    stage = og.sim.stage
    prim_path = "/World/r1pro_allegro"
    prim = stage.DefinePrim(prim_path, "Xform")
    prim.GetReferences().AddReference(str(BASE_USD))
    if PHYSICS_USD.exists():
        prim.GetReferences().AddReference(str(PHYSICS_USD))
        print(f"Physics layer loaded: {PHYSICS_USD}")

    # z offset so robot stands on ground
    xformable = UsdGeom.Xformable(prim)
    xformable.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 0.3))

    # --- Fix base_link (kinematic) so robot doesn't fall ---
    base_link_path = f"{prim_path}/base_link"
    base_link = stage.GetPrimAtPath(base_link_path)
    if base_link and base_link.IsValid():
        if not base_link.HasAPI(UsdPhysics.ArticulationRootAPI):
            UsdPhysics.ArticulationRootAPI.Apply(base_link)
        if not base_link.HasAPI(UsdPhysics.RigidBodyAPI):
            UsdPhysics.RigidBodyAPI.Apply(base_link)
        rb = UsdPhysics.RigidBodyAPI(base_link)
        rb.CreateKinematicEnabledAttr().Set(True)
        print("  base_link fixed (kinematic)")
    else:
        print(f"WARNING: base_link not found: {base_link_path}")

    # --- Find joints + add drives ---
    joint_scope_path = f"{prim_path}/joints"
    joint_scope = stage.GetPrimAtPath(joint_scope_path)

    all_joints = {}  # category -> list of joint dicts
    for cat in CATEGORIES:
        all_joints[cat] = []

    if joint_scope and joint_scope.IsValid():
        for child in joint_scope.GetChildren():
            type_name = child.GetTypeName()
            if "Fixed" in type_name:
                continue

            name = child.GetName()
            category = classify_joint(name)
            if category is None:
                continue

            lower = child.GetAttribute("physics:lowerLimit").Get()
            upper = child.GetAttribute("physics:upperLimit").Get()
            if lower is None or upper is None:
                continue

            # Higher gains for trunk joints (heavy upper body)
            if category == "trunk":
                stiff = args.stiffness * 100
                damp = args.damping * 20
                max_f = args.max_force * 10
            else:
                stiff = args.stiffness
                damp = args.damping
                max_f = args.max_force

            # Add position drive (PD controller)
            drive = UsdPhysics.DriveAPI.Apply(child, "angular")
            drive.CreateTypeAttr().Set(args.drive_type)
            drive.CreateMaxForceAttr().Set(max_f)
            drive.CreateStiffnessAttr().Set(stiff)
            drive.CreateDampingAttr().Set(damp)
            drive.CreateTargetPositionAttr().Set(0.0)

            all_joints[category].append({
                "name": name,
                "prim_path": str(child.GetPath()),
                "lower": lower,
                "upper": upper,
            })

    # --- Summary ---
    total = sum(len(v) for v in all_joints.values())
    print(f"\n{'='*70}")
    print(f"  R1Pro + Allegro  total {total} DOF")
    print(f"  drive: {args.drive_type}  stiffness: {args.stiffness}  damping: {args.damping}")
    print(f"{'='*70}")
    for cat in CATEGORIES:
        joints = all_joints[cat]
        if not joints:
            continue
        print(f"\n  [{cat}] {len(joints)} joints")
        print(f"  {'#':>3s}  {'joint name':30s}  {'lower':>8s}  {'upper':>8s}  {'range':>8s}")
        print(f"  {'---':>3s}  {'------------------------------':30s}  {'--------':>8s}  {'--------':>8s}  {'--------':>8s}")
        for i, j in enumerate(joints):
            r = j["upper"] - j["lower"]
            print(f"  {i:3d}  {j['name']:30s}  {j['lower']:8.1f}  {j['upper']:8.1f}  {r:8.1f}")

    # Filter by category
    if args.category == "all":
        test_categories = CATEGORIES
    else:
        test_categories = [args.category]

    test_joints = []
    for cat in test_categories:
        test_joints.extend(all_joints[cat])

    if not test_joints:
        print("\nERROR: No movable joints found for the selected category.")
        og.shutdown()
        return

    print(f"\n  Testing {len(test_joints)} joints (categories: {', '.join(test_categories)})")

    # --- Render + viewport setup ---
    og.sim.render()
    setup_viewport(
        position=(1.5, -2.5, 1.5),
        orientation_wxyz=(0.82, 0.35, 0.1, 0.44),
    )

    # --- Collect ALL joints (not just test targets) for default hold ---
    all_joint_list = []
    for cat in CATEGORIES:
        all_joint_list.extend(all_joints[cat])

    # Default position = 0.0 (clamp to joint range if 0 is outside)
    DEFAULT_POS = 0.0
    for j in all_joint_list:
        j["default"] = max(j["lower"], min(DEFAULT_POS, j["upper"]))

    def get_drive(joint):
        jp = stage.GetPrimAtPath(joint["prim_path"])
        return UsdPhysics.DriveAPI.Get(jp, "angular")

    def hold_all_default():
        """Set ALL joints to their default position."""
        for j in all_joint_list:
            get_drive(j).GetTargetPositionAttr().Set(j["default"])

    # --- Start simulation ---
    # Set all joints to default BEFORE play
    hold_all_default()
    og.sim.play()

    # Settle into default pose
    print("  Settling into default pose...")
    for _ in range(60):
        og.sim.step()

    steps_per_joint = args.steps_per_joint
    print(f"\n  {steps_per_joint} steps per joint (one-way), round-trip sweep")
    print(f"  Ctrl+C to quit\n")

    try:
        # --- Sequential joint test ---
        for j_idx, joint in enumerate(test_joints):
            name = joint["name"]
            lower = joint["lower"]
            upper = joint["upper"]
            default = joint["default"]

            print(f"  [{j_idx + 1:2d}/{len(test_joints)}] {name:30s}  ", end="", flush=True)

            # lower -> upper (only this joint moves, others hold default)
            for step in range(steps_per_joint):
                frac = step / max(steps_per_joint - 1, 1)
                target = lower + frac * (upper - lower)
                get_drive(joint).GetTargetPositionAttr().Set(target)
                og.sim.step()

            print(f"{lower:.1f} -> {upper:.1f}  ", end="", flush=True)

            # upper -> lower
            for step in range(steps_per_joint):
                frac = step / max(steps_per_joint - 1, 1)
                target = upper - frac * (upper - lower)
                get_drive(joint).GetTargetPositionAttr().Set(target)
                og.sim.step()

            # Reset this joint back to default
            get_drive(joint).GetTargetPositionAttr().Set(default)
            for _ in range(10):
                og.sim.step()

            print(f"-> default {default:.1f}  OK")

        # --- Wave animation (all tested joints simultaneously) ---
        print(f"\n  Wave animation ({len(test_joints)} joints)...")
        for cycle in range(3):
            for step in range(steps_per_joint * 2):
                frac = step / max(steps_per_joint * 2 - 1, 1)
                for j_idx, joint in enumerate(test_joints):
                    lower = joint["lower"]
                    upper = joint["upper"]
                    center = (lower + upper) / 2.0
                    half_range = (upper - lower) / 2.0
                    phase = j_idx / len(test_joints) * 2 * math.pi
                    val = center + half_range * math.sin(frac * 2 * math.pi + phase)
                    get_drive(joint).GetTargetPositionAttr().Set(val)
                og.sim.step()

        # --- Reset ALL joints to default ---
        hold_all_default()
        for _ in range(30):
            og.sim.step()

        print("\n  All joint tests done!")
        print("  Move camera with mouse/keyboard. Ctrl+C to quit.\n")

        while True:
            og.sim.render()

    except KeyboardInterrupt:
        print("\n  Shutting down...")

    og.shutdown()


if __name__ == "__main__":
    main()
