"""
Lab 3: 커스텀 로봇 등록 및 스폰
================================
r1pro_custom.yaml을 OmniGibson 로봇 정의 디렉토리에 등록하고,
시뮬레이션에서 스폰하여 원본 R1Pro와 비교합니다.

실행:
    # 1단계: 분석만 (시뮬레이션 불필요)
    python custom/lab3_custom_robot/spawn_custom_robot.py --analyze-only

    # 2단계: 커스텀 로봇 등록 + 시뮬레이션
    python custom/lab3_custom_robot/spawn_custom_robot.py --register-and-run

    # 등록 해제 (원본 복원)
    python custom/lab3_custom_robot/spawn_custom_robot.py --unregister

학습 목표:
    - 로봇 YAML 정의 파일의 등록 메커니즘 이해
    - 커스텀 YAML 작성 시 주의사항
    - 원본과 커스텀 로봇의 차이 확인
"""

import argparse
import shutil
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT_ROOT / "OmniGibson"))
sys.path.insert(0, str(PROJECT_ROOT / "bddl3"))

CUSTOM_YAML = Path(__file__).parent / "r1pro_custom.yaml"
DEFINITIONS_DIR = PROJECT_ROOT / "OmniGibson" / "omnigibson" / "robots" / "definitions"
TARGET_YAML = DEFINITIONS_DIR / "r1pro_custom.yaml"


def analyze_diff():
    """원본 r1pro.yaml과 커스텀 파일의 차이를 분석합니다."""
    from omegaconf import OmegaConf
    from omnigibson.robots.definition_schema import RobotDefinition

    schema = OmegaConf.structured(RobotDefinition)

    original = OmegaConf.to_container(
        OmegaConf.merge(schema, OmegaConf.load(DEFINITIONS_DIR / "r1pro.yaml")),
        resolve=True,
    )
    custom = OmegaConf.to_container(
        OmegaConf.merge(schema, OmegaConf.load(CUSTOM_YAML)),
        resolve=True,
    )

    print("=" * 70)
    print("  원본 R1Pro vs 커스텀 R1Pro 차이 분석")
    print("=" * 70)

    # 기본 컨트롤러 비교
    print("\n[1] 기본 컨트롤러 변경:")
    for key in original["default_controllers"]:
        orig_val = original["default_controllers"][key]
        cust_val = custom["default_controllers"][key]
        marker = " <-- 변경" if orig_val != cust_val else ""
        print(f"    {key:16s}: {orig_val:38s} -> {cust_val:38s}{marker}")

    # 속도 게인 비교
    print("\n[2] Primitives 속도 게인:")
    for field in ["linear_velocity_gain_for_primitives", "angular_velocity_gain_for_primitives"]:
        orig_val = original.get(field)
        cust_val = custom.get(field)
        marker = " <-- 변경" if orig_val != cust_val else ""
        print(f"    {field}: {orig_val} -> {cust_val}{marker}")

    # 작업 공간 비교
    print("\n[3] 팔 작업 공간 범위:")
    orig_ws = original.get("manipulation", {}).get("arm_workspace_range", {})
    cust_ws = custom.get("manipulation", {}).get("arm_workspace_range", {})
    for arm in ["left", "right"]:
        orig_range = orig_ws.get(arm, [])
        cust_range = cust_ws.get(arm, [])
        marker = " <-- 변경" if orig_range != cust_range else ""
        print(f"    {arm}: {orig_range} -> {cust_range}{marker}")

    # Untucked 포즈 비교
    print("\n[4] Untucked 포즈 변경된 조인트:")
    orig_untuck = original.get("mobile_manipulation", {}).get("untucked_default_joint_pos", [])
    cust_untuck = custom.get("mobile_manipulation", {}).get("untucked_default_joint_pos", [])
    for i in range(min(len(orig_untuck), len(cust_untuck))):
        if abs(orig_untuck[i] - cust_untuck[i]) > 0.01:
            print(f"    조인트[{i:2d}]: {orig_untuck[i]:+8.4f} -> {cust_untuck[i]:+8.4f}")


def register_robot():
    """커스텀 YAML을 OmniGibson definitions 디렉토리에 복사합니다."""
    if TARGET_YAML.exists():
        print(f"  이미 등록됨: {TARGET_YAML}")
        return True

    print(f"  복사: {CUSTOM_YAML}")
    print(f"    -> {TARGET_YAML}")
    shutil.copy2(CUSTOM_YAML, TARGET_YAML)
    print("  등록 완료!")
    return True


def unregister_robot():
    """커스텀 YAML을 definitions 디렉토리에서 제거합니다."""
    if not TARGET_YAML.exists():
        print("  커스텀 로봇이 등록되어 있지 않습니다.")
        return

    TARGET_YAML.unlink()
    print(f"  제거 완료: {TARGET_YAML}")


def setup_viewport():
    """RENDER_VIEWER_CAMERA=False로 숨겨진 Viewport를 복원하고 카메라를 설정합니다."""
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

    # RTX-Interactive (Path Tracing) 모드
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
    xf.AddTranslateOp().Set(Gf.Vec3d(-0.2, -2.7, 1.1))
    xf.AddOrientOp().Set(Gf.Quatf(0.73138017, 0.68196617, -0.00155408, -0.00166678))
    UsdGeom.Camera(prim).GetClippingRangeAttr().Set(Gf.Vec2f(0.01, 10000.0))
    UsdGeom.Camera(prim).GetFocalLengthAttr().Set(17.0)

    vps = list(omni.kit.viewport.window.get_viewport_window_instances())
    if vps:
        vps[0].viewport_api.set_active_camera(cam_path)
    print("  뷰포트 카메라 설정 완료")


def run_simulation():
    """커스텀 로봇을 시뮬레이션에서 스폰합니다."""
    import torch as th
    from omnigibson.macros import gm
    gm.USE_GPU_DYNAMICS = False
    gm.ENABLE_FLATCACHE = True
    gm.RENDER_VIEWER_CAMERA = False
    import omnigibson as og
    from omnigibson.robots import REGISTERED_ROBOTS

    # 등록 확인
    print(f"\n  등록된 로봇: {REGISTERED_ROBOTS}")
    if "r1pro_custom" not in REGISTERED_ROBOTS:
        print("  ERROR: r1pro_custom이 등록되지 않았습니다. --register-and-run을 사용하세요.")
        return

    # 환경 설정
    cfg = {
        "scene": {"type": "Scene"},
        "robots": [
            {
                "model": "r1pro_custom",
                "obs_modalities": [],
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
        ],
    }

    env = og.Environment(configs=cfg)
    robot = env.robots[0]

    # Viewport 복원 + 카메라 설정
    setup_viewport()

    # 로봇 정보 출력
    print(f"\n{'='*70}")
    print(f"  커스텀 로봇 스폰 완료!")
    print(f"{'='*70}")
    print(f"  모델:           {robot.model}")
    print(f"  Action 차원:    {robot.action_dim}")
    print(f"  컨트롤러 순서:  {robot.controller_order}")
    print(f"  팔 DOF (L):     {len(robot.arm_joint_names['left'])}")
    print(f"  팔 DOF (R):     {len(robot.arm_joint_names['right'])}")

    for name in robot.controller_order:
        ctrl = robot.controllers[name]
        print(f"    {name:16s} -> {ctrl.__class__.__name__} (dim={ctrl.command_dim})")

    # 리셋 및 간단한 시뮬레이션
    env.reset()
    robot.reset()

    print("\n  Untucked 포즈로 리셋 완료")
    joint_pos = robot.get_joint_positions()
    print(f"  현재 조인트 위치 (처음 12개): {joint_pos[:12].tolist()}")

    # 3000스텝 실행
    print("\n  3000 스텝 랜덤 액션 실행 중...")
    for step in range(3000):
        if step % 30 == 0:
            action = robot.action_space.sample() * 0.03
        env.step(action=action)

        if step % 500 == 0:
            joint_pos = robot.get_joint_positions()
            print(f"    Step {step:4d} | joints[0:6]={[f'{v:.3f}' for v in joint_pos[:6].tolist()]}")

    print("  시뮬레이션 완료!")
    og.shutdown()


def main():
    parser = argparse.ArgumentParser(description="Lab 3: 커스텀 로봇 등록 및 스폰")
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--analyze-only", action="store_true", help="차이 분석만 수행")
    group.add_argument("--register-and-run", action="store_true", help="등록 후 시뮬레이션 실행")
    group.add_argument("--unregister", action="store_true", help="커스텀 로봇 등록 해제")
    args = parser.parse_args()

    if args.analyze_only:
        analyze_diff()
    elif args.register_and_run:
        analyze_diff()
        print(f"\n{'='*70}")
        print("  커스텀 로봇 등록")
        print(f"{'='*70}")
        register_robot()
        run_simulation()
    elif args.unregister:
        unregister_robot()


if __name__ == "__main__":
    main()
