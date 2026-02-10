"""
Lab 1: R1Pro 로봇 구조 분석
===========================
R1Pro의 YAML 정의를 파싱하고, 실제 로봇 객체의 내부 구조를 출력합니다.
시뮬레이션 없이(headless) 로봇 정의만 분석하는 스크립트입니다.

실행:
    python custom/lab1_anatomy/inspect_r1pro.py

학습 목표:
    - R1Pro YAML 정의가 어떻게 Robot 클래스에 로딩되는지 이해
    - 컨트롤러 구조 (base, trunk, arm, gripper) 파악
    - 조인트/링크 이름 매핑 확인
    - R1 vs R1Pro 차이점 비교
"""

import os
import sys
from pathlib import Path
from omegaconf import OmegaConf

# 프로젝트 루트를 기준으로 import 경로 설정
PROJECT_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT_ROOT / "OmniGibson"))
sys.path.insert(0, str(PROJECT_ROOT / "bddl3"))

from omnigibson.robots.definition_schema import RobotDefinition


def load_robot_definition(model_name: str) -> dict:
    """로봇 YAML 정의를 로드하고 스키마와 병합합니다."""
    definition_dir = PROJECT_ROOT / "OmniGibson" / "omnigibson" / "robots" / "definitions"
    yaml_path = definition_dir / f"{model_name}.yaml"

    if not yaml_path.exists():
        raise FileNotFoundError(f"로봇 정의 파일을 찾을 수 없습니다: {yaml_path}")

    yaml_def = OmegaConf.load(yaml_path)
    schema = OmegaConf.structured(RobotDefinition)
    merged = OmegaConf.merge(schema, yaml_def)
    return OmegaConf.to_container(merged, resolve=True)


def print_section(title: str):
    print(f"\n{'='*60}")
    print(f"  {title}")
    print(f"{'='*60}")


def inspect_robot(model_name: str, definition: dict):
    """로봇 정의의 모든 구성 요소를 출력합니다."""

    print_section(f"{model_name} 로봇 정의 분석")

    # --- 1. 컨트롤러 순서 및 기본 컨트롤러 ---
    print_section("1. 컨트롤러 구조")
    print(f"  컨트롤러 순서 (action 차원 순서):")
    for i, name in enumerate(definition["raw_controller_order"]):
        default_ctrl = definition["default_controllers"].get(name, "N/A")
        print(f"    [{i}] {name:20s} -> {default_ctrl}")

    # --- 2. 이동 (Locomotion) ---
    loco = definition.get("locomotion")
    if loco:
        print_section("2. 이동 (Locomotion)")
        print(f"  베이스 조인트:        {loco.get('base_joint_names')}")
        print(f"  바닥 접촉 링크:      {loco.get('floor_touching_base_link_names')}")

    # --- 3. Holonomic Base ---
    holo = definition.get("holonomic_base")
    if holo:
        print_section("3. Holonomic Base")
        print(f"  구형 휠 근사:        {holo.get('force_sphere_wheel_approximation')}")
        print(f"  Base footprint link: {definition.get('base_footprint_link_name')}")

    # --- 4. Trunk (몸통) ---
    trunk = definition.get("articulated_trunk")
    if trunk:
        print_section("4. Trunk (몸통)")
        print(f"  트렁크 조인트 ({len(trunk['trunk_joint_names'])}개):")
        for j in trunk["trunk_joint_names"]:
            print(f"    - {j}")
        if trunk.get("trunk_link_names"):
            print(f"  트렁크 링크 ({len(trunk['trunk_link_names'])}개):")
            for l in trunk["trunk_link_names"]:
                print(f"    - {l}")

    # --- 5. 매니퓰레이션 (팔 + 그리퍼) ---
    manip = definition.get("manipulation")
    if manip:
        print_section("5. 매니퓰레이션 (팔 + 그리퍼)")
        print(f"  팔 개수:    {manip['n_arms']}")
        print(f"  팔 이름:    {manip['arm_names']}")

        for arm_name in manip["arm_names"]:
            print(f"\n  --- {arm_name.upper()} ARM ---")
            joints = manip["arm_joint_names"][arm_name]
            links = manip["arm_link_names"][arm_name]
            print(f"  조인트 ({len(joints)}개): {joints}")
            print(f"  링크   ({len(links)}개):  {links}")
            print(f"  EEF 링크:      {manip['eef_link_names'][arm_name]}")
            print(f"  핑거 링크:     {manip['finger_link_names'][arm_name]}")
            print(f"  핑거 조인트:   {manip['finger_joint_names'][arm_name]}")
            if manip.get("gripper_link_names") and arm_name in manip["gripper_link_names"]:
                print(f"  그리퍼 링크:   {manip['gripper_link_names'][arm_name]}")
            ws = manip.get("arm_workspace_range", {}).get(arm_name)
            if ws:
                print(f"  작업 공간 범위: {ws} (도)")

    # --- 6. Mobile Manipulation (Tuck/Untuck) ---
    mm = definition.get("mobile_manipulation")
    if mm:
        print_section("6. Mobile Manipulation (Tuck/Untuck)")
        tucked = mm["tucked_default_joint_pos"]
        untucked = mm["untucked_default_joint_pos"]
        print(f"  Tucked 포즈  ({len(tucked)}개 조인트): 첫 6개 = {tucked[:6]}")
        print(f"  Untucked 포즈 ({len(untucked)}개 조인트): 첫 6개 = {untucked[:6]}")

        # tuck/untuck 차이가 나는 조인트 표시
        diff_indices = [i for i in range(len(tucked)) if abs(tucked[i] - untucked[i]) > 0.01]
        if diff_indices:
            print(f"\n  Tuck <-> Untuck 차이나는 조인트 인덱스: {diff_indices}")
            for idx in diff_indices:
                print(f"    [{idx:2d}] tucked={tucked[idx]:+.4f}  untucked={untucked[idx]:+.4f}")

    # --- 7. 충돌 비활성화 쌍 ---
    coll = definition.get("disabled_collision_pairs")
    if coll:
        print_section("7. 비활성화된 충돌 쌍")
        for pair in coll:
            print(f"    {pair[0]:35s} <-> {pair[1]}")

    # --- 8. Primitives 속도 게인 ---
    print_section("8. Primitives 속도 게인")
    print(f"  선속도 게인: {definition.get('linear_velocity_gain_for_primitives')}")
    print(f"  각속도 게인: {definition.get('angular_velocity_gain_for_primitives')}")


def compare_r1_r1pro():
    """R1과 R1Pro의 주요 차이점을 비교합니다."""
    r1 = load_robot_definition("r1")
    r1pro = load_robot_definition("r1pro")

    print_section("R1 vs R1Pro 비교")

    # 팔 DOF 비교
    r1_manip = r1.get("manipulation", {})
    r1pro_manip = r1pro.get("manipulation", {})

    for arm in ["left", "right"]:
        r1_joints = r1_manip.get("arm_joint_names", {}).get(arm, [])
        r1pro_joints = r1pro_manip.get("arm_joint_names", {}).get(arm, [])
        print(f"\n  {arm.upper()} ARM DOF:")
        print(f"    R1:    {len(r1_joints)} DOF  {r1_joints}")
        print(f"    R1Pro: {len(r1pro_joints)} DOF  {r1pro_joints}")

    # 그리퍼 비교
    for arm in ["left", "right"]:
        r1_fingers = r1_manip.get("finger_joint_names", {}).get(arm, [])
        r1pro_fingers = r1pro_manip.get("finger_joint_names", {}).get(arm, [])
        print(f"\n  {arm.upper()} GRIPPER:")
        print(f"    R1:    {r1_fingers}")
        print(f"    R1Pro: {r1pro_fingers}")

    # R1Pro 고유 기능
    r1pro_gripper_links = r1pro_manip.get("gripper_link_names", {})
    if r1pro_gripper_links:
        print(f"\n  R1Pro 고유 - 그리퍼 링크 (RealSense 포함):")
        for arm, links in r1pro_gripper_links.items():
            print(f"    {arm}: {links}")

    # Tuck/Untuck 포즈 길이 비교
    r1_mm = r1.get("mobile_manipulation", {})
    r1pro_mm = r1pro.get("mobile_manipulation", {})
    r1_n = len(r1_mm.get("tucked_default_joint_pos", []))
    r1pro_n = len(r1pro_mm.get("tucked_default_joint_pos", []))
    print(f"\n  전체 조인트 수:")
    print(f"    R1:    {r1_n}개")
    print(f"    R1Pro: {r1pro_n}개")

    # 휠 링크 비교
    r1_wheels = r1.get("locomotion", {}).get("floor_touching_base_link_names", [])
    r1pro_wheels = r1pro.get("locomotion", {}).get("floor_touching_base_link_names", [])
    print(f"\n  바닥 접촉 링크:")
    print(f"    R1:    {r1_wheels}")
    print(f"    R1Pro: {r1pro_wheels}")


def list_all_robots():
    """등록된 모든 로봇을 나열합니다."""
    definition_dir = PROJECT_ROOT / "OmniGibson" / "omnigibson" / "robots" / "definitions"
    robots = sorted(p.stem for p in definition_dir.glob("*.yaml"))
    print_section("등록된 로봇 목록")
    for r in robots:
        print(f"    - {r}")
    return robots


def set_high_quality_render():
    """OmniGibson 기본 렌더링 설정을 복원합니다 (고품질)."""
    import carb

    s = carb.settings.get_settings()

    s.set_bool("/rtx/reflections/enabled", True)
    s.set_bool("/rtx/indirectDiffuse/enabled", True)
    s.set_bool("/rtx/ambientOcclusion/enabled", True)
    s.set_bool("/rtx/directLighting/sampledLighting/enabled", True)
    s.set_bool("/rtx/flow/enabled", True)
    s.set_bool("/rtx/translucency/enabled", True)
    s.set_bool("/rtx/shadows/enabled", True)
    s.set_int("/rtx/post/dlss/execMode", 0)
    s.set_bool("/rtx/post/aa/enabled", True)
    s.set_bool("/rtx/post/tonemap/enabled", True)
    s.set_bool("/rtx/post/denoiser/enabled", True)
    s.set_bool("/app/renderer/skipMaterialLoading", False)
    s.set_int("/rtx/raytracing/showLights", 1)
    s.set_float("/rtx/sceneDb/ambientLightIntensity", 1.0)

    # RTX-Interactive (Path Tracing) 모드로 설정
    s.set_string("/rtx/rendermode", "PathTracing")
    s.set_int("/rtx/pathtracing/spp", 1)
    s.set_int("/rtx/pathtracing/totalSpp", 64)
    s.set_int("/rtx/pathtracing/maxBounces", 4)
    s.set_int("/rtx/pathtracing/maxSpecularAndTransmissionBounces", 4)

    print("  렌더링: RTX-Interactive (Path Tracing)")


def setup_viewport_camera(position, orientation):
    """RENDER_VIEWER_CAMERA=False 상태에서 뷰포트 카메라를 직접 설정합니다.

    VisionSensor annotator가 Blackwell GPU에서 크래시하므로,
    USD Camera prim을 직접 생성하고 뷰포트에 연결합니다.
    또한 RENDER_VIEWER_CAMERA=False로 숨겨진 Viewport 창을 다시 표시합니다.

    Args:
        position: (x, y, z) 카메라 위치
        orientation: (x, y, z, w) 쿼터니언 (OmniGibson 규약)
    """
    from pxr import UsdGeom, Gf
    import omni.usd
    import omni.ui
    import omni.kit.viewport.window
    import omni.kit.app

    # 1) RENDER_VIEWER_CAMERA=False가 숨긴 Viewport 창을 다시 표시
    viewport_win = omni.ui.Workspace.get_window("Viewport")
    if viewport_win is not None:
        viewport_win.visible = True
        omni.kit.app.get_app().update()
        print("  Viewport 창 표시 완료")

    # 2) 렌더링 품질 설정
    set_high_quality_render()

    # 3) 뷰포트 해상도
    viewport_instances = list(omni.kit.viewport.window.get_viewport_window_instances())

    # 4) USD Camera prim 생성
    stage = omni.usd.get_context().get_stage()
    cam_path = "/World/viewport_camera"

    if not stage.GetPrimAtPath(cam_path).IsValid():
        UsdGeom.Camera.Define(stage, cam_path)

    prim = stage.GetPrimAtPath(cam_path)
    xformable = UsdGeom.Xformable(prim)

    xformable.ClearXformOpOrder()
    xformable.AddTranslateOp().Set(Gf.Vec3d(*position))

    # OmniGibson 쿼터니언 (x,y,z,w) -> USD 쿼터니언 (w,x,y,z)
    qx, qy, qz, qw = orientation
    xformable.AddOrientOp().Set(Gf.Quatf(qw, qx, qy, qz))

    # 클리핑 및 초점 거리 설정
    cam = UsdGeom.Camera(prim)
    cam.GetClippingRangeAttr().Set(Gf.Vec2f(0.01, 10000.0))
    cam.GetFocalLengthAttr().Set(17.0)

    # 5) 이 카메라를 뷰포트의 활성 카메라로 설정
    try:
        if viewport_instances:
            viewport_instances[0].viewport_api.set_active_camera(cam_path)
            print(f"  뷰포트 카메라 설정 완료: {cam_path}")
    except Exception as e:
        print(f"  뷰포트 카메라 설정 실패 (수동으로 뷰 조정 필요): {e}")


def run_visual(model_name: str = "r1pro", steps: int = 300):
    """시뮬레이션 창을 띄워 로봇을 시각적으로 확인합니다."""
    import torch as th
    from omnigibson.macros import gm
    gm.USE_GPU_DYNAMICS = False
    gm.ENABLE_FLATCACHE = True
    gm.RENDER_VIEWER_CAMERA = False  # VisionSensor annotator 우회
    import omnigibson as og

    print_section(f"{model_name} 시뮬레이션 시각 확인")

    cfg = {
        "scene": {"type": "Scene"},
        "robots": [
            {
                "model": model_name,
                "obs_modalities": [],
                "action_type": "continuous",
                "action_normalize": True,
                "grasping_mode": "physical",
                "default_reset_mode": "untuck",
            }
        ],
    }

    env = og.Environment(configs=cfg)
    robot = env.robots[0]

    # 뷰포트 카메라를 로봇 앞에 위치시킴
    # 기본 위치: 로봇에서 약 3m 떨어져 비스듬히 내려다보는 각도
    setup_viewport_camera(
        position=(-0.2, -2.7, 1.1),
        orientation=(0.68196617, -0.00155408, -0.00166678, 0.73138017),
    )

    print(f"  모델:          {robot.model}")
    print(f"  Action dim:    {robot.action_dim}")
    print(f"  컨트롤러:")
    for name in robot.controller_order:
        ctrl = robot.controllers[name]
        print(f"    {name:16s} -> {ctrl.__class__.__name__} (dim={ctrl.command_dim})")

    env.reset()
    robot.reset()

    print(f"\n  {steps} 스텝 실행 중... (Viewport 창에서 로봇을 확인하세요)")
    print(f"  창을 닫거나 Ctrl+C로 종료합니다.\n")

    for step in range(steps):
        if step % 30 == 0:
            action = robot.action_space.sample() * 0.05
        env.step(action=action)

        if step % 50 == 0:
            joint_pos = robot.get_joint_positions()
            print(f"  Step {step:3d} | joints[0:6]={[f'{v:.3f}' for v in joint_pos[:6].tolist()]}")

    print("\n  시뮬레이션 완료!")
    og.shutdown()


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Lab 1: R1Pro 로봇 구조 분석")
    parser.add_argument("--visual", action="store_true", help="시뮬레이션 창을 띄워 로봇 확인")
    parser.add_argument("--model", type=str, default="r1pro", help="시각화할 로봇 모델 (기본: r1pro)")
    parser.add_argument("--steps", type=int, default=3000, help="시뮬레이션 스텝 수 (기본: 3000)")
    args = parser.parse_args()

    if args.visual:
        # YAML 분석 출력 후 시뮬레이션 실행
        r1pro_def = load_robot_definition(args.model)
        inspect_robot(args.model, r1pro_def)
        run_visual(model_name=args.model, steps=args.steps)
    else:
        print("=" * 60)
        print("  Lab 1: R1Pro 로봇 구조 분석")
        print("=" * 60)

        list_all_robots()

        r1pro_def = load_robot_definition("r1pro")
        inspect_robot("R1Pro", r1pro_def)

        compare_r1_r1pro()

        print("\n\nLab 1 완료!")
        print("시뮬레이션 창으로 보려면: python custom/lab1_anatomy/inspect_r1pro.py --visual")
