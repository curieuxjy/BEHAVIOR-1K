#!/usr/bin/env python3
"""
Allegro Hand 관절별 DOF Range 테스트
=====================================
각 관절을 하나씩 순차적으로 하한→상한→하한 범위를 순회합니다.
관절 이름, 인덱스, 현재 각도를 실시간으로 출력합니다.

사용법:
    python custom/lab3_custom_robot/test_allegro_joints.py --hand right
    python custom/lab3_custom_robot/test_allegro_joints.py --hand left
    python custom/lab3_custom_robot/test_allegro_joints.py --hand right --steps-per-joint 120
"""

import argparse
import math
import os

import torch as th

from omnigibson.macros import gm

gm.USE_GPU_DYNAMICS = False
# Flatcache 비활성화: USD 속성 쓰기가 PhysX에 즉시 반영되도록
gm.ENABLE_FLATCACHE = False
gm.RENDER_VIEWER_CAMERA = False

import omnigibson as og


def setup_viewport(position=(-0.2, -2.7, 1.1), orientation_wxyz=(0.73138017, 0.68196617, -0.00155408, -0.00166678)):
    """Viewport를 복원하고 카메라/렌더 설정 (lab1~4 공통 패턴)."""
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
    UsdGeom.Camera(prim).GetFocalLengthAttr().Set(17.0)

    vps = list(omni.kit.viewport.window.get_viewport_window_instances())
    if vps:
        vps[0].viewport_api.set_active_camera(cam_path)
    print("  뷰포트 카메라/렌더 설정 완료")


def main():
    parser = argparse.ArgumentParser(
        description="Allegro Hand 관절별 DOF 테스트",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
예시:
  # 기본 실행
  python custom/lab3_custom_robot/test_allegro_joints.py --hand right

  # 부드럽고 자연스러운 움직임 (acceleration 모드 + 낮은 감쇠)
  python custom/lab3_custom_robot/test_allegro_joints.py --drive-type acceleration --stiffness 100 --damping 10

  # 빠르고 정확한 추적
  python custom/lab3_custom_robot/test_allegro_joints.py --stiffness 5000 --damping 200

  # 느리고 부드러운 애니메이션
  python custom/lab3_custom_robot/test_allegro_joints.py --steps-per-joint 120 --stiffness 500 --damping 50
""")
    parser.add_argument("--hand", choices=["right", "left"], default="right",
                        help="테스트할 손 (right 또는 left)")
    parser.add_argument("--steps-per-joint", type=int, default=60,
                        help="관절당 스텝 수 (기본: 60, 편도)")
    parser.add_argument("--stiffness", type=float, default=500,
                        help="드라이브 강성 (P 게인, 기본: 500)")
    parser.add_argument("--damping", type=float, default=50,
                        help="드라이브 감쇠 (D 게인, 기본: 50)")
    parser.add_argument("--max-force", type=float, default=1e3,
                        help="최대 토크 (기본: 1000)")
    parser.add_argument("--drive-type", choices=["force", "acceleration"], default="acceleration",
                        help="드라이브 모드 (기본: acceleration, 질량 무관)")
    args = parser.parse_args()

    hand = args.hand
    model_name = f"allegro_hand_{hand}"
    steps_per_joint = args.steps_per_joint

    base_usd = os.path.join(
        gm.DATA_PATH, "custom_dataset", "objects", "robot",
        model_name, "usd", "configuration", f"{model_name}_base.usd",
    )
    physics_usd = base_usd.replace("_base.usd", "_physics.usd")

    if not os.path.exists(base_usd):
        print(f"ERROR: USD 파일이 없습니다: {base_usd}")
        print("먼저 run_allegro_import.py로 변환하세요.")
        return

    print(f"Loading: {base_usd}")

    # 빈 씬 + 조명
    cfg = {
        "scene": {"type": "Scene"},
        "objects": [
            dict(type="LightObject", light_type="Sphere", name="light0",
                 radius=0.01, intensity=1e5, position=[-1.0, -1.0, 1.5]),
            dict(type="LightObject", light_type="Sphere", name="light1",
                 radius=0.01, intensity=1e5, position=[1.0, 1.0, 1.5]),
        ],
    }
    env = og.Environment(configs=cfg)

    # USD 로드
    stage = og.sim.stage
    prim_path = f"/World/{model_name}"
    prim = stage.DefinePrim(prim_path, "Xform")
    prim.GetReferences().AddReference(base_usd)
    if os.path.exists(physics_usd):
        prim.GetReferences().AddReference(physics_usd)
        print(f"Physics 레이어 로드: {physics_usd}")

    from pxr import UsdGeom, Gf, UsdPhysics

    # z축 30cm 오프셋
    xformable = UsdGeom.Xformable(prim)
    xformable.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 0.3))

    # base_link 고정 (공중에 떠있도록)
    base_link_path = f"{prim_path}/base_link"
    base_link = stage.GetPrimAtPath(base_link_path)
    if base_link and base_link.IsValid():
        if not base_link.HasAPI(UsdPhysics.ArticulationRootAPI):
            UsdPhysics.ArticulationRootAPI.Apply(base_link)
            print("  ArticulationRootAPI 추가 (base_link)")
        if not base_link.HasAPI(UsdPhysics.RigidBodyAPI):
            UsdPhysics.RigidBodyAPI.Apply(base_link)
        rb = UsdPhysics.RigidBodyAPI(base_link)
        rb.CreateKinematicEnabledAttr().Set(True)
        print("  base_link를 kinematic으로 고정")
    else:
        print(f"WARNING: base_link prim 없음: {base_link_path}")

    # 관절 찾기 + 드라이브 추가
    joint_scope_path = f"{prim_path}/joints"
    joint_scope = stage.GetPrimAtPath(joint_scope_path)

    joints = []
    if joint_scope and joint_scope.IsValid():
        for child in joint_scope.GetChildren():
            type_name = child.GetTypeName()
            if "Fixed" in type_name:
                continue

            name = child.GetName()
            lower = child.GetAttribute("physics:lowerLimit").Get()
            upper = child.GetAttribute("physics:upperLimit").Get()

            if lower is None or upper is None:
                continue

            # 위치 드라이브 추가 (PD controller)
            drive = UsdPhysics.DriveAPI.Apply(child, "angular")
            drive.CreateTypeAttr().Set(args.drive_type)
            drive.CreateMaxForceAttr().Set(args.max_force)
            drive.CreateStiffnessAttr().Set(args.stiffness)
            drive.CreateDampingAttr().Set(args.damping)
            drive.CreateTargetPositionAttr().Set(0.0)

            joints.append({
                "name": name,
                "prim_path": str(child.GetPath()),
                "lower": lower,
                "upper": upper,
            })

    if not joints:
        print("ERROR: 움직일 수 있는 관절을 찾지 못했습니다.")
        og.shutdown()
        return

    print(f"\n{'='*60}")
    print(f"  Allegro {hand.upper()} hand — 관절 {len(joints)}개")
    print(f"  drive: {args.drive_type}  stiffness: {args.stiffness}  damping: {args.damping}  maxForce: {args.max_force}")
    print(f"{'='*60}")
    print(f"  {'#':>3s}  {'관절 이름':20s}  {'하한(°)':>8s}  {'상한(°)':>8s}  {'범위(°)':>8s}")
    print(f"  {'---':>3s}  {'--------------------':20s}  {'--------':>8s}  {'--------':>8s}  {'--------':>8s}")
    for i, j in enumerate(joints):
        r = j["upper"] - j["lower"]
        print(f"  {i:3d}  {j['name']:20s}  {j['lower']:8.1f}  {j['upper']:8.1f}  {r:8.1f}")

    # 렌더 + 뷰포트 설정
    og.sim.render()
    setup_viewport(
        position=(0.0, -0.35, 0.45),
        orientation_wxyz=(0.835, 0.550, 0.0, 0.0),
    )

    # 시뮬레이션 시작
    og.sim.play()
    for _ in range(30):
        og.sim.step()

    print(f"\n  관절당 {steps_per_joint} 스텝 (편도), 왕복 순회합니다.")
    print(f"  Ctrl+C로 종료\n")

    def get_drive(joint):
        """관절의 DriveAPI를 매번 새로 가져옴 (stale reference 방지)."""
        jp = stage.GetPrimAtPath(joint["prim_path"])
        return UsdPhysics.DriveAPI.Get(jp, "angular")

    try:
        # ── 각 관절 순차 테스트 ──
        for j_idx, joint in enumerate(joints):
            name = joint["name"]
            lower = joint["lower"]
            upper = joint["upper"]

            print(f"  [{j_idx + 1:2d}/{len(joints)}] {name:20s}  ", end="", flush=True)

            # 하한 → 상한
            for step in range(steps_per_joint):
                frac = step / max(steps_per_joint - 1, 1)
                target = lower + frac * (upper - lower)
                get_drive(joint).GetTargetPositionAttr().Set(target)
                og.sim.step()

            print(f"{lower:.1f}° → {upper:.1f}°  ", end="", flush=True)

            # 상한 → 하한 (복귀)
            for step in range(steps_per_joint):
                frac = step / max(steps_per_joint - 1, 1)
                target = upper - frac * (upper - lower)
                get_drive(joint).GetTargetPositionAttr().Set(target)
                og.sim.step()

            # 중립 위치로 리셋
            center = (lower + upper) / 2.0
            get_drive(joint).GetTargetPositionAttr().Set(center)
            for _ in range(10):
                og.sim.step()

            print(f"→ 중립 {center:.1f}°  OK")

        # ── 전체 관절 동시 웨이브 ──
        print(f"\n  전체 관절 동시 웨이브 애니메이션...")
        for cycle in range(3):
            for step in range(steps_per_joint * 2):
                frac = step / max(steps_per_joint * 2 - 1, 1)
                for j_idx, joint in enumerate(joints):
                    lower = joint["lower"]
                    upper = joint["upper"]
                    center = (lower + upper) / 2.0
                    half_range = (upper - lower) / 2.0
                    # 관절마다 위상 오프셋 → 웨이브 효과
                    phase = j_idx / len(joints) * 2 * math.pi
                    val = center + half_range * math.sin(frac * 2 * math.pi + phase)
                    get_drive(joint).GetTargetPositionAttr().Set(val)
                og.sim.step()

        # ── 중립 위치로 리셋 ──
        for joint in joints:
            center = (joint["lower"] + joint["upper"]) / 2.0
            get_drive(joint).GetTargetPositionAttr().Set(center)
        for _ in range(30):
            og.sim.step()

        print("\n  모든 관절 테스트 완료!")
        print("  마우스/키보드로 카메라를 움직이세요. Ctrl+C로 종료.\n")

        # 유지
        while True:
            og.sim.render()

    except KeyboardInterrupt:
        print("\n종료합니다...")

    og.shutdown()


if __name__ == "__main__":
    main()
