#!/usr/bin/env python3
"""
Allegro Hand USD 시각 확인 스크립트
===================================
URDF→USD 변환 결과를 Isaac Sim 뷰포트에서 확인합니다.

OmniGibson의 USDObject 대신 직접 스테이지에 USD를 로드합니다.
(URDF 임포터 출력 구조가 OG의 entity_prim과 호환되지 않으므로)

사용법:
    python custom/lab3_custom_robot/view_allegro_usd.py --hand right
    python custom/lab3_custom_robot/view_allegro_usd.py --hand left

키보드/마우스로 카메라를 자유롭게 조작할 수 있습니다.
"""

import argparse
import os

import torch as th

from omnigibson.macros import gm

gm.USE_GPU_DYNAMICS = False
gm.ENABLE_FLATCACHE = True
gm.RENDER_VIEWER_CAMERA = False

import omnigibson as og
import omnigibson.lazy as lazy


def setup_viewport(position=(-0.2, -2.7, 1.1), orientation_wxyz=(0.73138017, 0.68196617, -0.00155408, -0.00166678)):
    """RENDER_VIEWER_CAMERA=False로 숨겨진 Viewport를 복원하고 카메라/렌더 설정합니다.

    다른 lab 스크립트(lab1~lab4)와 동일한 RTX 렌더링 설정을 적용합니다.
    """
    from pxr import UsdGeom, Gf
    import carb
    import omni.usd
    import omni.ui
    import omni.kit.viewport.window
    import omni.kit.app

    # Viewport 창 다시 표시
    vp_win = omni.ui.Workspace.get_window("Viewport")
    if vp_win is not None:
        vp_win.visible = True
        omni.kit.app.get_app().update()

    # 고품질 렌더링 (lab1 inspect_r1pro.py와 동일)
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

    # RTX-Interactive (Path Tracing) 모드
    s.set_string("/rtx/rendermode", "PathTracing")
    s.set_int("/rtx/pathtracing/spp", 1)
    s.set_int("/rtx/pathtracing/totalSpp", 64)
    s.set_int("/rtx/pathtracing/maxBounces", 4)
    s.set_int("/rtx/pathtracing/maxSpecularAndTransmissionBounces", 4)

    # 카메라 생성 및 뷰포트 연결
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
    print("  뷰포트 카메라/렌더 설정 완료 (RTX Path Tracing)")


def main():
    parser = argparse.ArgumentParser(description="Allegro Hand USD 뷰어")
    parser.add_argument(
        "--hand",
        choices=["right", "left"],
        default="right",
        help="확인할 손 (right 또는 left)",
    )
    args = parser.parse_args()

    hand = args.hand
    model_name = f"allegro_hand_{hand}"

    # _base.usd에 실제 로봇 지오메트리가 있음
    base_usd = os.path.join(
        gm.DATA_PATH, "custom_dataset", "objects", "robot",
        model_name, "usd", "configuration", f"{model_name}_base.usd",
    )

    if not os.path.exists(base_usd):
        print(f"ERROR: USD 파일이 없습니다: {base_usd}")
        print("먼저 run_allegro_import.py로 변환하세요.")
        return

    print(f"Loading: {base_usd}")

    # 빈 씬 + 조명으로 OG 환경 생성
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

    # 스테이지에 직접 USD 참조 추가 (z축 +0.3m 오프셋)
    stage = og.sim.stage
    prim_path = f"/World/{model_name}"
    prim = stage.DefinePrim(prim_path, "Xform")
    prim.GetReferences().AddReference(base_usd)

    # z축 30cm 위로 이동
    from pxr import UsdGeom, Gf
    xformable = UsdGeom.Xformable(prim)
    xformable.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 0.3))

    # physics USD도 레이어로 추가 (조인트 정보)
    physics_usd = base_usd.replace("_base.usd", "_physics.usd")
    if os.path.exists(physics_usd):
        prim.GetReferences().AddReference(physics_usd)
        print(f"Physics 레이어 로드: {physics_usd}")

    # 한 프레임 렌더링해서 지오메트리 표시
    og.sim.render()

    # Viewport 복원 + 렌더/카메라 설정 (lab1~4 공통)
    # 카메라 (0, -0.35, 0.45) → 손 중심 (0, 0, 0.3)을 바라보도록 설정
    # X축 회전 66.8°: q = (cos(33.4°), sin(33.4°), 0, 0)
    setup_viewport(
        position=(0.0, -0.35, 0.45),
        orientation_wxyz=(0.835, 0.550, 0.0, 0.0),
    )

    print("\n" + "=" * 50)
    print(f"  Allegro {hand.upper()} hand 시각화 중")
    print(f"  USD: {base_usd}")
    print("  마우스/키보드로 카메라를 움직이세요")
    print("  Ctrl+C로 종료")
    print("=" * 50 + "\n")

    # 렌더 루프
    try:
        for _ in range(100000):
            og.sim.render()
    except KeyboardInterrupt:
        print("\n종료합니다...")

    og.shutdown()


if __name__ == "__main__":
    main()
