"""
설치 검증 스크립트 (비전 센서 우회)
===================================
RTX 5090 + Isaac Sim 4.5 환경에서 렌더러 호환 문제를 우회하여
물리 시뮬레이션이 정상 동작하는지 검증합니다.

실행:
    python custom/test_install.py              # 창 띄움 (Viewport로 시각 확인)
    python custom/test_install.py --headless   # 창 없이 실행
"""

import argparse
import torch as th

parser = argparse.ArgumentParser()
parser.add_argument("--headless", action="store_true", help="창 없이 실행")
args = parser.parse_args()

# OmniGibson 매크로 설정 (import 전에 설정해야 함)
import omnigibson.macros as m
from omnigibson.macros import gm

gm.USE_GPU_DYNAMICS = False
gm.ENABLE_FLATCACHE = True
gm.RENDER_VIEWER_CAMERA = False  # VisionSensor annotator 우회 (syntheticdata 버그 회피)
gm.HEADLESS = args.headless      # False면 Viewport 창이 열림

import omnigibson as og

print("=" * 60)
print("  BEHAVIOR-1K 설치 검증 (비전 센서 우회)")
print("=" * 60)

# 환경 설정 — obs_modalities를 빈 리스트로 (비전 센서 비활성화)
cfg = {
    "scene": {"type": "Scene"},
    "robots": [
        {
            "model": "r1pro",
            "obs_modalities": [],     # 비전 센서 없음
            "action_type": "continuous",
            "action_normalize": True,
            "grasping_mode": "physical",
            "default_reset_mode": "untuck",
        }
    ],
}

print("\n[1/4] 환경 생성 중...")
env = og.Environment(configs=cfg)
robot = env.robots[0]

# 뷰포트 카메라 위치 설정 (RENDER_VIEWER_CAMERA=False이므로 직접 설정)
if not args.headless:
    try:
        from pxr import UsdGeom, Gf
        import carb
        import omni.usd
        import omni.ui
        import omni.kit.app
        import omni.kit.viewport.window

        # Viewport 창 다시 표시 (RENDER_VIEWER_CAMERA=False가 숨김)
        vp_win = omni.ui.Workspace.get_window("Viewport")
        if vp_win is not None:
            vp_win.visible = True
            omni.kit.app.get_app().update()

        # 렌더링 품질 최저로 설정
        s = carb.settings.get_settings()
        for key in ["/rtx/reflections/enabled", "/rtx/indirectDiffuse/enabled",
                     "/rtx/ambientOcclusion/enabled", "/rtx/directLighting/sampledLighting/enabled",
                     "/rtx/flow/enabled", "/rtx/translucency/enabled", "/rtx/caustics/enabled",
                     "/rtx/shadows/enabled", "/rtx/post/aa/enabled", "/rtx/post/tonemap/enabled",
                     "/rtx/post/denoiser/enabled"]:
            s.set_bool(key, False)
        s.set_bool("/app/renderer/skipMaterialLoading", True)
        s.set_int("/rtx/pathtracing/spp", 1)
        s.set_int("/rtx/pathtracing/totalSpp", 1)
        s.set_int("/app/renderer/resolution/width", 640)
        s.set_int("/app/renderer/resolution/height", 480)

        # 뷰포트 해상도 낮추기
        vps = list(omni.kit.viewport.window.get_viewport_window_instances())
        if vps:
            vps[0].viewport_api.set_texture_resolution((640, 480))

        # 카메라 설정
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
        if vps:
            vps[0].viewport_api.set_active_camera(cam_path)
        print("  뷰포트 카메라 설정 완료 (저품질 모드)")
    except Exception as e:
        print(f"  뷰포트 카메라 설정 실패 (수동으로 뷰 조정 필요): {e}")

print(f"\n[2/4] 로봇 정보:")
print(f"  모델:          {robot.model}")
print(f"  Action dim:    {robot.action_dim}")
print(f"  컨트롤러 순서: {robot.controller_order}")
for name in robot.controller_order:
    ctrl = robot.controllers[name]
    print(f"    {name:16s} -> {ctrl.__class__.__name__} (dim={ctrl.command_dim})")

print(f"\n[3/4] 시뮬레이션 100스텝 실행 중...")
env.reset()
robot.reset()

for step in range(100):
    if step % 30 == 0:
        action = robot.action_space.sample() * 0.05
    obs, reward, terminated, truncated, info = env.step(action=action)

    if step % 25 == 0:
        joint_pos = robot.get_joint_positions()
        print(f"  Step {step:3d} | joints[0:4]={joint_pos[:4].tolist()}")

print(f"\n[4/4] 정리 중...")
og.shutdown()

print("\n" + "=" * 60)
print("  설치 검증 성공!")
print("  물리 시뮬레이션 + 로봇 제어가 정상 동작합니다.")
print("  (비전 센서/렌더링은 Isaac Sim의 Blackwell 지원 후 사용 가능)")
print("=" * 60)
