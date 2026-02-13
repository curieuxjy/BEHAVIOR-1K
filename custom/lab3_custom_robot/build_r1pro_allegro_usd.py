"""
R1Pro + Allegro Hand USD 에셋 조립 스크립트 (통합 URDF 방식)
============================================================
R1Pro URDF에서 gripper를 제거하고 Allegro hand URDF를 병합한 뒤,
하나의 통합 URDF을 Isaac Sim URDF 임포터로 USD로 변환합니다.

이전 방식(USD 레벨 Sdf.CopySpec)의 문제를 근본적으로 해결:
  - URDF(XML) 단계에서 모든 조립 완료
  - 단일 URDF → USD 변환으로 일관된 결과
  - 스테이지/레이어 조작 불필요

실행 (OmniGibson/Isaac Sim 환경 필요):
    python custom/lab3_custom_robot/build_r1pro_allegro_usd.py

    # URDF 병합 분석만 (Isaac Sim 필요 없음)
    python custom/lab3_custom_robot/build_r1pro_allegro_usd.py --analyze-only

    # 빌드 + 시뮬레이터 미리보기
    python custom/lab3_custom_robot/build_r1pro_allegro_usd.py --preview

입력:
    - datasets/.../models/r1pro/urdf/r1pro.urdf
    - ../allegro_inhand_rotation/assets/allegro/allegro_right.urdf
    - ../allegro_inhand_rotation/assets/allegro/allegro_left.urdf

출력:
    - datasets/.../models/r1pro/r1pro_allegro/usd/r1pro_allegro.usda
"""

import argparse
import os
import shutil
import sys
import tempfile
import time
import xml.etree.ElementTree as ET
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
DATASETS_DIR = PROJECT_ROOT / "datasets" / "omnigibson-robot-assets"

# --- Input paths ---
# r1pro.urdf는 생성된 collision mesh(meshes/collision/)를 참조하지만 파일이 없음.
# r1pro_original.urdf는 동일한 링크/조인트 구조이면서 visual mesh를 collision에도 사용 (파일 존재).
R1PRO_URDF_PATH = DATASETS_DIR / "models" / "r1pro" / "urdf" / "r1pro_original.urdf"
ALLEGRO_URDF_DIR = PROJECT_ROOT.parent / "allegro_inhand_rotation" / "assets" / "allegro"
ALLEGRO_RIGHT_URDF = ALLEGRO_URDF_DIR / "allegro_right.urdf"
ALLEGRO_LEFT_URDF = ALLEGRO_URDF_DIR / "allegro_left.urdf"

# --- Output paths ---
OUTPUT_DIR = DATASETS_DIR / "models" / "r1pro" / "r1pro_allegro" / "usd"
OUTPUT_USD_PATH = OUTPUT_DIR / "r1pro_allegro.usda"

# --- R1Pro gripper parts to remove (per side) ---
GRIPPER_LINK_NAMES = [
    "{side}_gripper_link",
    "{side}_realsense_link",
    "{side}_gripper_finger_link1",
    "{side}_gripper_finger_link2",
]
GRIPPER_JOINT_NAMES = [
    "{side}_gripper_joint",
    "{side}_realsense_joint",
    "{side}_gripper_finger_joint1",
    "{side}_gripper_finger_joint2",
]

# --- Mount transform (from R1Pro URDF: arm_link7 → gripper_link) ---
MOUNT_OFFSET_XYZ = "-0.0295 0 -0.18065"

# --- Palm center offset from allegro base_link ---
PALM_CENTER_XYZ = "0 0 0.04"


# ========================================================================
# URDF 조작 함수
# ========================================================================

def remove_gripper_from_urdf(root, side):
    """R1Pro URDF에서 한쪽 gripper 링크/조인트를 제거합니다."""
    removed_links = []
    removed_joints = []

    for tmpl in GRIPPER_JOINT_NAMES:
        name = tmpl.format(side=side)
        for j in root.findall("joint"):
            if j.get("name") == name:
                root.remove(j)
                removed_joints.append(name)
                break

    for tmpl in GRIPPER_LINK_NAMES:
        name = tmpl.format(side=side)
        for l in root.findall("link"):
            if l.get("name") == name:
                root.remove(l)
                removed_links.append(name)
                break

    print(f"    {side}: 링크 {len(removed_links)}개, 조인트 {len(removed_joints)}개 제거")
    return removed_links, removed_joints


def create_prefixed_hand_elements(urdf_path, side):
    """Allegro hand URDF를 파싱하여 side-prefix + sanitize된 링크/조인트를 반환합니다.

    dots→underscores 변환 + palm_center 추가 포함.

    Returns:
        list[ET.Element]: R1Pro URDF에 추가할 link/joint 요소 목록
    """
    tree = ET.parse(str(urdf_path))
    root = tree.getroot()

    def prefix(name):
        sanitized = name.replace(".", "_")
        if name == "base_link":
            return f"{side}_allegro_base_link"
        return f"{side}_{sanitized}"

    # 링크 이름 prefix
    for link in root.findall("link"):
        link.set("name", prefix(link.get("name")))

    # 조인트 이름 + parent/child prefix
    for joint in root.findall("joint"):
        joint.set("name", prefix(joint.get("name")))
        p = joint.find("parent")
        if p is not None:
            p.set("link", prefix(p.get("link")))
        c = joint.find("child")
        if c is not None:
            c.set("link", prefix(c.get("link")))

    # 메시 경로 수정
    for mesh in root.findall(".//mesh"):
        fn = mesh.get("filename", "")
        # allegro/meshes/... → meshes/... (오른손 thumb 버그)
        if fn.startswith("allegro/meshes/"):
            fn = fn[len("allegro/"):]
        # 파일명 dots → underscores
        parts = fn.split("/")
        name_part, ext = os.path.splitext(parts[-1])
        parts[-1] = name_part.replace(".", "_") + ext
        mesh.set("filename", "/".join(parts))

    elements = []
    for link in root.findall("link"):
        elements.append(link)
    for joint in root.findall("joint"):
        elements.append(joint)

    # palm_center 링크 추가
    palm = ET.Element("link", name=f"{side}_palm_center")
    inertial = ET.SubElement(palm, "inertial")
    ET.SubElement(inertial, "mass", value="0.001")
    ET.SubElement(inertial, "origin", xyz="0 0 0", rpy="0 0 0")
    ET.SubElement(inertial, "inertia",
                  ixx="1e-7", iyy="1e-7", izz="1e-7", ixy="0", ixz="0", iyz="0")
    elements.append(palm)

    # palm_center 고정 조인트
    pj = ET.Element("joint", name=f"{side}_palm_center_joint", type="fixed")
    ET.SubElement(pj, "parent", link=f"{side}_allegro_base_link")
    ET.SubElement(pj, "child", link=f"{side}_palm_center")
    ET.SubElement(pj, "origin", xyz=PALM_CENTER_XYZ, rpy="0 0 0")
    elements.append(pj)

    links = [e for e in elements if e.tag == "link"]
    joints = [e for e in elements if e.tag == "joint"]
    print(f"    {side}: {len(links)}개 링크, {len(joints)}개 조인트")
    return elements


def sanitize_all_names(root):
    """통합 URDF의 모든 name 속성에서 SdfPath 비호환 문자(-, .)를 _로 변환합니다.

    R1Pro collision 요소: name="base_link-col-0" → name="base_link_col_0"
    Allegro 잔여 dots (있을 경우): name="link_0.0" → name="link_0_0"
    """
    import re
    fixed = 0
    for elem in root.iter():
        name = elem.get("name")
        if name and not re.match(r'^[a-zA-Z_][a-zA-Z0-9_]*$', name):
            new_name = name.replace("-", "_").replace(".", "_")
            elem.set("name", new_name)
            fixed += 1
    return fixed


def add_mount_joint(root, side):
    """arm_link7 → allegro_base_link 고정 조인트를 URDF에 추가합니다."""
    joint = ET.SubElement(root, "joint",
                          name=f"{side}_allegro_mount_joint", type="fixed")
    ET.SubElement(joint, "parent", link=f"{side}_arm_link7")
    ET.SubElement(joint, "child", link=f"{side}_allegro_base_link")
    # X축 기준 180° 회전 (π rad) — 손 방향을 gripper와 일치시킴
    ET.SubElement(joint, "origin", xyz=MOUNT_OFFSET_XYZ, rpy="3.14159265 0 0")
    print(f"    {side}: {side}_arm_link7 → {side}_allegro_base_link  xyz={MOUNT_OFFSET_XYZ} rpy=π,0,0")


# ========================================================================
# 메시 파일 복사
# ========================================================================

def copy_all_meshes(work_dir):
    """R1Pro + Allegro 메시 파일을 작업 디렉토리에 복사합니다."""
    # R1Pro 메시 복사
    r1pro_meshes_src = R1PRO_URDF_PATH.parent / "meshes"
    r1pro_meshes_dst = Path(work_dir) / "meshes"
    if r1pro_meshes_src.exists():
        shutil.copytree(str(r1pro_meshes_src), str(r1pro_meshes_dst))
        r1_count = sum(1 for _ in r1pro_meshes_dst.rglob("*.obj"))
        print(f"  R1Pro 메시: {r1_count}개 파일")

    # Allegro 메시 복사 (파일명 dots → underscores)
    allegro_src = ALLEGRO_URDF_DIR / "meshes" / "allegro"
    allegro_dst = Path(work_dir) / "meshes" / "allegro"
    allegro_dst.mkdir(parents=True, exist_ok=True)

    fixed = 0
    for fname in os.listdir(str(allegro_src)):
        src = allegro_src / fname
        if not src.is_file():
            continue
        name_part, ext = os.path.splitext(fname)
        new_fname = name_part.replace(".", "_") + ext
        shutil.copy2(str(src), str(allegro_dst / new_fname))
        if fname != new_fname:
            fixed += 1
    a_count = sum(1 for _ in allegro_dst.iterdir())
    print(f"  Allegro 메시: {a_count}개 파일 ({fixed}개 이름 변환)")


# ========================================================================
# URDF → USD 변환
# ========================================================================

def convert_urdf_to_usd(urdf_path, dest_usd_path):
    """URDF를 Isaac Sim URDF 임포터로 USD로 변환합니다."""
    import omnigibson.lazy as lazy

    print(f"    URDF: {urdf_path}")
    print(f"    USD:  {dest_usd_path}")
    dest_usd_path.parent.mkdir(parents=True, exist_ok=True)

    _, cfg = lazy.omni.kit.commands.execute("URDFCreateImportConfig")
    drive_mode = cfg.default_drive_type.__class__
    cfg.set_merge_fixed_joints(False)
    cfg.set_fix_base(False)
    cfg.set_import_inertia_tensor(True)
    cfg.set_convex_decomp(False)
    cfg.set_self_collision(False)
    cfg.set_default_drive_type(drive_mode.JOINT_DRIVE_NONE)
    cfg.set_default_drive_strength(0.0)
    cfg.set_default_position_drive_damping(0.0)
    cfg.set_distance_scale(1.0)
    cfg.set_density(0.0)
    cfg.set_up_vector(0, 0, 1)
    cfg.set_make_default_prim(True)
    cfg.set_create_physics_scene(False)

    result = lazy.omni.kit.commands.execute(
        "URDFParseAndImportFile",
        urdf_path=str(urdf_path),
        import_config=cfg,
        dest_path=str(dest_usd_path),
    )

    prim_path = result[1] if isinstance(result, tuple) and len(result) > 1 else result
    print(f"    변환 완료! root prim: {prim_path}")

    if dest_usd_path.exists():
        size_mb = dest_usd_path.stat().st_size / (1024 * 1024)
        print(f"    파일 크기: {size_mb:.1f} MB")
    else:
        print(f"    ERROR: 출력 파일 없음!")

    return prim_path


# ========================================================================
# USD 검증
# ========================================================================

def verify_output_usd(usd_path):
    """생성된 USD의 구조를 검증합니다.

    URDF 임포터는 운동학 트리에 따라 중첩된 계층 구조를 생성하므로
    stage.Traverse()로 전체 프림을 순회합니다.
    """
    from pxr import Usd, UsdPhysics

    stage = Usd.Stage.Open(str(usd_path))
    all_prims = list(stage.Traverse())
    print(f"  총 프림 수: {len(all_prims)}")

    # 프림 이름별 분류 (전체 계층 순회)
    allegro_links, allegro_joints = [], []
    r1pro_links, r1pro_joints = [], []
    gripper_found = []

    for prim in all_prims:
        name = prim.GetName()
        ptype = prim.GetTypeName()

        # 메시/비주얼/충돌 서브 프림 스킵
        if ptype in ("Mesh", "Scope", "Material", "Shader", "") or \
           name in ("visuals", "collisions", "mesh", "Looks", "joints"):
            continue

        is_allegro = ("allegro" in name or "palm_center" in name or
                      any(name.startswith(f"{s}_link_") or name.startswith(f"{s}_joint_")
                          for s in ["left", "right"]))

        if "gripper" in name:
            gripper_found.append(name)

        if "Joint" in ptype:
            (allegro_joints if is_allegro else r1pro_joints).append(name)
        elif ptype == "Xform":
            (allegro_links if is_allegro else r1pro_links).append(name)

    print(f"  R1Pro: {len(r1pro_links)} links, {len(r1pro_joints)} joints")
    print(f"  Allegro: {len(allegro_links)} links, {len(allegro_joints)} joints")

    # 핵심 프림 존재 확인 (전체 계층에서 이름으로 검색)
    prim_names = {p.GetName(): p.GetPrimPath().pathString for p in all_prims}
    critical = [
        "left_allegro_base_link", "right_allegro_base_link",
        "left_palm_center", "right_palm_center",
        "left_arm_link7", "right_arm_link7",
        "left_link_0_0", "right_link_0_0",
    ]
    all_ok = True
    for name in critical:
        if name in prim_names:
            print(f"    {name:35s} OK  ({prim_names[name]})")
        else:
            print(f"    {name:35s} MISSING")
            all_ok = False

    # Allegro 프림 샘플
    print(f"\n  Allegro 프림 샘플:")
    for name in sorted(allegro_links)[:6]:
        print(f"    [L] {name}")
    if len(allegro_links) > 6:
        print(f"    ... 외 {len(allegro_links) - 6}개")
    for name in sorted(allegro_joints)[:6]:
        print(f"    [J] {name}")
    if len(allegro_joints) > 6:
        print(f"    ... 외 {len(allegro_joints) - 6}개")

    # gripper 잔존 확인
    if gripper_found:
        print(f"  WARNING: gripper 프림 잔존: {gripper_found}")
    else:
        print(f"  OK: gripper 프림 없음")

    return all_ok


# ========================================================================
# 통합 URDF 빌드
# ========================================================================

def build_combined_urdf(work_dir):
    """R1Pro + Allegro 통합 URDF를 생성합니다.

    Returns:
        Path: 통합 URDF 파일 경로
    """
    print("=" * 70)
    print("  R1Pro + Allegro Hand 통합 URDF 빌드")
    print("=" * 70)

    for p, label in [(R1PRO_URDF_PATH, "R1Pro URDF"),
                      (ALLEGRO_RIGHT_URDF, "Allegro Right URDF"),
                      (ALLEGRO_LEFT_URDF, "Allegro Left URDF")]:
        assert p.exists(), f"{label} not found: {p}"

    # --- Step 1: R1Pro URDF 파싱 ---
    print(f"\n[Step 1] R1Pro URDF 파싱")
    tree = ET.parse(str(R1PRO_URDF_PATH))
    root = tree.getroot()
    n_links = len(root.findall("link"))
    n_joints = len(root.findall("joint"))
    print(f"  원본: {n_links}개 링크, {n_joints}개 조인트")

    # --- Step 2: Gripper 제거 ---
    print(f"\n[Step 2] Gripper 링크/조인트 제거")
    for side in ["left", "right"]:
        remove_gripper_from_urdf(root, side)
    n_links2 = len(root.findall("link"))
    n_joints2 = len(root.findall("joint"))
    print(f"  제거 후: {n_links2}개 링크 (-{n_links - n_links2}), "
          f"{n_joints2}개 조인트 (-{n_joints - n_joints2})")

    # --- Step 3: Allegro hand 병합 ---
    print(f"\n[Step 3] Allegro hand 전처리 및 병합")
    for side, urdf_path in [("right", ALLEGRO_RIGHT_URDF), ("left", ALLEGRO_LEFT_URDF)]:
        print(f"  [{side}]")
        elements = create_prefixed_hand_elements(urdf_path, side)
        for elem in elements:
            root.append(elem)
        print(f"    병합 완료: +{len(elements)}개 요소")

    # --- Step 4: 마운트 조인트 ---
    print(f"\n[Step 4] 마운트 조인트 추가")
    for side in ["left", "right"]:
        add_mount_joint(root, side)

    # --- Step 5: 메시 파일 복사 ---
    print(f"\n[Step 5] 메시 파일 복사")
    copy_all_meshes(work_dir)

    # --- Step 5.5: SdfPath 호환성 sanitize ---
    print(f"\n[Step 5.5] SdfPath 비호환 문자 sanitize (hyphens, dots → underscores)")
    n_fixed = sanitize_all_names(root)
    print(f"  {n_fixed}개 name 속성 수정")

    # --- Step 6: 통합 URDF 저장 ---
    total_links = len(root.findall("link"))
    total_joints = len(root.findall("joint"))
    print(f"\n[Step 6] 통합 URDF 저장")
    print(f"  총: {total_links}개 링크, {total_joints}개 조인트")

    root.set("name", "r1pro_allegro")
    combined_urdf = Path(work_dir) / "r1pro_allegro.urdf"
    tree.write(str(combined_urdf), xml_declaration=True, encoding="utf-8")
    urdf_size = combined_urdf.stat().st_size / 1024
    print(f"  저장: {combined_urdf.name} ({urdf_size:.0f} KB)")

    return combined_urdf


# ========================================================================
# Main
# ========================================================================

def main():
    parser = argparse.ArgumentParser(description="R1Pro + Allegro Hand USD 조립")
    parser.add_argument("--analyze-only", action="store_true",
                        help="URDF 병합 분석만 (Isaac Sim 불필요)")
    parser.add_argument("--preview", action="store_true",
                        help="빌드 후 시뮬레이터 미리보기")
    args = parser.parse_args()

    tmpdir = Path(tempfile.mkdtemp(prefix="r1pro_allegro_"))
    print(f"  작업 디렉토리: {tmpdir}")

    if args.analyze_only:
        # ---- URDF 병합만 (Isaac Sim 불필요) ----
        combined_urdf = build_combined_urdf(tmpdir)

        # 전체 링크/조인트 출력
        tree = ET.parse(str(combined_urdf))
        root = tree.getroot()
        print(f"\n{'=' * 70}")
        print(f"  === 전체 링크 ({len(root.findall('link'))}개) ===")
        for l in root.findall("link"):
            print(f"    {l.get('name')}")
        print(f"\n  === 전체 조인트 ({len(root.findall('joint'))}개) ===")
        for j in root.findall("joint"):
            p = j.find("parent").get("link") if j.find("parent") is not None else "?"
            c = j.find("child").get("link") if j.find("child") is not None else "?"
            print(f"    {j.get('name')} ({j.get('type')}): {p} → {c}")
        print(f"{'=' * 70}")

        shutil.rmtree(str(tmpdir), ignore_errors=True)
        return

    # ---- 전체 빌드 (Isaac Sim 필요) ----
    print("\n  Isaac Sim 초기화 중...")
    sys.path.insert(0, str(PROJECT_ROOT / "OmniGibson"))
    sys.path.insert(0, str(PROJECT_ROOT / "bddl3"))
    from omnigibson.macros import gm
    gm.USE_GPU_DYNAMICS = False
    gm.ENABLE_FLATCACHE = False
    gm.RENDER_VIEWER_CAMERA = False
    import omnigibson as og
    og.launch()
    print("  초기화 완료!\n")

    # Step 1-6: 통합 URDF 빌드
    combined_urdf = build_combined_urdf(tmpdir)

    # Step 7: URDF → USD 변환
    print(f"\n[Step 7] URDF → USD 변환")
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    og.sim.clear()
    prim_path = convert_urdf_to_usd(combined_urdf, OUTPUT_USD_PATH)

    # Step 8: 검증
    print(f"\n[Step 8] 출력 USD 검증")
    print(f"{'=' * 70}")
    ok = verify_output_usd(OUTPUT_USD_PATH)
    print(f"{'=' * 70}")

    # 정리
    shutil.rmtree(str(tmpdir), ignore_errors=True)

    if ok:
        print(f"\n  빌드 성공! → {OUTPUT_USD_PATH}")
    else:
        print(f"\n  빌드 완료 (검증 실패 항목 있음)")

    # 미리보기 (빌드 성공 시 자동)
    if ok:
        print(f"\n  시뮬레이터에 로드 중...")
        import omni.usd
        import omni.ui
        import omni.kit.viewport.window
        import omni.kit.app
        import carb
        from pxr import UsdGeom, Gf, UsdPhysics

        omni.usd.get_context().open_stage(str(OUTPUT_USD_PATH))
        for _ in range(10):
            og.sim.render()

        # --- Viewport 복원 (RENDER_VIEWER_CAMERA=False로 숨겨진 상태) ---
        vp_win = omni.ui.Workspace.get_window("Viewport")
        if vp_win is not None:
            vp_win.visible = True
            omni.kit.app.get_app().update()

        # 렌더링 설정 (inspect_r1pro.py set_high_quality_render와 동일)
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
        # RTX-Interactive (Path Tracing) 모드
        s.set_string("/rtx/rendermode", "PathTracing")
        s.set_int("/rtx/pathtracing/spp", 1)
        s.set_int("/rtx/pathtracing/totalSpp", 64)
        s.set_int("/rtx/pathtracing/maxBounces", 4)
        s.set_int("/rtx/pathtracing/maxSpecularAndTransmissionBounces", 4)

        # 카메라 생성
        preview_stage = omni.usd.get_context().get_stage()
        cam_path = "/World/preview_camera"
        if not preview_stage.GetPrimAtPath(cam_path).IsValid():
            UsdGeom.Camera.Define(preview_stage, cam_path)
        cam_prim = preview_stage.GetPrimAtPath(cam_path)
        xf = UsdGeom.Xformable(cam_prim)
        xf.ClearXformOpOrder()
        xf.AddTranslateOp().Set(Gf.Vec3d(1.5, -2.5, 2.0))
        xf.AddOrientOp().Set(Gf.Quatf(0.82, 0.35, 0.1, 0.44))
        UsdGeom.Camera(cam_prim).GetClippingRangeAttr().Set(Gf.Vec2f(0.01, 10000.0))
        UsdGeom.Camera(cam_prim).GetFocalLengthAttr().Set(18.0)

        # 뷰포트에 카메라 연결
        vps = list(omni.kit.viewport.window.get_viewport_window_instances())
        if vps:
            vps[0].viewport_api.set_active_camera(cam_path)

        # Ground plane
        gp = "/GroundPlane"
        if not preview_stage.GetPrimAtPath(gp).IsValid():
            ground = UsdGeom.Mesh.Define(preview_stage, gp)
            ground.GetPointsAttr().Set([
                Gf.Vec3f(-5, -5, 0), Gf.Vec3f(5, -5, 0),
                Gf.Vec3f(5, 5, 0), Gf.Vec3f(-5, 5, 0)])
            ground.GetFaceVertexCountsAttr().Set([4])
            ground.GetFaceVertexIndicesAttr().Set([0, 1, 2, 3])
            ground.GetNormalsAttr().Set([Gf.Vec3f(0, 0, 1)] * 4)
            UsdPhysics.CollisionAPI.Apply(ground.GetPrim())

        # 조명 추가
        light_path = "/World/preview_light"
        if not preview_stage.GetPrimAtPath(light_path).IsValid():
            light = UsdGeom.Xformable(
                preview_stage.DefinePrim(light_path, "SphereLight"))
            light.ClearXformOpOrder()
            light.AddTranslateOp().Set(Gf.Vec3d(0, -2, 3))
            preview_stage.GetPrimAtPath(light_path).GetAttribute("inputs:intensity").Set(30000.0)
            preview_stage.GetPrimAtPath(light_path).GetAttribute("inputs:radius").Set(0.1)

        print(f"  뷰포트 + 카메라 + 조명 설정 완료")

        view_sec = 120
        print(f"  시뮬레이터 창에서 확인하세요 ({view_sec}초, Ctrl+C로 종료)")
        try:
            for i in range(view_sec * 10):
                og.sim.render()
                time.sleep(0.1)
                if i % 100 == 0 and i > 0:
                    print(f"    남은: {view_sec - i // 10}초")
        except KeyboardInterrupt:
            print("\n  종료")

    og.shutdown()


if __name__ == "__main__":
    main()
