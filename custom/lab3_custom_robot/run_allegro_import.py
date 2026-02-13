#!/usr/bin/env python3
"""
Allegro Hand URDF → USD 변환 스크립트
=====================================
import_custom_robot.py의 파라미터 전달 버그와
convert_urdf_to_usd의 후처리 버그를 우회하여
Isaac Sim URDF 임포터를 직접 호출합니다.

사용법:
    python custom/lab3_custom_robot/run_allegro_import.py --hand right
    python custom/lab3_custom_robot/run_allegro_import.py --hand left
    python custom/lab3_custom_robot/run_allegro_import.py --hand both

우회하는 문제들:
    1. import_custom_robot.py: dataset_root 파라미터 누락
    2. import_og_asset_from_urdf: convert_urdf_to_usd에 잘못된 키워드 전달
    3. allegro_right.urdf: link_12.0 메시 경로에 잘못된 접두사
    4. URDF 링크/조인트/메시 이름에 '.' 포함 → USD SdfPath 에러
    5. convert_urdf_to_usd 후처리: 메시 계층 재구성 시 child "mesh" prim 누락
"""

import argparse
import os
import shutil
import xml.etree.ElementTree as ET

# --- 설정 ---
ALLEGRO_URDF_DIR = "/home/ai5090/Documents/allegro_inhand_rotation/assets/allegro"
WORK_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "_urdf_work")

HAND_CONFIGS = {
    "right": {
        "urdf_path": os.path.join(ALLEGRO_URDF_DIR, "allegro_right.urdf"),
        "name": "allegro_hand_right",
    },
    "left": {
        "urdf_path": os.path.join(ALLEGRO_URDF_DIR, "allegro_left.urdf"),
        "name": "allegro_hand_left",
    },
}


def sanitize_urdf(hand_key):
    """
    원본 URDF를 작업 디렉터리에 복사하고 USD 호환 문제를 수정합니다.

    수정 사항:
    - 메시 파일명의 '.' → '_' 변환 (USD SdfPath 호환)
    - 메시 경로 접두사 수정 ("allegro/meshes/..." → "meshes/...")
    - 링크/조인트 이름의 '.' → '_' 변환

    Returns:
        str: 수정된 URDF 복사본의 절대 경로
    """
    cfg = HAND_CONFIGS[hand_key]
    urdf_src = cfg["urdf_path"]

    # 이전 작업 디렉터리 정리 후 생성
    if os.path.exists(WORK_DIR):
        shutil.rmtree(WORK_DIR)
    os.makedirs(WORK_DIR, exist_ok=True)

    # 메시 파일 복사 + 이름 변경 (파일명의 '.' → '_')
    meshes_src = os.path.join(ALLEGRO_URDF_DIR, "meshes", "allegro")
    meshes_dst = os.path.join(WORK_DIR, "meshes", "allegro")
    os.makedirs(meshes_dst, exist_ok=True)

    mesh_file_fixes = 0
    for fname in os.listdir(meshes_src):
        src_file = os.path.join(meshes_src, fname)
        if not os.path.isfile(src_file):
            continue
        name_part, ext = os.path.splitext(fname)
        new_fname = name_part.replace(".", "_") + ext
        dst_file = os.path.join(meshes_dst, new_fname)
        shutil.copy2(src_file, dst_file)
        if fname != new_fname:
            mesh_file_fixes += 1

    if mesh_file_fixes:
        print(f"  {mesh_file_fixes}개 메시 파일 이름 변경 완료")

    # URDF 파싱 및 수정
    tree = ET.parse(urdf_src)
    root = tree.getroot()

    # 메시 참조 수정
    for link in root.findall("link"):
        for section in link.findall("visual") + link.findall("collision"):
            mesh = section.find(".//mesh")
            if mesh is not None:
                fname = mesh.attrib.get("filename", "")
                # 접두사 수정
                if fname.startswith("allegro/meshes/"):
                    fname = fname[len("allegro/"):]
                # 파일명의 '.' → '_'
                parts = fname.split("/")
                name_part, ext = os.path.splitext(parts[-1])
                parts[-1] = name_part.replace(".", "_") + ext
                mesh.attrib["filename"] = "/".join(parts)

    # 링크/조인트 이름의 '.' → '_'
    name_fixes = 0
    for link in root.findall("link"):
        if "." in link.attrib["name"]:
            link.attrib["name"] = link.attrib["name"].replace(".", "_")
            name_fixes += 1

    for joint in root.findall("joint"):
        if "." in joint.attrib["name"]:
            joint.attrib["name"] = joint.attrib["name"].replace(".", "_")
            name_fixes += 1
        parent = joint.find("parent")
        if parent is not None and "." in parent.attrib.get("link", ""):
            parent.attrib["link"] = parent.attrib["link"].replace(".", "_")
        child = joint.find("child")
        if child is not None and "." in child.attrib.get("link", ""):
            child.attrib["link"] = child.attrib["link"].replace(".", "_")

    if name_fixes:
        print(f"  {name_fixes}개 링크/조인트 이름 USD 호환 변환 완료")

    urdf_dst = os.path.join(WORK_DIR, os.path.basename(urdf_src))
    tree.write(urdf_dst, xml_declaration=True, encoding="utf-8")
    return urdf_dst


def import_single_hand(hand_key, overwrite=True):
    """하나의 Allegro hand URDF를 USD로 변환합니다."""
    cfg = HAND_CONFIGS[hand_key]
    model_name = cfg["name"]

    print(f"\n{'='*70}")
    print(f"  Allegro {hand_key.upper()} hand 변환 시작")
    print(f"  원본 URDF: {cfg['urdf_path']}")
    print(f"  Model: {model_name}")
    print(f"{'='*70}\n")

    if not os.path.exists(cfg["urdf_path"]):
        print(f"ERROR: URDF 파일이 존재하지 않습니다: {cfg['urdf_path']}")
        return None

    # --- Step 0: URDF 전처리 ---
    print("[0/3] URDF 전처리 (SdfPath 호환)...")
    urdf_path = sanitize_urdf(hand_key)
    print(f"  작업 URDF: {urdf_path}")

    # --- Step 1: 충돌 근사 생성 ---
    print("\n[1/3] 충돌 근사(collision approximation) 생성 중...")
    from omnigibson.utils.asset_conversion_utils import get_collision_approximation_for_urdf
    get_collision_approximation_for_urdf(
        urdf_path=urdf_path,
        collision_method="convex",
        hull_count=32,
    )

    # --- Step 2: Isaac Sim 시작 및 URDF 직접 임포트 ---
    print("\n[2/3] Isaac Sim URDF 임포터로 직접 변환 중...")
    os.environ["OMNIGIBSON_HEADLESS"] = "1"

    import omnigibson as og
    from omnigibson.macros import gm
    import omnigibson.lazy as lazy

    gm.ENABLE_FLATCACHE = False
    og.launch()

    # 출력 경로 설정
    output_dir = os.path.join(
        gm.DATA_PATH, "custom_dataset", "objects", "robot", model_name, "usd"
    )
    os.makedirs(output_dir, exist_ok=True)
    usd_path = os.path.join(output_dir, f"{model_name}.usda")

    # URDF 임포트 설정 생성
    _, import_config = lazy.omni.kit.commands.execute("URDFCreateImportConfig")
    drive_mode = import_config.default_drive_type.__class__

    import_config.set_merge_fixed_joints(False)
    import_config.set_convex_decomp(False)
    import_config.set_fix_base(False)
    import_config.set_import_inertia_tensor(True)
    import_config.set_distance_scale(1.0)
    import_config.set_density(0.0)
    import_config.set_default_drive_type(drive_mode.JOINT_DRIVE_NONE)
    import_config.set_default_drive_strength(0.0)
    import_config.set_default_position_drive_damping(0.0)
    import_config.set_self_collision(False)
    import_config.set_up_vector(0, 0, 1)
    import_config.set_make_default_prim(True)
    import_config.set_create_physics_scene(False)

    # URDF 임포트 실행
    result = lazy.omni.kit.commands.execute(
        "URDFParseAndImportFile",
        urdf_path=urdf_path,
        import_config=import_config,
        dest_path=usd_path,
    )

    print(f"  임포트 결과: {result}")

    # --- Step 3: 스테이지 저장 ---
    print("\n[3/3] USD 저장 중...")
    stage = lazy.isaacsim.core.utils.stage.get_current_stage()
    stage.Export(usd_path)

    file_size = os.path.getsize(usd_path) / (1024 * 1024)
    print(f"\n{'='*70}")
    print(f"  변환 완료!")
    print(f"  USD 파일: {usd_path}")
    print(f"  파일 크기: {file_size:.1f} MB")
    print(f"{'='*70}\n")

    # 정상 종료
    og.shutdown()
    return usd_path


def main():
    parser = argparse.ArgumentParser(description="Allegro Hand URDF → USD 변환")
    parser.add_argument(
        "--hand",
        choices=["right", "left", "both"],
        default="both",
        help="변환할 손 (right, left, both)",
    )
    parser.add_argument(
        "--overwrite",
        action="store_true",
        default=True,
        help="기존 파일 덮어쓰기",
    )
    args = parser.parse_args()

    hands = ["right", "left"] if args.hand == "both" else [args.hand]
    results = {}

    for hand in hands:
        usd_path = import_single_hand(hand, overwrite=args.overwrite)
        results[hand] = usd_path

    print("\n" + "=" * 70)
    print("  전체 결과 요약")
    print("=" * 70)
    for hand, path in results.items():
        status = "성공" if path else "실패"
        print(f"  {hand}: {status}")
        if path:
            print(f"    → {path}")
    print()


if __name__ == "__main__":
    main()
