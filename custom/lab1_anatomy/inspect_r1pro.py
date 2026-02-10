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


if __name__ == "__main__":
    print("=" * 60)
    print("  Lab 1: R1Pro 로봇 구조 분석")
    print("=" * 60)

    # 모든 로봇 나열
    list_all_robots()

    # R1Pro 상세 분석
    r1pro_def = load_robot_definition("r1pro")
    inspect_robot("R1Pro", r1pro_def)

    # R1 vs R1Pro 비교
    compare_r1_r1pro()

    print("\n\nLab 1 완료!")
