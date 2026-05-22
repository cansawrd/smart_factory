"""
warehouse1.usd에 prop + iw_hub 로봇을 배치하고 저장하는 스크립트

배치 방식:
  GRID    — 행/열 그리드로 자동 배치
  SCATTER — 지정 구역 안에 랜덤 산포
  FIXED   — 좌표 직접 지정
  ROBOTS  — iw_hub 로봇 (ActionGraph 포함)

Usage:
    isaac-python place_props_v1.py
"""
import sys
import random
from pathlib import Path
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

from pxr import Usd, UsdGeom, Gf, Sdf

# ── 경로 설정 ─────────────────────────────────────────────────────────────────
REPO_ROOT = Path(__file__).resolve().parents[1]
WAREHOUSE_DIR = REPO_ROOT / "warehouse"

WAREHOUSE_USD = str(WAREHOUSE_DIR / "warehouse1.usd")
OUTPUT_USD    = str(WAREHOUSE_DIR / "warehouse1_props.usd")

ASSET_DIR    = str(REPO_ROOT / "pod")
IW_HUB_USD   = str(REPO_ROOT / "Iw_hub" / "iw_hub_v1.usda")

# ── ROBOT 배치 설정 ───────────────────────────────────────────────────────────
# iw_hub_v1.usda 안의 로봇 prim 경로 (ActionGraph 포함)
# name: stage 내 prim 이름 (고유해야 함)
# pos: (x, y) 위치, rot_z: Z축 회전(도)
ROBOT_PROPS = [
    {
        "name":  "iw_hub_01",
        "pos":   (0.0, 0.0),
        "rot_z": 0.0,
    },
    {
        "name":  "iw_hub_02",
        "pos":   (3.0, 0.0),
        "rot_z": 90.0,
    },
]

# ── GRID 배치 설정 ────────────────────────────────────────────────────────────
GRID_PROPS = [
    # 강남 섹터
    {
        "usd":     f"{ASSET_DIR}/pod_stack_4.usda",
        "origin":  (-3.0, 8.0, 0.0),
        "rows":    2, "cols": 4,
        "row_gap": 3.5, "col_gap": 2.5,
        "name":    "pod_gangnam",
    },
    # 강서 섹터
    {
        "usd":     f"{ASSET_DIR}/pod_stack_4.usda",
        "origin":  (-3.0, -2.0, 0.0),
        "rows":    2, "cols": 4,
        "row_gap": 3.5, "col_gap": 2.5,
        "name":    "pod_gangseo",
    },
    # 경기 섹터
    {
        "usd":     f"{ASSET_DIR}/pod_stack_4.usda",
        "origin":  (-3.0, -12.0, 0.0),
        "rows":    2, "cols": 4,
        "row_gap": 3.5, "col_gap": 2.5,
        "name":    "pod_gyeonggi",
    },
]

# ── SCATTER 배치 설정 ─────────────────────────────────────────────────────────
SCATTER_PROPS = []

# ── FIXED 배치 설정 ──────────────────────────────────────────────────────────
FIXED_PROPS = []


# ── 배치 함수 ─────────────────────────────────────────────────────────────────

def _add_prim(stage, prim_path, usd_path, pos, rot_xyz, scale):
    prim = stage.DefinePrim(prim_path, "Xform")
    prim.GetReferences().AddReference(usd_path)
    xf = UsdGeom.Xformable(prim)
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(*pos))
    xf.AddRotateXYZOp().Set(Gf.Vec3f(*rot_xyz))
    xf.AddScaleOp().Set(Gf.Vec3f(*scale))


def _set_attr(stage, prim_path: str, attr_name: str, value):
    attr = stage.GetPrimAtPath(prim_path).GetAttribute(attr_name)
    if attr:
        attr.Set(value)


def _set_targets(stage, prim_path: str, rel_name: str, targets: list[str]):
    rel = stage.GetPrimAtPath(prim_path).GetRelationship(rel_name)
    if rel:
        rel.SetTargets([Sdf.Path(target) for target in targets])


def configure_robot_action_graph(stage, robot_root: str, robot_name: str):
    robot_prim = f"{robot_root}/iw_hub_sensors"
    graph = f"{robot_prim}/ActionGraph"

    _set_attr(stage, f"{graph}/ros2_subscribe_twist", "inputs:topicName", f"/{robot_name}/cmd_vel")
    _set_attr(stage, f"{graph}/ros2_subscribe_joint_state", "inputs:topicName", f"/{robot_name}/lift_cmd")

    _set_targets(stage, f"{graph}/articulation_controller", "inputs:targetPrim", [robot_prim])
    _set_targets(stage, f"{graph}/articulation_controller_01", "inputs:targetPrim", [robot_prim])
    _set_targets(stage, f"{graph}/isaac_compute_odometry_node", "inputs:chassisPrim", [robot_prim])
    _set_targets(stage, f"{graph}/ros2_publish_transform_tree", "inputs:targetPrims", [robot_prim])
    _set_attr(stage, f"{graph}/ros2_publish_transform_tree", "inputs:topicName", f"/{robot_name}/tf")

    _set_attr(stage, f"{graph}/ros2_publish_odometry", "inputs:topicName", f"/{robot_name}/odom")
    _set_attr(stage, f"{graph}/ros2_publish_odometry", "inputs:chassisFrameId", f"{robot_name}/base_link")
    _set_attr(stage, f"{graph}/ros2_publish_odometry", "inputs:odomFrameId", f"{robot_name}/odom")

    _set_targets(stage, f"{graph}/read_front_lidar", "inputs:lidarPrim", [f"{robot_prim}/front_2d_lidar"])
    _set_targets(stage, f"{graph}/read_back_lidar", "inputs:lidarPrim", [f"{robot_prim}/back_2d_lidar"])
    _set_attr(stage, f"{graph}/pub_front_lidar", "inputs:topicName", f"/{robot_name}/front_2d_lidar/scan")
    _set_attr(stage, f"{graph}/pub_front_lidar", "inputs:frameId", f"{robot_name}/front_2d_lidar")
    _set_attr(stage, f"{graph}/pub_back_lidar", "inputs:topicName", f"/{robot_name}/back_2d_lidar/scan")
    _set_attr(stage, f"{graph}/pub_back_lidar", "inputs:frameId", f"{robot_name}/back_2d_lidar")


def place_robots(stage, cfgs: list):
    """
    iw_hub_v1.usda를 참조해 로봇을 배치한다.
    iw_hub_v1.usda의 defaultPrim("World") 안에 iw_hub_sensors가 있고
    ActionGraph도 함께 포함된다.

    배치 경로: /World/Robots/<name>/iw_hub_sensors
    """
    UsdGeom.Xform.Define(stage, "/World/Robots")

    for cfg in cfgs:
        name   = cfg["name"]
        x, y   = cfg["pos"]
        rot_z  = cfg.get("rot_z", 0.0)

        # 로봇 컨테이너 prim — iw_hub_v1.usda의 defaultPrim("World") 내용이 여기 붙음
        prim_path = f"/World/Robots/{name}"
        prim = stage.DefinePrim(prim_path, "Xform")
        prim.GetReferences().AddReference(IW_HUB_USD)

        xf = UsdGeom.Xformable(prim)
        xf.ClearXformOpOrder()
        xf.AddTranslateOp().Set(Gf.Vec3d(x, y, 0.0))
        xf.AddRotateXYZOp().Set(Gf.Vec3f(0.0, 0.0, rot_z))
        xf.AddScaleOp().Set(Gf.Vec3f(1.0, 1.0, 1.0))

        configure_robot_action_graph(stage, prim_path, name)
        sys.stderr.write(f"[robot]   {name}: pos=({x}, {y}), rot_z={rot_z}\n")


def place_grid(stage, cfg: dict, counter: list):
    name    = cfg["name"]
    ox, oy, oz = cfg["origin"]
    for r in range(cfg["rows"]):
        for c in range(cfg["cols"]):
            x = ox + c * cfg["col_gap"]
            y = oy + r * cfg["row_gap"]
            idx = counter[0]
            counter[0] += 1
            path = f"/World/Props/{name}_{idx:03d}"
            _add_prim(stage, path, cfg["usd"],
                      (x, y, oz),
                      (0.0, 0.0, cfg.get("rot_z", 0.0)),
                      cfg.get("scale", (1.0, 1.0, 1.0)))
    total = cfg["rows"] * cfg["cols"]
    sys.stderr.write(f"[grid]    {name}: {total}개 배치\n")


def place_scatter(stage, cfg: dict, counter: list):
    name = cfg["name"]
    x_min, x_max, y_min, y_max = cfg["zone"]
    rz_lo, rz_hi = cfg.get("rot_z_range", (0.0, 360.0))
    rng = random.Random(cfg.get("seed", 0))

    for _ in range(cfg["count"]):
        x  = rng.uniform(x_min, x_max)
        y  = rng.uniform(y_min, y_max)
        rz = rng.uniform(rz_lo, rz_hi)
        idx = counter[0]
        counter[0] += 1
        path = f"/World/Props/{name}_{idx:03d}"
        _add_prim(stage, path, cfg["usd"],
                  (x, y, cfg.get("z", 0.0)),
                  (0.0, 0.0, rz),
                  cfg.get("scale", (1.0, 1.0, 1.0)))

    sys.stderr.write(f"[scatter] {name}: {cfg['count']}개 배치\n")


def place_fixed(stage, cfg: dict, counter: list):
    idx  = counter[0]
    counter[0] += 1
    name = cfg.get("name", f"prop_{idx:03d}")
    path = f"/World/Props/{name}"
    _add_prim(stage, path, cfg["usd"],
              cfg.get("pos",   (0.0, 0.0, 0.0)),
              cfg.get("rot",   (0.0, 0.0, 0.0)),
              cfg.get("scale", (1.0, 1.0, 1.0)))
    sys.stderr.write(f"[fixed]   {name}: pos={cfg.get('pos')}\n")


# ── 메인 ─────────────────────────────────────────────────────────────────────

def main():
    stage = Usd.Stage.CreateNew(OUTPUT_USD)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)

    stage.GetRootLayer().subLayerPaths.append(WAREHOUSE_USD)
    sys.stderr.write(f"[load]    창고(sublayer): {WAREHOUSE_USD}\n")

    world = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(world.GetPrim())

    UsdGeom.Xform.Define(stage, "/World/Props")

    # 로봇 배치 (ActionGraph 포함)
    place_robots(stage, ROBOT_PROPS)

    counter = [0]

    for cfg in GRID_PROPS:
        place_grid(stage, cfg, counter)

    for cfg in SCATTER_PROPS:
        place_scatter(stage, cfg, counter)

    for cfg in FIXED_PROPS:
        place_fixed(stage, cfg, counter)

    stage.GetRootLayer().Save()
    sys.stderr.write(f"\n[done]    로봇 {len(ROBOT_PROPS)}대, prop {counter[0]}개 배치\n")
    sys.stderr.write(f"[done]    저장: {OUTPUT_USD}\n")


main()
simulation_app.close()
