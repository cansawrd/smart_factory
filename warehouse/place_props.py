"""
warehouse_v4.usd에 prop을 배치하고 저장하는 스크립트

배치 방식 3가지:
  GRID    — 행/열 그리드로 자동 배치 (선반 줄 등)
  SCATTER — 지정 구역 안에 랜덤 산포 (박스, 팔레트 등)
  FIXED   — 좌표 직접 지정 (특수 장비 등)

Usage:
    isaac-python place_props.py
"""
import sys
import random
from pathlib import Path
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

from pxr import Usd, UsdGeom, Gf

# ── 경로 설정 ─────────────────────────────────────────────────────────────────
BASE_DIR = Path(__file__).resolve().parent

WAREHOUSE_USD = str(BASE_DIR / "warehouse_v5.usd")
OUTPUT_USD    = str(BASE_DIR / "warehouse_v5_props.usd")

ASSET_DIR = str(BASE_DIR.parent / "pod")  # prop USD 기본 경로

# ── GRID 배치 설정 ────────────────────────────────────────────────────────────
# 행/열 간격으로 자동 배치. 선반·기둥 줄 배치에 적합.
# origin: 그리드 시작점 (좌하단), rot_z: 전체 회전(도)
GRID_PROPS = [
    # 강남 섹터
    {
        "usd":     f"{ASSET_DIR}/pod_stack_4.usda",
        "origin":  (-3.0, 8.0, 0.0),   # 좌하단 시작점
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
# 지정 구역 안에 count개를 랜덤 산포. 박스·팔레트 등에 적합.
# zone: (x_min, x_max, y_min, y_max), seed: 재현성 위한 랜덤 시드
SCATTER_PROPS = [
    # {
    #     "usd":   f"{ASSET_DIR}/box.usd",
    #     "zone":  (-70.0, -30.0, -50.0, 50.0),  # (x_min, x_max, y_min, y_max)
    #     "count": 30,
    #     "z":     0.0,
    #     "rot_z_range": (0.0, 360.0),            # 랜덤 회전 범위 (도)
    #     "scale": (1.0, 1.0, 1.0),
    #     "name":  "box",
    #     "seed":  42,
    # },
]

# ── FIXED 배치 설정 ──────────────────────────────────────────────────────────
# 정확한 좌표가 필요한 특수 장비 등
FIXED_PROPS = [
    # {
    #     "usd":   f"{ASSET_DIR}/charger.usd",
    #     "pos":   (70.0, 0.0, 0.0),
    #     "rot":   (0.0, 0.0, 180.0),
    #     "scale": (1.0, 1.0, 1.0),
    #     "name":  "charger_main",
    # },
]


# ── 배치 함수 ─────────────────────────────────────────────────────────────────

def _add_prim(stage, prim_path, usd_path, pos, rot_xyz, scale):
    prim = stage.DefinePrim(prim_path, "Xform")
    prim.GetReferences().AddReference(usd_path)
    xf = UsdGeom.Xformable(prim)
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(*pos))
    xf.AddRotateXYZOp().Set(Gf.Vec3f(*rot_xyz))
    xf.AddScaleOp().Set(Gf.Vec3f(*scale))


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
    missing_assets = [
        path
        for path in [WAREHOUSE_USD, *[cfg["usd"] for cfg in GRID_PROPS + SCATTER_PROPS + FIXED_PROPS]]
        if not Path(path).exists()
    ]
    if missing_assets:
        for path in missing_assets:
            sys.stderr.write(f"[error]   파일 없음: {path}\n")
        raise FileNotFoundError("필요한 USD asset을 찾을 수 없습니다.")

    stage = Usd.Stage.CreateNew(OUTPUT_USD)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)

    # warehouse_v4를 sublayer로 로드
    # reference 방식은 prim 경로가 중첩되고 light가 소실되는 문제가 있음
    stage.GetRootLayer().subLayerPaths.append(WAREHOUSE_USD)
    sys.stderr.write(f"[load]    창고(sublayer): {WAREHOUSE_USD}\n")

    # default prim 설정 — 없으면 Isaac Sim에서 빨간색으로 표시됨
    world = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(world.GetPrim())

    UsdGeom.Xform.Define(stage, "/World/Props")

    counter = [0]  # prim 인덱스 공유 카운터

    for cfg in GRID_PROPS:
        place_grid(stage, cfg, counter)

    for cfg in SCATTER_PROPS:
        place_scatter(stage, cfg, counter)

    for cfg in FIXED_PROPS:
        place_fixed(stage, cfg, counter)

    stage.GetRootLayer().Save()
    sys.stderr.write(f"\n[done]    총 {counter[0]}개 prop 배치\n")
    sys.stderr.write(f"[done]    저장: {OUTPUT_USD}\n")


main()
simulation_app.close()
