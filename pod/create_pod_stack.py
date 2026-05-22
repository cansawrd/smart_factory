"""
Pod 4단 적재 USD 생성 스크립트
원본 pod.usda를 reference로 참조해 수직으로 4개 쌓는다.
출력: pod_stack_4.usda (이 스크립트와 같은 디렉토리)
"""
import sys
import os
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

from pxr import Usd, UsdGeom, UsdPhysics, Gf
import omni.usd

# ─── 파라미터 ────────────────────────────────────────────────────────────────

POD_TOTAL_HEIGHT = 0.265   # 다리 0.245 + 플랫폼 0.020
NUM_STAGES       = 4

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
POD_USD_PATH   = os.path.join(SCRIPT_DIR, "pod.usda")
OUTPUT_PATH    = os.path.join(SCRIPT_DIR, "pod_stack_4.usda")

# ─── 빌드 ────────────────────────────────────────────────────────────────────

stage = omni.usd.get_context().get_stage()

# 루트 Xform
UsdGeom.Xform.Define(stage, "/PodStack")

sys.stderr.write(f"[Stack] pod.usda: {POD_USD_PATH}\n")
sys.stderr.write(f"[Stack] stages: {NUM_STAGES}\n\n")

for i in range(NUM_STAGES):
    prim_path = f"/PodStack/Pod_{i}"
    z_offset  = i * POD_TOTAL_HEIGHT

    prim = stage.DefinePrim(prim_path, "Xform")
    prim.GetReferences().AddReference(POD_USD_PATH, "/Pod")

    xf = UsdGeom.Xformable(prim)
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, z_offset))

    platform_bottom = z_offset + 0.245
    platform_top    = z_offset + 0.265

    sys.stderr.write(f"[Stack]   Pod_{i}: Z offset={z_offset:.3f}m  "
                     f"platform {platform_bottom:.3f}~{platform_top:.3f}m\n")

total_h = NUM_STAGES * POD_TOTAL_HEIGHT
sys.stderr.write(f"\n[Stack] 전체 적재 높이: {total_h:.3f} m\n")

stage.SetDefaultPrim(stage.GetPrimAtPath("/PodStack"))
stage.Export(OUTPUT_PATH)
sys.stderr.write(f"[Stack] Saved → {OUTPUT_PATH}\n")

simulation_app.close()
