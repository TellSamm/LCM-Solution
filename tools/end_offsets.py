import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent))
from evaluate import run_bag, reference, load_all, ASSETS, ROOT
from tram_odometry.trackmap import TrackMap
from tram_odometry.estimator import Params
import numpy as np, pandas as pd
from concurrent.futures import ProcessPoolExecutor
M = TrackMap(ASSETS / "track_ring.json"); mk = M.markers; L = M.length
def term(k):
    for name in ["stop_S", "stop_T"]:
        if abs(((k - mk[name] + L / 2) % L) - L / 2) < 250: return name[-1]
    return "?"
def one(a):
    b, back = a
    r, O = run_bag(b, params=Params(offmap_start_back=back), verbose=True); ref = reference(load_all(b))
    out = dict(bag=b, back=back, init_dist=r["init_dist"], pos_mean=r["pos_mean"], final=r["final_err"])
    for lab, i in [("start", 5), ("mid", len(O) // 2), ("end", len(O) - 1)]:
        j = np.argmin(np.abs(ref.t.values - O.t.iloc[i])); k, dist = M.locate(ref.x.iloc[j], ref.y.iloc[j])
        out[lab + "_ds"] = round(((O.s.iloc[i] - k + L / 2) % L) - L / 2); out[lab + "_offmap"] = round(dist, 1); out[lab + "_term"] = term(k)
    return out
if __name__ == "__main__":
    summ = pd.read_csv(ROOT / "notes/bags_summary.csv"); bags = summ[(summ.m_cov >= 99) & (summ.dur_s > 600)].drop_duplicates("sig").bag.tolist()
    with ProcessPoolExecutor(10) as ex: rows = list(ex.map(one, [(b, bk) for bk in [0.0, 1.0] for b in bags], chunksize=6))
    R = pd.DataFrame(rows); pd.set_option("display.width", 250); pd.set_option("display.max_rows", 200)
    R0 = R[R.back == 0.0]
    print("=== back=0: along-track offset s_est - s_true (m) at start/mid/end, grouped by start terminus ===")
    print(R0.groupby("start_term")[["init_dist", "start_ds", "mid_ds", "end_ds", "end_offmap", "pos_mean", "final"]].median().round(1))
    print(R0.groupby("end_term")[["end_ds", "end_offmap", "final"]].median().round(1))
    print(R0.sort_values("init_dist", ascending=False)[["bag", "init_dist", "start_term", "start_ds", "mid_ds", "end_ds", "end_offmap", "end_term", "pos_mean", "final"]].head(20).to_string(index=False))
    R1 = R[R.back == 1.0]
    print("=== back=1 ==="); print(R1.groupby("start_term")[["mid_ds", "end_ds", "pos_mean", "final"]].median().round(1))
    R.to_csv(ROOT / "notes/end_offsets.csv", index=False)
