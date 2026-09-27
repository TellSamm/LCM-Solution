import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent))
from evaluate import run_bag, ROOT
from tram_odometry.estimator import Params
import pandas as pd
from concurrent.futures import ProcessPoolExecutor
def one(a):
    b, dec = a; r, _ = run_bag(b, params=Params(offmap_start_back=dec)); r["decay"] = dec; return r
if __name__ == "__main__":
    summ = pd.read_csv(ROOT / "notes/bags_summary.csv"); bags = summ[(summ.m_cov >= 99) & (summ.dur_s > 600)].drop_duplicates("sig").bag.tolist()
    with ProcessPoolExecutor(10) as ex: rows = list(ex.map(one, [(b, d) for d in [0.0, 0.7, 1.0, 1.3] for b in bags], chunksize=6))
    R = pd.DataFrame(rows); pd.set_option("display.width", 200)
    print(R.groupby("decay")[["pos_mean", "pos_p50", "final_err", "drift_pct"]].agg(["median", "mean"]).round(2).to_string())
    print(R[R.tram == "30618"].groupby("decay")[["pos_mean", "final_err"]].agg(["median", "mean"]).round(2).to_string())
    R[R.decay == 1.0].to_csv(ROOT / "notes/eval_v4.csv", index=False)
