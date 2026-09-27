"""Grid search of estimator parameters on the calibration bags (offline, parallel)."""
import sys, itertools
from pathlib import Path
import numpy as np, pandas as pd
from concurrent.futures import ProcessPoolExecutor
sys.path.insert(0, str(Path(__file__).parent))
from evaluate import run_bag
from paths import TABLES
from tram_odometry.estimator import Params

def one(args):
    bag, kw = args
    r, _ = run_bag(bag, params=Params(**kw)); r.update(kw); return r

if __name__ == "__main__":
    summ = pd.read_csv(TABLES / "bags_summary.csv")
    bags = summ[(summ.m_cov >= 99) & (summ.dur_s > 600)].drop_duplicates("sig").bag.tolist()
    grid = dict(q_accel=[0.3, 0.6, 1.0], r_wheel=[0.08, 0.15, 0.3], gate_sigma=[3.0, 4.0, 6.0])
    combos = [dict(zip(grid, v)) for v in itertools.product(*grid.values())]
    jobs = [(b, c) for c in combos for b in bags]
    with ProcessPoolExecutor(10) as ex: rows = list(ex.map(one, jobs, chunksize=8))
    R = pd.DataFrame(rows)
    keys = list(grid)
    agg = R.groupby(keys).agg(v_rmse=("v_rmse", "mean"), v_bias=("v_bias", lambda s: s.abs().mean()), pos_mean=("pos_mean", "mean"),
                              pos_p50=("pos_p50", "median"), final=("final_err", "median"), drift=("drift_pct", "mean"), rej=("rejected_pct", "mean")).round(3)
    pd.set_option("display.width", 200); print(agg.sort_values("v_rmse").to_string())
    R.to_csv(TABLES / "tune_grid.csv", index=False)
