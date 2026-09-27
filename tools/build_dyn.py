"""Step 3. Merge GNSS speed/acceleration, controller notch and wheel speeds into a uniform 10 Hz table
for the calibration bags -> cache/dyn10hz.parquet"""
import sys
from pathlib import Path
import numpy as np, pandas as pd
from scipy.signal import savgol_filter
sys.path.insert(0, str(Path(__file__).parent))
from cache import load_all
from paths import TABLES, CACHE

summ = pd.read_csv(TABLES / "bags_summary.csv")
use = summ[(summ.m_cov >= 99) & (summ.dur_s > 600)].drop_duplicates("sig")
rows = []
for b in use.bag:
    d = load_all(b); g, c, f, r = d["mvel"], d["cmd"], d["front"], d["rear"]
    t = g.stamp.values; v = np.hypot(g.vx.values, g.vy.values)
    tu = np.arange(t[0], t[-1], 0.1); vu = np.interp(tu, t, v); a = savgol_filter(vu, 15, 2, deriv=1, delta=0.1)
    notch = c.pos.values[np.clip(np.searchsorted(c.stamp.values, tu, side="right") - 1, 0, len(c) - 1)]
    fw = np.interp(tu, f.stamp.values, f.v.values) / 3.6; rw = np.interp(tu, r.stamp.values, r.v.values) / 3.6
    rows.append(pd.DataFrame(dict(bag=b, tram=b[:5], t=tu, v=vu, a=a, notch=notch, fw=fw, rw=rw)))
D = pd.concat(rows, ignore_index=True); D.to_parquet(CACHE / "dyn10hz.parquet"); print(f"{len(use)} bags, {len(D)} rows -> {CACHE / 'dyn10hz.parquet'}")
