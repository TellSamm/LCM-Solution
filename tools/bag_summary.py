"""Step 2. Per-bag summary (duration, GNSS coverage, duplicates, sensor stats) -> tools/data/bags_summary.csv"""
import sys, hashlib
from pathlib import Path
import numpy as np, pandas as pd
sys.path.insert(0, str(Path(__file__).parent))
from cache import bags, load_all
from paths import TABLES

rows = []; sig = {}
for b in bags():
    d = load_all(b); f, r, c = d["front"], d["rear"], d["cmd"]
    h = hashlib.md5(np.round(f.v.values, 3).tobytes() + np.round(c.pos.values).astype(np.int8).tobytes()).hexdigest()[:10]
    dur = c.stamp.iloc[-1] - c.stamp.iloc[0]; t0 = c.stamp.iloc[0]
    fi = np.interp(r.stamp, f.stamp, f.v); mv = fi > 5
    row = dict(bag=b, tram=b[:5], sig=h, dur_s=round(dur), vmax_kmh=round(f.v.max(), 1), fr_diff_kmh=round(np.abs(fi[mv] - r.v[mv]).mean(), 2) if mv.any() else np.nan,
               notch_min=int(c.pos.min()), notch_max=int(c.pos.max()), f_repeat=round((np.diff(f.v) == 0).mean() * 100), start=pd.to_datetime(t0, unit="s").strftime("%m-%d %H:%M"))
    for src in ["m", "r"]:
        gf, g = d[src + "fix"], d[src + "vel"]
        if len(gf) == 0 or len(g) == 0:
            row.update({src + "_cov": 0, src + "_gap": np.nan, src + "_first": np.nan, src + "_dist": np.nan}); continue
        sp = np.hypot(g.vx, g.vy); have = np.histogram(gf.stamp, np.arange(t0, t0 + dur + 1, 1.0))[0] > 0
        row.update({src + "_cov": round(have.mean() * 100), src + "_gap": round(np.diff(gf.stamp).max(), 1), src + "_first": round(gf.stamp.iloc[0] - t0, 1), src + "_dist": round(np.trapezoid(sp, g.stamp))})
    rows.append(row)
df = pd.DataFrame(rows); TABLES.mkdir(exist_ok=True); df.to_csv(TABLES / "bags_summary.csv", index=False)
print(f"{len(df)} bags, {df.sig.nunique()} unique; written to {TABLES / 'bags_summary.csv'}")
