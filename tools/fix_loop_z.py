"""Replace the linear z inside the terminus loops by the median GNSS height profile (alt - 3.0 m antenna height,
tram 30618 whose GNSS height matches pathgraph z within 0.1 m on the tracks). Recomputes grade. In-place on track_ring.json."""
import sys, json
from pathlib import Path
import numpy as np, pandas as pd
from scipy.spatial import cKDTree
from scipy.ndimage import uniform_filter1d
sys.path.insert(0, str(Path(__file__).parent))
import build_map as bm
from cache import load_all
from paths import TABLES, ASSETS
P = ASSETS / "track_ring.json"
R = json.load(open(P)); rx, ry, rz = map(np.asarray, (R["x"], R["y"], R["z"])); mk = R["markers"]; n = len(rx); tree = cKDTree(np.c_[rx, ry])
summ = pd.read_csv(TABLES / "bags_summary.csv"); use = summ[(summ.m_cov >= 99) & (summ.dur_s > 600) & (summ.tram == 30618)].drop_duplicates("sig")
sums = np.zeros(n); cnt = np.zeros(n); vals = [[] for _ in range(n // 10 + 1)]
for b in use.bag:
    t, x, y, z, sp, wsp = bm.baselink_track(load_all(b)); d, k = tree.query(np.c_[x, y]); m = (d < 3) & (sp > 0.7)
    for kk, zz in zip(k[m], z[m]): vals[kk // 10].append(zz - rz[kk])
prof = np.array([np.median(v) if len(v) >= 20 else np.nan for v in vals])
prof = pd.Series(prof).interpolate(limit_direction="both").fillna(0.0).values
corr = np.repeat(prof, 10)[:n]
loop = ((np.arange(n) > mk["A_end"]) & (np.arange(n) < mk["B_start"])) | (np.arange(n) > mk["B_end"])
corr = np.where(loop, corr, 0.0); corr = uniform_filter1d(corr, 31, mode="nearest")
z_new = rz + corr
print(f"loop z correction: min {corr.min():+.2f} max {corr.max():+.2f} m; applied to {loop.sum()} loop points")
R["z"] = z_new.round(3).tolist(); R["grade"] = uniform_filter1d(np.gradient(z_new), 31, mode="nearest").round(5).tolist()
json.dump(R, open(P, "w")); print("track_ring.json updated")
