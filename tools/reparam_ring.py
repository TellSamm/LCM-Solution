"""Re-parameterize the ring map by wheel odometry (offline, allowed).
For every training bag: true base_link position (GNSS, both antennas) -> ring index k(t) with continuity;
wheel-integrated travelled distance D(t) (front bogie, calibrated scale). In 20 m bins of k the local ratio
f = dD/dk is estimated over all bags; the ring is resampled so that 1 index == 1 m of *wheel travel*.
This removes junction/loop artefacts and makes the map consistent with what the estimator integrates."""
import sys, json
from pathlib import Path
import numpy as np, pandas as pd, yaml
from scipy.spatial import cKDTree
sys.path.insert(0, str(Path(__file__).parent))
import build_map as bm
from cache import load_all
from paths import TABLES, ASSETS
BIN = 20

def main():
    R = json.load(open(ASSETS / "track_ring.json")); rx, ry, rz = map(np.asarray, (R["x"], R["y"], R["z"])); n = len(rx); tree = cKDTree(np.c_[rx, ry])
    summ = pd.read_csv(TABLES / "bags_summary.csv"); use = summ[(summ.m_cov >= 99) & (summ.dur_s > 600)].drop_duplicates("sig")
    sumD = np.zeros(n // BIN + 1); sumK = np.zeros(n // BIN + 1); nb = np.zeros(n // BIN + 1)
    for b in use.bag:
        cal = yaml.safe_load(open(ASSETS / f"calib_{b[:5]}.yaml")); scale = cal["wheel_scale_front"]
        t, x, y, z, sp, wsp = bm.baselink_track(load_all(b)); wsp = wsp * scale
        D = np.r_[0, np.cumsum(0.5 * (wsp[1:] + wsp[:-1]) * np.diff(t))]
        d, k = tree.query(np.c_[x, y], k=8)   # candidates
        kprev = None; ks = np.full(len(x), -1)
        for i in range(len(x)):
            if d[i, 0] > 4: kprev = None; continue
            cands = [(kk, dd) for kk, dd in zip(k[i], d[i]) if dd < 4]
            if kprev is None: ks[i] = cands[0][0]
            else:
                exp = kprev + (D[i] - D[iprev]); best = min(cands, key=lambda c: abs(((c[0] - exp + n / 2) % n) - n / 2) + 0.5 * c[1])
                if abs(((best[0] - exp + n / 2) % n) - n / 2) > 30: kprev = None; continue
                ks[i] = best[0]
            kprev = ks[i]; iprev = i
        # increments over spans of >= 15 m of travel (ring index is integer: per-fix increments would be quantized)
        j = 0
        for i in range(len(x)):
            if ks[i] < 0: continue
            while j < len(x) and D[j] - D[i] < 15: j += 1
            if j >= len(x): break
            if ks[j] < 0: continue
            dk = ((ks[j] - ks[i] + n / 2) % n) - n / 2; dD = D[j] - D[i]
            if dk <= 5 or dk > 40: continue
            bidx = ks[i] // BIN; sumD[bidx] += dD; sumK[bidx] += dk; nb[bidx] += 1
    f = np.where(sumK > 50, sumD / np.maximum(sumK, 1e-9), np.nan)
    f = pd.Series(f).interpolate(limit_direction="both").clip(0.85, 1.15).values
    # pathgraph tracks are authoritative (1 m steps): keep f = 1 there, correct only inside the loops (with smoothing)
    mk0 = R["markers"]; bins = np.arange(len(f)) * BIN
    in_loop = ((bins > mk0["A_end"] - BIN) & (bins < mk0["B_start"])) | (bins > mk0["B_end"] - BIN)
    GAIN = float(sys.argv[1]) if len(sys.argv) > 1 else 0.5
    f = np.where(in_loop, 1.0 + GAIN * (f - 1.0), 1.0)
    f = pd.Series(f).rolling(3, center=True, min_periods=1).mean().values
    print(f"bins: {len(f)}, ratio dD/dk: median {np.nanmedian(f):.4f}, min {f.min():.3f}, max {f.max():.3f}")
    bad = np.where(np.abs(f - 1) > 0.03)[0]; print("bins with |f-1|>3%:", [(int(i * BIN), round(float(f[i]), 3)) for i in bad][:40])
    # new arc length per index
    fi = np.repeat(f, BIN)[:n]
    s_new = np.r_[0, np.cumsum(fi[:-1])]; Lnew = s_new[-1] + fi[-1]
    su = np.arange(0, Lnew, 1.0)
    X = np.interp(su, s_new, rx); Y = np.interp(su, s_new, ry); Z = np.interp(su, s_new, rz)
    m = len(su); dx = np.gradient(X); dy = np.gradient(Y); heading = np.arctan2(dy, dx)
    from scipy.ndimage import uniform_filter1d
    dh = np.diff(np.unwrap(heading)); curv = uniform_filter1d(np.r_[dh, dh[-1]], 15, mode="nearest"); grade = uniform_filter1d(np.gradient(Z), 31, mode="nearest")
    tree2 = cKDTree(np.c_[X, Y]); mk = {k: int(tree2.query([rx[v], ry[v]])[1]) for k, v in R["markers"].items()}
    out = dict(R); out.update(length=float(m), markers=mk, x=X.round(3).tolist(), y=Y.round(3).tolist(), z=Z.round(3).tolist(),
               heading=heading.round(5).tolist(), curv=curv.round(6).tolist(), grade=grade.round(5).tolist(), reparam="wheel_odometry")
    json.dump(out, open(ASSETS / "track_ring.json", "w"))
    print(f"ring re-parameterized: {n} -> {m} pts; markers {mk}")

if __name__ == "__main__":
    main()
