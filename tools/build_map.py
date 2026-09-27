"""Build a closed ring map (A track + Tallinskaya loop + B track + Shchukinskaya loop) from the
provided pathgraph JSONs and GNSS base_link tracks (offline, allowed by the rules).

Loop geometry: for every training bag the off-map head (before joining the main track) and tail
(after leaving it) are extracted; the arc-length parameter is the integral of GNSS speed (robust to
GNSS wander), xy are resampled every 1 m along it. The head/tail pieces that best cover the other
bags' points (most common branch) are chosen and spliced at the terminus stop.
Output: ros2_ws/src/tram_odometry/assets/track_ring.json
"""
import sys, json
from pathlib import Path
import numpy as np, pandas as pd
from scipy.ndimage import uniform_filter1d
from scipy.spatial import cKDTree
sys.path.insert(0, str(Path(__file__).parent))
from cache import load_all
from paths import TABLES, ASSETS, REPO
from pyproj import Transformer
TR = Transformer.from_crs("EPSG:4326", "EPSG:32637", always_xy=True)
X0, Y0 = 300000.0, 6100000.0
ANT_M, ANT_R = -9.873, 2.563
FR = ANT_R / (ANT_R - ANT_M)
DOCS = REPO / "tools" / "data"      # pathgraph JSONs from the organizers
OUT = ASSETS / "track_ring.json"
STOPS = {"S": np.array([103634.0, 86057.0]), "T": np.array([99015.0, 84949.0])}

def load_path(name):
    P = json.load(open(DOCS / name))["points"]
    return np.array([[p["x"], p["y"], p["z"]] for p in P])

def baselink_track(d):
    """t, x, y, z, speed of base_link (from master+rover antennas; master+velocity heading fallback)."""
    m, r = d["mfix"], d["rfix"]
    me, mn = TR.transform(m.lon.values, m.lat.values)
    vel = d["mvel"] if len(d["mvel"]) else d["rvel"]
    sp = np.interp(m.stamp, vel.stamp, np.hypot(vel.vx, vel.vy))
    # wheel speed (front bogie, km/h -> m/s) for the travel-distance parameter: exactly zero at standstill,
    # unlike |GNSS velocity| which is positively biased by noise while crawling/standing
    f = d["front"]; wsp = np.interp(m.stamp, f.stamp, f.v) / 3.6
    if len(r) > 10:
        re, rn = TR.transform(r.lon.values, r.lat.values)
        rx = np.interp(m.stamp, r.stamp, re); ry = np.interp(m.stamp, r.stamp, rn)
        x = rx + (me - rx) * FR; y = ry + (mn - ry) * FR
    else:
        vx = np.interp(m.stamp, vel.stamp, vel.vx); vy = np.interp(m.stamp, vel.stamp, vel.vy)
        hx = np.where(sp > 0.5, vx / np.maximum(sp, 1e-6), np.nan); hy = np.where(sp > 0.5, vy / np.maximum(sp, 1e-6), np.nan)
        hx = pd.Series(hx).ffill().bfill().values; hy = pd.Series(hy).ffill().bfill().values
        x = me - ANT_M * hx; y = mn - ANT_M * hy
    return m.stamp.values, x - X0, y - Y0, m.alt.values - 3.0, sp, wsp

def resample_by_travel(t, xy, sp, step=1.0):
    """Resample xy at 1 m of *travelled distance* (integral of wheel speed; increments below 0.3 m/s zeroed)."""
    v = np.where(sp > 0.3, sp, 0.0)
    s = np.r_[0, np.cumsum(0.5 * (v[1:] + v[:-1]) * np.diff(t))]
    # keep only strictly increasing s (drop standing samples)
    keep = np.r_[True, np.diff(s) > 0.05]
    s, xy = s[keep], xy[keep]
    su = np.arange(0, s[-1], step)
    return np.c_[np.interp(su, s, xy[:, 0]), np.interp(su, s, xy[:, 1]), np.interp(su, s, xy[:, 2])]

def resample_uniform(P, step=1.0):
    seg = np.hypot(*np.diff(P[:, :2], axis=0).T); s = np.r_[0, np.cumsum(seg)]
    su = np.arange(0, s[-1], step)
    return np.c_[[np.interp(su, s, P[:, k]) for k in range(P.shape[1])]].T

def smooth_xy(P, w=7):
    Q = P.copy(); Q[:, 0] = uniform_filter1d(P[:, 0], w, mode="nearest"); Q[:, 1] = uniform_filter1d(P[:, 1], w, mode="nearest"); return Q

def main():
    PA, PB = load_path("щукинская - таллинская.json"), load_path("таллинская - щукинская.json")
    treeA, treeB = cKDTree(PA[:, :2]), cKDTree(PB[:, :2])
    summ = pd.read_csv(TABLES / "bags_summary.csv")
    use = summ[(summ.m_cov >= 99) & (summ.dur_s > 600)].drop_duplicates("sig")
    pieces = {"S_tail": [], "S_head": [], "T_tail": [], "T_head": []}
    offmap_pts = {"S": [], "T": []}
    for b in use.bag:
        t, x, y, z, sp, wsp = baselink_track(load_all(b)); P = np.c_[x, y, z]
        dA, _ = treeA.query(P[:, :2]); dB, _ = treeB.query(P[:, :2])
        dirn = "A" if (dA < 3).mean() > (dB < 3).mean() else "B"
        on = (dA if dirn == "A" else dB) < 3; idx = np.where(on)[0]
        if len(idx) < 100: continue
        h = slice(0, idx[0] + 1); tl = slice(idx[-1], len(P))
        tree_dir = treeA if dirn == "A" else treeB
        # A: head at S, tail at T ; B: head at T, tail at S
        hk, tk = ("S", "T") if dirn == "A" else ("T", "S")
        for key, sl, term, side in [(f"{hk}_head", h, hk, "head"), (f"{tk}_tail", tl, tk, "tail")]:
            Q = resample_by_travel(t[sl], P[sl], wsp[sl])
            if len(Q) < 20 or len(Q) > 1500: continue
            free = Q[0, :2] if side == "head" else Q[-1, :2]
            if np.hypot(*(free - STOPS[term])) > 40: continue     # must start/end at the terminus stop
            # index on the pathgraph track where this piece attaches (head: its last point; tail: its first point)
            attach = int(tree_dir.query(Q[-1, :2] if side == "head" else Q[0, :2])[1])
            pieces[key].append((b, Q, attach))
            offmap_pts[term].append(P[sl][sp[sl] > 0.7][:, :2])
    for k, v in pieces.items(): print(f"{k}: {len(v)} candidate pieces")
    def best(key, term):
        pts = np.vstack(offmap_pts[term]); tree_pts = cKDTree(pts)
        scores = []
        for b, Q, attach in pieces[key]:
            # coverage: share of all off-map points (this terminus) within 3 m of this piece
            d, _ = cKDTree(Q[:, :2]).query(pts); cov = (d < 3).mean()
            # self-consistency: share of piece points within 3 m of at least 5 other points
            cnt = tree_pts.query_ball_point(Q[:, :2], 3.0, return_length=True); cons = (cnt >= 5).mean()
            scores.append((cov * cons, cov, cons, b, Q, attach))
        scores.sort(key=lambda s: -s[0]); s = scores[0]
        print(f"  {key}: best {s[3]} len {len(s[4])} m, coverage {s[1]:.2f}, consistency {s[2]:.2f}, attaches at track index {s[5]}")
        return s[4], s[5]
    def make_loop(term):
        (T, aT), (H, aH) = best(f"{term}_tail", term), best(f"{term}_head", term)
        d = np.hypot(H[:, 0] - T[-1, 0], H[:, 1] - T[-1, 1]); j = int(np.argmin(d))
        print(f"  {term} splice gap {d[j]:.1f} m at H[{j}]")
        return smooth_xy(np.vstack([T, H[j:]]), 5), aT, aH
    # tails leave the pathgraph track at index aT (may be before its end), heads join at index aH (may be after its start):
    # cut the pathgraph tracks accordingly so no metres are counted twice.
    LS, leaveB, joinA = make_loop("S"); LT, leaveA, joinB = make_loop("T")
    print(f"  track cuts: A[{joinA}:{leaveA}] of {len(PA)}, B[{joinB}:{leaveB}] of {len(PB)}")
    PA = PA[joinA:leaveA + 1]; PB = PB[joinB:leaveB + 1]
    def fix_z(L, z_start, z_end):
        L = L.copy(); L[:, 2] = np.linspace(z_start, z_end, len(L)); return L
    LT = fix_z(LT, PA[-1, 2], PB[0, 2]); LS = fix_z(LS, PB[-1, 2], PA[0, 2])
    ring = resample_uniform(np.vstack([PA, LT[1:], PB, LS[1:]]))
    n = len(ring)
    dx = np.gradient(ring[:, 0]); dy = np.gradient(ring[:, 1]); heading = np.arctan2(dy, dx)
    dh = np.diff(np.unwrap(heading)); curv = uniform_filter1d(np.r_[dh, dh[-1]], 15, mode="nearest")
    grade = uniform_filter1d(np.gradient(ring[:, 2]), 31, mode="nearest")
    tree0 = cKDTree(ring[:, :2])
    mk = dict(A_start=0, A_end=int(tree0.query(PA[-1, :2])[1]), B_start=int(tree0.query(PB[0, :2])[1]), B_end=int(tree0.query(PB[-1, :2])[1]),
              stop_S=int(tree0.query(STOPS["S"])[1]), stop_T=int(tree0.query(STOPS["T"])[1]))
    out = dict(frame="utm37n_offset", x0=X0, y0=Y0, step=1.0, length=float(n), closed=True, markers=mk,
               x=ring[:, 0].round(3).tolist(), y=ring[:, 1].round(3).tolist(), z=ring[:, 2].round(3).tolist(),
               heading=heading.round(5).tolist(), curv=curv.round(6).tolist(), grade=grade.round(5).tolist())
    OUT.parent.mkdir(parents=True, exist_ok=True); json.dump(out, open(OUT, "w"))
    print(f"ring: {n} pts = {n/1000:.2f} km; markers {mk}; grade {grade.min()*100:.1f}..{grade.max()*100:.1f}%; |curv| max {np.abs(curv).max():.3f}")
    # validation: share of moving fixes farther than 5 m from ring, per bag
    rows = []
    for b in use.bag:
        t, x, y, z, sp, wsp = baselink_track(load_all(b)); mv = sp > 0.7
        d, _ = tree0.query(np.c_[x, y][mv]); rows.append(dict(bag=b, far5=(d > 5).mean() * 100, med=np.median(d)))
    R = pd.DataFrame(rows); print(f"moving fixes >5 m from ring: median bag {R.far5.median():.1f}%, mean {R.far5.mean():.1f}%, bags <2%: {(R.far5 < 2).sum()}/{len(R)}")
    print("worst:", R.sort_values("far5", ascending=False).head(5).round(1).to_string(index=False))

if __name__ == "__main__":
    main()
