"""Offline evaluation: replay a bag's messages (stamp order) through the Estimator and compare
with the GNSS reference (base_link from master+rover antennas). Mirrors the judge's metrics:
velocity RMSE/MAE/bias, 2D/3D position error mean/max/RMSE, final drift (% of distance)."""
import sys, argparse, json
from pathlib import Path
import numpy as np, pandas as pd
sys.path.insert(0, str(Path(__file__).parent))
from paths import REPO, TABLES, ASSETS
sys.path.insert(0, str(REPO / "ros2_ws/src/tram_odometry"))
from cache import load_all
from tram_odometry.trackmap import TrackMap
from tram_odometry.traction import TractionModel
from tram_odometry.estimator import Estimator, Params
from pyproj import Transformer
TR = Transformer.from_crs("EPSG:4326", "EPSG:32637", always_xy=True)
X0, Y0 = 300000.0, 6100000.0
ANT_M, ANT_R = -9.873, 2.563; FR = ANT_R / (ANT_R - ANT_M)

def to_xy(df):
    e, n = TR.transform(df.lon.values, df.lat.values); return e - X0, n - Y0

def reference(d):
    m, r = d["mfix"], d["rfix"]
    if len(m) == 0 and len(r) == 0: return None
    if len(m) == 0:  # rover only
        x, y = to_xy(r); return pd.DataFrame(dict(t=r.stamp.values, x=x, y=y, alt=r.alt.values, src="rover"))
    mx, my = to_xy(m)
    if len(r) > 10:
        rx0, ry0 = to_xy(r); rx = np.interp(m.stamp, r.stamp, rx0); ry = np.interp(m.stamp, r.stamp, ry0)
        x = rx + (mx - rx) * FR; y = ry + (my - ry) * FR
    else:
        x, y = mx, my
    return pd.DataFrame(dict(t=m.stamp.values, x=x, y=y, alt=m.alt.values))

def inject(ev, kind, t_first, dur):
    """Synthetic input anomalies for robustness tests. Applied to wheel events in a window starting at 40% of the run."""
    if not kind: return ev
    T0 = t_first + 0.4 * dur; out = []; frozen = {}
    for e in ev:
        t, pr, which, val = e
        if which not in ("front", "rear"): out.append(e); continue
        if kind == "freeze_front" and which == "front" and T0 <= t < T0 + 8:
            frozen.setdefault(which, val); out.append((t, pr, which, frozen[which])); continue
        if kind == "freeze_both" and T0 <= t < T0 + 8:
            frozen.setdefault(which, val); out.append((t, pr, which, frozen[which])); continue
        if kind == "dropout" and T0 <= t < T0 + 3: continue
        if kind == "dropout_long" and T0 <= t < T0 + 10: continue
        if kind == "spikes" and int(t * 10) % 50 == 0: out.append((t, pr, which, val + 15.0)); continue
        if kind == "skid" and T0 <= t < T0 + 4: out.append((t, pr, which, 0.0)); continue
        if kind == "slip_front" and which == "front" and T0 <= t < T0 + 5: out.append((t, pr, which, val * 1.4 + 3)); continue
        if kind == "noise" : out.append((t, pr, which, max(0.0, val + __import__('random').gauss(0, 2.0)))); continue
        out.append(e)
    return out

def run_bag(bag, calib="auto", params=None, gnss_init_seconds=5.0, verbose=False, anomaly=None):
    d = load_all(bag)
    tram = bag[:5]
    cal = ASSETS / (f"calib_{tram}.yaml" if calib == "auto" else f"calib_{calib}.yaml")
    est = Estimator(TrackMap(ASSETS / "track_ring.json"), TractionModel(cal), params or Params())
    # event stream
    ev = []
    for k, which in [("front", "front"), ("rear", "rear")]:
        for t, v in zip(d[k].stamp.values, d[k].v.values): ev.append((t, 1, which, v))
    for t, n in zip(d["cmd"].stamp.values, d["cmd"].pos.values): ev.append((t, 0, "notch", n))
    t_first = min(e[0] for e in ev)
    for k, which in [("mfix", "master"), ("rfix", "rover")]:
        g = d[k]
        if len(g) == 0: continue
        x, y = to_xy(g)
        for t, la, lo, xi, yi in zip(g.stamp.values, g.lat.values, g.lon.values, x, y):
            if t - t_first <= gnss_init_seconds: ev.append((t, 2, which, (la, lo, xi, yi)))
    ev.sort(key=lambda e: (e[0], e[1]))
    ev = inject(ev, anomaly, t_first, ev[-1][0] - t_first)
    out = []
    for t, _, which, val in ev:
        if which == "notch": est.on_notch(t, val)
        elif which in ("front", "rear"):
            o = est.on_wheel(t, which, val)
            if o is not None: out.append(o)
        else: est.on_gnss(t, which, val[0], val[1], val[2], val[3])
    O = pd.DataFrame(out)
    ref = reference(d)
    res = dict(bag=bag, tram=tram, init=getattr(est, "init_source", "none"), init_dist=round(getattr(est, "init_dist", np.nan), 1),
               dist_m=round(est.distance), rejected_pct=round(100 * est.n_rejected / max(1, est.n_rejected + est.n_updates), 2))
    if ref is None or len(ref) < 10:
        return res, O
    # velocity reference: GNSS speed (master vel) interpolated to output stamps
    gv = d["mvel"] if len(d["mvel"]) else d["rvel"]
    vref = np.interp(O.t, gv.stamp, np.hypot(gv.vx, gv.vy))
    ev_ = O.v.values - vref
    # position reference: nearest-stamp within 0.05 s (judge tolerance)
    idx = np.searchsorted(ref.t.values, O.t.values); idx = np.clip(idx, 1, len(ref) - 1)
    near = np.where(np.abs(ref.t.values[idx] - O.t.values) < np.abs(ref.t.values[idx - 1] - O.t.values), idx, idx - 1)
    ok = np.abs(ref.t.values[near] - O.t.values) < 0.05
    ex = O.x.values[ok] - ref.x.values[near][ok]; ey = O.y.values[ok] - ref.y.values[near][ok]
    e2 = np.hypot(ex, ey)
    res.update(v_rmse=round(float(np.sqrt(np.mean(ev_ ** 2))), 3), v_mae=round(float(np.mean(np.abs(ev_))), 3), v_bias=round(float(np.mean(ev_)), 3),
               pos_mean=round(float(e2.mean()), 1), pos_rmse=round(float(np.sqrt((e2 ** 2).mean())), 1), pos_max=round(float(e2.max()), 1),
               pos_p50=round(float(np.median(e2)), 1), final_err=round(float(e2[-1]), 1), drift_pct=round(100 * float(e2[-1]) / max(1, est.distance), 2),
               matched_pct=round(100 * ok.mean()))
    if verbose:
        O["e2"] = np.nan; O.loc[np.where(ok)[0], "e2"] = e2; O["vref"] = vref
    return res, O

def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--bags", nargs="*"); ap.add_argument("--all", action="store_true"); ap.add_argument("--calib", default="auto")
    ap.add_argument("--gnss-init", type=float, default=5.0); ap.add_argument("--save", default=None); ap.add_argument("--anomaly", default=None)
    a = ap.parse_args()
    summ = pd.read_csv(TABLES / "bags_summary.csv")
    bags = a.bags or (summ[(summ.m_cov >= 99) & (summ.dur_s > 600)].drop_duplicates("sig").bag.tolist() if a.all else ["30618_0e41eac3", "30639_d601d28f", "30618_2050d396"])
    rows = []
    for b in bags:
        r, O = run_bag(b, a.calib, gnss_init_seconds=a.gnss_init, anomaly=a.anomaly); rows.append(r); print(r, flush=True)
    R = pd.DataFrame(rows)
    if a.save: R.to_csv(a.save, index=False)
    if len(R) > 1:
        pd.set_option("display.width", 250)
        print("\n=== summary ==="); print(R.describe().loc[["mean", "50%", "max"]].round(2).to_string())
        print(R.groupby("tram")[["v_rmse", "v_bias", "pos_mean", "pos_max", "final_err", "drift_pct", "rejected_pct"]].mean().round(2).to_string())

if __name__ == "__main__":
    main()
