"""Offline calibration (allowed by the rules): per-tram wheel-odometry scale and the
traction/braking acceleration table a(notch, v) from GNSS-referenced training bags.
Writes ros2_ws/src/tram_odometry/assets/calib_<tram>.yaml and calib_default.yaml.
"""
import sys, json
from pathlib import Path
import numpy as np, pandas as pd, yaml
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))
ASSETS = ROOT / "LCM-Solution/ros2_ws/src/tram_odometry/assets"
V_EDGES = np.arange(0, 17, 1.0)                 # speed bins 0..16 m/s, 1 m/s wide
V_CENTERS = (V_EDGES[:-1] + V_EDGES[1:]) / 2
NOTCHES = list(range(-15, 16))

def fill_table(tab):
    """tab: DataFrame notch x vbin with NaN. Fill along speed (ffill/bfill) then along notch."""
    tab = tab.reindex(index=NOTCHES)
    tab = tab.T.interpolate(limit_direction="both").T          # along speed
    tab = tab.interpolate(limit_direction="both")              # along notch
    return tab.fillna(0.0)

def main():
    D = pd.read_parquet(ROOT / "cache/dyn10hz.parquet")
    D["vbin"] = pd.cut(D.v, V_EDGES, labels=False, include_lowest=True)
    out_all = {}
    for tram, X in list(D.groupby("tram")) + [("default", D)]:
        mv = X.v > 0.5
        sf = float((X.fw[mv] * X.v[mv]).sum() / (X.v[mv] ** 2).sum())
        sr = float((X.rw[mv] * X.v[mv]).sum() / (X.v[mv] ** 2).sum())
        # accel table: median accel by notch & speed bin; require >= 30 samples per cell
        g = X[X.v > 0.3].groupby(["notch", "vbin"]).a
        med = g.median(); cnt = g.count()
        med[cnt < 30] = np.nan
        tab = fill_table(med.unstack().reindex(columns=range(len(V_CENTERS))))
        # physical sanity: traction >= 0 for notch>0, braking <= 0 for notch<0 (except notch -8 quirk kept as measured)
        for n in NOTCHES:
            if n > 0: tab.loc[n] = tab.loc[n].clip(lower=0.0)
            if n < 0: tab.loc[n] = tab.loc[n].clip(upper=0.0)
        # physical monotonicity: deeper braking notch never brakes less than the milder one (except the measured
        # notch -8 quirk, kept as is); traction non-decreasing with notch
        for n in range(-2, -16, -1):
            if n == -8: continue
            prev = n + 1 if n + 1 != -8 else n + 2
            tab.loc[n] = np.minimum(tab.loc[n].values, tab.loc[prev].values)
        for n in range(2, 16):
            tab.loc[n] = np.maximum(tab.loc[n].values, tab.loc[n - 1].values)
        # standstill drag/creep: notch 0 at v<0.3 -> 0
        # rolling resistance estimate: median decel at notch 0 while moving
        a0 = float(X[(X.notch == 0) & (X.v > 3)].a.median())
        calib = dict(tram=str(tram), wheel_scale_front=round(1 / sf, 5), wheel_scale_rear=round(1 / sr, 5),
                     kmh_to_ms=1 / 3.6, v_centers=V_CENTERS.tolist(), notches=NOTCHES,
                     accel_table=[[round(float(v), 4) for v in tab.loc[n].values] for n in NOTCHES],
                     coast_accel=round(a0, 4), n_samples=int(len(X)))
        yaml.safe_dump(calib, open(ASSETS / f"calib_{tram}.yaml", "w"), sort_keys=False)
        out_all[tram] = calib
        print(f"{tram}: scale front {1/sf:.5f} rear {1/sr:.5f}, coast a0 {a0:+.3f}, table rows {len(tab)}; "
              f"a(+15, 3m/s)={tab.loc[15].iloc[3]:+.2f} a(-10,5m/s)={tab.loc[-10].iloc[5]:+.2f} a(-8,5m/s)={tab.loc[-8].iloc[5]:+.2f}")
    # traction response lag: cross-correlate d(notch) with d(a)
    for tram, X in D.groupby("tram"):
        best = []
        for b, Y in X.groupby("bag"):
            n = Y.notch.values.astype(float); a = Y.a.values
            dn = np.diff(n); da = np.diff(a)
            cc = [np.corrcoef(dn[:-k] if k else dn, da[k:])[0, 1] for k in range(0, 30)]
            best.append(int(np.nanargmax(cc)))
        print(f"{tram}: response lag (notch -> accel) median {np.median(best)*0.1:.1f} s (per-bag range {min(best)*0.1:.1f}..{max(best)*0.1:.1f})")

if __name__ == "__main__":
    main()
