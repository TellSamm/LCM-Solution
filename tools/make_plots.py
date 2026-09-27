"""Figures for docs/ACCURACY_AND_PERFORMANCE.md (offline evaluation)."""
import sys
from pathlib import Path
import numpy as np, pandas as pd
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
sys.path.insert(0, str(Path(__file__).parent))
from evaluate import run_bag, ROOT, REPO
IMG = REPO / "docs/img"; IMG.mkdir(exist_ok=True)

def fig_velocity(bag="30618_0e41eac3"):
    r, O = run_bag(bag, verbose=True); t = O.t - O.t.iloc[0]
    fig, ax = plt.subplots(2, 1, figsize=(14, 7), sharex=True, gridspec_kw=dict(height_ratios=[3, 1]))
    ax[0].plot(t, O.vref, "k", lw=1.2, label="GNSS (эталон)"); ax[0].plot(t, O.v, "r", lw=0.8, alpha=0.8, label="оценка /result/velocity")
    ax[0].set_ylabel("скорость, м/с"); ax[0].legend(); ax[0].set_title(f"{bag}: скорость — RMSE {r['v_rmse']:.3f} м/с, bias {r['v_bias']:+.3f} м/с"); ax[0].grid(alpha=.3)
    ax[1].plot(t, O.v - O.vref, "b", lw=0.6); ax[1].set_ylim(-1, 1); ax[1].set_ylabel("ошибка, м/с"); ax[1].set_xlabel("время, с"); ax[1].grid(alpha=.3)
    ax[1].step(t, O.notch / 15, "g", lw=0.5, alpha=0.5, label="позиция контроллера /15"); ax[1].legend(loc="upper right")
    plt.tight_layout(); plt.savefig(IMG / "velocity_tracking.png", dpi=100); plt.close()
    # zoom on a start-stop cycle
    fig, ax = plt.subplots(figsize=(12, 4)); m = (t > 300) & (t < 420)
    ax.plot(t[m], O.vref[m], "k", lw=1.5, label="GNSS"); ax.plot(t[m], O.v[m], "r.", ms=2, label="оценка"); ax.step(t[m], O.notch[m] / 15 * 5, "g", lw=0.8, alpha=0.6, label="контроллер (×5/15)")
    ax.set_xlabel("время, с"); ax.set_ylabel("м/с"); ax.legend(); ax.grid(alpha=.3); ax.set_title("Разгон / торможение / остановка (фрагмент)")
    plt.tight_layout(); plt.savefig(IMG / "velocity_zoom.png", dpi=100); plt.close()

def fig_position(bags=("30618_0e41eac3", "30618_2050d396", "30639_2b4a6347", "30639_d927f360")):
    fig, ax = plt.subplots(figsize=(14, 5))
    for b in bags:
        r, O = run_bag(b, verbose=True); t = O.t - O.t.iloc[0]
        ax.plot(t, O.e2, lw=0.8, label=f"{b}: mean {r['pos_mean']:.1f} м, final {r['final_err']:.1f} м, drift {r['drift_pct']:.2f}%")
    ax.set_xlabel("время, с"); ax.set_ylabel("ошибка положения (2D), м"); ax.set_ylim(0, 80); ax.grid(alpha=.3); ax.legend(fontsize=8); ax.set_title("Ошибка положения относительно GNSS-эталона")
    plt.tight_layout(); plt.savefig(IMG / "position_error.png", dpi=100); plt.close()

def fig_trajectory(bag="30618_0e41eac3"):
    from evaluate import load_all, reference
    r, O = run_bag(bag, verbose=True); ref = reference(load_all(bag))
    fig, ax = plt.subplots(figsize=(12, 6)); ax.plot(ref.x, ref.y, "k", lw=2, label="GNSS base_link"); ax.plot(O.x, O.y, "r", lw=0.8, label="оценка")
    ax.set_aspect("equal"); ax.legend(); ax.grid(alpha=.3); ax.set_title(f"Траектория {bag} (UTM 37N − смещение), м"); plt.tight_layout(); plt.savefig(IMG / "trajectory.png", dpi=100); plt.close()

def fig_anomalies(bag="30618_0e41eac3"):
    fig, axs = plt.subplots(2, 2, figsize=(15, 8)); axs = axs.ravel()
    for ax, an, title in zip(axs, ["freeze_both", "skid", "dropout_long", "spikes"],
                             ["зависание обоих датчиков колёс (8 с)", "юз: колёса = 0 при движении (4 с)", "пропадание колёсных сообщений (10 с)", "выбросы +15 км/ч каждые 5 с"]):
        r, O = run_bag(bag, anomaly=an, verbose=True); t = O.t - O.t.iloc[0]; t0 = 0.4 * (t.iloc[-1]); m = (t > t0 - 15) & (t < t0 + 30)
        ax.plot(t[m], O.vref[m], "k", lw=1.5, label="GNSS"); ax.plot(t[m], O.v[m], "r", lw=1, label="оценка")
        st = O.status[m] > 0; ax.scatter(t[m][st], O.v[m][st], s=8, c="orange", zorder=5, label="отбраковано / флаг")
        ax.axvspan(t0, t0 + {"freeze_both": 8, "skid": 4, "dropout_long": 10, "spikes": 0}[an], color="red", alpha=0.08)
        ax.set_title(f"{title}: RMSE v {r['v_rmse']:.3f} м/с, ср. ошибка позиции {r['pos_mean']:.1f} м"); ax.grid(alpha=.3); ax.legend(fontsize=8); ax.set_xlabel("время, с")
    plt.tight_layout(); plt.savefig(IMG / "anomalies.png", dpi=100); plt.close()

def fig_summary():
    R = pd.read_csv(ROOT / "notes/eval_v3.csv")
    fig, axs = plt.subplots(1, 3, figsize=(15, 4))
    for ax, col, lab in zip(axs, ["v_rmse", "pos_mean", "drift_pct"], ["RMSE скорости, м/с", "средняя ошибка положения, м", "дрейф в конце, % дистанции"]):
        for tram, c in [("30618", "tab:blue"), ("30639", "tab:orange")]:
            ax.hist(R[R.tram == int(tram)][col], bins=20, alpha=0.6, color=c, label=f"трамвай {tram}")
        ax.set_xlabel(lab); ax.legend(); ax.grid(alpha=.3)
    fig.suptitle(f"Распределение метрик по {len(R)} обучающим прогонам с GNSS-эталоном"); plt.tight_layout(); plt.savefig(IMG / "summary_hist.png", dpi=100); plt.close()

if __name__ == "__main__":
    fig_velocity(); fig_position(); fig_trajectory(); fig_anomalies(); fig_summary(); print("figures written to", IMG)
