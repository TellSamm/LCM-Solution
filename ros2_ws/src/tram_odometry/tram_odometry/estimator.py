"""Core estimator (pure Python, no ROS): fuses wheel-speed odometry (front/rear bogie) with a
traction-model prediction in a 1-D Kalman filter with slip/stuck/glitch gating, integrates
distance along the ring map and outputs pose. Used by the ROS 2 node and by the offline evaluator."""
import math
from dataclasses import dataclass, field
from .trackmap import TrackMap
from .traction import TractionModel

@dataclass
class Params:
    publish_rate: float = 25.0
    # Kalman
    q_accel: float = 1.0         # process noise: model accel uncertainty (m/s^2), 1-sigma
    r_wheel: float = 0.10        # wheel-speed measurement noise (m/s), 1-sigma, when valid
    gate_sigma: float = 4.0      # innovation gate in sigmas
    gate_min: float = 0.5        # absolute minimum gate (m/s)
    # anomaly detection
    stuck_repeats: int = 4       # identical samples (>=) while model says speed changes -> stuck
    stuck_min_pred_change: float = 0.3   # m/s model-predicted change over the repeat window
    stuck_min_speed: float = 0.3         # frozen-at-zero is a legitimate standstill, never 'stuck'
    reject_timeout: float = 4.0  # s: after this long with all measurements rejected, re-accept
    max_dt: float = 0.5          # integration sub-step cap
    max_gap: float = 10.0        # longest input gap bridged by the model (s)
    adapt_r: bool = False        # adaptive measurement noise from innovation statistics (off: better on real data, see docs)
    r_max: float = 1.0
    adapt_window: int = 30
    adapt_threshold: float = 3.0 # inflate R only if robust innovation scale exceeds this x nominal
    v_max: float = 20.0
    # init
    init_wait: float = 3.0       # s after first input to wait for GNSS before falling back
    init_min_fixes: int = 5
    default_s0: float = 0.0
    default_heading_source: str = "map"
    # antennas (base_link frame)
    ant_master_x: float = -9.873
    ant_rover_x: float = 2.563
    scale_sigma: float = 0.003     # relative odometry scale uncertainty (drives position covariance)
    init_pos_sigma: float = 2.0
    offset_decay_m: float = 300.0  # off-map start: GNSS-vs-map offset is blended out over this travel distance (0 = snap to map)
    offmap_start_back: float = 1.0 # off-map start: assume the tram is `init_dist * this` metres BEFORE the nearest map point (it still has to drive there)

class Estimator:
    def __init__(self, trackmap: TrackMap, model: TractionModel, params: Params = None):
        self.map = trackmap; self.model = model; self.p = params or Params()
        self.v = 0.0; self.P = 1.0; self.s = self.p.default_s0
        self.t = None; self.notch = 0; self.notch_t = None
        self.a_model = 0.0
        self.initialized = False; self.init_fixes_m = []; self.init_fixes_r = []; self.first_input_t = None
        self.last_meas = {"front": None, "rear": None}; self.repeat = {"front": 0, "rear": 0}
        self.model_drift = {"front": 0.0, "rear": 0.0}   # model-integrated speed change since the sensor value last changed
        self.innov_hist = []; self.r_eff = self.p.r_wheel
        self.last_accept_t = None; self.status = {"front": 0, "rear": 0}   # 0 ok, 1 rejected(gate), 2 stuck, 3 gap
        self.distance = 0.0; self.n_updates = 0; self.n_rejected = 0
        self.Ps = self.p.init_pos_sigma ** 2; self.init_source = "none"; self.init_dist = float("nan")
        self.off_dx = 0.0; self.off_dy = 0.0; self.off_s0 = 0.0   # start offset (GNSS point minus map point) and travel origin
        self.moved = False   # set once wheel speed exceeds 0.5 m/s (tram has left its start position)

    # ---------------------------------------------------------------- inputs
    def on_notch(self, t, notch):
        self._touch(t)
        if self.initialized: self._predict(t)     # advance the model on the 25 Hz controller stream too
        self.notch = int(notch); self.notch_t = t

    def on_gnss(self, t, which, lat, lon, x=None, y=None):
        """Only used for initial alignment (first seconds). x,y already in local frame."""
        if self.initialized or x is None: return
        (self.init_fixes_m if which == "master" else self.init_fixes_r).append((t, x, y))
        self._try_init(t)

    def on_wheel(self, t, which, v_kmh):
        self._touch(t)
        if not self.initialized:
            waited = self.first_input_t is not None and t - self.first_input_t > self.p.init_wait
            self._try_init(t, force=waited and (self.moved or t - self.first_input_t > 60.0))
        scale = self.model.scale_front if which == "front" else self.model.scale_rear
        v_meas = max(0.0, float(v_kmh)) * self.model.kmh_to_ms * scale
        if v_meas > 0.5: self.moved = True
        dt = self._predict(t)
        # ---- anomaly checks
        st = 0
        last = self.last_meas[which]
        if last is not None and abs(v_meas - last) < 1e-9:
            self.repeat[which] += 1
        else:
            self.repeat[which] = 0; self.model_drift[which] = 0.0
        self.last_meas[which] = v_meas
        if (self.repeat[which] >= self.p.stuck_repeats and abs(self.model_drift[which]) > self.p.stuck_min_pred_change
                and v_meas > self.p.stuck_min_speed):
            st = 2   # bit-identical sensor value while the traction model says speed is changing -> frozen sensor
        innov = v_meas - self.v
        R = self.r_eff ** 2
        sigma = math.sqrt(self.P + R)
        gate = max(self.p.gate_min, self.p.gate_sigma * sigma)
        if st == 0 and abs(innov) > gate:
            st = 1   # slip / skid / glitch
        if self.p.adapt_r and st != 2:
            # adaptive measurement noise: robust scale (median-based) of ALL recent innovations, accepted or not.
            # Isolated slips/glitches do not move the median; a persistently noisy sensor does -> R inflates, gate widens.
            self.innov_hist.append(abs(innov))
            if len(self.innov_hist) > self.p.adapt_window: self.innov_hist.pop(0)
            if len(self.innov_hist) >= 10:
                mad = sorted(self.innov_hist)[len(self.innov_hist) // 2] * 1.4826
                self.r_eff = min(self.p.r_max, mad) if mad > self.p.adapt_threshold * self.p.r_wheel else self.p.r_wheel
        # re-accept after long rejection to avoid permanent divergence
        if st == 1 and self.last_accept_t is not None and t - self.last_accept_t > self.p.reject_timeout:   # never re-accept a frozen value
            st = 0; self.P = max(self.P, 1.0)
        self.status[which] = st
        if st == 0:
            K = self.P / (self.P + R)
            self.v = max(0.0, self.v + K * innov); self.P = (1 - K) * self.P
            self.last_accept_t = t; self.n_updates += 1

        else:
            self.n_rejected += 1
        # standstill lock: both wheels report 0 and we are (nearly) stopped -> exactly 0
        if v_meas == 0.0 and self.v < 0.3 and st == 0:
            self.v = 0.0
        return self.output() if self.initialized else None

    # ---------------------------------------------------------------- internals
    def _touch(self, t):
        if self.first_input_t is None: self.first_input_t = t
        if self.t is None: self.t = t

    def _predict(self, t):
        if self.t is None: self.t = t; return 0.0
        dt = t - self.t
        if dt <= 0: return 0.0
        dt_total = min(dt, self.p.max_gap)          # beyond max_gap the state is held (no phantom motion)
        remaining = dt_total
        while remaining > 1e-9:
            h = min(remaining, self.p.max_dt)
            self.a_model = self.model.accel(self.notch, self.v, self.map.grade_at(self.s), self.map.curv_at(self.s))
            v_new = min(self.p.v_max, max(0.0, self.v + self.a_model * h))
            ds = 0.5 * (self.v + v_new) * h
            self.s = self.map.wrap(self.s + ds); self.distance += ds
            self.Ps += (math.sqrt(self.P) * h) ** 2 + (self.p.scale_sigma * ds) ** 2
            for k in self.model_drift: self.model_drift[k] += v_new - self.v
            self.v = v_new; self.P += (self.p.q_accel * h) ** 2
            remaining -= h
        self.t = t
        return dt_total

    def _try_init(self, t, force=False):
        fm, fr = self.init_fixes_m, self.init_fixes_r
        if len(fm) >= self.p.init_min_fixes or (force and len(fm) >= 1):
            # base_link from both antennas if rover present, else master + map heading
            xm = sum(f[1] for f in fm[-self.p.init_min_fixes:]) / len(fm[-self.p.init_min_fixes:])
            ym = sum(f[2] for f in fm[-self.p.init_min_fixes:]) / len(fm[-self.p.init_min_fixes:])
            heading = None; x, y = xm, ym
            if len(fr) >= 1:
                xr = sum(f[1] for f in fr[-self.p.init_min_fixes:]) / len(fr[-self.p.init_min_fixes:])
                yr = sum(f[2] for f in fr[-self.p.init_min_fixes:]) / len(fr[-self.p.init_min_fixes:])
                dx, dy = xr - xm, yr - ym; L = math.hypot(dx, dy)
                if 8.0 < L < 16.0:                       # sane baseline (12.4 m nominal)
                    heading = math.atan2(dy, dx)
                    frac = self.p.ant_rover_x / (self.p.ant_rover_x - self.p.ant_master_x)
                    x = xr + (xm - xr) * frac; y = yr + (ym - yr) * frac
            k, dist = self.map.locate(x, y, heading)
            if heading is None:   # master only: shift along map heading at the located point
                hx, hy = math.cos(self.map.heading[k]), math.sin(self.map.heading[k])
                k, dist = self.map.locate(xm - self.p.ant_master_x * hx, ym - self.p.ant_master_x * hy, None)
            self.s = float(k); self.init_source = "gnss"; self.init_dist = dist
            if dist > 3.0 and self.p.offmap_start_back > 0:
                self.s = self.map.wrap(self.s - self.p.offmap_start_back * min(dist, 60.0))   # tram is off-map: it must still travel ~dist to reach the map
            if self.p.offset_decay_m > 0 and dist < 60.0:   # keep the true start point; blend onto the map while driving away
                mx, my, _, _ = self.map.pose(self.s); self.off_dx = x - mx; self.off_dy = y - my; self.off_s0 = self.distance
            self.initialized = True
        elif force:
            self.s = self.p.default_s0; self.init_source = "default"; self.init_dist = float("nan")
            self.initialized = True

    def predict_to(self, t):
        """Advance the model to time t without a measurement (used to publish on controller messages)."""
        if not self.initialized: return None
        self._predict(t); return self.output()

    # ---------------------------------------------------------------- output
    def output(self):
        x, y, z, hdg = self.map.pose(self.s)
        if self.off_dx or self.off_dy:
            w = max(0.0, 1.0 - (self.distance - self.off_s0) / self.p.offset_decay_m)
            x += w * self.off_dx; y += w * self.off_dy
        return dict(t=self.t, v=self.v, s=self.s, x=x, y=y, z=z, heading=hdg, a_model=self.a_model,
                    P=self.P, Ps=self.Ps, r_eff=self.r_eff, status=max(self.status.values()), notch=self.notch, initialized=self.initialized,
                    init_source=self.init_source)
