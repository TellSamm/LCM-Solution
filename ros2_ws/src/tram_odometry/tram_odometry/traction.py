"""Traction/braking model: acceleration a(notch, v) from a calibrated table (median measured
acceleration per controller notch and speed bin) plus map-based grade and curve resistance."""
import bisect, yaml
G = 9.81

class TractionModel:
    def __init__(self, calib_path):
        c = yaml.safe_load(open(calib_path))
        self.tram = str(c["tram"])
        self.scale_front = float(c["wheel_scale_front"]); self.scale_rear = float(c["wheel_scale_rear"])
        self.kmh_to_ms = float(c["kmh_to_ms"])
        self.vc = [float(v) for v in c["v_centers"]]; self.notches = list(c["notches"])
        self.table = [[float(a) for a in row] for row in c["accel_table"]]     # [notch_idx][v_idx]
        self.n0 = self.notches[0]
        self.coast = float(c.get("coast_accel", 0.0))
        self.k_curve = float(c.get("k_curve", 3.0))

    @staticmethod
    def _interp(x, xs, ys):
        """Linear interpolation with clamping at the ends (same semantics as numpy.interp)."""
        if x <= xs[0]: return ys[0]
        if x >= xs[-1]: return ys[-1]
        j = bisect.bisect_right(xs, x); x0, x1 = xs[j - 1], xs[j]
        return ys[j - 1] + (ys[j] - ys[j - 1]) * (x - x0) / (x1 - x0)

    def accel(self, notch, v, grade=0.0, curv=0.0):
        """Model longitudinal acceleration (m/s^2) for controller notch at speed v (m/s)."""
        notch = int(max(self.notches[0], min(self.notches[-1], notch)))
        row = self.table[notch - self.n0]
        a = self._interp(v, self.vc, row)
        # the table is a route-average; add deviation of local grade from zero and curve drag
        a -= G * grade
        a -= self.k_curve * abs(curv)                             # curve resistance, ~0.1 m/s^2 at R=30 m
        if v <= 0.05 and a < 0:                                  # cannot decelerate below standstill
            a = 0.0
        return a
