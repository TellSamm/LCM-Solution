"""Ring track map: 1 m-spaced closed polyline (A track + loops + B track) in the local metric
frame (UTM 37N minus (300000, 6100000)) with z, heading, curvature, grade per point."""
import json, math
import numpy as np

class TrackMap:
    def __init__(self, path):
        d = json.load(open(path))
        self.x = np.asarray(d["x"]); self.y = np.asarray(d["y"]); self.z = np.asarray(d["z"])
        self.heading = np.asarray(d["heading"]); self.curv = np.asarray(d["curv"]); self.grade = np.asarray(d["grade"])
        self.n = len(self.x); self.length = float(d.get("length", self.n)); self.step = float(d.get("step", 1.0))
        self.markers = d.get("markers", {}); self.x0 = d.get("x0", 300000.0); self.y0 = d.get("y0", 6100000.0)

    # ---- s -> pose -------------------------------------------------------------------------
    def wrap(self, s):
        return s % self.length

    def pose(self, s):
        """Interpolated (x, y, z, heading) at arc-length s (metres, wraps around the ring)."""
        s = self.wrap(s); i = int(s); f = s - i; j = (i + 1) % self.n
        x = self.x[i] + f * (self.x[j] - self.x[i]); y = self.y[i] + f * (self.y[j] - self.y[i]); z = self.z[i] + f * (self.z[j] - self.z[i])
        return x, y, z, self.heading[i]

    def grade_at(self, s):
        return float(self.grade[int(self.wrap(s))])

    def curv_at(self, s):
        return float(self.curv[int(self.wrap(s))])

    # ---- xy -> s --------------------------------------------------------------------------
    def candidates(self, x, y, radius=15.0):
        """All local-minimum ring indices within radius of (x,y): (idx, dist) sorted by dist."""
        d = np.hypot(self.x - x, self.y - y)
        near = np.where(d < radius)[0]
        if len(near) == 0:
            k = int(np.argmin(d)); return [(k, float(d[k]))]
        # split into contiguous runs (each run = one passing of the track), take best of each
        runs = np.split(near, np.where(np.diff(near) > 3)[0] + 1)
        out = [(int(r[np.argmin(d[r])]), float(d[r].min())) for r in runs]
        # ring wrap: merge first and last run if adjacent
        if len(out) > 1 and out[0][0] < 3 and out[-1][0] > self.n - 4:
            out = [min(out[0], out[-1], key=lambda t: t[1])] + out[1:-1]
        return sorted(out, key=lambda t: t[1])

    def locate(self, x, y, heading=None, radius=15.0):
        """Best ring index for a position; if heading (rad) given, prefer candidates whose track
        heading agrees (resolves the two parallel opposite-direction tracks)."""
        cands = self.candidates(x, y, radius)
        if heading is None or len(cands) == 1:
            return cands[0][0], cands[0][1]
        def score(c):
            k, dist = c; dh = abs(math.remainder(self.heading[k] - heading, 2 * math.pi))
            return dist + 20.0 * (dh > math.pi / 2)   # heavy penalty for opposite direction
        best = min(cands, key=score); return best[0], best[1]

    @staticmethod
    def yaw_to_quat(yaw):
        return (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))
