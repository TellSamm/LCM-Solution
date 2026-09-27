"""ROS 2 Humble node: model-based backup odometry for the tram.
Subscribes: /vehicle/front_bogie_velocity, /vehicle/rear_bogie_velocity (tram_vehicle_msgs/VelocitySensor, km/h),
            /vehicle/driver_position_cmd (tram_vehicle_msgs/DriverControllerCommand),
            /sensing/gnss/{master,rover}/fix (sensor_msgs/NavSatFix) -- ONLY during the first
            `gnss_init_seconds` after the first input, for initial alignment; ignored afterwards.
Publishes: /result/velocity (tram_vehicle_msgs/VelocitySensor, m/s), /result/position (nav_msgs/Odometry),
           /result/diagnostics (diagnostic_msgs/DiagnosticArray).
"""
import math, time, os
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from ament_index_python.packages import get_package_share_directory
from tram_vehicle_msgs.msg import VelocitySensor, DriverControllerCommand
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from .trackmap import TrackMap
from .traction import TractionModel
from .estimator import Estimator, Params

# UTM zone 37N (WGS84) forward projection, no external deps (Karney/Snyder series, mm-level for Moscow)
def latlon_to_utm37(lat_deg, lon_deg):
    a = 6378137.0; f = 1 / 298.257223563; k0 = 0.9996; lon0 = math.radians(39.0)
    e2 = f * (2 - f); ep2 = e2 / (1 - e2)
    lat = math.radians(lat_deg); lon = math.radians(lon_deg)
    N = a / math.sqrt(1 - e2 * math.sin(lat) ** 2); T = math.tan(lat) ** 2; C = ep2 * math.cos(lat) ** 2; A = math.cos(lat) * (lon - lon0)
    M = a * ((1 - e2 / 4 - 3 * e2 ** 2 / 64 - 5 * e2 ** 3 / 256) * lat - (3 * e2 / 8 + 3 * e2 ** 2 / 32 + 45 * e2 ** 3 / 1024) * math.sin(2 * lat)
             + (15 * e2 ** 2 / 256 + 45 * e2 ** 3 / 1024) * math.sin(4 * lat) - (35 * e2 ** 3 / 3072) * math.sin(6 * lat))
    x = k0 * N * (A + (1 - T + C) * A ** 3 / 6 + (5 - 18 * T + T ** 2 + 72 * C - 58 * ep2) * A ** 5 / 120) + 500000.0
    y = k0 * (M + N * math.tan(lat) * (A ** 2 / 2 + (5 - T + 9 * C + 4 * C ** 2) * A ** 4 / 24 + (61 - 58 * T + T ** 2 + 600 * C - 330 * ep2) * A ** 6 / 720))
    return x, y

def stamp_to_sec(st): return st.sec + st.nanosec * 1e-9

class OdometryNode(Node):
    def __init__(self):
        super().__init__("tram_odometry")
        share = get_package_share_directory("tram_odometry")
        dp = self.declare_parameter; dyn = ParameterDescriptor(dynamic_typing=True)
        dp("map_file", os.path.join(share, "assets", "track_ring.json"))
        dp("calib", "auto", dyn)            # auto | 30618 | 30639 | default
        dp("tram_id", "30618", dyn)         # used when calib == auto; "" -> averaged table
        dp("default_start", "stop_S", dyn)  # stop_S | stop_T | A_start | B_start | <number: s in metres>
        dp("gnss_init_seconds", 5.0)        # GNSS accepted only this long after the first input
        dp("init_wait", 3.0)
        dp("init_min_fixes", 5)
        dp("frame_id", "map"); dp("child_frame_id", "base_link")
        dp("q_accel", 1.0); dp("r_wheel", 0.10); dp("gate_sigma", 4.0); dp("gate_min", 0.5)
        dp("stuck_repeats", 4); dp("stuck_min_pred_change", 0.4); dp("reject_timeout", 4.0)
        dp("scale_sigma", 0.003); dp("adapt_r", False); dp("max_gap", 10.0); dp("publish_on_notch", True); dp("gap_fill_rate", 10.0); dp("log_period", 10.0); dp("max_predict_gap", 1.0); dp("max_hold_gap", 5.0)
        g = lambda n: self.get_parameter(n).value
        self.map = TrackMap(g("map_file"))
        calib = g("calib"); tram = str(g("tram_id"))
        if calib == "auto": calib = tram if tram in ("30618", "30639") else "default"
        self.model = TractionModel(os.path.join(share, "assets", f"calib_{calib}.yaml"))
        p = Params(q_accel=g("q_accel"), r_wheel=g("r_wheel"), gate_sigma=g("gate_sigma"), gate_min=g("gate_min"),
                   stuck_repeats=int(g("stuck_repeats")), stuck_min_pred_change=g("stuck_min_pred_change"), reject_timeout=g("reject_timeout"),
                   init_wait=g("init_wait"), init_min_fixes=int(g("init_min_fixes")), scale_sigma=g("scale_sigma"), adapt_r=bool(g("adapt_r")), max_gap=float(g("max_gap")))
        ds = str(g("default_start"))
        p.default_s0 = float(self.map.markers.get(ds, ds if ds.replace(".", "", 1).isdigit() else 0))
        self.est = Estimator(self.map, self.model, p)
        self.gnss_init_seconds = float(g("gnss_init_seconds")); self.frame_id = g("frame_id"); self.child = g("child_frame_id")
        self.publish_on_notch = bool(g("publish_on_notch")); self.max_predict_gap = float(g("max_predict_gap")); self.max_hold_gap = float(g("max_hold_gap"))
        qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(VelocitySensor, "/vehicle/front_bogie_velocity", lambda m: self.on_wheel(m, "front"), qos)
        self.create_subscription(VelocitySensor, "/vehicle/rear_bogie_velocity", lambda m: self.on_wheel(m, "rear"), qos)
        self.create_subscription(DriverControllerCommand, "/vehicle/driver_position_cmd", self.on_notch, qos)
        self.create_subscription(NavSatFix, "/sensing/gnss/master/fix", lambda m: self.on_fix(m, "master"), qos)
        self.create_subscription(NavSatFix, "/sensing/gnss/rover/fix", lambda m: self.on_fix(m, "rover"), qos)
        self.pub_v = self.create_publisher(VelocitySensor, "/result/velocity", 10)
        self.pub_p = self.create_publisher(Odometry, "/result/position", 10)
        self.pub_d = self.create_publisher(DiagnosticArray, "/result/diagnostics", 10)
        self.first_input_t = None; self.last_pub_stamp = None; self.last_pub_wall = None; self.last_out = None
        self.lat_sum = 0.0; self.lat_max = 0.0; self.lat_n = 0; self.pub_count = 0; self.t_log = time.monotonic()
        self.create_timer(1.0 / float(g("gap_fill_rate")), self.gap_fill)
        self.create_timer(float(g("log_period")), self.log_stats)
        self.get_logger().info(f"tram_odometry started: calib={calib}, map={self.map.n} pts ({self.map.length/1000:.2f} km), default_start={ds} (s={p.default_s0:.0f})")

    # ---------------------------------------------------------------- callbacks
    def on_notch(self, msg):
        t0 = time.perf_counter(); t = stamp_to_sec(msg.header.stamp); self._first(t)
        self.est.on_notch(t, msg.position)
        if self.publish_on_notch:
            out = self.est.predict_to(t)
            if out: self.publish(out, msg.header.stamp, t0)

    def on_wheel(self, msg, which):
        t0 = time.perf_counter(); t = stamp_to_sec(msg.header.stamp); self._first(t)
        v = msg.velocity
        if not math.isfinite(v): return
        out = self.est.on_wheel(t, which, v)
        if out: self.publish(out, msg.header.stamp, t0)

    def on_fix(self, msg, which):
        t = stamp_to_sec(msg.header.stamp)
        if self.first_input_t is None or self.est.initialized: return
        # GNSS is accepted for the initial alignment only: within gnss_init_seconds of the first input, or for as long
        # as the tram has not started moving yet (still standing at the terminus) — never after motion has begun.
        if t - self.first_input_t > self.gnss_init_seconds and self.est.moved: return
        if not (math.isfinite(msg.latitude) and math.isfinite(msg.longitude)) or msg.latitude == 0.0: return
        e, n = latlon_to_utm37(msg.latitude, msg.longitude)
        self.est.on_gnss(t, which, msg.latitude, msg.longitude, e - self.map.x0, n - self.map.y0)
        if self.est.initialized: self.get_logger().info(f"initialized from GNSS ({which}): s={self.est.s:.0f} m, dist to track {self.est.init_dist:.1f} m")

    def _first(self, t):
        if self.first_input_t is None: self.first_input_t = t

    def gap_fill(self):
        """Keep >=10 Hz output if inputs stall: re-publish the last state advanced by wall-clock elapsed time."""
        if self.last_out is None or self.last_pub_wall is None: return
        gap = time.monotonic() - self.last_pub_wall
        if gap < 0.15: return
        if gap <= self.max_predict_gap:
            # short input dropout: advance the model to keep >=10 Hz output with consistent stamps
            t = self.last_pub_stamp_sec + gap; out = self.est.predict_to(t)
            if out:
                self.publish(out, rclpy.time.Time(seconds=t).to_msg(), time.perf_counter(), gap_fill=True)
        elif gap <= self.max_hold_gap:
            # long dropout (or bag finished): hold the last state frozen, flag it; do not integrate a stale notch
            out = dict(self.last_out); out["status"] = 3
            self.publish(out, rclpy.time.Time(seconds=self.last_pub_stamp_sec + gap).to_msg(), time.perf_counter(), gap_fill=True, advance=False)

    # ---------------------------------------------------------------- output
    def publish(self, out, stamp, t0, gap_fill=False, advance=True):
        vmsg = VelocitySensor(); vmsg.header.stamp = stamp; vmsg.header.frame_id = self.child; vmsg.velocity = float(out["v"])
        o = Odometry(); o.header.stamp = stamp; o.header.frame_id = self.frame_id; o.child_frame_id = self.child
        o.pose.pose.position.x = float(out["x"]); o.pose.pose.position.y = float(out["y"]); o.pose.pose.position.z = float(out["z"])
        qx, qy, qz, qw = self.map.yaw_to_quat(out["heading"]); o.pose.pose.orientation.x = qx; o.pose.pose.orientation.y = qy; o.pose.pose.orientation.z = qz; o.pose.pose.orientation.w = qw
        Ps = float(out["Ps"]); cov = [0.0] * 36; cov[0] = Ps; cov[7] = Ps; cov[14] = 4.0; cov[35] = 0.01; o.pose.covariance = cov
        o.twist.twist.linear.x = float(out["v"]); tc = [0.0] * 36; tc[0] = float(out["P"]); o.twist.covariance = tc
        self.pub_v.publish(vmsg); self.pub_p.publish(o)
        if advance: self.last_out = out; self.last_pub_stamp_sec = stamp_to_sec(stamp); self.last_pub_wall = time.monotonic()
        self.pub_count += 1
        lat = time.perf_counter() - t0; self.lat_sum += lat; self.lat_max = max(self.lat_max, lat); self.lat_n += 1
        if self.pub_count % 10 == 0 or out["status"]:
            da = DiagnosticArray(); da.header.stamp = stamp
            st = DiagnosticStatus(name="tram_odometry", hardware_id=self.model.tram)
            st.level = DiagnosticStatus.WARN if out["status"] else DiagnosticStatus.OK
            st.message = {0: "ok", 1: "wheel measurement rejected (slip/skid/glitch), model only", 2: "wheel sensor frozen, model only", 3: "gap"}[int(out["status"])]
            st.values = [KeyValue(key=k, value=str(v)) for k, v in dict(slip_status=int(out["status"]), front_status=self.est.status["front"], rear_status=self.est.status["rear"],
                        v_mps=round(out["v"], 3), s_m=round(out["s"], 1), a_model=round(out["a_model"], 3), notch=out["notch"], v_sigma=round(math.sqrt(out["P"]), 3),
                        pos_sigma=round(math.sqrt(Ps), 2), init_source=out["init_source"], rejected=self.est.n_rejected, gap_fill=gap_fill).items()]
            da.status = [st]; self.pub_d.publish(da)

    def log_stats(self):
        if self.lat_n == 0: return
        dt = time.monotonic() - self.t_log
        self.get_logger().info(f"rate {self.pub_count/dt:.1f} Hz | latency mean {1e3*self.lat_sum/self.lat_n:.2f} ms max {1e3*self.lat_max:.2f} ms | v={self.est.v:.2f} m/s s={self.est.s:.0f} m | rejected {self.est.n_rejected} | init {self.est.init_source}")
        self.lat_sum = 0.0; self.lat_max = 0.0; self.lat_n = 0; self.pub_count = 0; self.t_log = time.monotonic()

def main(args=None):
    rclpy.init(args=args); node = OdometryNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.try_shutdown()

if __name__ == "__main__":
    main()
