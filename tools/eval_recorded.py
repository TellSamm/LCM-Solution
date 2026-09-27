"""Runs INSIDE the container (ROS 2 Humble). Compares the recorded /result/* bag with the GNSS
reference in the input bag, mirroring the judge: nearest-stamp matching within 0.05 s, velocity
RMSE/MAE/bias, position error (2D and 3D) mean/RMSE/max, final error and drift (% of distance).
usage: eval_recorded.py <input_bag_dir> <result_bag_dir>"""
import sys, math
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
sys.path.insert(0, "/ws/install/tram_odometry/lib/python3.10/site-packages")
from tram_odometry.node import latlon_to_utm37
X0, Y0 = 300000.0, 6100000.0
ANT_M, ANT_R = -9.873, 2.563; FR = ANT_R / (ANT_R - ANT_M)

def read(bag, topics):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id="sqlite3"), rosbag2_py.ConverterOptions("", ""))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    out = {t: [] for t in topics}
    while r.has_next():
        topic, data, _ = r.read_next()
        if topic in out: out[topic].append(deserialize_message(data, get_message(types[topic])))
    return out

def stamp(m): return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9

def main(inp, res, node_log=None, res_log=None):
    ref = read(inp, ["/sensing/gnss/master/fix", "/sensing/gnss/rover/fix", "/sensing/gnss/master/vel", "/sensing/gnss/rover/vel"])
    out = read(res, ["/result/velocity", "/result/position"])
    V = out["/result/velocity"]; P = out["/result/position"]
    if not P: print("ОШИБКА: выходные сообщения не записаны"); return
    tP = np.array([stamp(m) for m in P]); dur = tP[-1] - tP[0]; rate = len(P) / max(dur, 1e-9)
    # latency from the node log, resources from the sampler log
    lat_mean = lat_max = cpu = rss = None
    if node_log:
        import re
        L = [re.search(r"latency mean ([\d.]+) ms max ([\d.]+) ms", l) for l in open(node_log, errors="ignore")]
        L = [(float(m.group(1)), float(m.group(2))) for m in L if m]
        if L: lat_mean = sum(a for a, _ in L) / len(L); lat_max = max(b for _, b in L)
    if res_log:
        R = [l.split() for l in open(res_log, errors="ignore") if "cpu=" in l]
        if R: cpu = sum(float(r[1][4:-1]) for r in R) / len(R); rss = max(float(r[2][4:-2]) for r in R)
    ok = lambda c: "OK" if c else "НЕТ"
    print()
    print("=" * 78); print(" ИТОГ ПРОВЕРКИ - LCM Solution / tram_odometry"); print("=" * 78)
    print(f" Прогон: {inp.rstrip('/').split('/')[-1]}   длительность {dur:.0f} с   записано сообщений: {len(V)} velocity, {len(P)} position")
    print()
    print(" РЕАЛЬНОЕ ВРЕМЯ И РЕСУРСЫ                 требование        измерено        статус")
    print(f"   частота /result/velocity,/position     >= 10 Гц          {rate:6.1f} Гц       {ok(rate >= 10)}")
    if lat_mean is not None:
        print(f"   задержка вход->публикация, средняя     <= 100 мс         {lat_mean:6.2f} мс       {ok(lat_mean <= 100)}")
        print(f"   задержка вход->публикация, пик         <= 250 мс         {lat_max:6.2f} мс       {ok(lat_max <= 250)}")
    if cpu is not None:
        print(f"   CPU ноды (среднее)                     <= 200 % (2 ядра) {cpu:6.1f} %        {ok(cpu <= 200)}")
        print(f"   память ноды (макс RSS)                 <= 512 МБ         {rss:6.1f} МБ       {ok(rss <= 512)}")
    fixm = ref["/sensing/gnss/master/fix"] or ref["/sensing/gnss/rover/fix"]; fixr = ref["/sensing/gnss/rover/fix"]
    vel = ref["/sensing/gnss/master/vel"] or ref["/sensing/gnss/rover/vel"]
    if len(fixm) < 10:
        print("\n ТОЧНОСТЬ: в этом bag нет GNSS-эталона - метрики точности недоступны (выходы записаны в results/)"); print("=" * 78); return
    tm = np.array([stamp(m) for m in fixm]); xm, ym = np.array([latlon_to_utm37(m.latitude, m.longitude) for m in fixm]).T
    if len(fixr) > 10 and ref["/sensing/gnss/master/fix"]:
        tr = np.array([stamp(m) for m in fixr]); xr0, yr0 = np.array([latlon_to_utm37(m.latitude, m.longitude) for m in fixr]).T
        xr = np.interp(tm, tr, xr0); yr = np.interp(tm, tr, yr0); xb = xr + (xm - xr) * FR; yb = yr + (ym - yr) * FR
    else:
        xb, yb = xm, ym
    xb -= X0; yb -= Y0
    zb = np.array([m.altitude for m in fixm]) - 3.0     # antenna height above base_link (rail head): GNSS alt - 3.0 matches pathgraph z within 0.1 m
    # velocity
    tv = np.array([stamp(m) for m in vel]); sv = np.array([math.hypot(m.twist.linear.x, m.twist.linear.y) for m in vel])
    tV = np.array([stamp(m) for m in V]); vV = np.array([m.velocity for m in V])
    okv = (tV >= tv[0]) & (tV <= tv[-1]); e = vV[okv] - np.interp(tV[okv], tv, sv)
    # position: nearest stamp within 0.05 s
    px = np.array([m.pose.pose.position.x for m in P]); py = np.array([m.pose.pose.position.y for m in P]); pz = np.array([m.pose.pose.position.z for m in P])
    idx = np.clip(np.searchsorted(tm, tP), 1, len(tm) - 1)
    near = np.where(np.abs(tm[idx] - tP) < np.abs(tm[idx - 1] - tP), idx, idx - 1); okp = np.abs(tm[near] - tP) < 0.05
    e2 = np.hypot(px[okp] - xb[near][okp], py[okp] - yb[near][okp]); ez = pz[okp] - zb[near][okp]; e3 = np.sqrt(e2 ** 2 + ez ** 2)
    dist = np.trapz(sv, tv) if hasattr(np, "trapz") else np.trapezoid(sv, tv)
    print()
    print(" ТОЧНОСТЬ ОТНОСИТЕЛЬНО GNSS-ЭТАЛОНА (эталон: base_link по двум антеннам; сопоставление по метке времени, допуск 0.05 с)")
    print(f"   скорость: RMSE {np.sqrt(np.mean(e**2)):.3f} м/с   MAE {np.mean(np.abs(e)):.3f} м/с   смещение {np.mean(e):+.3f} м/с   (сопоставлено {okv.sum()} сообщений)")
    print(f"   положение (2D): средняя {e2.mean():.1f} м   медиана {np.median(e2):.1f} м   RMSE {np.sqrt((e2**2).mean()):.1f} м   максимум {e2.max():.1f} м   (сопоставлено {okp.mean()*100:.0f} % сообщений)")
    print(f"   положение (3D, с высотой): средняя {e3.mean():.1f} м   RMSE {np.sqrt((e3**2).mean()):.1f} м   максимум {e3.max():.1f} м   |ошибка z| средняя {np.abs(ez).mean():.1f} м")
    print(f"   конец прогона: ошибка {e2[-1]:.1f} м (3D {e3[-1]:.1f} м) после {dist:.0f} м пути  ->  накопленный дрейф {100*e2[-1]/max(dist,1):.2f} % дистанции")
    print()
    print(f" Файлы: results/<bag>_result/ (rosbag2 с /result/*), results/<bag>_node.log (частота и задержка каждые 10 с), results/<bag>_resources.log (CPU/ОЗУ)")
    print("=" * 78)

if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2], *(sys.argv[3:5]))
