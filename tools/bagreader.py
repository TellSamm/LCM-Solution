"""Read hackathon rosbags into pandas DataFrames without ROS."""
from pathlib import Path
import numpy as np, pandas as pd
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore, get_types_from_msg

from paths import MSG_DIR

def make_typestore():
    ts = get_typestore(Stores.ROS2_HUMBLE)
    add = {}
    for f in MSG_DIR.glob("*.msg"):
        add.update(get_types_from_msg(f.read_text(), f"tram_vehicle_msgs/msg/{f.stem}"))
    ts.register(add)
    return ts

TS = make_typestore()

def read_bag(bag_dir):
    """Return dict topic -> DataFrame with t (s, bag receive time), stamp (s, header), fields."""
    out = {}
    with AnyReader([Path(bag_dir)], default_typestore=TS) as r:
        for conn in r.connections:
            rows = []
            for _, t, raw in r.messages(connections=[conn]):
                m = r.deserialize(raw, conn.msgtype)
                h = m.header
                st = h.stamp.sec + h.stamp.nanosec * 1e-9
                row = {"t": t * 1e-9, "stamp": st}
                if conn.msgtype.endswith("VelocitySensor"):
                    row["v"] = m.velocity
                elif conn.msgtype.endswith("DriverControllerCommand"):
                    row["pos"] = m.position
                elif conn.msgtype.endswith("NavSatFix"):
                    row.update(lat=m.latitude, lon=m.longitude, alt=m.altitude,
                               status=m.status.status, cov0=m.position_covariance[0])
                elif conn.msgtype.endswith("TwistStamped"):
                    row.update(vx=m.twist.linear.x, vy=m.twist.linear.y, vz=m.twist.linear.z)
                rows.append(row)
            out[conn.topic] = pd.DataFrame(rows)
    return out
