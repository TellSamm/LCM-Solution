from pathlib import Path
import pandas as pd, numpy as np
from paths import CACHE
T = {"front": "vehicle__front_bogie_velocity", "rear": "vehicle__rear_bogie_velocity",
     "cmd": "vehicle__driver_position_cmd", "mfix": "sensing__gnss__master__fix",
     "mvel": "sensing__gnss__master__vel", "rfix": "sensing__gnss__rover__fix", "rvel": "sensing__gnss__rover__vel"}
def bags():
    return sorted(p.name for p in CACHE.iterdir() if (p / "done").exists())
def load(bag, key):
    return pd.read_parquet(CACHE / bag / (T[key] + ".parquet"))
def load_all(bag):
    return {k: load(bag, k) for k in T}
