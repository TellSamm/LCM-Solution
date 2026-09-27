"""Step 1. Read every bag once (rosbags, no ROS needed) and store each topic as parquet in cache/<bag>/.
usage: python tools/build_cache.py            (bags in <repo>/data or $TRAM_DATA_DIR)"""
import sys, time
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor
sys.path.insert(0, str(Path(__file__).parent))
from bagreader import read_bag
from paths import DATA, CACHE

def work(bag):
    out = CACHE / bag.name
    if (out / "done").exists(): return bag.name, "cached"
    out.mkdir(parents=True, exist_ok=True)
    d = read_bag(bag)
    for topic, df in d.items(): df.to_parquet(out / (topic.strip("/").replace("/", "__") + ".parquet"), index=False)
    (out / "done").touch(); return bag.name, f"{sum(len(v) for v in d.values())} msgs"

if __name__ == "__main__":
    bags = sorted(p for p in DATA.iterdir() if p.is_dir() and (p / "metadata.yaml").exists())
    t0 = time.time()
    with ProcessPoolExecutor(8) as ex:
        for name, res in ex.map(work, bags): print(f"{name}: {res}", flush=True)
    print(f"{len(bags)} bags, done in {time.time()-t0:.0f}s")
