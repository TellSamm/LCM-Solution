"""Common paths for the offline tools. Bags are expected in <repo>/data/<bag_id>/ (or set TRAM_DATA_DIR);
intermediate caches go to <repo>/cache (git-ignored); small derived tables live in tools/data."""
import os
from pathlib import Path
REPO = Path(__file__).resolve().parents[1]
DATA = Path(os.environ.get("TRAM_DATA_DIR", REPO / "data"))
CACHE = Path(os.environ.get("TRAM_CACHE_DIR", REPO / "cache"))
TABLES = REPO / "tools" / "data"
ASSETS = REPO / "ros2_ws" / "src" / "tram_odometry" / "assets"
MSG_DIR = REPO / "ros2_ws" / "src" / "tram_vehicle_msgs" / "msg"
