import cProfile
import importlib.util
from pathlib import Path
import pstats
import numpy as np

root = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location('review_ground', root / 'pointcloud_fast_review.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
points = np.load(root / 'request_audit/request_40__spot1_ouster_front_points_filtered.npz')['body']
plane = module.local_ground_plane(points, 8.)
profile = cProfile.Profile()
for _ in range(10):
    profile.runcall(module.supported_ground_patches, points, plane, 8., .1, .35)
pstats.Stats(profile).sort_stats('cumtime').print_stats(25)
