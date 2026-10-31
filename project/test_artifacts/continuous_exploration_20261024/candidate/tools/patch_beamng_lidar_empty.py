"""Make the BeamNG ROS2 LiDAR publisher tolerate empty startup frames."""
from __future__ import annotations

import argparse
import shutil
import time
from pathlib import Path


OLD = '''        data = self.sensor.poll()
        if isinstance(data["colours"], list):
            data["colours"] = np.array(data["colours"], dtype=np.uint8).reshape(-1, 3)
        points = cast(np.ndarray, data["pointCloud"])
        colours = cast(np.ndarray, data["colours"])
        colours = colours[:, [2, 1, 0]]  # RGB -> BGR
'''

NEW = '''        data = self.sensor.poll()
        # BeamNG can publish an empty first LiDAR frame while the sensor is
        # warming up. Depending on the backend, pointCloud/colours may be a
        # list, a 1-D empty ndarray, or a regular Nx3 array. Normalize all
        # cases before the RGB->BGR indexing below.
        points = np.asarray(data.get("pointCloud", []), dtype=np.float32)
        if points.size == 0:
            points = np.empty((0, 3), dtype=np.float32)
        else:
            points = points.reshape(-1, 3)
        colours = np.asarray(data.get("colours", []), dtype=np.uint8)
        if colours.size == 0:
            colours = np.zeros((points.shape[0], 3), dtype=np.uint8)
        else:
            colours = colours.reshape(-1, 3)
            if colours.shape[0] != points.shape[0]:
                colours = np.resize(colours, (points.shape[0], 3))
        colours = colours[:, [2, 1, 0]]  # RGB -> BGR
'''


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--workspace', type=Path, required=True)
    args = parser.parse_args()
    path = (args.workspace.expanduser().resolve() / 'beamng_ros2' /
            'beamng_ros2' / 'publishers' / 'sensors.py')
    if not path.is_file():
        raise FileNotFoundError(path)
    text = path.read_text(encoding='utf-8')
    if 'Normalize all' in text:
        print('empty LiDAR frame patch already applied')
        return 0
    if text.count(OLD) != 1:
        raise RuntimeError('unexpected LidarPublisher source; refusing to patch')
    stamp = time.strftime('%Y%m%d-%H%M%S')
    shutil.copy2(path, path.with_name(path.name + f'.pre_empty_lidar_{stamp}.bak'))
    path.write_text(text.replace(OLD, NEW, 1), encoding='utf-8')
    print(f'empty LiDAR frame patch applied to {path}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
