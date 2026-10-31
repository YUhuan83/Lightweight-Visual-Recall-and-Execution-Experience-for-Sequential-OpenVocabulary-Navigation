"""Read-only launch prerequisite inventory. Never reads license contents."""
import argparse
import importlib.metadata
import json
import os
from pathlib import Path
import platform
import shutil
import subprocess


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--beamng-home', required=True)
    parser.add_argument('--checkpoint-root', required=True)
    args = parser.parse_args()
    home = Path(args.beamng_home).expanduser()
    ckpts = Path(args.checkpoint_root).expanduser()
    checks = {}
    checks['linux_x64'] = platform.system() == 'Linux' and platform.machine() in ('x86_64', 'amd64')
    checks['beamng_binary'] = (home/'BinLinux/BeamNG.tech.x64').is_file()
    checks['license_file_present_only'] = (home/'tech.key').is_file()
    checks['ros_humble_setup'] = Path('/opt/ros/humble/setup.bash').is_file()
    for name in ('ros2','colcon','nvidia-smi','vulkaninfo'):
        checks[f'command_{name}'] = shutil.which(name) is not None
    for name in ('c-radio_v3-b_half.pth.tar','frontier_head.ckpt','trav_head.ckpt',
                 'siglip2_ms/config.json','siglip2_ms/model.safetensors'):
        checks[f'weight_{name}'] = (ckpts/name).is_file()
    packages = {}
    for name in ('beamngpy','torch','transformers','numpy'):
        try:
            packages[name] = importlib.metadata.version(name)
        except importlib.metadata.PackageNotFoundError:
            packages[name] = None
    gpu = None
    if shutil.which('nvidia-smi'):
        try:
            result = subprocess.run(['nvidia-smi','--query-gpu=name,driver_version,memory.total',
                                     '--format=csv,noheader'],capture_output=True,text=True,timeout=15)
            gpu = {'returncode': result.returncode, 'output': result.stdout.strip(), 'error': result.stderr.strip()}
        except (OSError, subprocess.TimeoutExpired) as exc:
            gpu = {'error': str(exc)}
    report = dict(checks=checks, missing=[key for key,value in checks.items() if not value],
                  python=platform.python_version(),packages=packages,gpu=gpu,
                  ros_distro=os.environ.get('ROS_DISTRO'),
                  full_pipeline_ready=False,
                  limitations=['File presence does not verify licensing, weights, or Vulkan rendering.',
                               'Simulator version must match BeamNGpy and the pinned bridge.',
                               'Calibration, upstream clock acquisition, vehicle control and full algorithm runtime remain unverified.'])
    print(json.dumps(report,indent=2,ensure_ascii=False))
    return 0 if all(checks.values()) else 2


if __name__ == '__main__':
    raise SystemExit(main())
