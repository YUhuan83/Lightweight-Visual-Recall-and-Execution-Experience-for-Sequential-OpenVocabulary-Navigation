"""Snapshot the actually imported bridge/SDK time and publication implementation."""
from pathlib import Path
import hashlib
import importlib
import inspect
import json
import sys
from beamng_ros2.vehicle import VehicleNode
from beamng_ros2.publishers.sensors import LidarPublisher
from beamng_ros2.utils import float_to_time,float_to_duration
from beamngpy.sensors.lidar import Lidar

run=Path(sys.argv[1]);destination=run/'source_provenance';destination.mkdir(exist_ok=False)
objects=dict(bridge_offset=VehicleNode._compute_time_offset,bridge_poll=VehicleNode._publisher_callback if hasattr(VehicleNode,'_publisher_callback') else VehicleNode,
             lidar_data=LidarPublisher.get_data,lidar_publish=LidarPublisher.publish,
             sdk_lidar_poll=Lidar.poll,float_to_time=float_to_time,float_to_duration=float_to_duration)
modules={inspect.getmodule(value).__name__ for value in objects.values()}
modules.update(base.__module__ for base in LidarPublisher.__mro__ if base.__module__.startswith('beamng_ros2'))
files={}
for name in sorted(modules):
    source=Path(importlib.import_module(name).__file__)
    data=source.read_bytes();target=destination/(name.replace('.','_')+'.py')
    target.write_bytes(data)
    files[name]=dict(imported_path=str(source),snapshot=str(target.relative_to(run)),sha256=hashlib.sha256(data).hexdigest())
methods={name:inspect.getsource(value) for name,value in objects.items() if name!='bridge_poll'}
report=dict(scope=__doc__,files=files,methods=methods,production_modified=False)
(run/'sensor_source_provenance.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(dict(files=files,lidar_publish=methods['lidar_publish'],production_modified=False),indent=2))
