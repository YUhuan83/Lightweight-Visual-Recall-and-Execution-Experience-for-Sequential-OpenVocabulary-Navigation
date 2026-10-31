"""Capture an overhead view while the full test keeps running."""
from pathlib import Path
import time
from beamngpy import BeamNGpy
from beamngpy.sensors import Camera
root=Path('/home/ly/full_town_memory_20261002/body_collision_full_trial1')
b=BeamNGpy('127.0.0.1',25252,socket_timeout=8).open(launch=False)
camera=None
try:
    camera=Camera('body_collision_trial_overhead',b,vehicle=None,is_static=True,is_dir_world_space=True,
        pos=(745.,-9.,72.),dir=(0.,0.,-1.),up=(0.,1.,0.),resolution=(960,720),
        field_of_view_y=65,requested_update_time=.1,is_render_colours=True,
        is_render_annotations=False,is_render_depth=False)
    for attempt in range(15):
        time.sleep(.2)
        frame=camera.poll().get('colour')
        if frame is not None:
            frame.save(root/'stall_overhead.png')
            print('Saved current stall_overhead.png without stepping or moving vehicle',flush=True)
            break
    else: raise RuntimeError('No overhead frame')
finally:
    if camera is not None: camera.remove()
    b.disconnect()

