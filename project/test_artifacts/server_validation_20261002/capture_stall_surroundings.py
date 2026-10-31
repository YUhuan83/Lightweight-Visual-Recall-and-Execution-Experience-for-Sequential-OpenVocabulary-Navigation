"""Capture fixed overhead and rear-side sensors without moving the vehicle."""
import json
from pathlib import Path
import time
from beamngpy import BeamNGpy
from beamngpy.sensors import Camera
ROOT=Path('/home/ly/full_town_memory_20261002')
b=BeamNGpy('127.0.0.1',25252,socket_timeout=8);b.open(launch=False)
try:
    for name,pos,direction,up in [
        ('stall_overhead',(724.,-13.,66.),(0.,0.,-1.),(0.,1.,0.)),
        ('stall_rear_side',(729.,-5.,56.),(-7.,-8.,-3.),(0.,0.,1.))]:
        camera=Camera(name,b,vehicle=None,is_static=True,is_dir_world_space=True,
            pos=pos,dir=direction,up=up,resolution=(960,720),field_of_view_y=55,
            requested_update_time=.1,is_render_colours=True,is_render_annotations=False,is_render_depth=False)
        try:
            for attempt in range(10):
                b.control.step(6,wait=True)
                data=camera.poll()
                if data.get('colour') is not None:
                    data['colour'].save(ROOT/(name+'.png'));break
                time.sleep(.1)
            else:raise RuntimeError('No image from '+name)
            print('Saved',name,flush=True)
        finally:camera.remove()
finally:b.disconnect()
