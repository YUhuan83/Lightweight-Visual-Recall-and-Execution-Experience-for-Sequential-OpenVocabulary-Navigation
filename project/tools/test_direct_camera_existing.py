import time
from beamngpy import BeamNGpy, Vehicle
from beamngpy.connection import Connection
from beamngpy.sensors import Camera

bng = BeamNGpy("127.0.0.1", 25252).open(launch=False)
vehicle = Vehicle("ego", "sbr")
endpoint = bng.vehicles.start_connection(vehicle, vehicle.extensions)
vehicle.connection = Connection("127.0.0.1", int(endpoint["result"]), timeout=30.0)
vehicle.connect(bng, tries=1)
cam = Camera(
    "diag_cam", bng, vehicle=vehicle,
    pos=(0.0, -1.5, 2.35), dir=(0.0, -1.0, 0.0), up=(0.0, 0.0, 1.0),
    resolution=(960, 640), field_of_view_y=60.0,
    requested_update_time=0.2, is_using_shared_memory=False,
    is_streaming=False, is_render_colours=True,
    is_render_annotations=False, is_render_instance=False,
    is_render_depth=False, is_visualised=False,
)
try:
    for i in range(15):
        d = cam.poll_raw()
        print(i, {k: (None if v is None else len(v)) for k,v in d.items()}, flush=True)
        time.sleep(0.5)
finally:
    try:
        cam.remove()
    except Exception as exc:
        print("camera remove", repr(exc))
    try:
        vehicle.disconnect()
    except Exception as exc:
        print("vehicle disconnect", repr(exc))
    bng.disconnect()
