import time
from beamngpy import BeamNGpy
b=BeamNGpy('127.0.0.1',25252,socket_timeout=4).open(launch=False)
try:
    start=time.monotonic()
    response=b.control.queue_lua_command(
        "return jsonEncode({mission=getMissionFilename(),running=be:getPhysicsRunning()})",response=True)
    print('health_seconds',round(time.monotonic()-start,2),response,flush=True)
    start=time.monotonic()
    b.control.pause()
    print('pause_seconds',round(time.monotonic()-start,2),flush=True)
except Exception as exc:
    print(type(exc).__name__,str(exc),flush=True)
finally:
    b.disconnect()
