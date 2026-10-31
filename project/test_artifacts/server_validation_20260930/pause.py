from beamngpy import BeamNGpy
b=BeamNGpy('127.0.0.1',25252)
b.open(launch=False)
try:
    b.control.pause()
    print('SIMULATION_PAUSED',flush=True)
finally:
    b.disconnect()
