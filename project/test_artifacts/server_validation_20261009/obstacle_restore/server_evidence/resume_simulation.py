from pathlib import Path
import json
from beamngpy import BeamNGpy
from beamngpy.connection.prefixed_length_socket import PrefixedLengthSocket
from wildos_nav2_adapter.fail_closed_transport import install_fail_closed_reads
install_fail_closed_reads(PrefixedLengthSocket)
b=BeamNGpy('127.0.0.1',25252,home='/home/ly/beamng-install-0.39.4.0/BeamNG.tech.v0.39.4.0',user='/home/ly/beamng-east-coast-user/current',headless=True,quit_on_close=False,socket_timeout=10)
b.open(launch=False)
b.control.resume()
Path('/home/ly/town_memory_validation_20261009/obstacle_restore/resume_simulation.json').write_text(json.dumps({'GE_handshake':True,'simulation_resumed':True,'motion_enabled':False}))
b.disconnect()
print('SIMULATION_RESUMED',flush=True)
