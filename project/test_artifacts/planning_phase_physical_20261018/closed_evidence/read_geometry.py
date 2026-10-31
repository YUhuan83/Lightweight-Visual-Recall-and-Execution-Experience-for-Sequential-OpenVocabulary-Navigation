"""Engine-only read of bbox vs measured rear axle while all demand is braking."""
from pathlib import Path
import json,sys,time
import numpy as np
import rclpy
from std_msgs.msg import String
from beamngpy import BeamNGpy
from beamngpy.connection.prefixed_length_socket import PrefixedLengthSocket
from wildos_nav2_adapter.fail_closed_transport import install_fail_closed_reads
from wildos_nav2_adapter.vehicle_geometry import SBR_BASE_OFFSET,SBR_FRONT_EXTENT,SBR_REAR_EXTENT,SBR_HALF_WIDTH
from wildos_nav2_adapter.physical_vehicle import read_physics_snapshot,physical_snapshot_errors
from bbox_geometry import rear_axle_extents

output=Path(sys.argv[1]);assert not output.exists()
install_fail_closed_reads(PrefixedLengthSocket)
rclpy.init();node=rclpy.create_node('stopped_bbox_geometry_observer')
latest={};report=dict(passed=False,scope=__doc__,samples=[],errors=[])
node.create_subscription(String,'/wildos/native_control_status',lambda m:latest.update(json.loads(m.data)),10)
bng=None
try:
    bng=BeamNGpy('127.0.0.1',25252,socket_timeout=5).open(launch=False)
    for _ in range(5):
        began=time.monotonic()
        while time.monotonic()-began<2.:
            rclpy.spin_once(node,timeout_sec=.02)
            if latest and 0<=time.time()-latest['wall_time']<.3:break
        assert latest,'No Native samples received in geometry observer'
        assert latest['speed']<.05,('Vehicle not stationary',latest['speed'])
        assert latest['sent']['throttle']==0. and latest['sent']['brake']>=.9
        physics=read_physics_snapshot(bng)
        assert not physical_snapshot_errors(physics)
        # Same read-only engine request used by installed GEVehiclesApi;
        # do not connect to or compete for the vehicle control TCP client.
        response=bng.vehicles._send(dict(type='GetBBoxCorners',vid='ego')).recv('BBoxCorners')
        corners=np.asarray(response['points'],dtype=float)
        local,extents=rear_axle_extents(corners,latest['position'],latest['direction'],
                                       latest['up'],SBR_BASE_OFFSET)
        report['samples'].append(dict(native_wall=latest['wall_time'],native=latest,
            physics=physics,world_corners=corners.tolist(),rear_axle_corners=local.tolist(),extents=extents))
        time.sleep(.1)
    maxima={k:max(x['extents'][k] for x in report['samples']) for k in ('front','rear','half_width')}
    nominal=dict(front=SBR_FRONT_EXTENT,rear=SBR_REAR_EXTENT,half_width=SBR_HALF_WIDTH)
    report.update(observed_maxima=maxima,nominal=nominal,
                  exceeding={k:maxima[k]-nominal[k] for k in nominal})
    report['passed']=all(maxima[k]<=nominal[k]+.03 for k in nominal)
except BaseException as exc:
    report['errors'].append(repr(exc))
finally:
    if bng:bng.disconnect()
    node.destroy_node();rclpy.shutdown()
    output.write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps({k:v for k,v in report.items() if k!='samples'},indent=2))
raise SystemExit(not report['passed'])
