"""Read closed physical bag; compare returned maneuver and exact reverse on every public map.

Public OccupancyGrid snapshots are not the private C++ map version at search time.
This diagnostic never publishes, opens BeamNG, or commands motion.
"""
import hashlib
import json
import math
from pathlib import Path
import sqlite3

from nav_msgs.msg import OccupancyGrid
from rclpy.serialization import deserialize_message
from wildos_nav2_adapter.graph_route import validate_vehicle_path, validate_local_prefix

root = Path('/home/ly/vehicle_profile_repair_20261019/short_20261019_225630')
output = Path(__file__).resolve().parent / 'closed_map_review'
output.mkdir(exist_ok=False)
report = json.loads((root / 'execution/result.json').read_text())
x, y, yaw = report['initial']
direct = [(x - 3. * i / 60 * math.cos(yaw), y - 3. * i / 60 * math.sin(yaw)) for i in range(61)]
planned = report['plans'][0]
db = root / 'interfaces/interfaces_0.db3'
connection = sqlite3.connect('file:' + str(db) + '?mode=ro', uri=True)
results = []
for topic_id, topic in connection.execute('select id,name from topics where id in (12,15)'):
    for index, (timestamp, payload) in enumerate(connection.execute(
            'select timestamp,data from messages where topic_id=? order by timestamp,id', (topic_id,))):
        grid = deserialize_message(payload, OccupancyGrid)
        item = dict(topic=topic, index=index, bag_timestamp_ns=timestamp,
                    source_timestamp_ns=grid.header.stamp.sec * 10**9 + grid.header.stamp.nanosec,
                    frame=grid.header.frame_id, width=grid.info.width, height=grid.info.height,
                    resolution=grid.info.resolution, origin=[grid.info.origin.position.x,grid.info.origin.position.y])
        for name, points, headings in [('direct_reverse',direct,[yaw]*61),
                                       ('returned_maneuver',planned['points'],planned['headings'])]:
            reason = validate_vehicle_path(points, headings, [direct[0],points[-1]], grid, points[-1],
                                            start=direct[0], minimum_turning_radius=8., heading_quantization_bins=72)
            local_reason = validate_local_prefix(points,headings,grid,direct[0],yaw)
            item[name] = dict(full_path_reason=reason,local_prefix_reason=local_reason)
        # Lossless public grid + exact request fixture, not an inferred private map.
        fixture = output / ('global' if topic_id==15 else 'local')
        fixture = fixture.with_name(fixture.name + f'_{index}.txt')
        desired = direct[-1]
        with fixture.open('x') as stream:
            stream.write(f'{grid.info.width} {grid.info.height} {grid.info.resolution:.17g} '
                         f'{grid.info.origin.position.x:.17g} {grid.info.origin.position.y:.17g}\n')
            stream.write(f'{x:.17g} {y:.17g} {yaw:.17g} '
                         f'{desired[0]:.17g} {desired[1]:.17g} {yaw:.17g}\n')
            stream.write(' '.join(map(str,grid.data))+'\n')
        item['fixture'] = fixture.name
        results.append(item)
connection.close()
result = dict(scope=__doc__, db_sha256=hashlib.sha256(db.read_bytes()).hexdigest(),
              initial=report['initial'], desired=[*direct[-1],yaw],
              physical_motion_goals=report['motion_goals'], snapshots=results)
(output/'result.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(dict(initial=result['initial'],desired=result['desired'],snapshots=results),indent=2))
