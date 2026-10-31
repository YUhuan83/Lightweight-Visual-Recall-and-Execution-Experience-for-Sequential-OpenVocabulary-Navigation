"""Software planning controls, no DDS or BeamNG execution."""
import csv,json,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT/'nav2_adapter'),str(ROOT/'nav2_adapter/test')]
from offline_ros_fakes import install
install()
from test_graph_path_bridge_routing import GraphBridgeRoutingTests
from wildos_nav2_adapter.motion_path import motion_sections
from wildos_nav2_adapter.tracking_preview import preview_tracking
from wildos_nav2_adapter.graph_route import validate_vehicle_path

output=[]
for file in sorted(Path(__file__).parent.glob('recorded_town_replanned*.csv')):
    rows=list(csv.reader(file.open()))[1:]
    start=tuple(map(float,rows[0][1:]));pp=[tuple(map(float,r[1:])) for r in rows[1:]]
    pts=[p[:2] for p in pp];hh=[p[2] for p in pp]
    grid=GraphBridgeRoutingTests.make_grid()
    grid.info.width=grid.info.height=320
    grid.info.resolution=.25
    grid.info.origin.position.x=690.;grid.info.origin.position.y=-40.
    grid.data=[0]*102400
    reason=validate_vehicle_path(pts,hh,pts,grid,pts[-1],start=start[:2],
        minimum_turning_radius=8.,heading_quantization_bins=72)
    out=[];current=start
    for first,last in motion_sections(pts):
        r=preview_tracking(pts[first:last+1],hh[first:last+1],current[:2],current[2],8.)
        out.append(dict(reached=r.reached,reason=r.reason,closest=r.closest_m,poses=last-first+1))
        current=r.trajectory[-1]
    output.append(dict(file=file.name,path_poses=len(pp),geometry_issue=reason,sections=out))
target=Path(__file__).parent/'planning_matrix.json'
target.write_text(json.dumps(output,indent=2),encoding='utf8')
print(json.dumps(output,indent=2))
