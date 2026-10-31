"""Overlay only this arc-heading delta onto frozen v22, preserving unrelated dirty work."""
import ast,hashlib,json,zipfile
from pathlib import Path
repo=Path(r'C:\Users\Yu\Desktop\拟真');art=repo/'test_artifacts/fresh_command_retry_20261023';base=art/'v22_base'
names=['nav2_adapter/wildos_nav2_adapter/graph_path_bridge.py','nav2_adapter/wildos_nav2_adapter/graph_route.py',
       'nav2_adapter/test/test_graph_path_bridge_routing.py','nav2_adapter/test/test_graph_route.py']
payload={}
for name in names:
    old=(base/Path(name).name).read_text(encoding='utf8');new=(repo/name).read_text(encoding='utf8')
    if name.endswith('/graph_path_bridge.py'):
        old=old.replace('                          route_candidates, validate_vehicle_path, validate_local_prefix,',
                        '                          route_candidates, forward_arc_arrival_heading,\n                          validate_vehicle_path, validate_local_prefix,',1)
        marker='        if path is validated_path and route == self.current_route_points():\n'
        assert old.count(marker)==1;old=old.replace(marker,'            if self.compare_curvature_heading(path, target, cost):\n                return\n'+marker)
        start=new.index('    def compare_curvature_heading(');end=new.index('    def exploration_motion_issue(',start)
        marker='    def exploration_motion_issue(';assert old.count(marker)==1;old=old.replace(marker,new[start:end]+marker)
    elif name.endswith('/graph_route.py'):
        start=new.index('def forward_arc_arrival_heading(');end=new.index('def route_candidates(',start)
        marker='def route_candidates(';assert old.count(marker)==1;old=old.replace(marker,new[start:end]+marker)
    elif name.endswith('/test_graph_path_bridge_routing.py'):
        start=new.index('    def prepare_non_directional_shunt_result(');end=new.index('    def test_two_headings_choose_shorter_path_without_reversals(',start)
        marker='    def test_two_headings_choose_shorter_path_without_reversals(';assert old.count(marker)==1;old=old.replace(marker,new[start:end]+marker)
    else:
        old=old.replace('    validate_vehicle_path, validate_ideal_path, validate_ideal_local_prefix,',
                        '    validate_vehicle_path, validate_ideal_path, validate_ideal_local_prefix,\n    forward_arc_arrival_heading,',1)
        start=new.index('    def test_forward_arc_arrival_heading_matches_circle_and_rotation(');end=new.index('    def test_same_raw_smooth_fixtures_as_actual_cpp_contract(',start)
        marker='    def test_same_raw_smooth_fixtures_as_actual_cpp_contract(';assert old.count(marker)==1;old=old.replace(marker,new[start:end]+marker)
    ast.parse(old);payload[name]=old
with zipfile.ZipFile(art/'v23_delta.zip','w',zipfile.ZIP_DEFLATED) as z:
    z.writestr('base_hashes.json',json.dumps({name:hashlib.sha256((base/Path(name).name).read_bytes()).hexdigest() for name in names}))
    for name,value in payload.items():z.writestr(name,value)
print(json.dumps({name:hashlib.sha256(value.encode()).hexdigest() for name,value in payload.items()},indent=2))
