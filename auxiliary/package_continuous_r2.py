import ast,hashlib,json,zipfile
from pathlib import Path
root=Path(r'C:\Users\Yu\Desktop\拟真\test_artifacts\continuous_exploration_20261024');candidate=root/'candidate';base=root/'server_v23_base'
payload={};hashes={}
with zipfile.ZipFile(root/'continuous_delta.zip') as z:
    for name in ('nav2_adapter/wildos_nav2_adapter/graph_path_bridge.py','nav2_adapter/wildos_nav2_adapter/continuous_graph_path_bridge.py','nav2_adapter/test/test_continuous_exploration_ros.py'):
        hashes[name]=hashlib.sha256(z.read(name)).hexdigest();payload[name]=(candidate/name).read_text(encoding='utf8')
        if name.endswith('/graph_path_bridge.py'):
            s=z.read(name).decode()
            needle='        self.init_memory_selection()\n        self.add_on_set_parameters_callback(self.on_parameters)'
            assert s.count(needle)==1
            s=s.replace(needle,'        self.init_memory_selection()\n        self.declare_strategy_parameters()\n        self.add_on_set_parameters_callback(self.on_parameters)')
            needle='    def status(self, state, **details):'
            assert s.count(needle)==1
            s=s.replace(needle,'    def declare_strategy_parameters(self):\n        """Declare immutable strategy settings before installing runtime guards."""\n        pass\n\n'+needle)
            payload[name]=s
    name='nav2_adapter/launch/physical_vehicle_control.launch.py';raw=z.read(name);s=raw.decode();hashes[name]=hashlib.sha256(raw).hexdigest()
    needle='        graph_profile = graph_geometry_profile(yaml.safe_load(stream))';assert s.count(needle)==1
    s=s.replace(needle,"        nav_config = yaml.safe_load(stream)\n        graph_profile = graph_geometry_profile(nav_config)\n        planner_radius = float(nav_config['planner_server']['ros__parameters']['CarLike']['minimum_turning_radius'])")
    needle="                 'planner_id': 'CarLike', 'minimum_turning_radius': 8.0}],";assert s.count(needle)==1
    s=s.replace(needle,"                 'planner_id': 'CarLike', 'minimum_turning_radius': 8.0,\n                 'planner_minimum_turning_radius': planner_radius}],");payload[name]=s
for name in ('tools/server_test_preflight.py','tools/wildos_console.py'):
    raw=(base/Path(name).name).read_bytes();s=raw.decode();hashes[name]=hashlib.sha256(raw).hexdigest()
    if name.endswith('server_test_preflight.py'):
        needle="    'graph_bridge': ('graph_path_bridge',),";assert s.count(needle)==1;s=s.replace(needle,"    'graph_bridge': ('graph_path_bridge', 'continuous_graph_path_bridge'),")
        needle='        for owner, fields in expected_runtime.items():\n';assert s.count(needle)==1
        new=(candidate/name).read_text(encoding='utf8');start=new.index('        continuous_owner = any(');end=new.index(needle,start)
        s=s.replace(needle,new[start:end]+needle)
    else:
        needle="                   'cmd_vel_to_control', 'kinematic_vehicle_control', 'graph_path_bridge',";assert s.count(needle)==1
        s=s.replace(needle,needle+"\n                   'continuous_graph_path_bridge',")
    ast.parse(s);payload[name]=s
name='tools/test_continuous_process_roles.py';payload[name]=(candidate/name).read_text(encoding='utf8')
with zipfile.ZipFile(root/'continuous_r2_delta.zip','w',zipfile.ZIP_DEFLATED) as z:
    z.writestr('base_hashes.json',json.dumps(hashes))
    for name,s in payload.items():ast.parse(s);z.writestr(name,s)
print(json.dumps({name:hashlib.sha256(s.encode()).hexdigest() for name,s in payload.items()},indent=2))
