"""Apply scheduling hooks to exact frozen server sources, not the dirty main tree."""
import ast,hashlib,json,zipfile
from pathlib import Path
root=Path(r'C:\Users\Yu\Desktop\拟真\test_artifacts\continuous_exploration_20261024')
candidate=root/'candidate';base=root/'server_v23_base';payload={}
name='nav2_adapter/wildos_nav2_adapter/graph_path_bridge.py';s=(base/'graph_path_bridge.py').read_text(encoding='utf8');new=(candidate/name).read_text(encoding='utf8')
needle="        metadata = self.exploration_metadata(self.route)\n        if metadata is not None and metadata['exploration_id'] == identity:\n";assert s.count(needle)==1
s=s.replace(needle,"        metadata = self.exploration_metadata(self.route)\n        if self.retain_motion_for_retired_intent(data, metadata):\n            return\n        if metadata is not None and metadata['exploration_id'] == identity:\n")
needle='            elif (index+1 < len(self.candidates)\n';assert s.count(needle)==1;s=s.replace(needle,'            elif (self.should_compare_endpoint_headings(path)\n                  and index+1 < len(self.candidates)\n')
start=new.index('    def prepare_intent_change(');end=new.index('    def compare_curvature_heading(',start)
needle='    def compare_curvature_heading(';assert s.count(needle)==1;s=s.replace(needle,new[start:end]+needle);payload[name]=s
name='nav2_adapter/wildos_nav2_adapter/visible_approach.py';s=(base/'visible_approach.py').read_text(encoding='utf8')
needle='        draining_motion = self.retired_motion is not None\n';assert s.count(needle)==1;s=s.replace(needle,'        draining_motion = (self.retired_motion is not None\n                           or self.prepare_intent_change())\n')
start=s.index('    def release_visible_approach(');pos=s.index("        if getattr(self, 'lookback_active', False):\n",start)
s=s[:pos]+"        if self.release_continuous_visible_intent(reason):\n            return\n"+s[pos:];payload[name]=s
name='nav2_adapter/wildos_nav2_adapter/confirmed_approach.py';s=(base/'confirmed_approach.py').read_text(encoding='utf8')
needle='        self.reset()\n        # Ownership passes';assert s.count(needle)==1;s=s.replace(needle,'        if not self.prepare_intent_change():\n            self.reset()\n        # Ownership passes');payload[name]=s
for name in ('nav2_adapter/setup.py','nav2_adapter/launch/physical_vehicle_control.launch.py'):
    s=(base/Path(name).name).read_text(encoding='utf8')
    if name.endswith('setup.py'):
        needle='            "graph_path_bridge = wildos_nav2_adapter.graph_path_bridge:main",';assert s.count(needle)==1
        s=s.replace(needle,needle+'\n            "continuous_graph_path_bridge = wildos_nav2_adapter.continuous_graph_path_bridge:main",')
    else:
        needle="executable='graph_path_bridge'";assert s.count(needle)==1;s=s.replace(needle,"executable='continuous_graph_path_bridge'")
    payload[name]=s
for name in ('nav2_adapter/wildos_nav2_adapter/continuous_graph_path_bridge.py','nav2_adapter/test/test_continuous_exploration_ros.py'):
    payload[name]=(candidate/name).read_text(encoding='utf8')
hashes={name:hashlib.sha256((base/Path(name).name).read_bytes()).hexdigest() for name in payload if (base/Path(name).name).exists()}
with zipfile.ZipFile(root/'continuous_delta.zip','w',zipfile.ZIP_DEFLATED) as archive:
    archive.writestr('base_hashes.json',json.dumps(hashes))
    for name,text in payload.items():ast.parse(text);archive.writestr(name,text)
print(json.dumps({name:hashlib.sha256(value.encode()).hexdigest() for name,value in payload.items()},indent=2))
