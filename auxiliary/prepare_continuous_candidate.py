"""Add explicit scheduling hooks only inside the preserved candidate checkout."""
import ast,json,zipfile
from pathlib import Path
repo=Path(r'C:\Users\Yu\Desktop\拟真');root=repo/'test_artifacts/continuous_exploration_20261024';candidate=root/'candidate'
assert (root/'baseline_manifest.json').is_file()
with zipfile.ZipFile(root/'baseline_source.zip') as archive:
    for name in archive.namelist():
        if '/resource/' in name and not name.startswith('_git_state/'):
            p=candidate/name;p.parent.mkdir(parents=True,exist_ok=True);p.write_bytes(archive.read(name))

path=candidate/'nav2_adapter/wildos_nav2_adapter/graph_path_bridge.py';s=path.read_text(encoding='utf8')
needle="        metadata = self.exploration_metadata(self.route)\n        if metadata is not None and metadata['exploration_id'] == identity:\n"
assert s.count(needle)==1
s=s.replace(needle,"        metadata = self.exploration_metadata(self.route)\n        if self.retain_motion_for_retired_intent(data, metadata):\n            return\n        if metadata is not None and metadata['exploration_id'] == identity:\n")
needle='            elif (index+1 < len(self.candidates)\n'
assert s.count(needle)==1;s=s.replace(needle,'            elif (self.should_compare_endpoint_headings(path)\n                  and index+1 < len(self.candidates)\n')
needle='    def compare_curvature_heading('
hooks='''    def prepare_intent_change(self):
        return False

    def retain_motion_for_retired_intent(self, data, metadata):
        return False

    def release_continuous_visible_intent(self, reason):
        return False

    def should_compare_endpoint_headings(self, path):
        return True

'''
assert s.count(needle)==1;s=s.replace(needle,hooks+needle);ast.parse(s);path.write_text(s,encoding='utf8')

path=candidate/'nav2_adapter/wildos_nav2_adapter/visible_approach.py';s=path.read_text(encoding='utf8')
needle='        draining_motion = self.retired_motion is not None\n'
assert s.count(needle)==1;s=s.replace(needle,'        draining_motion = (self.retired_motion is not None\n                           or self.prepare_intent_change())\n')
needle='        if getattr(self, \'lookback_active\', False):\n'
start=s.index('    def release_visible_approach(');pos=s.index(needle,start)
s=s[:pos]+"        if self.release_continuous_visible_intent(reason):\n            return\n"+s[pos:];ast.parse(s);path.write_text(s,encoding='utf8')

path=candidate/'nav2_adapter/wildos_nav2_adapter/confirmed_approach.py';s=path.read_text(encoding='utf8')
needle='        self.reset()\n        # Ownership passes'
assert s.count(needle)==1;s=s.replace(needle,'        if not self.prepare_intent_change():\n            self.reset()\n        # Ownership passes');ast.parse(s);path.write_text(s,encoding='utf8')

path=candidate/'nav2_adapter/setup.py';s=path.read_text(encoding='utf8')
needle='            "graph_path_bridge = wildos_nav2_adapter.graph_path_bridge:main",'
assert s.count(needle)==1;s=s.replace(needle,needle+'\n            "continuous_graph_path_bridge = wildos_nav2_adapter.continuous_graph_path_bridge:main",');path.write_text(s,encoding='utf8')
path=candidate/'nav2_adapter/launch/physical_vehicle_control.launch.py';s=path.read_text(encoding='utf8')
assert s.count("executable='graph_path_bridge'")==1;s=s.replace("executable='graph_path_bridge'","executable='continuous_graph_path_bridge'");path.write_text(s,encoding='utf8')

# Keep the unrelated, not-implemented 0.3s parameter experiment in its archive.
# The continuous-only candidate retains the baseline's actual 2s setting.
path=candidate/'nav2_adapter/test/test_cmd_vel_to_control_ros.py';s=path.read_text(encoding='utf8')
begin=s.index('    def physical_shift_profile(');end=s.index('    def test_opposite_rolling_brakes_in_both_gears(',begin)
(root/'pending_shift_profile_tests.txt').write_text(s[begin:end],encoding='utf8')
s=s[:begin]+s[end:];ast.parse(s);path.write_text(s,encoding='utf8')
print(json.dumps(dict(candidate=str(candidate),baseline_untouched=True,shift_confirmation_seconds=2.,pending_shift_tests=str(root/'pending_shift_profile_tests.txt'))))
