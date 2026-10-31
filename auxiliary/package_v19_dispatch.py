"""Package only synchronous dispatch reuse against the frozen v18 files."""
import hashlib
import json
import zipfile
from pathlib import Path

repo = Path(r'C:\Users\Yu\Desktop\拟真')
art = repo/'test_artifacts/fresh_command_retry_20261023'
base = art/'v18_base'
members = ['nav2_adapter/wildos_nav2_adapter/graph_path_bridge.py',
           'nav2_adapter/test/test_graph_path_bridge_routing.py',
           'nav2_adapter/test/test_planner_execution_ownership_ros.py']
payload = {}
old = (base/'graph_path_bridge.py').read_text(encoding='utf-8')
new = (repo/members[0]).read_text(encoding='utf-8')
start = old.index('    def on_plan_result(')
end = old.index('    def exploration_motion_issue(', start)
body = old[start:end]
assert body.count('        self.full_path_pub.publish(path)') == 1
body = body.replace('        self.full_path_pub.publish(path)',
                    '        validated_path = path\n        self.full_path_pub.publish(path)')
branch = new[new.index('        if path is validated_path'):new.index('    def exploration_motion_issue(')]
assert body.endswith('        self.execute_checked_path(path, target)\n\n')
body = body[:-len('        self.execute_checked_path(path, target)\n\n')]+branch
old = old[:start]+body+old[end:]
start = old.index('    def execute_checked_path(')
end = old.index('    def send_motion_segment(', start)
body = old[start:end]
needle = '        self.pending = False\n        if not self.follow_enabled:'
assert body.count(needle) == 1
handoff = new[new.index('        self.dispatch_validated_path(path, target, points, headings)\n\n    def dispatch_validated_path'):new.index('        if not self.follow_enabled:', new.index('    def dispatch_validated_path'))]
body = body.replace(needle, handoff+'        if not self.follow_enabled:')
payload[members[0]] = old[:start]+body+old[end:]

old = (base/'test_graph_path_bridge_routing.py').read_text(encoding='utf-8')
new = (repo/members[1]).read_text(encoding='utf-8')
marker = '    def test_local_collision_prevents_follow_despite_free_global_map('
tests = new[new.index('    def prepare_synchronous_dispatch_result('):new.index(marker)]
assert old.count(marker) == 1 and 'prepare_synchronous_dispatch_result' not in old
payload[members[1]] = old.replace(marker, tests+marker)
old = (base/'test_planner_execution_ownership_ros.py').read_text(encoding='utf-8')
new = (repo/members[2]).read_text(encoding='utf-8')
assert old.replace('self.node.execute_checked_path = Mock()', 'self.node.send_motion_segment = Mock()').replace('self.node.execute_checked_path.assert_called_once()', 'self.node.send_motion_segment.assert_called_once()') == new
payload[members[2]] = new
hashes = {name: hashlib.sha256((base/Path(name).name).read_bytes()).hexdigest() for name in members}
with zipfile.ZipFile(art/'v19_delta.zip', 'w', zipfile.ZIP_DEFLATED) as archive:
    archive.writestr('base_hashes.json', json.dumps(hashes))
    for name, content in payload.items():
        compile(content, name, 'exec')
        archive.writestr(name, content)
print(json.dumps({name: hashlib.sha256(content.encode()).hexdigest() for name, content in payload.items()}, indent=2))
