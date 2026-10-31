"""Remove this turn's config/test additions; retain the user's other files."""
from pathlib import Path
import hashlib

root=Path(r'C:\Users\Yu\Desktop\拟真')
evidence=Path(__file__).resolve().parent/'main_change'
for relative,backup in (
    ('nav2_adapter/config/nav2_ackermann_control.yaml','nav2_ackermann_control.before.yaml'),
    ('nav2_adapter/test/test_runtime_contract.py','test_runtime_contract.before.py'),
):
    path=root/relative;raw=path.read_bytes();nl='\r\n' if b'\r\n' in raw else '\n'
    text=raw.decode('utf-8')
    if path.suffix=='.yaml':
        remove=f'          # Include delayed scans between map updates; expire in source time.{nl}          observation_persistence: 1.0{nl}'
        assert text.count(remove)==1;text=text.replace(remove,'')
    else:
        first=text.index('    def test_marking_window_covers_recorded_intermediate_frame_and_expires(self):')
        last=text.index('    def test_frontier_missing_window_is_explicit_in_graph_entry_and_runtime_gate(self):',first)
        text=text[:first]+text[last:]
    final=text.encode('utf-8')
    assert final==(evidence/backup).read_bytes(), 'Other edits exist: inspect rather than overwrite'
    path.write_bytes(final)
    print(relative,hashlib.sha256(final).hexdigest())
