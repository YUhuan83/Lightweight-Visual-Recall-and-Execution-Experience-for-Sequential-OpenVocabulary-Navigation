"""Restore the pre-fence diagnostic test source only when its recorded hash matches."""
from pathlib import Path
import hashlib
import json

root=Path(__file__).resolve().parent
record=json.loads((root/'frame_20261019_083934/result.json').read_text())
source=(root/'test_frame_clock.py').read_text()
source=source.replace('from execute_recorded import post_enable_owner_ready\n','')
begin=source.index('    def test_pre_enable_coherent_cached_owner_cannot_admit_action')
end=source.index('    def test_bridge_anchor_round_trip_preserves_engine_elapsed_time')
source=source[:begin]+source[end:]
data=source.encode()
want=record['diagnostic_files']['test_frame_clock.py']
assert hashlib.sha256(data).hexdigest()==want
target=root/'test_frame_clock.before_enable_fence.py'
assert not target.exists();target.write_bytes(data)
print(json.dumps(dict(restored=str(target),recorded_sha256=want)))
