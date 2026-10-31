import hashlib
import json
from pathlib import Path

ROOT=Path(__file__).resolve().parents[2]
source=ROOT/'test_artifacts/green_house_search_20261014_trial1/evidence/green_house_trial1/first_abort_interface_fixture.json'
case=json.loads(source.read_text())
poses=case['topics']['/wildos/planned_path'][-1]['poses']
result={'source_sha256':hashlib.sha256(source.read_bytes()).hexdigest(),
        'scope':'Recorded single motion section and actual Bridge dispatch pose; no plant or obstacles',
        'frame':'spot1/odom','points':[p[:2] for p in poses],'headings':[p[2] for p in poses],
        'actual_start_pose':case['last_plan_status']['actual_start_pose'],
        'position_tolerance_m':.2,'heading_tolerance_rad':.35,'execution_radius_m':8.,
        'cpp_zero_delay_closest_m':.412002}
(ROOT/'nav2_adapter/test/fixtures/town_first_cusp_tracking_20261014.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf8')
