import csv
import hashlib
import json
from pathlib import Path

ROOT=Path(__file__).resolve().parents[2]
source=ROOT/'test_artifacts/green_house_search_20261014_trial1/evidence/green_house_trial1/first_abort_interface_fixture.json'
case=json.loads(source.read_text())
path=case['topics']['/wildos/planned_path'][-1]['poses']
target=ROOT/'ackermann_rpp/test/fixtures/town_first_cusp_section_20261014.csv'
if target.exists(): raise FileExistsError(target)
with target.open('w',newline='',encoding='ascii') as handle:
    writer=csv.writer(handle,lineterminator='\n')
    writer.writerow(['kind','x','y','yaw'])
    writer.writerow(['start',*case['last_plan_status']['actual_start_pose']])
    for pose in path: writer.writerow(['path',*pose])
print(json.dumps({'poses':len(path),'source_sha256':hashlib.sha256(source.read_bytes()).hexdigest(),
                  'fixture_sha256':hashlib.sha256(target.read_bytes()).hexdigest()}))
