import hashlib,json,re,zipfile
from pathlib import Path
repo=Path(r'C:\Users\Yu\Desktop\拟真');root=repo/'test_artifacts/continuous_exploration_20261024';candidate=root/'candidate'
names={'test_artifacts/server_validation_20261002/house_localization_replay.json',
       'test_artifacts/server_validation_20261002/accepted_memory_prefix_snapshot.npz',
       'test_artifacts/server_validation_20261002/prefix_replay.json'}
names.add('captures/20261008_B_green_house_station_trial12_repeat_geometry/stationary_sensor_calibration.json')
for directory in (repo/'nav2_adapter/test',repo/'wildos_port/repro_check/src/nebula2-wildos/visual_navigation/test'):
    names.update(str(p.relative_to(repo)).replace('\\','/') for p in directory.rglob('*') if p.is_file() and p.suffix in ('.npz','.npy','.png','.jpg'))
manifest={}
with zipfile.ZipFile(root/'baseline_test_fixtures.zip','w',zipfile.ZIP_DEFLATED) as z:
    for name in sorted(names):
        data=(repo/name).read_bytes();manifest[name]=dict(sha256=hashlib.sha256(data).hexdigest(),bytes=len(data))
        z.writestr(name,data);target=candidate/name;target.parent.mkdir(parents=True,exist_ok=True);target.write_bytes(data)
(root/'baseline_test_fixtures_manifest.json').write_text(json.dumps(manifest,indent=2),encoding='utf8')
print(json.dumps(dict(files=len(manifest),bytes=sum(v['bytes'] for v in manifest.values()))))
