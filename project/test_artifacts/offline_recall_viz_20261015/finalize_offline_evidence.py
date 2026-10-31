"""Seal current evidence and verify frozen sources in an independent unpack."""
import hashlib,json,os,shutil,subprocess,sys,zipfile
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2];OUT=Path(__file__).parent
EXPECTED='dfdb40d7493274da9b0397c8a8dc3e3473368ff2cbdff3a12ef11c3ccd65eb53'
bundle=ROOT/'official_wildos_beamng_bundle.zip'
assert hashlib.sha256(bundle.read_bytes()).hexdigest()==EXPECTED
manifest=json.loads(bundle.with_suffix('.manifest.json').read_text());assert manifest['sha256']==EXPECTED
unpacked=OUT/'final_unpacked';unpacked.mkdir(exist_ok=False)
with zipfile.ZipFile(bundle) as z:
    assert z.testzip() is None and set(z.namelist())==set(manifest['members'])
    for name,digest in manifest['members'].items():
        path=(unpacked/name).resolve();assert path.is_relative_to(unpacked.resolve())
        data=z.read(name);assert hashlib.sha256(data).hexdigest()==digest
        assert data==(ROOT/name).read_bytes(),name
        path.parent.mkdir(parents=True,exist_ok=True);path.write_bytes(data)
environment=dict(os.environ)
for key in ('PYTHONPATH','PYTHONHOME'):environment.pop(key,None)
checks=[]
for name,argv in (
    ('unpacked_callbacks',[sys.executable,'tools/offline_callback_check.py']),
    ('unpacked_goal_chain',[sys.executable,'-m','unittest','discover','-s','tools','-p','test_offline_goal_chain_replay.py']),
    ('qt_layout',[sys.executable,'tools/configure_rviz_layout.py','--check'])):
    result=subprocess.run(argv,cwd=unpacked,env=environment,capture_output=True,timeout=60)
    (OUT/(name+'.log')).write_bytes(result.stdout+result.stderr)
    checks.append({'name':name,'returncode':result.returncode});assert result.returncode==0,name
source=ROOT/'test_artifacts/offline_navigation'
for name in ('report.json','report.md','callback_report.json','test_offline_goal_chain_replay.log',
             'test_recall_component_names.log','test_scene_display.log','test_rviz_configuration.log',
             'test_graph_route.log','test_tracking_preview.log','test_dds_profile.log'):
    shutil.copyfile(source/name,OUT/name)
report=json.loads((OUT/'report.json').read_text());counts={}
for row in report['checks']:counts[row['status']]=counts.get(row['status'],0)+1
assert counts.get('FAIL',0)==0
renames=json.loads((OUT/'rename_equivalence.json').read_text());assert renames['passed']
callback=json.loads((OUT/'callback_report.json').read_text());assert callback['passed']
freeze=ROOT/'deploy/20261015_offline_recall_viz_candidate';freeze.mkdir(exist_ok=False)
for suffix in ('.zip','.manifest.json','.sha256'):
    shutil.copyfile(bundle.with_suffix(suffix),freeze/('official_wildos_beamng_bundle'+suffix))
summary={'date':'2026-10-15','offline_only':True,'counts':counts,'callback_tests':callback['tests'],
    'synthetic_goal_chain_tests':4,'rename_equivalent_production_files':len(renames['files']),
    'unchanged_cpp_files':renames['unchanged_cpp_files'],'independent_checks':checks,
    'frozen_bundle_sha256':EXPECTED,'members':len(manifest['members']),
    'freeze_path':str(freeze),'field_acceptance':False,'commit_created':False,
    'pending':['Actual Qt5/RViz/Ogre layout and DDS stream/overlay validation',
               'Physical tracking and complete green-house search; 20-round gate remains unmet'],
    'warnings':[row for row in report['checks'] if row['status']=='WARN']}
(OUT/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding='utf8')
(freeze/'CANDIDATE.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding='utf8')
print(json.dumps({k:v for k,v in summary.items() if k not in ('warnings','pending')},ensure_ascii=False))
