import ast,difflib,hashlib,json,zipfile
from pathlib import Path
art=Path(r'C:\Users\Yu\Desktop\拟真\test_artifacts');out=art/'confirmation_window_repair_20261025'
c=out/'candidate';base=art/'execution_handoff_repair_20261025/candidate'
base_names=set(json.loads((out/'base_hashes.json').read_text()))
extra={'nav2_adapter/test/test_confirmation_profile_ros.py','nav2_adapter/test/fixtures/v29_r2_sparse_localizations.json'}
names=base_names|extra
changed={n for n in names if not (base/n).exists() or hashlib.sha256((base/n).read_bytes()).hexdigest()!=hashlib.sha256((c/n).read_bytes()).hexdigest()}
runtime={n for n in changed if '/test/' not in n}
expected={'official_adapter/launch/official_graph_stack.launch.py',
'wildos_port/repro_check/src/nebula2-wildos/visual_navigation/visual_navigation/explorfm_triangulation/localization_geometry.py',
'wildos_port/repro_check/src/nebula2-wildos/visual_navigation/visual_navigation/explorfm_triangulation/obj_mask_triangulation.py'}
assert runtime==expected,runtime
launch='official_adapter/launch/official_graph_stack.launch.py'
old=(base/launch).read_text(encoding='utf8');new=(c/launch).read_text(encoding='utf8')
assert new.replace('                         "confirmation_window_sec": 60.0,\n','')==old
diff=[]
with zipfile.ZipFile(out/'v30_delta_from_v28.zip','w',zipfile.ZIP_DEFLATED) as z:
    z.writestr('base_hashes.json',json.dumps({n:hashlib.sha256((base/n).read_bytes()).hexdigest() for n in changed if (base/n).exists()}))
    for n in sorted(changed):
        z.write(c/n,n)
        if n.endswith('.py'):
            a=(base/n).read_text(encoding='utf8') if (base/n).exists() else ''
            b=(c/n).read_text(encoding='utf8')
            diff.extend(difflib.unified_diff(a.splitlines(True),b.splitlines(True),fromfile='v28/'+n,tofile='v30/'+n))
(out/'v28_to_v30.diff').write_text(''.join(diff),encoding='utf8')
report=dict(runtime={n:hashlib.sha256((c/n).read_bytes()).hexdigest() for n in sorted(runtime)},
            changed_members=len(changed),confirmation_history_sec=60.,latest_localization_age_sec=5.,support=4,
            scope='v29_r2 range geometry plus one explicit launch parameter; v28 vehicle/planning/control unchanged')
(out/'source_scope.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
