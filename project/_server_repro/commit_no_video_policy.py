from pathlib import Path
import os, subprocess, tempfile, zipfile, hashlib, json

root = Path(__file__).resolve().parents[1]
paths = [
    'AGENTS.md', 'docs/PROJECT_MEMORY.md',
    'nav2_adapter/wildos_nav2_adapter/cmd_vel_to_control.py',
    'nav2_adapter/test/test_cmd_vel_to_control_ros.py',
    'tools/town_physical_single_target_trial.py', 'tools/town_trial_harness.py',
    'tools/test_town_trial_harness.py', 'tools/server_test_preflight.py',
    'tools/image_stream_health.py', 'tools/test_image_stream_health.py',
    'tools/test_physical_trial_policy.py', 'tools/check_control_freshness_dds.py',
    'test_artifacts/offline_navigation/no_video_callbacks.log',
    'test_artifacts/server_validation_20261010/no_video_validation_evidence.zip',
]
def git(*args, env=None):
    return subprocess.check_output(['git', *args], cwd=root, env=env)
def unrelated():
    return [r for r in git('ls-files','--stage','-z').split(b'\0') if r and
            r.split(b'\t',1)[1].decode('utf-8') not in paths]
before=unrelated()
with tempfile.TemporaryDirectory(dir=root/'_server_repro') as temporary:
    env=os.environ.copy(); env['GIT_INDEX_FILE']=str(Path(temporary)/'index')
    git('read-tree','HEAD',env=env)
    git('add','--',*paths,env=env)
    git('-c','core.whitespace=blank-at-eol,blank-at-eof,space-before-tab,cr-at-eol',
        'diff','--cached','--check','--',*[p for p in paths if p.endswith(('.py','.md'))],env=env)
    print(git('commit','-m','fix: decouple physical trials from video and expose brake freshness',env=env).decode())
for p in paths:
    entry=git('ls-tree','HEAD','--',p).decode().split('\t',1)[0].split()
    git('update-index','--add','--cacheinfo',','.join((entry[0],entry[2],p)))
assert before==unrelated(), 'Unrelated staged entries changed'
print('COMMIT',git('rev-parse','HEAD').decode().strip())
source_paths=[p for p in paths if p.endswith('.py')]
archive=root/'_server_repro/no_video_freshness_candidate_final.zip'
with zipfile.ZipFile(archive,'w',zipfile.ZIP_DEFLATED) as z:
    for p in source_paths:z.write(root/p,p)
    manifest={p:hashlib.sha256((root/p).read_bytes()).hexdigest() for p in source_paths}
    z.writestr('no_video_final_sources.json',json.dumps(manifest,indent=2))
print('DEPLOY',str(archive),hashlib.sha256(archive.read_bytes()).hexdigest(),archive.stat().st_size)
