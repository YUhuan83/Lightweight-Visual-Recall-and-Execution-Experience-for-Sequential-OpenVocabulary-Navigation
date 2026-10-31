from pathlib import Path
import os, subprocess, tempfile, sys
root=Path(__file__).resolve().parents[1]
baseline=['tools/build_official_wildos.sh','tools/package_official_bundle.py']
paths=baseline if sys.argv[1]=='baseline' else (
    [p.relative_to(root).as_posix() for p in (root/'ackermann_smac').rglob('*') if p.is_file()]
    +baseline+['nav2_adapter/package.xml','nav2_adapter/config/nav2_ackermann_control.yaml',
    'nav2_adapter/wildos_nav2_adapter/runtime_contract.py','nav2_adapter/test/test_runtime_contract.py',
    'nav2_adapter/wildos_nav2_adapter/graph_path_bridge.py','nav2_adapter/test/test_initial_path_join_ros.py',
    'tools/test_start_connector_field_evidence.py','tools/test_continuous_start_evidence.py',
    'tools/check_continuous_start_execution.py','tools/test_continuous_start_execution.py',
    'docs/PROJECT_MEMORY.md','test_artifacts/server_validation_20261011/CONTINUOUS_START.md',
    'test_artifacts/server_validation_20261011/continuous_start_evidence.zip',
    'test_artifacts/offline_navigation/continuous_start_callbacks.log',
    'test_artifacts/offline_navigation/continuous_start_all_callbacks.log',
    'test_artifacts/offline_navigation/continuous_start_regressions.log'])
def git(*args,env=None): return subprocess.check_output(['git',*args],cwd=root,env=env)
def unrelated():
    return [r for r in git('ls-files','--stage','-z').split(b'\0') if r and
        r.split(b'\t',1)[1].decode('utf-8') not in paths]
before=unrelated()
with tempfile.TemporaryDirectory(dir=root/'_server_repro') as temporary:
    env=os.environ.copy();env['GIT_INDEX_FILE']=str(Path(temporary)/'index')
    git('read-tree','HEAD',env=env);git('add','--',*paths,env=env)
    git('-c','core.whitespace=blank-at-eol,blank-at-eof,space-before-tab,cr-at-eol',
        'diff','--cached','--check','--',*[p for p in paths if p.endswith(('.cpp','.hpp','.xml','.yaml','.py','.md'))],env=env)
    print(git('commit','-m',('chore: checkpoint existing deployment tools before planner integration'
        if sys.argv[1]=='baseline' else 'fix: connect continuous vehicle starts to bounded Smac planning'),env=env).decode())
for p in paths:
    entry=git('ls-tree','HEAD','--',p).decode().split('\t',1)[0].split()
    git('update-index','--add','--cacheinfo',','.join((entry[0],entry[2],p)))
assert before==unrelated(),'Unrelated staged entries changed'
print('COMMIT',git('rev-parse','HEAD').decode().strip())
