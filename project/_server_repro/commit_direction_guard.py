from pathlib import Path
import os,subprocess,tempfile

root=Path(__file__).resolve().parents[1]
paths=[
    'ackermann_rpp/src/controller.cpp', 'ackermann_rpp/test/test_controller.cpp',
    'ackermann_rpp/test/fixtures/pilot3_motion_sections.csv', 'ackermann_rpp/README.md',
    'docs/PROJECT_MEMORY.md',
    'test_artifacts/server_validation_20261010/PILOT3_DIRECTION_REPAIR.md',
    'test_artifacts/server_validation_20261010/pilot3_closed_evidence.zip',
    'test_artifacts/server_validation_20261010/direction_validation_evidence.zip',
    'test_artifacts/offline_navigation/direction_runtime_contract.log',
]
def git(*args,env=None):
    return subprocess.check_output(['git',*args],cwd=root,env=env)
def unrelated():
    return [r for r in git('ls-files','--stage','-z').split(b'\0') if r and
            r.split(b'\t',1)[1].decode('utf-8') not in paths]
before=unrelated()
with tempfile.TemporaryDirectory(dir=root/'_server_repro') as temporary:
    env=os.environ.copy();env['GIT_INDEX_FILE']=str(Path(temporary)/'index')
    git('read-tree','HEAD',env=env);git('add','--',*paths,env=env)
    git('-c','core.whitespace=blank-at-eol,blank-at-eof,space-before-tab,cr-at-eol',
        'diff','--cached','--check','--',*[p for p in paths if p.endswith(('.cpp','.csv','.md'))],env=env)
    print(git('commit','-m','fix: preserve Ackermann path direction after endpoint pruning',env=env).decode())
for p in paths:
    entry=git('ls-tree','HEAD','--',p).decode().split('\t',1)[0].split()
    git('update-index','--add','--cacheinfo',','.join((entry[0],entry[2],p)))
assert before==unrelated(),'Unrelated staged entries changed'
print('COMMIT',git('rev-parse','HEAD').decode().strip())
