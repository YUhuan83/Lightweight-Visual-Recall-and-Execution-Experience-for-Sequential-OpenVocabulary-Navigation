"""Preserve the current dirty source/index state before an isolated architecture experiment."""
import hashlib,json,os,subprocess,time,zipfile
from pathlib import Path
repo=Path(r'C:\Users\Yu\Desktop\拟真')
root=repo/'test_artifacts/continuous_exploration_20261024'
assert not root.exists(),'Do not overwrite an existing baseline'
root.mkdir()
def git(*args):return subprocess.run(['git',*args],cwd=repo,check=True,capture_output=True).stdout
head=git('rev-parse','HEAD').decode().strip()
tracked=set(git('ls-files','-z').decode().split('\0'))-{''}
untracked=set(git('ls-files','--others','--exclude-standard','-z').decode().split('\0'))-{''}
source_suffixes={'.py','.cpp','.hpp','.h','.c','.cmake','.yaml','.yml','.xml','.json','.toml','.ini','.cfg','.sh','.ps1','.bat','.rviz','.md','.txt','.msg','.srv','.action','.lua','.jbeam','.urdf','.pc','.csv'}
skip={'test_artifacts','captures','assets','_server_repro','.git','build','install','log','__pycache__','.venv','node_modules','.deps','ckpts','datasets','.pytest_cache'}
def source(name):
    p=Path(name)
    return not any(part in skip for part in p.parts) and (p.suffix.lower() in source_suffixes or p.name in {'CMakeLists.txt','package.xml','resource','LICENSE'})
names=sorted(tracked | {name for name in untracked if source(name)})
manifest={};missing=[];candidate=root/'candidate';candidate.mkdir()
with zipfile.ZipFile(root/'baseline_source.zip','w',zipfile.ZIP_DEFLATED) as archive:
    for name in names:
        path=repo/name
        if not path.is_file():missing.append(name);continue
        data=path.read_bytes();manifest[name]=dict(sha256=hashlib.sha256(data).hexdigest(),bytes=len(data),tracked=name in tracked)
        archive.writestr(name,data)
        if source(name):
            target=candidate/name;target.parent.mkdir(parents=True,exist_ok=True);target.write_bytes(data)
    for name,args in [('head.diff',('diff','--binary','HEAD')),('index.diff',('diff','--cached','--binary')),('worktree.diff',('diff','--binary'))]:
        data=git(*args);archive.writestr('_git_state/'+name,data)
    archive.writestr('_git_state/status.txt',git('status','--porcelain=v1'))
    archive.writestr('_git_state/HEAD.txt',head+'\n')
with zipfile.ZipFile(root/'baseline_source.zip') as archive:
    assert archive.testzip() is None
    for name,info in manifest.items():
        assert hashlib.sha256(archive.read(name)).hexdigest()==info['sha256'],name
        assert hashlib.sha256((repo/name).read_bytes()).hexdigest()==info['sha256'],name
receipt=dict(created_wall=time.time(),head=head,archive=str(root/'baseline_source.zip'),
    archive_sha256=hashlib.sha256((root/'baseline_source.zip').read_bytes()).hexdigest(),members=manifest,
    missing_tracked_paths=missing,candidate=str(candidate),
    scope='All tracked working files plus non-runtime untracked source/config/docs; HEAD/index/worktree binary diffs retained. Models, captures, build outputs and untracked historical test artifacts remain in place and are not copied.',
    validation='ZIP integrity, every member SHA256 and source still matches; no Git commit, index or branch mutation',
    known_pending='Four physical shift-profile tests were added before an unexecuted 0.3s parameter change; two expected failures remain. Field v23 and complete flow unaccepted.')
(root/'baseline_manifest.json').write_text(json.dumps(receipt,ensure_ascii=False,indent=2),encoding='utf8')
print(json.dumps({k:v for k,v in receipt.items() if k!='members'},ensure_ascii=False,indent=2))
print('files',len(manifest),'bytes',sum(m['bytes'] for m in manifest.values()))
