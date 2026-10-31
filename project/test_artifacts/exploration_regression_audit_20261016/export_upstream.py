from pathlib import Path
import difflib
import hashlib
import json
import subprocess

ROOT = Path(__file__).resolve().parents[2]
OUT = Path(__file__).resolve().parent
SOURCE = Path('C:/Users/Yu/nebula2-wildos')
HEAD = 'ffab44cb5f36e5508fbe29d3dc5bcd5fe69cb572'
FILES = [
    'graphnav_planner/src/planner_node.cpp', 'graphnav_planner/src/planner.cpp',
    'graphnav_mapper/src/mapper.cpp', 'graphnav_mapper/src/navigation_graph.cpp',
    'visual_navigation/visual_navigation/wildos/nav.py',
]
records = []
for relative in FILES:
    target = OUT/'upstream'/relative
    old = target.read_bytes() if target.exists() else subprocess.check_output(['git','-C',str(SOURCE),'show',HEAD+':'+relative])
    target.parent.mkdir(parents=True,exist_ok=True)
    target.write_bytes(old)
    current = ROOT/'wildos_port/repro_check/src/nebula2-wildos'/relative
    new = current.read_bytes()
    diff = ''.join(difflib.unified_diff(old.decode().replace('\r\n','\n').splitlines(True),new.decode().replace('\r\n','\n').splitlines(True),
        fromfile=HEAD+'/'+relative,tofile='current/'+relative))
    diff_path=OUT/'diffs'/(relative.replace('/','__')+'.diff')
    diff_path.parent.mkdir(exist_ok=True)
    diff_path.write_text(diff,encoding='utf8')
    records.append(dict(file=relative,upstream_sha256=hashlib.sha256(old).hexdigest(),
        current_sha256=hashlib.sha256(new).hexdigest(),upstream_lines=len(old.splitlines()),
        current_lines=len(new.splitlines())))
(OUT/'upstream_git_identity.json').write_text(json.dumps(dict(head=HEAD,files=records),indent=2),encoding='utf8')
print(json.dumps(records,indent=2))
