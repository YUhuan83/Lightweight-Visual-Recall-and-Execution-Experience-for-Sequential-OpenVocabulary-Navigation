"""Read-only project log inventory; follow no links and print no log contents."""
import collections,json,os,sys
from pathlib import Path
roots=[Path(p).resolve() for p in sys.argv[2:]]
rows=[];counts=collections.Counter();sizes=collections.Counter()
for root in roots:
 for parent,dirs,files in os.walk(root,followlinks=False):
  dirs[:]=[d for d in dirs if d not in ('.git','.venv','node_modules') and not (Path(parent)/d).is_symlink()]
  for name in files:
   p=Path(parent)/name
   if p.is_symlink():continue
   kind=('log' if name.endswith('.log') or '.log.' in name else 'jsonl' if name.endswith('.jsonl') else 'recording' if name.endswith(('.db3','.mcap','.bag')) else None)
   if kind is None:continue
   try:s=p.stat()
   except OSError:continue
   counts[kind]+=1;sizes[kind]+=s.st_size
   rows.append(dict(path=str(p),root=str(root),kind=kind,bytes=s.st_size,mtime=s.st_mtime))
out=dict(roots=list(map(str,roots)),counts=dict(counts),bytes=dict(sizes),files=rows)
Path(sys.argv[1]).write_text(json.dumps(out,indent=2),encoding='utf8')
print(json.dumps(dict(counts=out['counts'],bytes=out['bytes'],largest_logs=sorted((r for r in rows if r['kind']=='log'),key=lambda r:r['bytes'],reverse=True)[:18]),indent=2))
