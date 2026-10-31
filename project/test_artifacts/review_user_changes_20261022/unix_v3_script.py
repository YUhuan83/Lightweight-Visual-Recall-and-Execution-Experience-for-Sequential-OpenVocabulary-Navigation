from pathlib import Path
p=Path('/home/ly/execution_continuity_review_20261022/v3/build.sh')
p.write_bytes(p.read_bytes().replace(b'\r\n',b'\n'))
