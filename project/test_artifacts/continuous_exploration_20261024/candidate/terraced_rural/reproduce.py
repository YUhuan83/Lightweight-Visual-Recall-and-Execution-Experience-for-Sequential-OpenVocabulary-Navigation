"""Execute whole ZIP reproducibility and blocked verification, then restore default build."""
import argparse
import json
import subprocess
import sys
from pathlib import Path
import build


def main(home):
    hashes=[]
    cases=[(20260911,'passable'),(20260911,'passable'),(20260912,'passable'),(20260911,'blocked'),(20260911,'passable')]
    for seed,mode in cases:
        subprocess.run([sys.executable,str(build.ROOT/'build.py'),'--seed',str(seed),'--mode',mode],check=True)
        m=json.loads((build.ROOT/'generated/build_manifest.json').read_text())
        hashes.append(dict(seed=seed,mode=mode,sha256=m['zip_sha256'],bytes=m['zip_bytes']))
        if mode=='blocked' or len(hashes)==len(cases):
            subprocess.run([sys.executable,str(build.ROOT/'verify.py'),'--home',str(home)],check=True)
    assert hashes[0]['sha256']==hashes[1]['sha256']==hashes[4]['sha256']
    assert hashes[0]['sha256']!=hashes[2]['sha256']!=hashes[3]['sha256']
    build.write_json(build.ROOT/'generated/reproducibility.json',dict(successful=True,cases=hashes,
        blocked_and_final_verification_passed=True,game_started=False))


if __name__=='__main__':
    p=argparse.ArgumentParser(); p.add_argument('--home',type=Path,required=True)
    main(p.parse_args().home)
