"""Check frozen A/B sources, memory configuration and offline contracts."""
import argparse
import ast
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys

ROOT=Path(__file__).resolve().parent
CORE=Path('wildos_port/repro_check/src/nebula2-wildos')


def git(root,*args):
    return subprocess.run(['git','-C',str(root),*args],capture_output=True,text=True,
                          encoding='utf8',errors='replace',check=True).stdout.strip()


def tracked(root):
    return {n:hashlib.sha256((root/n).read_bytes()).hexdigest()
            for n in git(root,'ls-files').splitlines()}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--peer',type=Path)
    parser.add_argument('--callbacks',action='store_true')
    parser.add_argument('--syntax-only',action='store_true')
    args=parser.parse_args()
    arm=json.loads((ROOT/'experiment_arm.json').read_text(encoding='utf8'))
    if arm['arm']=='R':
        raise RuntimeError('R is reference review only; upstream identity and interface compatibility are not qualified')
    if git(ROOT,'status','--porcelain'):
        raise RuntimeError('Experimental worktree contains unfrozen changes')
    files=tracked(ROOT)
    for name in files:
        if name.endswith('.py'):
            ast.parse((ROOT/name).read_text(encoding='utf-8-sig'),filename=name)
    config=(ROOT/CORE/'visual_navigation/configs/beamng_ablation.yaml').read_text(encoding='utf8')
    expected='memory_experiment_enabled: '+('true' if arm['memory'] else 'false')
    if config.count('memory_experiment_enabled:')!=1 or expected not in config:
        raise AssertionError('Arm identity differs from installed-source configuration')
    if args.peer:
        peer=Path(os.path.normpath(str(ROOT/args.peer)))
        if git(peer,'status','--porcelain'):
            raise RuntimeError('Peer has unfrozen changes')
        peer_arm=json.loads((peer/'experiment_arm.json').read_text(encoding='utf8'))
        if {arm['arm'],peer_arm['arm']}!={'A','B'} or arm['memory']==peer_arm['memory']:
            raise AssertionError('Expected one A and one B')
        other=tracked(peer)
        changed=sorted(n for n in files.keys()|other.keys() if files.get(n)!=other.get(n))
        allowed=sorted(['experiment_arm.json',(CORE/'visual_navigation/configs/beamng_ablation.yaml').as_posix()])
        if changed!=allowed:
            raise AssertionError('Uncontrolled A/B differences: '+repr(changed))
        # Require the YAML difference to be exactly the master gate.
        peer_config=(peer/CORE/'visual_navigation/configs/beamng_ablation.yaml').read_text(encoding='utf8')
        normalize=lambda t:t.replace('memory_experiment_enabled: true','memory_experiment_enabled: false')
        if normalize(config)!=normalize(peer_config):
            raise AssertionError('A/B parameters differ beyond memory gate')
        print('A/B tracked code and shared configuration identical; only memory gate and arm metadata differ')
    if not args.syntax_only:
        for script in ('test_memory_gate.py','tools/test_beamng_kinematic_math.py'):
            subprocess.run([sys.executable,str(ROOT/script)],cwd=ROOT,check=True)
        if args.callbacks:
            subprocess.run([sys.executable,str(ROOT/'tools/offline_callback_check.py')],cwd=ROOT,check=True)
    report=dict(arm=arm['arm'],commit=git(ROOT,'rev-parse','HEAD'),tracked_files=len(files),
        clean=True,peer_checked=bool(args.peer),syntax_only=args.syntax_only,
        ros_build_run=False,simulator_run=False)
    print(json.dumps(report,ensure_ascii=False,indent=2))


if __name__=='__main__':
    main()
