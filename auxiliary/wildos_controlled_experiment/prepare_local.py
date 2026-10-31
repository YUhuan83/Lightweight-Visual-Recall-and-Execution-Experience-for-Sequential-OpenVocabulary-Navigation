"""Create isolated experimental Git snapshots without changing the source project."""
from pathlib import Path
import ast
import hashlib
import json
import os
import shutil
import subprocess

HERE = Path(__file__).resolve().parent
SOURCE = HERE.parent.parent / '拟真'
REPO = HERE / 'repository'
CORE = Path('wildos_port/repro_check/src/nebula2-wildos')
REF = Path('wildos_port/workspace/src/nebula2-wildos')
NAV = CORE / 'visual_navigation/visual_navigation/wildos/nav.py'
SKIP_PARTS = {'.git', '__pycache__', '.pytest_cache', 'build', 'install', 'log', 'ckpts', 'assets'}
ALLOWED = {'.py', '.cpp', '.hpp', '.h', '.xml', '.yaml', '.yml', '.msg', '.srv',
           '.action', '.cfg', '.txt', '.cmake', '.sh', '.lua', '.rviz', '.md', '.json', '.npz'}


def run(*args, cwd=REPO):
    result = subprocess.run(args, cwd=cwd, capture_output=True, text=True,
                            encoding='utf8', errors='replace', check=True)
    return result.stdout.strip()


def collect(directory):
    result = {}
    for p in sorted(directory.rglob('*')):
        rel = p.relative_to(directory)
        if (not p.is_file() or p.is_symlink() or any(s in SKIP_PARTS for s in rel.parts)
                or p.name.endswith(('.bak', '.pre_beamng.bak'))
                or '.pre_beamng_' in p.name or p.stat().st_size > 2_000_000):
            continue
        if p.suffix in ALLOWED or p.name in ('LICENSE', 'COPYING', 'CMakeLists.txt') or 'resource' in rel.parts:
            result[rel.as_posix()] = p
    return result


def digest(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()


def copy_tree(source, destination):
    files = collect(source)
    for name,p in files.items():
        target = destination / name
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(p, target)
    return {name:digest(p) for name,p in files.items()}


def replace_once(text, old, new):
    if text.count(old) != 1:
        raise ValueError('Expected one source anchor: '+old[:100])
    return text.replace(old, new, 1)


def experiment_gate(text):
    text = replace_once(text, '        "visual_memory_enabled": True,',
        '        "memory_experiment_enabled": True,\n        "visual_memory_enabled": True,')
    text = replace_once(text, '        self.visual_memory = None',
        '        self.memory_experiment_enabled = bool(config.memory_experiment_enabled)\n'
        '        self.get_logger().info(f"EXPERIMENT_MEMORY_ENABLED={self.memory_experiment_enabled}")\n'
        '        self.visual_memory = None')
    text = replace_once(text, '        if config.visual_memory_enabled:',
        '        if self.memory_experiment_enabled and config.visual_memory_enabled:')
    text = replace_once(text, '        if config.visual_memory_map_enabled:',
        '        if self.memory_experiment_enabled and config.visual_memory_map_enabled:')
    text = replace_once(text,
        '            if self.object_search_mode and config.semantic_landmark_memory_enabled else None)',
        '            if self.memory_experiment_enabled and self.object_search_mode\n'
        '            and config.semantic_landmark_memory_enabled else None)')
    text = replace_once(text, '    def object_confirmation_callback(self, message):\n',
        '    def object_confirmation_callback(self, message):\n'
        '        if not getattr(self, "memory_experiment_enabled", True):\n            return\n')
    text = replace_once(text, '    def _reuse_remembered_object_goal(self, query):\n',
        '    def _reuse_remembered_object_goal(self, query):\n'
        '        if not getattr(self, "memory_experiment_enabled", True):\n'
        '            self.remembered_candidates_publisher.publish(String(data=json.dumps({\n'
        '                "query": query, "views": [],\n'
        '                "issued_at_ns": self.get_clock().now().nanoseconds,\n'
        '                "query_started_ns": self.object_query_started_ns})))\n'
        '            return\n')
    text = replace_once(text, '        memory_embedding = self.pool_memory_embedding(spatial_feats)',
        '        memory_embedding = (self.pool_memory_embedding(spatial_feats)\n'
        '                            if self.memory_experiment_enabled else None)')
    begin = text.index('            self.last_scene_features = {')
    end = text.index('\n\n            text_sim_spatial, binary_mask', begin)
    block = text[begin:end]
    text = text[:begin] + '            if self.memory_experiment_enabled:\n' + '\n'.join(
        '    '+line if line else line for line in block.splitlines()) + text[end:]
    ast.parse(text)
    return text


def commit(message):
    run('git', 'add', '--all')
    run('git', '-c', 'user.name=Codex Experiment Preparation', '-c',
        'user.email=codex-experiment@local.invalid', 'commit', '-m', message)
    return run('git', 'rev-parse', 'HEAD')


def write(path, text):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding='utf8', newline='\n')


def main():
    if REPO.exists():
        raise RuntimeError('Destination already exists; refusing to overwrite experimental history')
    # Record only selected code packages; exclude videos, credentials, weights,
    # build output and unrelated project files. Original index remains untouched.
    before = run('git', 'status', '--porcelain=v1', '--untracked-files=normal', cwd=SOURCE)
    REPO.mkdir()
    run('git', 'init', '-b', 'experiment/common')
    run('git', 'config', 'core.autocrlf', 'false')
    inventory = {}
    for relative, destination in ((CORE, CORE), (REF, Path('reference/nebula2-wildos')),
                                   (Path('official_adapter'), Path('official_adapter')),
                                   (Path('nav2_adapter'), Path('nav2_adapter'))):
        inventory[str(relative)] = copy_tree(SOURCE/relative, REPO/destination)
    for name in ('wildosKinematic.lua',):
        src = SOURCE/'tools/beamng_extensions'/name
        write(REPO/'tools/beamng_extensions'/name, src.read_text(encoding='utf8'))
        inventory['tools/beamng_extensions/'+name] = digest(src)
    for name in ('offline_callback_check.py', 'test_beamng_kinematic_math.py', 'verify_beamng_math_source.py'):
        shutil.copyfile(SOURCE/'tools'/name, REPO/name)
    # These tools retain their original tools/ location for ROOT resolution.
    for name in ('offline_callback_check.py', 'test_beamng_kinematic_math.py', 'verify_beamng_math_source.py'):
        target = REPO/'tools'/name
        target.parent.mkdir(exist_ok=True)
        (REPO/name).rename(target)
    fixtures = ('accepted_memory_prefix_snapshot.npz', 'prefix_replay.json', 'house_localization_replay.json')
    for name in fixtures:
        target = REPO/'test_artifacts/server_validation_20261002'/name
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(SOURCE/'test_artifacts/server_validation_20261002'/name, target)
    shutil.copyfile(SOURCE/'evaluation/controlled_ablation_20261007/EXPERIMENT_PLAN.md', REPO/'EXPERIMENT_PLAN.md')
    write(REPO/'.gitignore', '__pycache__/\n*.pyc\n.pytest_cache/\nbuild/\ninstall/\nlog/\nresults/\n.runtime/\n'
        'test_artifacts/*\n!test_artifacts/server_validation_20261002/\n'
        'test_artifacts/server_validation_20261002/*\n'+''.join(
            '!test_artifacts/server_validation_20261002/'+name+'\n' for name in fixtures))
    write(REPO/'source_origin.json', json.dumps(dict(
        source_root=str(SOURCE), original_project_has_commit=False,
        upstream_reference_verified=False, reference='Local retained source, not authenticated upstream',
        files=inventory), ensure_ascii=False, indent=2))
    snapshot = commit('Snapshot local reference and shared BeamNG execution source for experiments')
    run('git', 'tag', 'snapshot/local-20261007', snapshot)
    nav = REPO/NAV
    write(nav, experiment_gate(nav.read_text(encoding='utf8')))
    # All memory mechanisms are active in B; A disables them through one gate.
    # Heading policy, frontier history, uncertainty and detection logic remain
    # identical in both arms and are explicitly outside this first ablation.
    config = (SOURCE/'official_adapter/config/beamng_wildos.yaml').read_text(encoding='utf8')
    config += '\n# Controlled ablation: only this gate differs between A/B.\n'
    config += 'memory_experiment_enabled: false\nvisual_memory_enabled: true\n'
    config += 'visual_memory_map_enabled: true\nsemantic_landmark_memory_enabled: true\n'
    config += 'visual_memory_path: ":memory:"\nvisual_memory_map_path: ":memory:"\n'
    config += 'visual_memory_map_save_images: false\npublish_legacy_remembered_goal: false\n'
    write(REPO/CORE/'visual_navigation/configs/beamng_ablation.yaml', config)
    write(REPO/'experiment_arm.json', json.dumps(dict(
        arm='A', memory=False, heading_policy='Current shared policy, held constant',
        paper_reference_verified=False, trial_state_reset_required=True), indent=2))
    # Auxiliary scripts are supplied beside this preparation script.
    for name in ('verify_experiment.py', 'test_memory_gate.py', 'build_ros.sh', 'run_vision.sh', 'README.md'):
        shutil.copyfile(HERE/'templates'/name, REPO/name)
    common = commit('Add shared complete memory gate and isolated experiment tooling; default arm A')
    run('git', 'branch', 'experiment/memory-off', common)
    run('git', 'tag', 'experiment/memory-off', common)
    run('git', 'switch', '-c', 'experiment/memory-on')
    config_path = REPO/CORE/'visual_navigation/configs/beamng_ablation.yaml'
    write(config_path, config_path.read_text(encoding='utf8').replace(
        'memory_experiment_enabled: false', 'memory_experiment_enabled: true'))
    arm = json.loads((REPO/'experiment_arm.json').read_text(encoding='utf8'))
    arm.update(arm='B', memory=True)
    write(REPO/'experiment_arm.json', json.dumps(arm, indent=2))
    memory = commit('Arm B: enable visual memory only; retain identical heading and execution code')
    run('git', 'tag', 'experiment/memory-on', memory)
    run('git', 'switch', 'experiment/common')
    # R preserves an inspectable baseline, while explicitly preventing launch
    # before original runtime/message compatibility has been qualified.
    run('git', 'switch', '-c', 'experiment/reference-review')
    arm = dict(arm='R', memory=False, paper_reference_verified=False,
               runnable=False, reason='Reference provenance and common-interface adaptation need review')
    write(REPO/'experiment_arm.json', json.dumps(arm, indent=2))
    reference = commit('Reference review arm: preserve raw local source; block unsupported runtime claims')
    run('git', 'tag', 'experiment/reference-review', reference)
    run('git', 'switch', 'experiment/common')
    checkouts = {}
    for key,branch in (('A-memory-off','experiment/memory-off'),
                       ('B-memory-on','experiment/memory-on'),
                       ('R-reference-review','experiment/reference-review')):
        path = HERE/'checkouts'/key
        run('git', 'worktree', 'add', str(path), branch)
        checkouts[key] = str(path)
    after = run('git', 'status', '--porcelain=v1', '--untracked-files=normal', cwd=SOURCE)
    if before != after:
        raise RuntimeError('Source project status changed during preparation; inspect before using experiments')
    report = dict(snapshot=snapshot, common=common, A=common, B=memory, R=reference,
                  worktrees=checkouts, original_project_status_unchanged=True,
                  simulator_run=False, ros_build_run=False, upstream_reference_verified=False)
    write(HERE/'prepared_versions.json', json.dumps(report, ensure_ascii=False, indent=2))
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
