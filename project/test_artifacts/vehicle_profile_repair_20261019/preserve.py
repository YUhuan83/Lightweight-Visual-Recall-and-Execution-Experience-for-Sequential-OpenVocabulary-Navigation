from pathlib import Path
import hashlib
import json
import shutil

ROOT = Path(__file__).resolve().parents[2]
OUT = Path(__file__).resolve().parent
PREFIX = 'wildos_port/repro_check/src/nebula2-wildos/graphnav_mapper/'
names = [PREFIX + p for p in ('include/graphnav_mapper/mapper.hpp', 'src/mapper.cpp',
          'src/mapper_node.cpp', 'src/navigation_graph.cpp', 'test/test_mapper.cpp')]
names += ['official_adapter/launch/official_graph_stack.launch.py',
          'nav2_adapter/launch/physical_vehicle_control.launch.py',
          'nav2_adapter/wildos_nav2_adapter/runtime_contract.py',
          'nav2_adapter/test/test_runtime_contract.py']
dest = OUT / 'before'
dest.mkdir(exist_ok=False)
manifest = {}
for name in names:
    target = dest / name
    target.parent.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(ROOT / name, target)
    manifest[name] = hashlib.sha256(target.read_bytes()).hexdigest()
(OUT / 'before_manifest.json').write_text(json.dumps(manifest, indent=2) + '\n', encoding='utf8')
print('Preserved', len(manifest), 'sources/tests')
