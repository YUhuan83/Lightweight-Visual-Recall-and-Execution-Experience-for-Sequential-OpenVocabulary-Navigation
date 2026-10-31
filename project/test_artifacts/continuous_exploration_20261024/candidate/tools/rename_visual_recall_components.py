"""One-time source naming migration; keep legacy module/API aliases intact."""
from pathlib import Path
import ast
import io
import tokenize

ROOT=Path(__file__).resolve().parents[1]
VIS=ROOT/'wildos_port/repro_check/src/nebula2-wildos/visual_navigation'
PACKAGE=VIS/'visual_navigation/wildos'
RENAMES=(
    ('visual_memory','VisualMemory','execution_experience_store','ExecutionExperienceStore'),
    ('visual_memory_map','VisualMemoryMap','graph_observation_store','GraphObservationStore'),
    ('viewpoint_memory','ViewpointMemory','confirmed_viewpoint_cache','ConfirmedViewpointCache'),
    ('semantic_landmark_memory','SemanticLandmarkMemory','passive_landmark_index','PassiveLandmarkIndex'))

def main():
    originals={old:(PACKAGE/(old+'.py')).read_text(encoding='utf8') for old,_,_,_ in RENAMES}
    for old,old_class,new,new_class in RENAMES:
        destination=PACKAGE/(new+'.py')
        if destination.exists():raise RuntimeError('Migration already applied: '+str(destination))
        destination.write_bytes(originals[old].replace('class '+old_class+':','class '+new_class+':').encode())
    names={old_class:new_class for _,old_class,_,new_class in RENAMES}
    changed=[]
    files=list((VIS/'visual_navigation').rglob('*.py'))+list((VIS/'test').glob('*.py'))
    files+=list((ROOT/'nav2_adapter/test').glob('*.py'))+list((ROOT/'tools').glob('*.py'))
    legacy={PACKAGE/(old+'.py') for old,_,_,_ in RENAMES}
    for path in files:
        if path in legacy or path==Path(__file__):continue
        original=path.read_text(encoding='utf-8-sig')
        tokens=[]
        for token in tokenize.generate_tokens(io.StringIO(original).readline):
            if token.type==tokenize.NAME and token.string in names:
                token=token._replace(string=names[token.string])
            tokens.append(token)
        source=tokenize.untokenize(tokens)
        for old,_,new,_ in RENAMES:
            source=source.replace('from visual_navigation.wildos.'+old+' import ',
                                  'from visual_navigation.wildos.'+new+' import ')
        if source!=original:
            ast.parse(source);path.write_bytes(source.encode());changed.append(str(path.relative_to(ROOT)))
    for old,old_class,new,new_class in RENAMES:
        # Re-export public helpers too: old imports and module-level monkeypatches
        # retain the same Python objects, with no second cache/database instance.
        wrapper=('"""Compatibility import; canonical implementation: '+new+'."""\n'
                 'from .'+new+' import *  # noqa: F401,F403\n'+old_class+' = '+new_class+'\n')
        (PACKAGE/(old+'.py')).write_bytes(wrapper.encode())
    print('\n'.join(changed))

def rename_policy():
    directory=ROOT/'nav2_adapter/wildos_nav2_adapter'
    original=directory/'memory_selection.py';destination=directory/'recall_candidate_policy.py'
    if destination.exists():raise RuntimeError('Policy migration already applied')
    destination.write_bytes(original.read_text().replace('class MemorySelection:',
                                                        'class RecallCandidatePolicy:').encode())
    original.write_bytes(b'"""Compatibility import for the recall candidate policy."""\n'
        b'from .recall_candidate_policy import *\nMemorySelection = RecallCandidatePolicy\n')
    path=directory/'graph_path_bridge.py'
    source=path.read_text().replace('from .memory_selection import MemorySelection',
        'from .recall_candidate_policy import RecallCandidatePolicy').replace(
        'ConfirmedTargetApproach, MemorySelection, Node','ConfirmedTargetApproach, RecallCandidatePolicy, Node')
    ast.parse(source);path.write_bytes(source.encode())

if __name__=='__main__':
    import sys
    rename_policy() if sys.argv[1:]==['--policy'] else main()
