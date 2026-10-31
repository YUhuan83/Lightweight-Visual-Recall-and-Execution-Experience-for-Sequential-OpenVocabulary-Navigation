"""Check the naming-only scope against the pre-rename frozen source."""
import ast,hashlib,json,zipfile
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
OLD='71f9ed53b5e0262c400017f7751efef5f5dcac929ade0809343d7d1f43047660'
PREFIX='wildos_port/repro_check/src/nebula2-wildos/visual_navigation/visual_navigation/wildos/'
entries=[('visual_memory','VisualMemory','execution_experience_store','ExecutionExperienceStore'),
         ('visual_memory_map','VisualMemoryMap','graph_observation_store','GraphObservationStore'),
         ('viewpoint_memory','ViewpointMemory','confirmed_viewpoint_cache','ConfirmedViewpointCache'),
         ('semantic_landmark_memory','SemanticLandmarkMemory','passive_landmark_index','PassiveLandmarkIndex')]
names={new_class:old_class for _,old_class,_,new_class in entries}
names['RecallCandidatePolicy']='MemorySelection'
modules={new:old for old,_,new,_ in entries};modules['recall_candidate_policy']='memory_selection'

class Normalize(ast.NodeTransformer):
    def visit_Name(self,node):node.id=names.get(node.id,node.id);return node
    def visit_ClassDef(self,node):
        node.name=names.get(node.name,node.name);return self.generic_visit(node)
    def visit_ImportFrom(self,node):
        if node.module:
            parts=node.module.split('.');parts[-1]=modules.get(parts[-1],parts[-1]);node.module='.'.join(parts)
        for alias in node.names:alias.name=names.get(alias.name,alias.name)
        return node
    def generic_visit(self,node):
        node=super().generic_visit(node)
        if isinstance(node,(ast.Module,ast.ClassDef,ast.FunctionDef,ast.AsyncFunctionDef)):
            if node.body and isinstance(node.body[0],ast.Expr) and isinstance(node.body[0].value,ast.Constant) and isinstance(node.body[0].value.value,str):
                node.body=node.body[1:]
        return node

def normalized(source):return ast.dump(Normalize().visit(ast.parse(source)),include_attributes=False)
archive=ROOT/('official_wildos_beamng_bundle.rollback.'+OLD+'.zip')
assert hashlib.sha256(archive.read_bytes()).hexdigest()==OLD
report={'baseline':OLD,'scope':'AST equivalence after canonical name/import normalization; not navigation acceptance','files':{}}
with zipfile.ZipFile(archive) as z:
    pairs=[(PREFIX+old+'.py',PREFIX+new+'.py') for old,_,new,_ in entries]
    pairs += [(PREFIX+'nav.py',PREFIX+'nav.py'),
              ('nav2_adapter/wildos_nav2_adapter/memory_selection.py','nav2_adapter/wildos_nav2_adapter/recall_candidate_policy.py'),
              ('nav2_adapter/wildos_nav2_adapter/graph_path_bridge.py','nav2_adapter/wildos_nav2_adapter/graph_path_bridge.py')]
    for old,new in pairs:
        before=z.read(old).decode('utf-8-sig');after=(ROOT/new).read_text(encoding='utf-8-sig')
        same=normalized(before)==normalized(after)
        report['files'][new]={'normalized_ast_equal':same,'old_sha256':hashlib.sha256(before.encode()).hexdigest(),
                             'new_sha256':hashlib.sha256(after.encode()).hexdigest()}
        assert same,new
    cpp=[n for n in z.namelist() if n.startswith(('ackermann_smac/','ackermann_rpp/')) and n.endswith(('.cpp','.hpp','CMakeLists.txt'))]
    assert all(z.read(n)==(ROOT/n).read_bytes() for n in cpp)
    report['unchanged_cpp_files']=len(cpp)
report['passed']=True
Path(__file__).with_name('rename_equivalence.json').write_text(json.dumps(report,indent=2))
print(json.dumps({'passed':True,'equivalent_files':len(pairs),'unchanged_cpp_files':len(cpp)}))
