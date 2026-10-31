"""Complete the requested single-empty-path forward-probe entry; preserve v73 field evidence."""
import json,shutil
from pathlib import Path
base=Path('/home/ly/execution_continuity_review_20261022/v73_document_plan')
root=base.with_name('v74_document_handoff');assert not root.exists();root.mkdir()
shutil.copytree(base/'candidate',root/'candidate',symlinks=True,ignore=shutil.ignore_patterns('__pycache__'))
(root/'full').mkdir()
for p in (base/'full').iterdir():
    if p.is_file() and p.suffix in ('.py','.json'):(root/'full'/p.name).write_text(p.read_text().replace(str(base),str(root)))
(root/'full/candidate').symlink_to('../candidate',target_is_directory=True)
config=json.loads((root/'full/config.json').read_text())
config['setups']=[str(base/'install/local_setup.bash') if x==str(root/'install/local_setup.bash') else x for x in config['setups']]
config['setups'].append(str(root/'install/local_setup.bash'))
(root/'full/config.json').write_text(json.dumps(config,indent=2)+'\n')
lines=(base/'run_full.sh').read_text().splitlines();last=max(i for i,x in enumerate(lines) if x.startswith('source '))
lines.insert(last+1,f'source {root}/install/local_setup.bash')
(root/'run_full.sh').write_text('\n'.join(x if x.startswith('source ') else x.replace(str(base),str(root)) for x in lines)+'\n')
for name in ('audit_fluency_120s.py','audit_target_flow.py','audit_target_path_interface.py'):
    shutil.copy2(base/name,root/name)
p=root/'candidate/nav2_adapter/wildos_nav2_adapter/continuous_graph_path_bridge.py';s=p.read_text();where=s.index('    def target_forward_candidates(self, start):')
s=s[:where]+'''    def candidate_failed(self, reason):
        if (self.odom is not None and self.candidate_index < len(self.candidates)
                and (reason=='empty_path' or reason.startswith('local_ahead_'))
                and not any(r=='empty_path' or r.startswith('local_ahead_') for r in self.failure_reasons)):
            # One bounded local attempt per existing candidate batch, including
            # a forward graph path that failed. Reuse the current failure cache.
            probes=self.target_forward_candidates(xy(self.odom.pose.pose))
            probes=[p for p in probes if not any(distance(p[:2],c[:2])<.25
                and abs(math.remainder(p[2]-c[2],2*math.pi))<.1 for c in self.candidates)]
            self.candidates[self.candidate_index+1:self.candidate_index+1]=probes
        return super().candidate_failed(reason)

''' +s[where:];p.write_text(s)
p=root/'candidate/nav2_adapter/test/test_continuous_exploration_ros.py';s=p.read_text();where=s.index('    def test_missing_execution_progress_does_not_start_obstacle_replan')
s=s[:where]+'''    def test_single_empty_forward_plan_probes_sides_before_the_existing_graph_fallback(self):
        n=self.node;n.odom.pose.pose.orientation.w=1.
        n.upstream_target_approach=True;n.object_destination=fixtures.route((0.,0.),(20.,0.)).poses[-1]
        n.expected=(20.,0.);n.route=self.route((20.,0.),'frontier-A');n.route_endpoint=(20.,0.)
        n.candidates=[(20.,0.,0.,20.),(-12.,0.,math.pi,12.)];n.candidate_index=0
        n.active_plan_start=(0.,0.);n.vehicle_pose_feasible=Mock(return_value=True);n.try_candidate=Mock()
        n.candidate_failed('empty_path')
        self.assertEqual(n.candidates[1][:2],(8.,0.))
        self.assertIn((8.,-2.),[c[:2] for c in n.candidates])
        self.assertIn((8.,2.),[c[:2] for c in n.candidates])
        self.assertEqual(n.candidates[-1][:2],(-12.,0.))
        before=list(n.candidates);n.candidate_failed('empty_path')
        self.assertEqual(n.candidates,before)

''' +s[where:];p.write_text(s)
print(root)
