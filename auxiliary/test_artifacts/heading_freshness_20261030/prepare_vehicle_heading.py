"""Prioritize the already existing vehicle arrival heading for normal frontiers."""
import hashlib,json,shutil,sys
from pathlib import Path
base=Path('/home/ly/execution_continuity_review_20261022/v71_heading_freshness')
root=base.with_name('v72_vehicle_heading')
relative=Path('nav2_adapter/wildos_nav2_adapter/continuous_graph_path_bridge.py')
if '--apply' not in sys.argv:
    assert not root.exists();root.mkdir()
    shutil.copytree(base/'candidate',root/'candidate',symlinks=True,ignore=shutil.ignore_patterns('__pycache__'))
    (root/'full').mkdir()
    for p in (base/'full').iterdir():
        if p.is_file() and p.suffix in ('.py','.json'):
            (root/'full'/p.name).write_text(p.read_text().replace(str(base),str(root)))
    (root/'full/candidate').symlink_to('../candidate',target_is_directory=True)
    config=json.loads((root/'full/config.json').read_text())
    config['setups']=[str(base/'install/local_setup.bash') if x==str(root/'install/local_setup.bash') else x for x in config['setups']]
    config['setups'].append(str(root/'install/local_setup.bash'))
    (root/'full/config.json').write_text(json.dumps(config,indent=2)+'\n')
    lines=(base/'run_full.sh').read_text().splitlines();last=max(i for i,x in enumerate(lines) if x.startswith('source '))
    lines.insert(last+1,f'source {root}/install/local_setup.bash')
    (root/'run_full.sh').write_text('\n'.join(x if x.startswith('source ') else x.replace(str(base),str(root)) for x in lines)+'\n')
    for name in ('audit_fluency_120s.py','audit_target_3d.py','audit_target_flow.py'):
        shutil.copy2(base/name,root/name)
    test=root/'candidate/nav2_adapter/test/test_continuous_exploration_ros.py'
    text=test.read_text().replace('def test_explicit_observation_heading_is_not_replaced(self):',
                                'def test_frontier_view_heading_is_a_fallback_not_a_vehicle_constraint(self):')
    old='''        n.begin_exploration_comparison()
        self.assertEqual(n.candidates[0][2],0.)

    def test_arrival_proposal'''
    new='''        n.begin_exploration_comparison()
        self.assertAlmostEqual(n.candidates[0][2],2*math.atan2(1.,6.))
        self.assertIn((6.,1.,0.,math.hypot(6.,1.)),n.candidates)

    def test_remembered_and_confirmed_observation_orientations_stay_required(self):
        n=self.node;n.follow_enabled=True;n.odom.pose.pose.orientation.w=1.
        n.route=self.route((6.,1.),'frontier-A');n.route_endpoint=(6.,1.)
        n.route_metadata[stamp_ns(n.route)]['observation_heading']=0.
        for attribute in ('memory_goal','confirmed_route_goal'):
            setattr(n,attribute,(6.,1.))
            n.candidates=[(6.,1.,0.,math.hypot(6.,1.))]
            n.begin_exploration_comparison()
            self.assertEqual(n.candidates[0][2],0.)
            setattr(n,attribute,None)

    def test_blocked_vehicle_arrival_keeps_the_existing_view_fallback(self):
        n=self.node;n.follow_enabled=True;n.odom.pose.pose.orientation.w=1.
        n.route=self.route((6.,1.),'frontier-A');n.route_endpoint=(6.,1.)
        n.route_metadata[stamp_ns(n.route)]['observation_heading']=0.
        n.candidates=[(6.,1.,0.,math.hypot(6.,1.))]
        n.vehicle_pose_feasible=Mock(return_value=False)
        n.begin_exploration_comparison()
        self.assertEqual(n.candidates[0][2],0.)

    def test_arrival_proposal'''
    assert old in text;test.write_text(text.replace(old,new))
else:
    path=root/'candidate'/relative;text=path.read_text()
    old='''            required = self.exploration_observation_heading()
            destination = self.exploration_destination() or self.route_endpoint
            explicit = required is not None and destination is not None and distance(target,destination)<1e-6
'''
    assert old in text;text=text.replace(old,'')
    old='            if not explicit and heading is not None and self.vehicle_pose_feasible(target,heading):'
    assert old in text;text=text.replace(old,'            if heading is not None and self.vehicle_pose_feasible(target,heading):')
    text=text.replace('# yaw; explicit observation and remembered orientations stay intact.',
        '# yaw. Frontier viewing yaw remains a fallback; remembered and\n        # confirmed observation orientations keep their existing contracts.')
    path.write_text(text)
    manifest=json.loads((base/'full/frozen_source_manifest.json').read_text());changed=[]
    for name,old in manifest['members'].items():
        digest=hashlib.sha256((root/'candidate'/name).read_bytes()).hexdigest()
        if digest!=old:changed.append(name)
        manifest['members'][name]=digest
    (root/'full/frozen_source_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    (root/'candidate_scope.json').write_text(json.dumps(dict(changed=changed,baseline=str(base),
        no_new_runtime_components=True,arrival='XYZ distance <3m',search_s=120,
        reason='Use the existing vehicle-feasible arrival yaw before a semantic frontier viewing yaw; retain fallback and memory contracts.'),indent=2))
    print((root/'candidate_scope.json').read_text())
