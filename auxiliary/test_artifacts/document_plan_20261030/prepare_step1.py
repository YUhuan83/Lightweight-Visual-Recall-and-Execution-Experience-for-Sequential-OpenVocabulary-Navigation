"""User-approved plan, step 1: planar visual-target arrival; preserve measured XYZ."""
import hashlib,json,shutil
from pathlib import Path
base=Path('/home/ly/execution_continuity_review_20261022/v71_heading_freshness')
root=base.with_name('v73_document_plan')
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
for name in ('audit_fluency_120s.py','audit_target_flow.py','audit_target_path_interface.py'):
    shutil.copy2(base/name,root/name)
package=root/'candidate/nav2_adapter'
path=package/'wildos_nav2_adapter/graph_path_bridge.py';s=path.read_text()
old='        goal_distance = math.dist(position, object_position)'
assert old in s;s=s.replace(old,'''        if not all(map(math.isfinite, position + object_position)):
            return
        goal_distance = math.dist(position[:2], object_position[:2])''')
s=s.replace('# WildOS PlannerNode: (goal_vec - odom_vec).norm() < goal_radius.',
            '# Visual surfaces may be above the road. Keep XYZ, use the approved XY arrival metric.')
s=s.replace("arrival_basis='upstream_goal_distance_3d',","arrival_basis='visual_goal_distance_xy',")
path.write_text(s)
path=package/'wildos_nav2_adapter/arrival_criteria.py';s=path.read_text()
s=s.replace("if status.get('arrival_basis') == 'upstream_goal_distance_3d':", "if status.get('arrival_basis') in ('upstream_goal_distance_3d', 'visual_goal_distance_xy'):")
old='''            goal_distance = math.dist(position, goal)
            observed_distance = math.dist(position, observed)'''
new='''            if not all(math.isfinite(float(v)) for point in (position, goal, observed) for v in point):
                return False
            dimensions = 2 if status['arrival_basis'] == 'visual_goal_distance_xy' else 3
            goal_distance = math.dist(position[:dimensions], goal[:dimensions])
            observed_distance = math.dist(position[:dimensions], observed[:dimensions])'''
assert old in s;s=s.replace(old,new)
s=s.replace('and math.dist(goal, observed) < limit)', 'and math.dist(goal[:dimensions], observed[:dimensions]) < limit)')
path.write_text(s)
path=package/'test/test_upstream_target_approach_ros.py';s=path.read_text()
start=s.index('    def test_vertical_gap_is_part_of_arrival(self):')
end=s.index('    def test_real_arrival_event_',start)
s=s[:start]+'''    def test_visual_surface_height_is_preserved_but_does_not_block_planar_arrival(self):
        self.b.on_waypoint(self.goal(2.999, 0., 8.))
        self.assertTrue(self.b.object_found)
        self.assertEqual(self.b.last_status['arrival_basis'],'visual_goal_distance_xy')
        self.assertEqual(self.b.last_status['object_position'],[2.999,0.,8.])
        self.assertAlmostEqual(self.b.last_status['object_distance_m'],2.999)

    def test_planar_three_metre_boundary_is_strict_at_any_height(self):
        self.b.on_waypoint(self.goal(3., 0., 8.))
        self.assertFalse(self.b.object_found)
        self.b.on_waypoint(self.goal(3.001, 0., 8.))
        self.assertFalse(self.b.object_found)
        self.b.on_waypoint(self.goal(2., 2., 8.))
        self.assertTrue(self.b.object_found)
        self.assertAlmostEqual(self.b.last_status['object_distance_m'],math.sqrt(8.))

    def test_nonfinite_height_is_not_a_valid_visual_localization(self):
        self.b.on_waypoint(self.goal(1., 0., math.nan))
        self.assertFalse(self.b.object_found)

    def test_independent_visual_arrival_uses_the_same_planar_metric(self):
        from wildos_nav2_adapter.arrival_criteria import arrival_status_matches
        self.b.on_waypoint(self.goal(2.999, 0., 8.))
        confirmation=json.loads(self.fixture.confirmation(point=(2.999,0.,8.)).data)
        self.assertTrue(arrival_status_matches(self.b.last_status,'house',confirmation))
        confirmation['object_position']=[3.,0.,8.]
        self.assertFalse(arrival_status_matches(self.b.last_status,'house',confirmation))

''' +s[end:]
path.write_text(s)
(root/'candidate_scope.json').write_text(json.dumps(dict(baseline=str(base),stage='step1',
    arrival='visual XY distance strictly <3m; measured XYZ retained',new_runtime_components=False),indent=2))
print(root)
