"""User-approved step 2: covered local lookahead and bounded same-target forward probes."""
from pathlib import Path
root=Path('/home/ly/execution_continuity_review_20261022/v73_document_plan')
package=root/'candidate/nav2_adapter'
p=package/'wildos_nav2_adapter/graph_route.py';s=p.read_text()
s=s.replace('def cell_cost(grid, x, y):','def cell_cost(grid, x, y, *, outside_cost=100):')
old='''    if not (0 <= mx < info.width and 0 <= my < info.height):
        return 100'''
assert old in s;s=s.replace(old,old.replace('return 100','return outside_cost'),1)
s=s.replace('padding=0.01, limit=5):','padding=0.01, limit=5, covered_only=False):',1)
old='''        if not (0 <= cell[0] < info.width and 0 <= cell[1] < info.height):
            return [(round(forward, 2), round(left, 2))]'''
new='''        if not covered_only and not (0 <= cell[0] < info.width and 0 <= cell[1] < info.height):
            return [(round(forward, 2), round(left, 2))]'''
assert old in s;s=s.replace(old,new,1)
old='''    for mx, my in sorted(cells):
        if grid.data[my*info.width+mx] >= 100:'''
new='''    for mx, my in sorted(cells):
        if covered_only and not (0 <= mx < info.width and 0 <= my < info.height):
            continue
        if grid.data[my*info.width+mx] >= 100:'''
assert old in s;s=s.replace(old,new,1)
s=s.replace('def validate_local_prefix(points, headings, grid, start, start_heading, maximum=2.0):',
    'def validate_local_prefix(points, headings, grid, start, start_heading, maximum=2.0, *, covered_only=False):')
old='''    if footprint_blockers(grid, start, start_heading, limit=1):
        return 'local_start_footprint_blocked' '''.rstrip()
new='''    if footprint_blockers(grid, start, start_heading, limit=1, covered_only=covered_only):
        return 'local_start_footprint_blocked' '''.rstrip()
assert old in s;s=s.replace(old,new,1)
start=s.index('def validate_local_prefix(');end=s.index('\ndef validate_ideal_local_prefix',start)
part=s[start:end].replace('cost = cell_cost(grid, *p)','cost = cell_cost(grid, *p, outside_cost=-1 if covered_only else 100)')
part=part.replace('if cost < 0:', 'if cost < 0 and not covered_only:')
part=part.replace('footprint_blockers(grid, p, h, limit=1)', 'footprint_blockers(grid, p, h, limit=1, covered_only=covered_only)')
s=s[:start]+part+s[end:];p.write_text(s)
p=package/'wildos_nav2_adapter/continuous_graph_path_bridge.py';s=p.read_text()
where=s.index('    def begin_exploration_comparison(self):')
s=s[:where]+'''    def target_forward_candidates(self, start):
        if (not self.upstream_target_approach or self.object_destination is None
                or self.expected is None or self.memory_goal is not None or self.memory_selecting):
            return []
        gap=distance(start,self.expected)
        if gap<=3.:return []
        ux,uy=(self.expected[0]-start[0])/gap,(self.expected[1]-start[1])/gap
        advance=min(8.,gap)
        result=[]
        for offset in (0.,-2.,2.):
            point=(start[0]+advance*ux-offset*uy,start[1]+advance*uy+offset*ux)
            heading=forward_arc_arrival_heading(start,yaw(self.odom.pose.pose),point,
                max(self.minimum_turning_radius,float(self.get_parameter('planner_minimum_turning_radius').value)))
            if heading is None:heading=math.atan2(point[1]-start[1],point[0]-start[0])
            if (not self.goal_recently_failed(start,point,heading)
                    and self.vehicle_pose_feasible(point,heading)):
                result.append((*point,heading,distance(start,point)))
        return result

    def validate_path(self, points, headings, route, grid, target, *, start=None, target_heading=None):
        if (self.upstream_target_approach and self.object_destination is not None
                and self.expected is not None and start is not None
                and distance(start,target)<=math.hypot(8.,2.)+.01
                and (target[0]-start[0])*(self.expected[0]-start[0])
                    +(target[1]-start[1])*(self.expected[1]-start[1])>0):
            # These bounded same-target probes deliberately precede a graph
            # detour. Smac and the full body/map checks still validate them.
            route=[start,target]
        reason=super().validate_path(points,headings,route,grid,target,
                                    start=start,target_heading=target_heading)
        if (reason is None and self.object_destination is not None
                and self.carrier_model=='ackermann' and self.local_grid is not None
                and self.local_grid.header.frame_id==grid.header.frame_id
                and self.local_freshness_issue() is None and start is not None):
            reason=validate_local_prefix(points,headings,self.local_grid,start,
                yaw(self.odom.pose.pose),maximum=CONTINUOUS_HANDOFF_DISTANCE_M,covered_only=True)
            if reason:reason='local_ahead_'+reason
        return reason

''' +s[where:]
old='''    def begin_exploration_comparison(self):
        # Keep WildOS'''
new='''    def begin_exploration_comparison(self):
        if self.candidates and self.odom is not None and self.expected is not None:
            start=xy(self.odom.pose.pose);first=self.candidates[0]
            away=((first[0]-start[0])*(self.expected[0]-start[0])
                  +(first[1]-start[1])*(self.expected[1]-start[1])<=0)
            if away:
                probes=self.target_forward_candidates(start)
                self.candidates=probes+[c for c in self.candidates if not any(
                    distance(c[:2],p[:2])<1e-6 and abs(math.remainder(c[2]-p[2],2*math.pi))<1e-6 for p in probes)]
        # Keep WildOS'''
assert old in s;s=s.replace(old,new,1)
old='''            or self.validate_local_path(points,headings,self.local_grid,
                xy(self.odom.pose.pose),yaw(self.odom.pose.pose)))'''
new='''            or validate_local_prefix(points,headings,self.local_grid,
                xy(self.odom.pose.pose),yaw(self.odom.pose.pose),
                maximum=CONTINUOUS_HANDOFF_DISTANCE_M,covered_only=True))'''
assert old in s;s=s.replace(old,new,1);p.write_text(s)
p=package/'wildos_nav2_adapter/graph_path_bridge.py';s=p.read_text()
old="        if (reason == 'empty_path' and self.active_plan_start is not None"
new="        if ((reason == 'empty_path' or reason.startswith('local_ahead_')) and self.active_plan_start is not None"
assert old in s;s=s.replace(old,new,1);p.write_text(s)
p=package/'test/test_graph_route.py';s=p.read_text();where=s.index('    def test_forward_arc_arrival_heading_matches_circle_and_rotation')
s=s[:where]+'''    def test_local_lookahead_detects_inscribed_obstacle_beyond_imminent_prefix(self):
        from wildos_nav2_adapter.graph_route import validate_local_prefix
        m=grid();set_cell(m,7.,0.,99)
        points=[(0.,0.),(10.,0.)];headings=[0.,0.]
        self.assertIsNone(validate_local_prefix(points,headings,m,(0.,0.),0.))
        self.assertEqual(validate_local_prefix(points,headings,m,(0.,0.),0.,8.,covered_only=True),'local_collision_cell')

    def test_covered_lookahead_does_not_treat_map_border_as_an_obstacle(self):
        from wildos_nav2_adapter.graph_route import validate_local_prefix
        m=grid(width=44,height=44)
        points=[(0.,0.),(10.,0.)];headings=[0.,0.]
        self.assertIsNone(validate_local_prefix(points,headings,m,(0.,0.),0.,8.,covered_only=True))
        set_cell(m,4.,0.,100)
        self.assertIsNotNone(validate_local_prefix(points,headings,m,(0.,0.),0.,8.,covered_only=True))

''' +s[where:];p.write_text(s)
p=package/'test/test_continuous_exploration_ros.py';s=p.read_text();where=s.index('    def test_missing_execution_progress_does_not_start_obstacle_replan')
s=s[:where]+'''    def test_local_obstacle_at_eight_metres_replans_without_canceling_motion(self):
        n=self.moving();handle=n.follow_handle
        n.local_grid.data[10*n.local_grid.info.width+18]=99
        self.assertIsNotNone(n.active_window())
        n.start_segment=Mock();n.maybe_extend_path()
        n.start_segment.assert_called_once_with(continuous=True)
        handle.cancel_goal_async.assert_not_called()

    def test_explicit_target_tries_bounded_forward_candidates_before_away_graph_exit(self):
        n=self.node;n.odom.pose.pose.orientation.w=1.
        n.upstream_target_approach=True;n.object_destination=fixtures.route((0.,0.),(20.,0.)).poses[-1]
        n.expected=(20.,0.);n.route=self.route((-12.,0.),'frontier-A');n.route_endpoint=(-12.,0.)
        n.candidates=[(-12.,0.,math.pi,12.)];n.vehicle_pose_feasible=Mock(return_value=True)
        n.begin_exploration_comparison()
        self.assertEqual(n.candidates[0][:2],(8.,0.))
        self.assertEqual(n.candidates[-1][:2],(-12.,0.))
        failed=n.candidates[0];n.remember_failed_goal((0.,0.),failed[:2],failed[2])
        self.assertNotIn(failed,n.target_forward_candidates((0.,0.)))
        n.memory_goal=(20.,0.)
        self.assertEqual(n.target_forward_candidates((0.,0.)),[])

''' +s[where:];p.write_text(s)
print('Step2 applied; no runtime components added')
