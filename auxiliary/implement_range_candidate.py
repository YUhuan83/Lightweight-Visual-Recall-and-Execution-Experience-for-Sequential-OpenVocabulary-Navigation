from pathlib import Path
root=Path(r'C:\Users\Yu\Desktop\拟真\test_artifacts\range_candidate_20261027\candidate')
p=root/'official_adapter/beamng_wildos_adapter/goal_standoff.py';s=p.read_text(encoding='utf8')
old='''        navigation={}
        if (str(self.get_parameter('carrier_model').value).lower()=='ackermann'
                and self.observation_minimum_turning_radius>0):
            selected=self.vehicle_observation_pose(evidence['bearing'])'''
new='''        navigation={}
        bearing=evidence['bearing']
        candidate=None
        # A current supported range is useful for approaching to observe, even
        # before temporal consensus can confirm the object's identity. Reuse
        # only this exact mask's accepted measurement; never carry a range to
        # a new mask/instance or turn it into arrival evidence.
        if self.pending_confirmation is None and self.observations:
            latest=self.observations[-1]
            if latest['observation_stamp_ns']==evidence['stamp']:
                point=latest['position']
                measured_bearing=math.atan2(point[1]-robot.y,point[0]-robot.x)
                if abs(math.remainder(measured_bearing-bearing,2*math.pi))<=.35:
                    candidate=latest
                    bearing=measured_bearing
        if (str(self.get_parameter('carrier_model').value).lower()=='ackermann'
                and self.observation_minimum_turning_radius>0):
            if candidate is None:
                selected=self.vehicle_observation_pose(bearing)
            else:
                clearance=observation_clearance(self.get_parameter('camera_offset').value,
                    float(self.get_parameter('minimum_camera_target_range').value),
                    STANDOFF_ARRIVAL_TOLERANCE_M)
                selected=self.vehicle_observation_pose(bearing,tuple(candidate['position'][:2]),clearance)'''
assert old in s;s=s.replace(old,new,1)
# Keep ideal-point legacy proposals unchanged; only actual car proposals use range.
s=s.replace("        else:\n            point = observation_step(grid", "        else:\n            candidate=None\n            bearing=evidence['bearing']\n            point = observation_step(grid",1)
old="""            'bearing': evidence['bearing'], 'evidence': 'bearing_only',
            'position_confirmed': False, 'search_confirmed': False,**navigation})))"""
new="""            'bearing': bearing, 'evidence': 'range_candidate' if candidate else 'bearing_only',
            'position_confirmed': False, 'search_confirmed': False,**navigation,
            **(dict(target_range_known=True,object_position=candidate['position'],
                    localization_source=candidate['source']) if candidate else {})})))"""
assert old in s;s=s.replace(old,new,1)
s=s.replace('''        if len(self.observations) < minimum_support:
            return''','''        if len(self.observations) < minimum_support:
            self.publish_visible_step()
            return''',1)
s=s.replace('''        clusters = supported_position_clusters(values, radius, minimum_support)
        if not clusters:
            return''','''        clusters = supported_position_clusters(values, radius, minimum_support)
        if not clusters:
            self.publish_visible_step()
            return''',1)
p.write_text(s,encoding='utf8')
p=root/'nav2_adapter/wildos_nav2_adapter/visible_approach.py';s=p.read_text(encoding='utf8')
s=s.replace("or data.get('evidence') != 'bearing_only'", "or data.get('evidence') not in ('bearing_only','range_candidate')",1)
old="""            if self.visible_hint and stamp < self.visible_hint['observation_stamp_ns']:
                return"""
new="""            if data['evidence']=='range_candidate':
                target=data.get('object_position',())
                if (data.get('target_range_known') is not True
                        or data.get('localization_source') not in ('lidar','multiview')
                        or len(target)!=3 or not all(map(math.isfinite,target))):
                    return
            if self.visible_hint and stamp < self.visible_hint['observation_stamp_ns']:
                return"""
assert old in s;s=s.replace(old,new,1)
s=s.replace('target_range_known=False','target_range_known=bool((self.visible_hint or {}).get(\'target_range_known\'))')
p.write_text(s,encoding='utf8')
