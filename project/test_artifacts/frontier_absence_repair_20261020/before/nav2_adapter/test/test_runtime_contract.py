import ast
import copy
import importlib.util
from pathlib import Path
import unittest
import yaml
from wildos_nav2_adapter.runtime_contract import navigation_runtime_contract
from wildos_nav2_adapter.runtime_contract import graph_geometry_profile
from wildos_nav2_adapter.physical_vehicle import physical_snapshot_errors

ROOT = Path(__file__).resolve().parents[2]


class RuntimeContractTests(unittest.TestCase):
    def setUp(self):
        self.config = yaml.safe_load((ROOT/'nav2_adapter/config/nav2_ackermann_control.yaml').read_text())

    def test_graph_and_costmaps_share_measured_geometry_not_point_robot_clearance(self):
        profile = graph_geometry_profile(self.config)
        self.assertAlmostEqual(profile['robot_minimum_clearance'], 1.06)
        self.assertEqual(profile['robot_body_front_extent'], 3.4)
        self.assertEqual(profile['robot_body_rear_extent'], 1.05)
        self.assertEqual(profile['robot_body_half_width'], 1.05)
        self.assertEqual(profile['robot_ground_clearance'], -1.)
        self.assertEqual(profile['robot_body_height'], -1.)
        self.assertFalse(profile['robot_holonomic'])
        self.assertEqual(profile['robot_min_turning_radius'], 8.)
        required = navigation_runtime_contract(self.config, 'ackermann')
        for key, value in profile.items():
            self.assertEqual(required['/spot1/graphnav_mapper'][key], value)
        self.assertEqual(required['/spot1/graphnav_planner']['robot_profile_id'],
                         required['/spot1/graphnav_mapper']['robot_profile_id'])

    def test_graph_geometry_refuses_disagreeing_or_invalid_footprints(self):
        for key, value in (('footprint_padding', .02), ('footprint_padding', -.1),
                           ('footprint', '[]'), ('footprint', '[[1, 1], [1, 0], [-1, -1], [-1, 1]]')):
            changed = copy.deepcopy(self.config)
            changed['global_costmap']['global_costmap']['ros__parameters'][key] = value
            with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                graph_geometry_profile(changed)

    def test_physical_stack_requires_one_curvature_planner_and_native_state(self):
        required = navigation_runtime_contract(self.config, 'ackermann')
        self.assertEqual(required['/planner_server']['planner_plugins'], ['CarLike'])
        speed = required['/wildos_cmd_vel_to_control']
        self.assertEqual(speed['control_topic'], '/wildos/native_control_command')
        self.assertEqual(speed['control_protocol'], 'wildos_source_control_v1')
        self.assertEqual(required['/wildos_native_vehicle_control']['control_protocol'], speed['control_protocol'])
        self.assertEqual(required['/wildos_native_vehicle_control']['control_topic'], speed['control_topic'])
        self.assertTrue(required['/wildos_graph_path_bridge']['require_source_control'])
        self.assertEqual(speed['state_topic'], '/vehicles/ego/sensors/state')
        self.assertTrue(speed['require_stamped_command'])
        self.assertEqual(speed['cmd_vel_topic'],'/wildos/cmd_vel_stamped')
        self.assertEqual(speed['cmd_vel_frame'],'spot1/nav_base_link')
        self.assertIs(speed['use_sim_time'],False)
        radius = required['/wildos_graph_path_bridge']['minimum_turning_radius']
        self.assertGreater(required['/planner_server']['CarLike.minimum_turning_radius'],radius)
        self.assertEqual(radius,required['/controller_server']['FollowPath.minimum_turning_radius'])
        self.assertEqual(radius, speed['minimum_turning_radius'])

    def test_physical_launch_and_runtime_gate_agree_on_source_control_wire_contract(self):
        tree=ast.parse((ROOT/'nav2_adapter/launch/physical_vehicle_control.launch.py').read_text(encoding='utf8'))
        launched={}
        for node in ast.walk(tree):
            if not isinstance(node,ast.Call) or not isinstance(node.func,ast.Name) or node.func.id!='Node':continue
            keywords={k.arg:k.value for k in node.keywords}
            name=keywords.get('name');params=keywords.get('parameters')
            if not isinstance(name,ast.Constant) or not isinstance(params,ast.List):continue
            literals={}
            for item in params.elts:
                if not isinstance(item,ast.Dict):continue
                for key,value in zip(item.keys,item.values):
                    try:literals[ast.literal_eval(key)]=ast.literal_eval(value)
                    except (ValueError,TypeError):pass
            launched['/'+name.value]=literals
        required=navigation_runtime_contract(self.config,'ackermann')
        for owner in ('/wildos_cmd_vel_to_control','/wildos_native_vehicle_control'):
            for key in ('control_protocol','control_topic'):
                self.assertEqual(launched[owner][key],required[owner][key])
        for key in ('cmd_vel_topic','require_stamped_command','cmd_vel_frame','use_sim_time'):
            self.assertEqual(launched['/wildos_cmd_vel_to_control'][key],
                             required['/wildos_cmd_vel_to_control'][key])
        self.assertIs(launched['/wildos_graph_path_bridge']['require_source_control'],True)

    def test_original_empty_obstacles_or_disabled_collision_cannot_pass(self):
        for changed in ('collision', 'local_costmap', 'global_costmap'):
            config = copy.deepcopy(self.config)
            if changed == 'collision':
                config['controller_server']['ros__parameters']['FollowPath']['use_collision_detection'] = False
            else:
                config[changed][changed]['ros__parameters']['obstacle_layer']['observation_sources'] = ''
            with self.subTest(changed=changed), self.assertRaises(ValueError):
                navigation_runtime_contract(config, 'ackermann')

    def test_reintroduced_navfn_fallback_cannot_pass_physical_gate(self):
        self.config['planner_server']['ros__parameters']['planner_plugins'].insert(0, 'GridBased')
        with self.assertRaises(ValueError):
            navigation_runtime_contract(self.config, 'ackermann')

    def test_continuous_start_plugin_and_bounded_search_must_match_live(self):
        required = navigation_runtime_contract(self.config, 'ackermann')['/planner_server']
        self.assertEqual(required['CarLike.plugin'], 'wildos_ackermann_smac/Planner')
        self.assertEqual(required['CarLike.max_total_planning_time'], 2.)
        for key, value in [('plugin', 'nav2_smac_planner/SmacPlannerHybrid'),
                           ('max_total_planning_time', 5.), ('connector_search_time', 1.),
                           ('connector_max_evaluations', 100000), ('connector_min_section', .2)]:
            config = copy.deepcopy(self.config)
            config['planner_server']['ros__parameters']['CarLike'][key] = value
            with self.subTest(key=key), self.assertRaises(ValueError):
                navigation_runtime_contract(config, 'ackermann')

    def test_rotation_reverse_and_curvature_drift_are_rejected(self):
        for key, value in (('use_rotate_to_heading', True), ('allow_reversing', False),
                           ('regulated_linear_scaling_min_radius', 0.0)):
            config = copy.deepcopy(self.config)
            config['controller_server']['ros__parameters']['FollowPath'][key] = value
            with self.subTest(key=key), self.assertRaises(ValueError):
                navigation_runtime_contract(config, 'ackermann')

    def test_tracking_preview_and_controller_share_fixed_lookahead(self):
        for key,value in [('use_velocity_scaled_lookahead_dist',True),
                          ('lookahead_dist',6.),('max_robot_pose_search_dist',20.)]:
            config=copy.deepcopy(self.config)
            config['controller_server']['ros__parameters']['FollowPath'][key]=value
            with self.subTest(key=key),self.assertRaises(ValueError):
                navigation_runtime_contract(config,'ackermann')

    def test_planning_headroom_does_not_change_physical_command_limit(self):
        required=navigation_runtime_contract(self.config,'ackermann')
        self.assertEqual(required['/planner_server']['CarLike.minimum_turning_radius'],12.)
        self.assertEqual(required['/controller_server']['FollowPath.minimum_turning_radius'],8.)
        self.assertEqual(required['/wildos_cmd_vel_to_control']['minimum_turning_radius'],8.)
        for radius in (8.,6.,20.):
            config=copy.deepcopy(self.config)
            config['planner_server']['ros__parameters']['CarLike']['minimum_turning_radius']=radius
            with self.subTest(radius=radius),self.assertRaises(ValueError):
                navigation_runtime_contract(config,'ackermann')

    def test_collision_prediction_must_use_the_actuated_curvature(self):
        for key,value in [('plugin','nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController'),
                          ('minimum_turning_radius',4.)]:
            config=copy.deepcopy(self.config)
            config['controller_server']['ros__parameters']['FollowPath'][key]=value
            with self.subTest(key=key),self.assertRaises(ValueError):
                navigation_runtime_contract(config,'ackermann')

    def test_wrong_car_origin_or_point_footprint_is_rejected(self):
        options = self.config['local_costmap']['local_costmap']['ros__parameters']
        for footprint in ('[]', '[[-2,1],[2,1],[2,-1],[-2,-1]]'):
            options['footprint'] = footprint
            with self.subTest(footprint=footprint), self.assertRaises(ValueError):
                navigation_runtime_contract(self.config, 'ackermann')

    def test_absolute_obstacle_height_does_not_drop_elevated_town(self):
        options = self.config['global_costmap']['global_costmap']['ros__parameters']
        options['obstacle_layer']['cloud']['max_obstacle_height'] = 2.0
        with self.assertRaises(ValueError):
            navigation_runtime_contract(self.config, 'ackermann')

    def test_target_path_checker_cannot_finish_outside_arrival_tolerance(self):
        self.config['controller_server']['ros__parameters']['maneuver_goal_checker']['xy_goal_tolerance'] = 1.0
        with self.assertRaises(ValueError):
            navigation_runtime_contract(self.config, 'ackermann')

    def test_smac_inflation_must_cover_rear_axle_body_not_only_width(self):
        for radius in (2.0, 3.57, 3.74, float('nan'), float('inf')):
            config = copy.deepcopy(self.config)
            config['global_costmap']['global_costmap']['ros__parameters']['inflation_layer']['inflation_radius'] = radius
            with self.subTest(radius=radius), self.assertRaises(ValueError):
                navigation_runtime_contract(config, 'ackermann')
        required = navigation_runtime_contract(self.config, 'ackermann')
        self.assertEqual(required['/global_costmap/global_costmap']['inflation_layer.inflation_radius'], 3.75)
        self.assertEqual(required['/global_costmap/global_costmap']['inflation_layer.cost_scaling_factor'], 2.0)
        # RPP checks its actual footprint and does not use the Smac shortcut.
        self.assertEqual(required['/local_costmap/local_costmap']['inflation_layer.inflation_radius'], 1.6)

    def test_smac_coverage_gate_tracks_grid_and_padding_changes(self):
        for key, value in (('resolution', .5), ('footprint_padding', .2),
                           ('resolution', 0.), ('footprint_padding', float('nan'))):
            config = copy.deepcopy(self.config)
            config['global_costmap']['global_costmap']['ros__parameters'][key] = value
            with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                navigation_runtime_contract(config, 'ackermann')

    def test_cusp_is_not_an_observation_region_or_a_moving_success(self):
        required=navigation_runtime_contract(self.config,'ackermann')['/controller_server']
        self.assertEqual(required['cusp_goal_checker.xy_goal_tolerance'],.2)
        for key,value in [('plugin','nav2_controller::SimpleGoalChecker'),
                          ('xy_goal_tolerance',.6),('trans_stopped_velocity',.25),('stateful',True)]:
            config=copy.deepcopy(self.config)
            config['controller_server']['ros__parameters']['cusp_goal_checker'][key]=value
            with self.subTest(key=key),self.assertRaises(ValueError):
                navigation_runtime_contract(config,'ackermann')

    def test_ideal_diagnostic_config_retains_its_existing_contract(self):
        config = yaml.safe_load((ROOT/'nav2_adapter/config/nav2_official_control.yaml').read_text())
        required = navigation_runtime_contract(config, 'ideal_point')
        self.assertNotIn('/wildos_cmd_vel_to_control', required)
        self.assertEqual(required['/local_costmap/local_costmap']['robot_radius'], .15)


class RealPhysicsEvidenceTests(unittest.TestCase):
    def setUp(self):
        self.snapshot = dict(kinematicExtensionLoaded=False, collisionOverrideLoaded=False,
            configKinematic=False, configBodyDisabled=False, nodeCount=723, fixedNodeCount=0,
            massKg=1500., beamCount=2100, bodyCollisionNodeCount=200, wheelCount=4,
            wheels=[dict(treadNodeCount=40, collisionNodeCount=40, fixedNodeCount=0) for _ in range(4)],
            cameras={'front': 1}, lidars={'front': 2})

    def test_dynamic_body_and_all_four_tires_pass(self):
        self.assertEqual(physical_snapshot_errors(self.snapshot), [])

    def test_ackermann_named_fixed_carrier_does_not_pass(self):
        self.snapshot.update(carrierModel='ackermann', fixedNodeCount=723, configKinematic=True)
        self.assertTrue(physical_snapshot_errors(self.snapshot))

    def test_each_missing_physics_layer_or_sensor_fails(self):
        for key, value in (('kinematicExtensionLoaded', True), ('collisionOverrideLoaded', True),
                           ('bodyCollisionNodeCount', 0), ('beamCount', 0),
                           ('massKg', float('nan')), ('cameras', {}), ('lidars', {})):
            snapshot = copy.deepcopy(self.snapshot)
            snapshot[key] = value
            with self.subTest(key=key):
                self.assertTrue(physical_snapshot_errors(snapshot))

    def test_one_disabled_or_fixed_tire_cannot_hide_in_aggregate(self):
        for key, value in (('collisionNodeCount', 0), ('fixedNodeCount', 40)):
            snapshot = copy.deepcopy(self.snapshot)
            snapshot['wheels'][2][key] = value
            with self.subTest(key=key):
                self.assertTrue(physical_snapshot_errors(snapshot))

    def test_engine_evidence_is_required_not_just_mode_name(self):
        self.assertTrue(physical_snapshot_errors(dict(carrierModel='ackermann')))


class RestoreBaselineTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        spec = importlib.util.spec_from_file_location('restore_checked', ROOT/'tools/restore_physical_vehicle.py')
        cls.tool = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(cls.tool)

    def test_original_config_is_retained_and_override_metadata_removed(self):
        source = dict(model='sbr', partConfigFilename='vehicles/sbr/TT_AWD_S2_DCT.pc',
                      partsTree={'slots': {'wheels': 'stock'}}, wildosKinematicProfile=False,
                      wildosCollisionProfileVersion='old')
        result = self.tool.original_config({'config': source})
        self.assertEqual(result['partsTree'], source['partsTree'])
        self.assertNotIn('wildosKinematicProfile', result)
        self.assertIn('wildosKinematicProfile', source)

    def test_disabled_saved_config_or_wrong_vehicle_is_not_original(self):
        for changes in ({'model': 'other'}, {'partsTree': {}}, {'wildosKinematicProfile': True},
                        {'wildosBodyCollisionDisabled': True}):
            source = dict(model='sbr', partConfigFilename='vehicles/sbr/TT_AWD_S2_DCT.pc',
                          partsTree={'slots': {'wheels': 'stock'}})
            source.update(changes)
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                self.tool.original_config(source)


if __name__ == '__main__':
    unittest.main()


class ObservationContractTests(unittest.TestCase):
    def test_runtime_parameter_reader_preserves_ros_array_types(self):
        from types import SimpleNamespace as S
        from wildos_nav2_adapter.runtime_contract import runtime_parameter_value
        offset = [1.252, 0., 1.56]
        self.assertEqual(runtime_parameter_value(S(type=8, double_array_value=offset), offset), offset)
        self.assertIsNone(runtime_parameter_value(S(type=9, string_array_value=['1.252','0','1.56']), offset))
        self.assertEqual(runtime_parameter_value(S(type=9, string_array_value=['CarLike']), ['CarLike']), ['CarLike'])
        self.assertIsNone(runtime_parameter_value(S(type=8, double_array_value=[1.]), ['CarLike']))
        self.assertIsNone(runtime_parameter_value(S(type=2, integer_value=1), True))
        self.assertIsNone(runtime_parameter_value(S(type=3, double_value=1.), 1))

    def test_observation_range_is_shared_and_mismatch_is_rejected(self):
        from wildos_nav2_adapter.runtime_contract import observation_runtime_contract
        value = observation_runtime_contract(dict(observation_range_m=40., frontiers_range=40.))
        self.assertEqual(value['/wildos']['visual_frontier_range_m'], 40.)
        self.assertEqual(value['/wildos_official_cloud_filter']['max_range'], 40.)
        for v in (0., float('nan'), float('inf'), 9.):
            with self.assertRaises(ValueError):
                observation_runtime_contract(dict(observation_range_m=v, frontiers_range=40.))
