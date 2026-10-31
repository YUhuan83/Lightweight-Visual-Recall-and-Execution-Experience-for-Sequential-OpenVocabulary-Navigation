"""Navigation file/live-parameter contracts, independent of ROS and BeamNG."""
import ast
import math
from .vehicle_geometry import (SBR_FRONT_EXTENT, SBR_REAR_EXTENT,
                               SBR_HALF_WIDTH, SBR_WHEELBASE)
from .arrival_criteria import STANDOFF_ARRIVAL_TOLERANCE_M
from .motion_path import ACKERMANN_CUSP_POSITION_TOLERANCE_M
from .sensor_geometry import SBR_LIDAR_OFFSET


def runtime_parameter_value(value, expected):
    """Decode ParameterValue only when its ROS type matches the contract."""
    scalar = {bool: (1, 'bool_value'), int: (2, 'integer_value'),
              float: (3, 'double_value'), str: (4, 'string_value')}
    arrays = {bool: (6, 'bool_array_value'), int: (7, 'integer_array_value'),
              float: (8, 'double_array_value'), str: (9, 'string_array_value')}
    is_array = isinstance(expected, list)
    if is_array:
        element_type = type(expected[0]) if expected else str
        if any(type(element) is not element_type for element in expected):
            raise ValueError('Runtime parameter contract arrays must have one type')
        specification = arrays.get(element_type)
    else:
        specification = scalar.get(type(expected))
    if specification is None or value.type != specification[0]:
        return None
    observed = getattr(value, specification[1])
    return list(observed) if is_array else observed


def observation_runtime_contract(config):
    value = float(config['observation_range_m'])
    if (not math.isfinite(value) or value <= 0
            or value != float(config['frontiers_range'])):
        raise ValueError('Visual and point-cloud observation ranges must agree')
    return {'/wildos': {'visual_frontier_range_m': value},
            '/wildos_official_cloud_filter': {'max_range': value}}


def graph_geometry_profile(config):
    """Derive necessary graph clearance from the actual rectangular footprint.

    The inscribed circle is a lower bound valid at every heading. It does not
    certify the long front overhang or a turning sweep; Smac still checks the
    full padded polygon. Unknown vertical/dynamic quantities remain unknown.
    """
    profiles = []
    for name in ('local_costmap', 'global_costmap'):
        options = config[name][name]['ros__parameters']
        polygon = ast.literal_eval(options.get('footprint', '[]'))
        if (len(polygon) != 4 or any(len(p) != 2 for p in polygon)
                or not all(math.isfinite(v) for p in polygon for v in p)):
            raise ValueError(name + ': graph geometry needs a finite rectangular footprint')
        front = max(p[0] for p in polygon)
        rear = -min(p[0] for p in polygon)
        half_width = max(abs(p[1]) for p in polygon)
        expected = {(front, half_width), (front, -half_width),
                    (-rear, half_width), (-rear, -half_width)}
        padding = float(options['footprint_padding'])
        if (set(map(tuple, polygon)) != expected or min(front, rear, half_width) <= 0
                or not math.isfinite(padding) or padding < 0):
            raise ValueError(name + ': invalid rear-reference rectangle or padding')
        profiles.append((front, rear, half_width, padding))
    if profiles[0] != profiles[1]:
        raise ValueError('Graph/local/global footprint geometry differs')
    front, rear, half_width, padding = profiles[0]
    radius = float(config['controller_server']['ros__parameters']['FollowPath']['minimum_turning_radius'])
    if not math.isfinite(radius) or radius <= 0:
        raise ValueError('Invalid physical turning radius')
    return dict(robot_body_front_extent=float(front), robot_body_rear_extent=float(rear),
                robot_body_half_width=float(half_width), robot_footprint_padding=padding,
                robot_minimum_clearance=min(front, rear, half_width) + padding,
                robot_body_height=-1.0, robot_ground_clearance=-1.0,
                robot_min_turning_radius=radius, robot_holonomic=False)


def navigation_runtime_contract(config, carrier_model):
    """Required live parameters for the measured SBR or ideal diagnostic rig."""
    if carrier_model not in ('ideal_point', 'ackermann'):
        raise ValueError('Unknown carrier model')
    physical = carrier_model == 'ackermann'
    controller = config['controller_server']['ros__parameters']['FollowPath']
    if controller.get('use_collision_detection') is not True:
        raise ValueError('FollowPath collision checking is disabled')
    required = {'/controller_server': {'FollowPath.use_collision_detection': True}}
    if physical:
        planner = config['planner_server']['ros__parameters']
        car = planner.get('CarLike', {})
        if (planner.get('planner_plugins') != ['CarLike']
                or car.get('plugin') != 'wildos_ackermann_smac/Planner'
                or car.get('motion_model_for_search') != 'REEDS_SHEPP'
                or car.get('minimum_turning_radius') != 12.0
                or car.get('angle_quantization_bins') != 72
                or controller.get('plugin') != 'wildos_ackermann_rpp/Controller'
                or controller.get('minimum_turning_radius') != 8.0
                or controller.get('use_rotate_to_heading') is not False
                or controller.get('allow_reversing') is not True
                or controller.get('regulated_linear_scaling_min_radius') != 8.0
                or controller.get('use_velocity_scaled_lookahead_dist') is not False
                or controller.get('lookahead_dist') != 3.0
                or controller.get('max_robot_pose_search_dist') != 10.0):
            raise ValueError('Physical planning/controller curvature contract differs')
        required['/planner_server'] = {
            'planner_plugins': ['CarLike'], 'CarLike.plugin': car['plugin'],
            'CarLike.motion_model_for_search': 'REEDS_SHEPP',
            'CarLike.minimum_turning_radius': 12.0, 'CarLike.angle_quantization_bins': 72}
        connector = {'max_planning_time': 1.5, 'max_total_planning_time': 2.0,
                     'connector_search_time': .2, 'connector_max_evaluations': 6000,
                     'connector_max_candidates': 6, 'connector_min_section': .4}
        if any(car.get(key) != value for key, value in connector.items()):
            raise ValueError('Physical continuous-start planner budgets/sections differ')
        required['/planner_server'].update({'CarLike.' + key: value
                                           for key, value in connector.items()})
        required['/controller_server'].update({
            'FollowPath.plugin': 'wildos_ackermann_rpp/Controller',
            'FollowPath.minimum_turning_radius': 8.0,
            'FollowPath.lookahead_dist': 3.0,
            'FollowPath.use_velocity_scaled_lookahead_dist': False,
            'FollowPath.max_robot_pose_search_dist': 10.0,
            'FollowPath.use_rotate_to_heading': False, 'FollowPath.allow_reversing': True,
            'FollowPath.regulated_linear_scaling_min_radius': 8.0})
        checks = config['controller_server']['ros__parameters']
        if checks.get('maneuver_goal_checker', {}).get('xy_goal_tolerance') != STANDOFF_ARRIVAL_TOLERANCE_M:
            raise ValueError('Target endpoint checker and arrival tolerance differ')
        required['/controller_server']['maneuver_goal_checker.xy_goal_tolerance'] = STANDOFF_ARRIVAL_TOLERANCE_M
        cusp = {'plugin': 'nav2_controller::StoppedGoalChecker',
                'xy_goal_tolerance': ACKERMANN_CUSP_POSITION_TOLERANCE_M,
                'yaw_goal_tolerance': .35, 'stateful': False,
                'trans_stopped_velocity': .05, 'rot_stopped_velocity': .05}
        if ('cusp_goal_checker' not in checks.get('goal_checker_plugins', [])
                or any(checks.get('cusp_goal_checker', {}).get(key) != value for key,value in cusp.items())):
            raise ValueError('Physical cusp must use accurate pose and measured stopped velocity')
        required['/controller_server']['goal_checker_plugins'] = checks['goal_checker_plugins']
        required['/controller_server'].update({'cusp_goal_checker.'+key:value for key,value in cusp.items()})
        required['/wildos_graph_path_bridge'] = {
            'carrier_model': 'ackermann', 'planner_id': 'CarLike', 'minimum_turning_radius': 8.0,
            'require_source_control': True,
            'object_arrival_distance': 2.0, 'traffic_light_arrival_distance': 2.0}
        required['/beamng_wildos_adapter'] = {'carrier_model': 'ackermann',
                                             'lidar_offset': list(SBR_LIDAR_OFFSET),
                                             'obstacle_min_height': .1, 'ground_max_slope': .35}
        required['/beamng_vehicle_goal_standoff'] = {
            'carrier_model': 'ackermann', 'vehicle_front_extent': SBR_FRONT_EXTENT,
            'observation_minimum_turning_radius': max(car['minimum_turning_radius'],controller['minimum_turning_radius'])}
        required['/wildos_official_cloud_filter'] = {
            'filter_footprint_returns': True, 'footprint_front_extent': SBR_FRONT_EXTENT,
            'footprint_rear_extent': SBR_REAR_EXTENT, 'footprint_half_width': SBR_HALF_WIDTH,
            'obstacle_min_height': .1, 'ground_max_slope': .35}
        required['/spot1/graphnav_mapper'] = dict(
            graph_geometry_profile(config), robot_profile_id='default',
            use_occupancy_input=False, robot_max_slope=.35)
        # Keep the existing same-process profile identity: the visual scorer
        # and execution feedback also use "default". Renaming it here alone
        # would silently bypass those consumers' physical assessments.
        required['/spot1/graphnav_planner'] = {
            'robot_profile_id': 'default', 'missing_frontier_timeout_seconds': 10.}
        required['/wildos_cmd_vel_to_control'] = {
            'allow_reverse': True, 'minimum_turning_radius': 8.0,
            'wheelbase': SBR_WHEELBASE, 'steer_sign': -1.0,
            'max_steering': .6, 'max_steer_angle': .54,
            'state_topic': '/vehicles/ego/sensors/state',
            'cmd_vel_topic': '/wildos/cmd_vel_stamped',
            'require_stamped_command': True, 'cmd_vel_frame': 'spot1/nav_base_link',
            'use_sim_time': False, 'control_topic': '/wildos/native_control_command',
            'control_protocol': 'wildos_source_control_v1'}
        required['/wildos_native_vehicle_control'] = {
            'control_topic': '/wildos/native_control_command', 'control_protocol': 'wildos_source_control_v1'}
    for name in ('local_costmap', 'global_costmap'):
        options = config[name][name]['ros__parameters']
        if options.get('robot_base_frame') != 'spot1/nav_base_link':
            raise ValueError(name + ': wrong navigation frame')
        fields = {'robot_base_frame': 'spot1/nav_base_link',
                  'inflation_layer.inflation_radius': options['inflation_layer']['inflation_radius']}
        if physical:
            polygon = ast.literal_eval(options.get('footprint', '[]'))
            expected = [[SBR_FRONT_EXTENT, SBR_HALF_WIDTH], [SBR_FRONT_EXTENT, -SBR_HALF_WIDTH],
                        [-SBR_REAR_EXTENT, -SBR_HALF_WIDTH], [-SBR_REAR_EXTENT, SBR_HALF_WIDTH]]
            if polygon != expected:
                raise ValueError(name + ': footprint differs from rear-axle SBR geometry')
            if options['inflation_layer']['inflation_radius'] < SBR_HALF_WIDTH:
                raise ValueError(name + ': inflation smaller than inscribed radius')
            if name == 'global_costmap':
                padding = options['footprint_padding']
                resolution = options['resolution']
                inflation = options['inflation_layer']['inflation_radius']
                if (not all(map(math.isfinite, (padding, resolution, inflation)))
                        or padding < 0 or resolution <= 0):
                    raise ValueError(name + ': invalid Smac collision geometry')
                # Humble Smac skips rectangular checks below its circumscribed
                # inflation cost. A truncated field can leave that center at 0
                # while a long body corner intersects a lethal cell.
                circumscribed = max(math.hypot(x + math.copysign(padding, x),
                                              y + math.copysign(padding, y))
                                    for x, y in polygon)
                minimum_inflation = circumscribed + resolution / math.sqrt(2)
                if inflation < minimum_inflation:
                    raise ValueError(name + ': inflation does not cover Smac body collision shortcut')
                fields.update(resolution=resolution,
                              **{'inflation_layer.cost_scaling_factor':
                                 options['inflation_layer']['cost_scaling_factor']})
            fields.update(footprint=options['footprint'], footprint_padding=options['footprint_padding'])
        else:
            radius = options['robot_radius']
            if (not math.isfinite(radius) or radius < options['resolution']/2.0
                    or options['inflation_layer']['inflation_radius'] < radius):
                raise ValueError(name + ': invalid RPP collision sampling radius')
            fields['robot_radius'] = radius
        layer = options['obstacle_layer']
        if (layer.get('enabled') is not True or layer.get('footprint_clearing_enabled') is not True
                or set(layer.get('observation_sources', '').split()) != {'cloud', 'clearing_cloud'}):
            raise ValueError(name + ': missing marking/clearing contract')
        fields.update({'obstacle_layer.enabled': True, 'obstacle_layer.footprint_clearing_enabled': True,
                       'obstacle_layer.observation_sources': 'cloud clearing_cloud'})
        for key, expected in (('min_obstacle_height', -1000.0), ('max_obstacle_height', 1000.0)):
            if layer.get(key) != expected:
                raise ValueError(name + ': invalid world-height bound ' + key)
            fields['obstacle_layer.' + key] = expected
        for source, marking, clearing in (('cloud', True, False), ('clearing_cloud', False, True)):
            values = layer.get(source, {})
            if physical:
                if values.get('sensor_frame') != 'spot1/lidar_ray_origin':
                    raise ValueError(name + ': ' + source + ' must use physical LiDAR ray origin')
                fields['obstacle_layer.' + source + '.sensor_frame'] = 'spot1/lidar_ray_origin'
            for key, expected in (('topic', '/wildos/points' if marking else '/wildos/clearing_points'),
                                  ('data_type', 'PointCloud2'), ('marking', marking), ('clearing', clearing),
                                  ('min_obstacle_height', -1000.0), ('max_obstacle_height', 1000.0)):
                if values.get(key) != expected:
                    raise ValueError(name + ': invalid ' + source + '.' + key)
                fields['obstacle_layer.' + source + '.' + key] = expected
        required['/' + name + '/' + name] = fields
    return required
