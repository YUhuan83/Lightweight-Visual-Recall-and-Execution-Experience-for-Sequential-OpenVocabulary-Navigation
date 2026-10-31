"""Persistent physical-vehicle experiment console. Linux server entry point.

No ROS or BeamNG imports until after setup/DDS bootstrap and process checks.
Commands are argv arrays throughout; configuration is trusted local input.
"""
import argparse
from collections import deque
import hashlib
import json
import math
import os
from pathlib import Path
import queue
import signal
import subprocess
import sys
import threading
import time


ROOT = Path(__file__).resolve().parents[1]
HELP = '输入目标（建议英文）；/stop 停车；/status 状态；/quit 停车并退出；/help 帮助'


def record_console_event(node, kind, details):
    # Match Harness.event's mapping merge: plan_status legitimately carries query.
    item = {'elapsed': time.monotonic()-node.started_at, 'phase': node.phase,
            'query': node.query, 'kind': kind, **details}
    node.events.append(item)
    node.log.write(json.dumps(item)+'\n')
    node.log.flush()
    return item


def write_json(path, value):
    temporary = path.with_suffix('.next.json')
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2)+'\n', encoding='utf8')
    temporary.replace(path)


def valid_query(value):
    value = value.strip()
    if not value or len(value) > 200 or any(ord(c) < 32 for c in value):
        raise ValueError('目标须为 1–200 个可打印字符')
    return ' '.join(value.casefold().split())


def proc_identity(pid):
    """Linux start time + boot ID protects against PID reuse (including spaces in comm)."""
    base = Path('/proc')/str(pid)
    raw = (base/'stat').read_text()
    tail = raw[raw.rfind(')')+2:].split()
    return {'pid': pid, 'start': tail[19],
            'boot': Path('/proc/sys/kernel/random/boot_id').read_text().strip()}


def group_members(pgid):
    members = []
    for path in Path('/proc').iterdir():
        if not path.name.isdigit():
            continue
        try:
            raw = (path/'stat').read_text()
            if raw[raw.rfind(')')+2:].split()[0] in ('Z', 'X'):
                continue
            if os.getpgid(int(path.name)) == pgid:
                members.append(proc_identity(int(path.name)))
        except (ProcessLookupError, FileNotFoundError):
            pass
    return members


class OwnedProcesses:
    def __init__(self, output, cwd):
        self.output, self.cwd = output, cwd
        self.children = []
        self.known = {}
        self.last_scan = 0.

    def spawn(self, role, argv, env=None, cwd=None):
        with (self.output/(role+'.log')).open('xb') as log:
            child = subprocess.Popen(argv, cwd=cwd or self.cwd, env=env, stdin=subprocess.DEVNULL,
                                     stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        self.children.append((role, child, proc_identity(child.pid)))
        self.known[child.pid] = [self.children[-1][2]]
        (self.output/(role+'.pid')).write_text(str(child.pid)+'\n')
        write_json(self.output/'owned_processes.json', [dict(role=r, **i) for r, _, i in self.children])
        return child

    def healthy(self):
        for role, child, identity in self.children:
            if child.poll() is not None or proc_identity(child.pid) != identity:
                raise RuntimeError(f'{role} 进程已退出或身份变化；查看 {role}.log')
        if time.monotonic()-self.last_scan >= 1:
            for _, child, _ in self.children:
                self.known[child.pid] = group_members(child.pid)
            self.last_scan = time.monotonic()

    def close(self):
        errors = []
        # Children are group leaders. Never signal an unrelated/reused PID.
        for role, child, identity in reversed(self.children):
            try:
                members = group_members(child.pid)
                if not members:
                    child.poll()
                    continue
                # A process-group ID remains reserved while descendants exist. All
                # members must be from this boot and born after this owned leader.
                if any(m['boot'] != identity['boot'] or int(m['start']) < int(identity['start'])
                       or (m['pid'] == child.pid and m != identity) for m in members):
                    raise RuntimeError('process identity/group changed')
                if not any(m in self.known.get(child.pid, []) for m in members):
                    raise RuntimeError('no surviving owned group identity; refusing signal')
                for sig, timeout in ((signal.SIGINT, 12), (signal.SIGTERM, 5), (signal.SIGKILL, 3)):
                    os.killpg(child.pid, sig)
                    deadline = time.monotonic()+timeout
                    while time.monotonic() < deadline:
                        child.poll()  # reap leader before testing whether group is empty
                        if not group_members(child.pid):
                            break
                        time.sleep(.1)
                    if not group_members(child.pid):
                        break
                else:
                    raise RuntimeError('owned group still exists after SIGKILL')
            except Exception as exc:
                errors.append(f'{role}: {exc}')
        return errors


class Experiment:
    """One backend for the entire console: query changes never reconstruct the model/map."""
    def __init__(self, backend, output, timeout=420., clock=time.monotonic):
        self.backend, self.output, self.timeout, self.clock = backend, output, timeout, clock
        self.active = None
        self.count = 0
        self.fault = False
        self.last_query = None

    def start(self, query):
        if self.fault:
            raise RuntimeError('停车/运行故障后禁止重新解锁，请退出并检查日志')
        if self.active:
            raise ValueError('正在搜寻；请先 /stop 再输入新目标')
        query = valid_query(query)
        if query == self.last_query:
            raise ValueError('当前query已是该目标，现有接口不会创建新一轮；请输入另一目标，或退出重开会话')
        if self.count >= 100:
            raise ValueError('本会话已达100目标上限，请结束并归档')
        self.count += 1
        folder = self.output/f'goal_{self.count:03d}'
        folder.mkdir()
        self.active = dict(query=query, output=str(folder), started=self.clock(),
                           passed=False, state='preparing')
        self.last_query = query
        try:
            self.backend.begin(query, folder)
            self.active['started'] = self.clock()
            self.active['state'] = 'searching'
        except BaseException as exc:
            self.active['failure_reason'] = repr(exc)
            self.finish('user_stopped' if isinstance(exc, InterruptedError) else 'start_failed')
            raise

    def tick(self):
        self.backend.poll()
        if not self.active:
            return
        evidence = self.backend.arrival(self.active['query'])
        if evidence:
            self.active['arrival'] = evidence
            self.finish('automatic_arrival_pending_visual_review')
        elif self.clock()-self.active['started'] >= self.timeout:
            self.finish('timeout')

    def finish(self, reason):
        report = self.active
        try:
            if report:
                report['state'] = reason
                if reason != 'automatic_arrival_pending_visual_review':
                    try:
                        report['failure_image'] = self.backend.failure_image()
                    except Exception as exc:
                        report['failure_image_error'] = repr(exc)
            self.backend.stop()
        except BaseException as exc:
            self.fault = True
            if report:
                report['stop_error'] = repr(exc)
                report['state'] = 'unsafe_stop_or_runtime_failure'
            raise
        finally:
            if report:
                report['elapsed_s'] = self.clock()-report['started']
                write_json(Path(report['output'])/'result.json', report)
                print(f"\n{report['state']}: {report['query']}\n证据：{report['output']}", flush=True)
            self.active = None


class PhysicalBackend:
    def __init__(self, config, output, owned, interrupted):
        self.config, self.output, self.owned = config, output, owned
        self.node = self.bng = self.bag = None
        self.interrupted = interrupted
        self.stopping = False
        self.ros_started = False
        self.goal_dir = output

    def initialize(self):
        import yaml
        self.initial_query = valid_query(yaml.safe_load(
            (ROOT/'official_adapter/config/beamng_wildos.yaml').read_text())[
                'object_search_config']['text_queries'][0])
        from beamngpy import BeamNGpy
        from beamngpy.connection.prefixed_length_socket import PrefixedLengthSocket
        from wildos_nav2_adapter.fail_closed_transport import install_fail_closed_reads
        import rclpy
        from rclpy.signals import SignalHandlerOptions
        from town_trial_harness import Harness
        owner = self

        class InterruptibleHarness(Harness):
            def event(self, kind, **details):
                record_console_event(self, kind, details)

            def spin(self, seconds, done=lambda: False):
                def checked():
                    if owner.interrupted.is_set() and not owner.stopping:
                        raise InterruptedError('用户请求停止')
                    return done()
                return super().spin(seconds, checked)

        install_fail_closed_reads(PrefixedLengthSocket)
        self.bng = BeamNGpy('127.0.0.1', 25252, socket_timeout=5).open(launch=False)
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        self.ros_started = True
        vision_pid = int((self.output/'vision.pid').read_text())
        self.node = InterruptibleHarness(self.output, vision_pid, carrier_model='ackermann', record_video=False)
        self.node.events = deque(maxlen=2048)
        self.bng.control.resume()
        self.stop()

    def begin(self, query, folder):
        import shutil
        if shutil.disk_usage(self.output).free < 2*1024**3:
            raise RuntimeError('剩余磁盘不足2GiB，拒绝新目标')
        self.owned.healthy()
        self.goal_dir = folder
        self.bng.control.resume()
        self.stop(pause=False)
        for name, filename in [('log', 'events.jsonl'), ('native_log', 'native_samples.jsonl'),
                               ('sensor_timing_log', 'sensor_timing.jsonl')]:
            getattr(self.node, name).close()
            setattr(self.node, name, (folder/filename).open('x', encoding='utf8'))
        self.node.out = folder
        self.node.events.clear()
        self.node.phase = 'interactive_search'
        if not self.node.spin(8, lambda: bool(self.node.frames) and 'odom' in self.node.state):
            raise RuntimeError('缺少相机或里程计')
        topics = ['/spot1/object_search/query', '/spot1/object_mask', '/spot1/object_localization',
                  '/spot1/object_confirmation', '/spot1/remembered_object_candidates',
                  '/spot1/visible_target_approach', '/wildos/memory_selection', '/wildos/plan_status',
                  '/wildos/planned_path', '/wildos/native_control_status', '/wildos/control_status',
                  '/wildos/cmd_vel', '/wildos/cmd_vel_stamped', '/wildos/native_control', '/wildos/native_control_command',
                  '/wildos/execution_feedback',
                  '/tf', '/spot1/odom', '/spot1/measurement_timing', '/spot1/local_terrain',
                  '/spot1/local_map', '/global_costmap/costmap', '/local_costmap/costmap',
                  '/received_global_plan', '/spot1/graphnav_planner/path',
                  '/spot1/graphnav_planner/rejected_route_endpoint',
                  '/spot1/graphnav_planner/observed_route_endpoint',
                  '/spot1/graphnav_planner/retired_exploration',
                  '/spot1/graphnav_planner/vehicle_choice',
                  '/spot1/graphnav_planner/selection_ack',
                  '/spot1/traversal_feedback', '/spot1/graphnav_planner/route_status']
        with (folder/'bag.log').open('xb') as log:
            self.bag = subprocess.Popen(['ros2', 'bag', 'record', '-o', str(folder/'interfaces'), *topics],
                stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        if not self.node.spin(8, lambda: any(i.node_name.startswith('rosbag2_recorder') for i in
                self.node.get_subscriptions_info_by_topic('/spot1/object_search/query'))):
            raise RuntimeError('接口日志记录器未就绪')
        self.node.last_status = None
        self.node.change_query(query)
        if not self.node.spin(8, lambda: bool(self.node.last_status)
                and self.node.last_status.get('query') == query
                and self.node.last_status.get('schema_version') == 2
                and self.node.last_status.get('stamp_ns', 0) >= self.node.query_started_ns):
            raise RuntimeError('Bridge未确认当前目标及真实车体profile')
        self.node.watch()
        if self.interrupted.is_set():
            raise InterruptedError('用户请求停止')
        self.node.set_bool('/wildos_cmd_vel_to_control', 'enable', True)
        if self.interrupted.is_set():
            raise InterruptedError('用户请求停止')
        self.node.set_bool('/wildos_graph_path_bridge', 'follow_path', True)

    def poll(self):
        self.node.spin(.05)
        self.owned.healthy()
        if self.bag and self.bag.poll() is not None:
            raise RuntimeError('接口日志记录器意外退出')
        # Paused idle telemetry can legitimately stop advancing. Query begin/watch
        # and every active tick enforce the existing freshness contract.
        if self.bag:
            self.node.watch()

    def arrival(self, query):
        evidence = self.node.arrival_evidence(query, minimum_mask_count=3)
        if evidence:
            return dict(evidence=evidence, images=self.node.save_target_images(query))
        return None

    def failure_image(self):
        return self.node.save_failure_image() if self.node else {'available': False}

    def stop(self, pause=True):
        from check_continuous_start_execution import settle
        from wildos_nav2_adapter.physical_vehicle import read_physics_snapshot, physical_snapshot_errors
        errors = []
        self.stopping = True
        try:
            if self.bng:
                self.bng.control.resume()
            if self.node:
                try:
                    self.node.stop()
                except Exception as exc:
                    errors.append('control gates: '+repr(exc))
            if self.bng:
                try:
                    after, samples = settle(lambda: read_physics_snapshot(self.bng),
                                            self.node.spin if self.node else time.sleep)
                    write_json(self.goal_dir/'stop.json', dict(physics=after, samples=samples))
                    errors.extend(physical_snapshot_errors(after))
                except Exception as exc:
                    errors.append('sustained stop: '+repr(exc))
                finally:
                    if pause:
                        self.bng.control.pause()
            if self.bag:
                try:
                    if self.bag.poll() is None:
                        self.bag.send_signal(signal.SIGINT)
                    self.bag.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    self.bag.terminate()
                    self.bag.wait(timeout=5)
                    errors.append('recorder forced termination; bag may be incomplete')
                finally:
                    self.bag = None
            if errors:
                raise RuntimeError('; '.join(errors))
        finally:
            self.stopping = False

    def close(self):
        if self.node:
            for handle in (self.node.log, self.node.native_log, self.node.sensor_timing_log):
                handle.close()
            self.node.destroy_node()
        if self.ros_started:
            import rclpy
            rclpy.shutdown()
        if self.bng:
            self.bng.disconnect()


def wait_ready(owned, predicate, interrupted, timeout):
    deadline = time.monotonic()+timeout
    while time.monotonic() < deadline:
        owned.healthy()
        if interrupted.is_set():
            raise InterruptedError('启动被取消')
        if predicate():
            return
        time.sleep(.2)
    raise RuntimeError('启动就绪等待超时')


def prepare(config, output, owned, interrupted):
    """Only for a new empty server session; never replace an existing map/owner."""
    from server_test_preflight import process_rows
    conflicting = {'BeamNG.tech.x64', 'attach_current_vehicle_sensors.py', 'native_vehicle_control',
                   'cmd_vel_to_control', 'kinematic_vehicle_control', 'graph_path_bridge',
                   'planner_server', 'controller_server', 'visual_navigation.wildos.nav',
                   'obj_mask_triangulation.py'}
    rows = process_rows()
    leftovers = [r for r in rows if any(t in conflicting or Path(t).name in conflicting for t in r['tokens'])]
    if leftovers:
        write_json(output/'startup_conflicts.json', leftovers)
        raise RuntimeError('服务器存在旧仿真/导航拥有者；拒绝叠加启动，见startup_conflicts.json')
    home = Path(config['beamng_home'])
    engine = owned.spawn('engine', [str(home/'BinLinux/BeamNG.tech.x64'), '-headless', '-nosfx',
        '-userpath', config['beamng_user'], '-tcom', '-tport', '25252',
        '-tcom-listen-ip', '127.0.0.1', '-nosteam'], cwd=home)
    import socket
    def listening():
        try:
            with socket.create_connection(('127.0.0.1', 25252), timeout=.3):
                return True
        except OSError:
            return False
    wait_ready(owned, listening, interrupted, 90)
    scene = owned.spawn('scene', [sys.executable, str(ROOT/'tools/wildos_console_scene.py'),
        '--home', str(home), '--user', config['beamng_user'], '--output', str(output/'scene.json')])
    deadline = time.monotonic()+120
    while scene.poll() is None:
        if interrupted.is_set() or time.monotonic() > deadline:
            raise RuntimeError('场景准备取消或超时')
        time.sleep(.2)
    if scene.returncode or not json.loads((output/'scene.json').read_text())['passed']:
        raise RuntimeError('Town场景准备失败；查看scene.log')
    # Completed preparation is not a persistent owner.
    owned.children = [entry for entry in owned.children if entry[0] != 'scene']
    owned.spawn('sensor', [sys.executable, str(ROOT/'tools/attach_current_vehicle_sensors.py'),
        '--rig', str(ROOT/'config/sbr_physical_sensor_rig.json'), '--output', str(output/'sensor_attachment.json'),
        '--engine-pid', str(engine.pid), '--resume'])
    def sensor_ready():
        path = output/'sensor_attachment.json'
        try:
            return path.exists() and json.loads(path.read_text()).get('ready')
        except json.JSONDecodeError:
            return False
    wait_ready(owned, sensor_ready, interrupted, 40)
    port = json.loads((output/'sensor_attachment.json').read_text())['vehicle_port']
    owned.spawn('stack', ['ros2', 'launch', 'wildos_nav2_adapter', 'physical_vehicle_control.launch.py',
                         'vehicle_port:='+str(port)])
    from ament_index_python.packages import get_package_share_directory
    share = Path(get_package_share_directory('beamng_wildos_adapter'))/'config'
    vision_env = dict(os.environ, CUDA_VISIBLE_DEVICES=config.get('cuda_device', '0'))
    vision = config['vision_python']
    import importlib.util
    spec = importlib.util.find_spec('visual_navigation')
    if not spec or not spec.submodule_search_locations:
        raise RuntimeError('visual_navigation未安装/不在PYTHONPATH')
    tri = Path(next(iter(spec.submodule_search_locations)))/'explorfm_triangulation/obj_mask_triangulation.py'
    owned.spawn('triangulation', [vision, str(tri), '--config', str(share/'beamng_triangulation.yaml')], vision_env)
    owned.spawn('vision', [vision, '-m', 'visual_navigation.wildos.nav', '--config',
        str(share/'beamng_wildos.yaml'), '--do_object_search', 'true', '--memory_in_process_only'], vision_env)
    if config.get('rviz', False):
        owned.spawn('rviz', ['ros2', 'launch', 'wildos_nav2_adapter', 'beamng_rviz.launch.py'])
    # Model subscription is created after construction; wait before strict preflight.
    import rclpy
    rclpy.init()
    node = rclpy.create_node('wildos_console_startup_probe')
    try:
        def model_ready():
            rclpy.spin_once(node, timeout_sec=.05)
            return any(i.node_name == 'wildos' for i in
                       node.get_subscriptions_info_by_topic('/spot1/object_search/query'))
        wait_ready(owned, model_ready, interrupted, 180)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    # Startup settling time for mapper/TF discovery, without opening either control gate.
    settle_until = time.monotonic()+3
    wait_ready(owned, lambda: time.monotonic() >= settle_until, interrupted, 5)
    check = owned.spawn('preflight', [sys.executable, str(ROOT/'tools/server_test_preflight.py'),
        '--session-dir', str(output), '--arm', str(ROOT), '--carrier-model', 'ackermann',
        '--require-local-dds', '--output', str(output/'server_preflight.json')])
    deadline = time.monotonic()+150
    while check.poll() is None:
        if interrupted.is_set() or time.monotonic() > deadline:
            raise RuntimeError('启动检查取消或超时')
        time.sleep(.2)
    if check.returncode:
        raise RuntimeError('启动预检失败，未解锁控制；查看server_preflight.json / preflight.log')
    owned.children = [entry for entry in owned.children if entry[0] != 'preflight']


def bootstrap(config_path, config):
    """Source setup scripts with positional arguments, never interpolate shell code."""
    setups = config['setups']
    for path in [*setups, config['python'], config['vision_python']]:
        if not Path(path).is_file():
            raise ValueError('配置路径不存在: '+path)
    env = dict(os.environ, ROS_DOMAIN_ID=str(config.get('domain', 88)),
               DISPLAY=config.get('display', ':138'))
    env['PYTHONPATH'] = os.pathsep.join([str(ROOT/'tools'), *config.get('python_paths', []),
                                      env.get('PYTHONPATH', '')])
    code = 'set -e; n="$1"; shift; for ((i=0;i<n;i++)); do source "$1"; shift; done; exec "$@"'
    command = [config['python'], '-m', 'wildos_nav2_adapter.dds_profile', '--profile',
        str(ROOT/'nav2_adapter/config/fastdds_local_udp.xml'), '--', config['python'],
        str(Path(__file__).resolve()), '--runtime', '--config', str(config_path)]
    os.execvpe('bash', ['bash', '-c', code, 'wildos-console', str(len(setups)), *setups, *command], env)


def verify_sources(config):
    """Refuse mixed old/new installed Python, launch and parameter files before engine start."""
    import importlib.util
    from ament_index_python.packages import get_package_share_directory, get_package_prefix
    packages = {
        'wildos_nav2_adapter': ROOT/'nav2_adapter/wildos_nav2_adapter',
        'beamng_wildos_adapter': ROOT/'official_adapter/beamng_wildos_adapter',
        'visual_navigation': ROOT/'wildos_port/repro_check/src/nebula2-wildos/visual_navigation/visual_navigation',
    }
    hashes = {}
    for name, source in packages.items():
        spec = importlib.util.find_spec(name)
        if not spec or not spec.submodule_search_locations:
            raise RuntimeError('未安装Python包: '+name)
        installed = Path(next(iter(spec.submodule_search_locations)))
        for path in source.rglob('*.py'):
            if '__pycache__' in path.parts:
                continue
            relative = path.relative_to(source)
            actual = installed/relative
            if not actual.is_file() or actual.read_bytes() != path.read_bytes():
                raise RuntimeError(f'安装版本与本次源码不一致: {name}/{relative}')
            hashes[str(path.relative_to(ROOT))] = hashlib.sha256(path.read_bytes()).hexdigest()
    for name, source in [('wildos_nav2_adapter', ROOT/'nav2_adapter'),
                         ('beamng_wildos_adapter', ROOT/'official_adapter')]:
        share = Path(get_package_share_directory(name))
        for subdir in ('config', 'launch'):
            for path in (source/subdir).glob('*'):
                if path.is_file():
                    actual = share/subdir/path.name
                    if not actual.is_file() or actual.read_bytes() != path.read_bytes():
                        raise RuntimeError('安装配置/launch与源码不一致: '+str(path))
    for package, name in [('wildos_ackermann_smac', 'libwildos_ackermann_smac.so'),
                          ('wildos_ackermann_rpp', 'libwildos_ackermann_rpp.so'),
                          ('graphnav_planner', 'libgraphnav_planner_component.so')]:
        library = Path(get_package_prefix(package))/'lib'/name
        digest = hashlib.sha256(library.read_bytes()).hexdigest()
        if config.get('library_sha256', {}).get(name) != digest:
            raise RuntimeError('C++插件不符初始构建身份；重新准备隔离安装: '+str(library))
        hashes[str(library)] = digest
    return hashes


def signal_console(folder):
    identity = json.loads((folder/'console_identity.json').read_text())
    pid = identity['pid']
    if proc_identity(pid) != identity:
        raise RuntimeError('会话已退出或PID已复用；未发送信号')
    if str(Path(__file__).resolve()).encode() not in (Path('/proc')/str(pid)/'cmdline').read_bytes().split(b'\0'):
        raise RuntimeError('PID不是本交互入口；未发送信号')
    os.kill(pid, signal.SIGTERM)
    print('已请求停车并退出；完成结果将写入 '+str(folder/'session_result.json'))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument('--config', type=Path)
    mode.add_argument('--stop', type=Path, metavar='SESSION_DIR')
    parser.add_argument('--runtime', action='store_true', help=argparse.SUPPRESS)
    parser.add_argument('--check-config', action='store_true')
    args = parser.parse_args()
    if args.stop:
        signal_console(args.stop)
        return 0
    config = json.loads(args.config.read_text(encoding='utf-8-sig'))
    for key in ('setups', 'python', 'vision_python', 'beamng_home', 'beamng_user', 'output_root'):
        if not config.get(key):
            parser.error('缺少配置项: '+key)
    timeout = float(config.get('goal_timeout_s', 420))
    if not math.isfinite(timeout) or not 1 <= timeout <= 420:
        parser.error('goal_timeout_s须在1–420秒')
    if args.check_config:
        print(json.dumps(config, ensure_ascii=False, indent=2))
        return 0
    if sys.platform != 'linux':
        parser.error('此入口运行于Linux服务器；本机通过SSH使用，参见docs/SELF_SERVICE_CONSOLE.md')
    if not args.runtime:
        bootstrap(args.config.resolve(), config)
    output = Path(config['output_root'])/time.strftime('session_%Y%m%d_%H%M%S')
    output.mkdir(parents=True, exist_ok=False)
    # Exclusive machine-level flock: also prevents two consoles starting before discovery.
    import fcntl
    lock = open('/tmp/wildos_physical_console.lock', 'a')
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        raise RuntimeError('另一个WildOS交互入口仍在运行')
    stopped, quit_requested = threading.Event(), threading.Event()
    commands = queue.Queue(maxsize=16)
    def request_quit(*_):
        stopped.set(); quit_requested.set()
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, request_quit)
    def read_input():
        try:
            for line in sys.stdin:
                text = line.strip()
                if text in ('/quit', '/exit'):
                    request_quit(); return
                if text == '/stop':
                    stopped.set()
                    continue
                try:
                    commands.put_nowait(text)
                except queue.Full:
                    print('输入队列已满，请稍后再试', flush=True)
        finally:
            request_quit()
    threading.Thread(target=read_input, daemon=True).start()
    owned = OwnedProcesses(output, ROOT)
    backend = PhysicalBackend(config, output, owned, stopped)
    experiment = Experiment(backend, output, timeout)
    errors = []
    try:
        print('正在启动；加载一次Town地图，保持真实车体。日志：'+str(output), flush=True)
        write_json(output/'console_identity.json', proc_identity(os.getpid()))
        write_json(output/'verified_sources.json', verify_sources(config))
        prepare(config, output, owned, stopped)
        backend.initialize()
        experiment.last_query = backend.initial_query
        print(HELP, flush=True)
        print('模型初始query为 '+backend.initial_query+'；请先输入一个不同的目标。', flush=True)
        began = time.monotonic()
        while not quit_requested.is_set():
            if time.monotonic()-began > 8*3600:
                raise RuntimeError('会话达到8小时归档上限')
            if stopped.is_set():
                experiment.finish('user_stopped')
                # Discard targets typed before the stop, so cancellation cannot rearm.
                while not commands.empty():
                    commands.get_nowait()
                stopped.clear()
                print('已停车并暂停，可输入下一个目标。', flush=True)
            try:
                text = commands.get_nowait()
            except queue.Empty:
                text = None
            if text is not None:
                if text == '/status':
                    print(json.dumps(experiment.active or {'state': 'idle', 'goals': experiment.count},
                                     ensure_ascii=False), flush=True)
                elif text in ('/help', ''):
                    print(HELP, flush=True)
                elif text.startswith('/'):
                    print('未知命令。'+HELP, flush=True)
                else:
                    try:
                        experiment.start(text)
                    except ValueError as exc:
                        print(str(exc), flush=True)
                    except InterruptedError:
                        continue
            try:
                experiment.tick()
            except InterruptedError:
                # stop/quit during ACK/discovery is handled by the same stop path.
                continue
    except InterruptedError as exc:
        if not quit_requested.is_set() and not stopped.is_set():
            errors.append(repr(exc))
    except BaseException as exc:
        errors.append(repr(exc))
        if experiment.active:
            experiment.active['failure_reason'] = repr(exc)
        print('运行异常：'+repr(exc), flush=True)
    finally:
        try:
            experiment.finish('session_exit')
        except BaseException as exc:
            errors.append('stop: '+repr(exc))
        # Gates/SDK stop attempted before terminating owned processes, including engine.
        try:
            backend.close()
        except Exception as exc:
            errors.append('close: '+repr(exc))
        errors.extend(owned.close())
        write_json(output/'session_result.json', dict(errors=errors, goals=experiment.count,
                   field_acceptance=False, video_recording_enabled=False))
        print('会话结束。日志：'+str(output)+ ('；退出有错误，请检查session_result.json' if errors else ''), flush=True)
        lock.close()
    return 1 if errors else 0


if __name__ == '__main__':
    raise SystemExit(main())
