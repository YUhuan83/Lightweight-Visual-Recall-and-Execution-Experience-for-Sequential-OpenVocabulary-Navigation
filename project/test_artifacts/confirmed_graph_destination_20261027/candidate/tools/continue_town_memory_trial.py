"""Bounded continuation of a timed-out search; never relabel the original trial."""
import ast
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time


root = Path(os.environ.get('WILDOS_TEST_SESSION_DIR', '/home/ly/town_memory_validation_20261008'))
name = sys.argv[1]
prior = root / sys.argv[2]
out = root/name
assert not out.exists()
events = [json.loads(line) for line in (prior/'events.jsonl').read_text().splitlines()]
ack = next(e for e in events if e.get('kind') == 'vision_query_accepted')
source = Path(__file__).with_name('town_sequential_memory_trial.py')
tree = ast.parse(source.read_text())
prefix = []
for statement in tree.body:
    if (isinstance(statement, ast.Assign)
            and any(isinstance(t, ast.Name) and t.id == 'node' for t in statement.targets)):
        break
    prefix.append(statement)
ns = {'__name__': 'continuation_harness', '__file__': str(source)}
exec(compile(ast.Module(body=prefix, type_ignores=[]), str(source), 'exec'), ns)
node = ns['Harness'](ns['out'], ns['vision_pid'])
node.query_started_ns = int(ack['payload']['query_started_ns'])
node.phase = 'house_continuation'
report = dict(passed=False, continuation_of=prior.name, original_result_unchanged=True,
              map_reload_count=0, vehicle_teleports=0, vision_pid=ns['vision_pid'],
              code_commit=subprocess.check_output(['git','rev-parse','HEAD'], text=True).strip())
recorders = []
logs = []
try:
    for label, command in (
        ('rviz', ['ffmpeg','-hide_banner','-loglevel','warning','-f','x11grab',
                  '-filter_threads','1','-video_size','1280x1024','-framerate','10','-i',':137',
                  '-c:v','libx264','-threads','1','-preset','ultrafast','-pix_fmt','yuv420p',str(out/'rviz_live.mp4')]),
        ('bag', ['ros2','bag','record','-o',str(out/'bag'),
                 *json.loads((prior/'full_test.json').read_text())['topics_recorded']])):
        log=(out/(label+'.log')).open('w'); logs.append(log)
        recorders.append(subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True))
    node.event('continuation_started', prior=prior.name, query_started_ns=node.query_started_ns)
    ns['beamng'].control.resume()
    node.set_bool('/wildos_cmd_vel_to_control', 'enable', True)
    node.set_bool('/wildos_graph_path_bridge', 'follow_path', True)
    for query, budget in (('green residential house',150.), ('gas station',300.)):
        if query == 'gas station':
            if not report.get('house_arrival'):
                break
            node.phase = 'station_return'
            node.change_query(query)
        started=time.monotonic(); latest_progress=0.; accepted=None
        while time.monotonic()-started < budget:
            node.spin(.2)
            item=node.arrival_evidence(query,minimum_mask_count=2)
            if item and query == 'gas station':
                status=item['status']; selection=node.state.get('selection',{})
                gap=status.get('memory_region_distance_m'); tolerance=status.get('memory_region_tolerance_m')
                if not (selection.get('status')=='memory_candidate_selected'
                        and selection.get('query')==query
                        and selection.get('view_id')
                        and status.get('memory_view_id','').split('~approach',1)[0]
                            == selection['view_id'].split('~approach',1)[0]
                        and isinstance(gap,(int,float)) and math.isfinite(gap)
                        and isinstance(tolerance,(int,float)) and math.isfinite(tolerance)
                        and gap <= tolerance):
                    item=None
            if item:
                accepted=item
                node.save_target_images(query)
                break
            if time.monotonic()-latest_progress > 10.:
                latest_progress=time.monotonic()
                node.event('continuation_progress',position=node.state.get('native',{}).get('position'),
                           status=node.last_status,mask_count=node.masks.get(query,{}).get('count',0))
        report['house_arrival' if query=='green residential house' else 'station_arrival']=accepted
        report['house_continuation_elapsed_s' if query=='green residential house' else 'station_elapsed_s']=time.monotonic()-started
    report['passed']=bool(report.get('house_arrival') and report.get('station_arrival'))
    report['vision_process_retained']=Path('/proc',str(ns['vision_pid']),'status').exists()
    report['final_position']=node.state.get('native',{}).get('position')
except Exception as exc:
    report['error']=str(exc)
    raise
finally:
    node.stop()
    ns['beamng'].control.pause()
    report['final_status']=node.last_status
    (out/'continuation_test.json').write_text(json.dumps(report,indent=2))
    for proc in recorders:
        if proc.poll() is None:
            proc.send_signal(signal.SIGINT)
            try:proc.wait(timeout=20)
            except subprocess.TimeoutExpired:proc.terminate();proc.wait(timeout=5)
    node.video.release()
    for stream in (node.log,node.native_log,node.sensor_timing_log,*logs):stream.close()
    node.destroy_node()
    ns['beamng'].disconnect()
    ns['rclpy'].shutdown()
print(json.dumps({k:v for k,v in report.items() if not isinstance(v,dict)},indent=2))
