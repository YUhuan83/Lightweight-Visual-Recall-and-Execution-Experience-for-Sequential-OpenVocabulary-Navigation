"""User-requested RViz recording around the unchanged full field driver."""
from pathlib import Path
import hashlib
import json
import shutil

ROOT=Path(__file__).resolve().parent;field=ROOT/'full'
source=(field/'driver_full.py').read_text()
source=source.replace('import signal\n','import signal\nimport re\nfrom recording_window import select_rviz_window\n')
shutil.copy2(ROOT/'recording_window.py',field/'recording_window.py')
source=source.replace('child=None;session=None;halt=False','child=None;session=None;halt=False\nvideo_child=None;video_log=None')
source=source.replace('complete_navigation_passed=False,video=False)',
    'complete_navigation_passed=False,video=False,video_requested=True,video_recording_mode="User-requested external RViz X11 recording; console internal recorder remains disabled")')
helper='''
def start_video():
    global video_child,video_log
    tree=subprocess.check_output(['xwininfo','-display',':148','-root','-tree'],text=True,timeout=10)
    (OUT/'xwindow_tree.txt').write_text(tree)
    windows=[select_rviz_window(tree)]
    assert len(windows)==1,windows
    info=subprocess.check_output(['xwininfo','-display',':148','-id',windows[0]],text=True,timeout=10)
    (OUT/'rviz_window.txt').write_text(info)
    width=int(re.search(r'Width:\\s+(\\d+)',info).group(1));height=int(re.search(r'Height:\\s+(\\d+)',info).group(1))
    assert width>=1000 and height>=600,(width,height)
    capture=['-f','x11grab','-draw_mouse','0','-framerate','10','-video_size',f'{width}x{height}','-window_id',str(int(windows[0],16)),'-i',':148.0']
    report['video_capture']=dict(window_id=windows[0],width=width,height=height,fps=10,
                                 started_wall_ns=time.time_ns(),started_monotonic=time.monotonic(),
                                 source='Actual RViz window, no composed mock map',path=str(OUT/'rviz_live.mp4'))
    video_log=(OUT/'video_encoder.log').open('w')
    video_child=subprocess.Popen(['ffmpeg','-hide_banner','-loglevel','warning','-n',*capture,
        '-t','540','-an','-vf','pad=ceil(iw/2)*2:ceil(ih/2)*2','-c:v','libx264','-threads','2',
        '-preset','ultrafast','-crf','23','-pix_fmt','yuv420p','-movflags','+faststart',str(OUT/'rviz_live.mp4')],
        stdin=subprocess.PIPE,stdout=video_log,stderr=subprocess.STDOUT,text=True,start_new_session=True)
    (OUT/'video.pid').write_text(str(video_child.pid))
    time.sleep(1.)
    assert video_child.poll() is None,'RViz recorder exited; inspect encoder log'
    report['video']=True
    subprocess.run(['ffmpeg','-hide_banner','-loglevel','error','-n',*capture,'-frames:v','1',str(OUT/'rviz_preview.png')],
                   check=True,timeout=10,capture_output=True)

def stop_video():
    if video_child is None:return
    if video_child.poll() is None:
        try:
            video_child.stdin.write('q\\n');video_child.stdin.flush();video_child.wait(timeout=15)
        except (BrokenPipeError,subprocess.TimeoutExpired):
            video_child.send_signal(signal.SIGINT)
            try:video_child.wait(timeout=10)
            except subprocess.TimeoutExpired:raise RuntimeError('Owned video recorder remains live')
    if video_log:video_log.close()
    report['video_encoder_exit']=video_child.returncode
    video=OUT/'rviz_live.mp4'
    if video.exists():
        data=json.loads(subprocess.check_output(['ffprobe','-v','error','-show_format','-show_streams','-of','json',str(video)],text=True,timeout=15))
        (OUT/'video_probe.json').write_text(json.dumps(data,indent=2)+'\\n')
        stream=next(s for s in data['streams'] if s['codec_type']=='video')
        assert float(data['format']['duration'])>1. and int(stream['nb_frames'])>10
        report['video_capture'].update(duration_s=float(data['format']['duration']),frames=int(stream['nb_frames']),
                                       sha256=hashlib.sha256(video.read_bytes()).hexdigest(),verified_container=True)
'''
source=source.replace('try:\n    result=subprocess.run(',helper+'\ntry:\n    result=subprocess.run(',1)
needle="    send('green residential house')";assert source.count(needle)==1
source=source.replace(needle,"    start_video()\n"+needle)
needle='finally:\n    halt=False';assert source.count(needle)==1
source=source.replace(needle,"finally:\n    try:stop_video()\n    except Exception as error:report['errors'].append('Video: '+repr(error))\n    halt=False")
compile(source,str(field/'driver_recorded.py'),'exec')
(field/'driver_recorded.py').write_text(source)
shell=(ROOT/'run_full.sh').read_text().replace('full/driver_full.py','full/driver_recorded.py')
(ROOT/'run_recorded.sh').write_text(shell)
(ROOT/'recording_harness_identity.json').write_text(json.dumps(dict(
    requested_by_user=True,production_navigation_unchanged=True,
    files={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in (field/'driver_recorded.py',ROOT/'run_recorded.sh')}
),indent=2)+'\n')
print('Prepared recorded driver; navigation source/config unchanged')
