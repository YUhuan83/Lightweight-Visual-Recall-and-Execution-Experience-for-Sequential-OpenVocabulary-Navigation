from pathlib import Path
import json,hashlib,subprocess
r=Path('/home/ly/planning_phase_physical_20261018');t=sorted(r.glob('field_yaw_left90_search_*'))[-1]
assert (t/'result.json').is_file()
v=t/'rviz_live.mp4';assert v.is_file()
probe=subprocess.run(['ffprobe','-v','error','-show_entries','format=duration:stream=codec_name,nb_frames,width,height,r_frame_rate','-of','json',str(v)],capture_output=True,text=True,check=True)
decode=subprocess.run(['ffmpeg','-nostdin','-v','error','-i',str(v),'-f','null','-'],capture_output=True,text=True)
h=hashlib.sha256()
with v.open('rb') as f:
 for b in iter(lambda:f.read(4194304),b''):h.update(b)
o=dict(path=str(v),bytes=v.stat().st_size,sha256=h.hexdigest(),probe=json.loads(probe.stdout),decode_returncode=decode.returncode,decode_errors=decode.stderr)
(r/'video_verification.json').write_text(json.dumps(o,indent=2));assert decode.returncode==0;print(json.dumps(o,indent=2))
