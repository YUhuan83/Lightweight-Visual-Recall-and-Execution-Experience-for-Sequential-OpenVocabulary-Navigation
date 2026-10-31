#!/usr/bin/env bash
set -eo pipefail
REMOTE=/home/ly/source_control_repair_20261017
source /opt/ros/humble/setup.bash
source /home/ly/beamng_ros2_0394_ws/install/setup.bash
source /home/ly/town_memory_validation_20261014/curvature_contract_candidate1/install/setup.bash
source /home/ly/wildos-console/install/setup.bash
source /home/ly/exploration_owner_repair_20261016/final_install/setup.bash
source /home/ly/frontier_cost_repair_20261016/install/setup.bash
source /home/ly/frontier_cost_repair_20261016/release_install/setup.bash
source /home/ly/exploration_scope_repair_20261016/install/setup.bash
source /home/ly/frontier_resolution_repair_20261016/install/setup.bash
source /home/ly/frontier_motion_handoff_20261016/install/setup.bash
source "$REMOTE/install/setup.bash"
python3 "$REMOTE/field/verify_before.py" > "$REMOTE/post_run_verification.json"
python3 "$REMOTE/field/analyze_closed.py" > "$REMOTE/closed_analysis.log"
python3 "$REMOTE/analyze_source_closed.py" > "$REMOTE/source_analysis.log"
python3 "$REMOTE/review_field.py" > "$REMOTE/closed_summary.log"
python3 "$REMOTE/inspect_closed_progress.py" > "$REMOTE/target_analysis.log"
python3 - "$REMOTE" <<'PY'
from pathlib import Path
import hashlib,json,subprocess,sys
r=Path(sys.argv[1]);t=sorted(r.glob('field_scope_baseline_search_*'))[-1]
assert (t/'result.json').is_file()
video=t/'rviz_live.mp4';assert video.is_file()
probe=subprocess.run(['ffprobe','-v','error','-show_entries','format=duration:stream=codec_name,nb_frames,width,height,r_frame_rate','-of','json',str(video)],capture_output=True,text=True,check=True)
decode=subprocess.run(['ffmpeg','-nostdin','-v','error','-i',str(video),'-f','null','-'],capture_output=True,text=True)
report=dict(path=str(video),bytes=video.stat().st_size,sha256=hashlib.sha256(video.read_bytes()).hexdigest(),probe=json.loads(probe.stdout),decode_returncode=decode.returncode,decode_errors=decode.stderr)
(r/'video_verification.json').write_text(json.dumps(report,indent=2)+'\n')
assert decode.returncode==0,report
print(json.dumps(report,indent=2))
PY
python3 "$REMOTE/seal_field.py" > "$REMOTE/archive.log"
