from pathlib import Path
base=Path(r'C:\Users\Yu\Desktop\Beamng')
for old,new in [('validate_v29_r2_remote.py','validate_v30_remote.py'),('run_v29_r2_field.py','run_v30_field.py'),
                ('run_v29_r2_validation_field.sh','run_v30_validation_field.sh')]:
    s=(base/old).read_text().replace('v29_range_consistency_r2','v30_confirmation_history')
    s=s.replace('/tmp/validate_v29_r2_remote.py','/tmp/validate_v30_remote.py').replace('/tmp/run_v29_r2_field.py','/tmp/run_v30_field.py')
    # build.sh already performs the full real-ROS review.
    s=s.replace('/home/ly/beamng_ros2_0394_ws/.venv/bin/python real_ros_review.py >real_ros_review.log 2>&1\n','')
    (base/new).write_text(s,newline='\n')
