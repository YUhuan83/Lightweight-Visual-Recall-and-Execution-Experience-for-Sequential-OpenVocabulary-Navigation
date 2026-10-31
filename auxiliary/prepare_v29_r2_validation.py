from pathlib import Path
base=Path(r'C:\Users\Yu\Desktop\Beamng')
for old,new in [('validate_v29_remote.py','validate_v29_r2_remote.py'),
                ('run_v29_field.py','run_v29_r2_field.py'),
                ('run_v29_validation_field.sh','run_v29_r2_validation_field.sh')]:
    s=(base/old).read_text().replace('v29_range_consistency','v29_range_consistency_r2')
    s=s.replace('/tmp/validate_v29_remote.py','/tmp/validate_v29_r2_remote.py').replace('/tmp/run_v29_field.py','/tmp/run_v29_r2_field.py')
    (base/new).write_text(s,newline='\n')
