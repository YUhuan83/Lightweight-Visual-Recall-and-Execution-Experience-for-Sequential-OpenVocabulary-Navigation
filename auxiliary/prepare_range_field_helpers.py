from pathlib import Path
aux=Path(__file__).parent
for n in ('run_confirmed_progress_trial.py','read_vehicle_observation_live.py','audit_vehicle_observation_closed.py','audit_full_path_closed.py'):
 p=aux/n;s=p.read_text(encoding='utf8');s=s.replace("'v48_range_candidate')","'v48_range_candidate','v49_localization_range')");p.write_text(s,encoding='utf8')
p=aux/'audit_range_hint_closed.py';s=p.read_text(encoding='utf-8-sig').replace('import collections,json,math,sqlite3','import collections,json,math,sqlite3,sys').replace("root=base/'v48_range_candidate'","root=base/(sys.argv[1] if len(sys.argv)>1 else 'v48_range_candidate')");p.write_text(s,encoding='utf8')
s=(aux/'audit_range_candidate_closed.sh').read_text(encoding='utf-8-sig').replace('v48_range_candidate','v49_localization_range').replace('python3 audit_range_hint_closed.py','python3 audit_range_hint_closed.py v49_localization_range');(aux/'audit_localization_range_closed.sh').write_text(s,encoding='utf8',newline='\n')
