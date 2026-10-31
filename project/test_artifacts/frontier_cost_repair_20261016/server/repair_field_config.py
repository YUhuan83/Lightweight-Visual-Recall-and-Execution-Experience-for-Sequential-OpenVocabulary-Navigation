from pathlib import Path
import json
r=Path('/home/ly/frontier_cost_repair_20261016');field=r/'field'
path=field/'sensor_full_console.json'
(field/'config_startup_failure.json').write_bytes(path.read_bytes())
cfg=json.loads(Path('/home/ly/exploration_owner_field_20261016/sensor_full_console.json').read_text())
deployed=json.loads(Path('/home/ly/wildos-console/console.json').read_text())
cfg['setups']=deployed['setups'];cfg['library_sha256']=deployed['library_sha256']
assert cfg['rviz'] is True and cfg['domain']==88 and cfg['display']==':148'
assert cfg['output_root']=='/home/ly/wildos-console/audit_results' and cfg['goal_timeout_s']==420
path.write_text(json.dumps(cfg,indent=2))
startup=Path('/home/ly/wildos-console/results/session_20261016_144941')
report=dict(reason='Diagnostic configuration copied from non-RViz console default; preflight correctly kept gates closed',
    driver='/home/ly/wildos-console/field_frontier_cost_search_20261016_144940',
    session=str(startup),result=json.loads((startup/'session_result.json').read_text()),
    visual_preflight=json.loads((startup/'server_preflight.json').read_text()),
    goals=0,video_available=False,repair='Reused previous verified field configuration; new setup prefixes/library identities only')
(field/'startup_failure.json').write_text(json.dumps(report,indent=2))
print(json.dumps(dict(rviz=cfg['rviz'],output=cfg['output_root'],setups=cfg['setups']),indent=2))
