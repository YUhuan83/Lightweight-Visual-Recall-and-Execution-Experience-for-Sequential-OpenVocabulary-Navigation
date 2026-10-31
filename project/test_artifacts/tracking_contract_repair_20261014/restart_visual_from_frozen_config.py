"""Restart only the failed model, with one authoritative installed config."""
import hashlib,json,os,subprocess,time
from pathlib import Path
BASE=Path('/home/ly/town_memory_validation_20261014/dds_transport_candidate')
CORE=BASE.parent/'tracking_contract_candidate'
session=BASE/'green_house_session1'
old_pid=int((session/'vision.pid').read_text())
assert not Path(f'/proc/{old_pid}').exists(),'Prior model still exists'
config=CORE/'install/share/beamng_wildos_adapter/config/beamng_wildos.yaml'
assert config.read_bytes()==(BASE/'bundle/official_adapter/config/beamng_wildos.yaml').read_bytes()
report={'scope':'Model process/configuration repair; target trial not started',
    'previous_model_pid':old_pid,'old_failure_log':'vision.log',
    'configuration':str(config),'config_sha256':hashlib.sha256(config.read_bytes()).hexdigest()}
with (session/'vision_config_retry1.log').open('x') as log:
    args=['/home/ly/vision-cu128/bin/python','-m','visual_navigation.wildos.nav',
        '--config',str(config),'--do_object_search','true','--memory_in_process_only']
    p=subprocess.Popen(args,cwd=BASE/'bundle',env=dict(os.environ,CUDA_VISIBLE_DEVICES='0'),
        stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
(session/'vision_failed.pid').write_text(str(old_pid)+chr(10))
(session/'vision.pid').write_text(str(p.pid)+chr(10))
report['new_model_pid']=p.pid;report['argv']=args
(session/'visual_configuration_repair.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report))
