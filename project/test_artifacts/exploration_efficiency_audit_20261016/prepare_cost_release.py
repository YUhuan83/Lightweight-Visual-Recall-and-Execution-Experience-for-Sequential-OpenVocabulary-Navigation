from pathlib import Path
import json
import shutil
root=Path(__file__).resolve().parents[2]
m=json.loads((root/'official_wildos_beamng_bundle.manifest.json').read_text(encoding='utf8'))
d=root/'deploy/20261016_frontier_cost_candidate';d.mkdir(exist_ok=False)
for name in ('official_wildos_beamng_bundle.zip','official_wildos_beamng_bundle.manifest.json','official_wildos_beamng_bundle.sha256'):
    shutil.copy2(root/name,d/name)
source=(Path(__file__).parent/'finalize_cost.sh').read_text(encoding='utf8')
source=source.replace('05d9d784aabd308a91323c0fe8cb10523e72c0fbc2a2436d0c8c6ace74f2f976',m['sha256'])
for old,new in (('final.zip','release.zip'),('final.manifest.json','release.manifest.json'),
                ('final_candidate','release_candidate'),('final_install','release_install'),
                ('final_build','release_build'),('final_identity.json','release_identity.json')):
    source=source.replace(old,new)
(Path(__file__).parent/'release_cost.sh').write_text(source,encoding='utf8',newline='\n')
print(m['sha256'])
