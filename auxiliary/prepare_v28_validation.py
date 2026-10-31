from pathlib import Path
base=Path(r'C:\Users\Yu\Desktop\Beamng')
s=(base/'validate_v27_remote.py').read_text().replace('v27_fresh_bearing','v28_intent_handoff')
s=s.replace("json.loads((root/'receipt.json').read_text())['changed']",
            "json.loads((root/'full/frozen_source_manifest.json').read_text())['members']")
(base/'validate_v28_remote.py').write_text(s)
s=(base/'run_v27_field.py').read_text().replace('v27_fresh_bearing','v28_intent_handoff')
(base/'run_v28_field.py').write_text(s)
