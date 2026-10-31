from pathlib import Path
root=Path(r'C:\Users\Yu\Desktop\Beamng')
s=(root/'collect_continuous_compact.py').read_text(encoding='utf8')
s=s.replace('v24_continuous_exploration_r2','v25_fast_shift').replace('continuous_exploration_r2_trials_20261024','fast_shift_trials_20261024').replace('continuous_r2_compact.tar.gz','fast_shift_compact.tar.gz')
s=s.replace("'continuous_r2_delta_receipt.json'","'fast_shift_delta_receipt.json','loaded_shift_profile.json','shift_wait_audit.json'")
(root/'collect_fast_shift_compact.py').write_text(s,encoding='utf8')
