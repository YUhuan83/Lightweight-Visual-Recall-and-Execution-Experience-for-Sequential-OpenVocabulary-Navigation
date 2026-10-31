from pathlib import Path
source=Path(__file__).with_name('freeze_rolling_trial.py').read_text()
source=source.replace("root=base/'v38_rolling_observation'","root=base/'v41_general_observation'")
exec(compile(source,__file__,'exec'))
