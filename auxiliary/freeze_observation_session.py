from pathlib import Path
source=(Path('/home/ly/execution_continuity_review_20261022')/'freeze_rolling_trial.py').read_text()
source=source.replace("root=base/'v38_rolling_observation'","root=base/'v39_observation_session'")
exec(compile(source,__file__,'exec'))
