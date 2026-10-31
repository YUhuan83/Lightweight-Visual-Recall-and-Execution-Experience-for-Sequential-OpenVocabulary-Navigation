from pathlib import Path
base=Path('/home/ly/execution_continuity_review_20261022')
source=(base/'prepare_rolling_remote.py').read_text()
source=source.replace("target=base/'v38_rolling_observation'","target=base/'v39_observation_session'")
source=source.replace("base/'rolling_delta.zip'","base/'observation_session_delta.zip'")
source=source.replace("('test_rolling_observation_ros', ","('test_observation_session_ros', 'test_rolling_observation_ros', ")
exec(compile(source,__file__,'exec'))
