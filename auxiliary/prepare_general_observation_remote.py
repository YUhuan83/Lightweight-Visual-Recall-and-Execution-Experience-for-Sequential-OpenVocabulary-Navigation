from pathlib import Path
source=Path(__file__).with_name('prepare_observation_heading_remote.py').read_text()
source=source.replace("source=base/'v39_observation_session';target=base/'v40_observation_heading'","source=base/'v40_observation_heading';target=base/'v41_general_observation'")
source=source.replace("base/'observation_heading_delta.zip'","base/'general_observation_delta.zip'")
source=source.replace("('test_recovery_heading_ros', ","('test_general_observation_ros', ")
exec(compile(source,__file__,'exec'))
