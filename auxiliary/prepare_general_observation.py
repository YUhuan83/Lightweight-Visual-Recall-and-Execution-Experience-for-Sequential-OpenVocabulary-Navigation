from pathlib import Path
source=(Path(__file__).with_name('prepare_observation_heading.py')).read_text(encoding='utf8')
source=source.replace('observation_session_20261026/candidate','observation_heading_20261026/candidate')
source=source.replace("root=repo/'test_artifacts/observation_heading_20261026'","root=repo/'test_artifacts/general_observation_20261026'")
exec(compile(source,__file__,'exec'))
