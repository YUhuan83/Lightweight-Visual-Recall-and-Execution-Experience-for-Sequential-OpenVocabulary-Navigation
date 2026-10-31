from pathlib import Path
source=Path(__file__).with_name('prepare_observation_heading_remote.py').read_text()
source=source.replace("source=base/'v39_observation_session';target=base/'v40_observation_heading'",
 "source=base/'v45_approach_contract';target=base/'v46_approach_feedback'")
source=source.replace('observation_heading_delta.zip','approach_feedback_delta.zip').replace('test_recovery_heading_ros','test_approach_feedback_ros')
exec(compile(source,__file__,'exec'))
run=target/'run_full.sh';text=run.read_text()
marker=f'source {target}/install/local_setup.bash';assert marker in text
run.write_text(text.replace(marker,f'source {base}/v42_retirement_boundary/install/local_setup.bash\n'+marker,1))
