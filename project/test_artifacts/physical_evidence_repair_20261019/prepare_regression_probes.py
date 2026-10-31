from pathlib import Path
ROOT=Path(__file__).resolve().parents[2];OUT=Path(__file__).resolve().parent
source=(ROOT/'test_artifacts/mapper_replay_20261019/replay_mapper.py').read_text(encoding='utf8')
source=source.replace("os.environ['ROS_DOMAIN_ID']=='191'","os.environ['ROS_DOMAIN_ID']=='199'").replace('ros_domain=191','ros_domain=199')
anchor="    prefix=Path(get_package_prefix('graphnav_mapper'))\n";assert source.count(anchor)==1
source=source.replace(anchor,anchor+"    assert prefix==ROOT/'candidate6/install/graphnav_mapper',str(prefix)\n")
(OUT/'replay_candidate.py').write_text(source,encoding='utf8',newline='\n')
source=(ROOT/'test_artifacts/recall_contract_repair_20261019/probe_decision_dds.py').read_text(encoding='utf8')
source=source.replace('ROOT=Path(__file__).resolve().parents[3]',"ROOT=Path(__file__).resolve().parent/'candidate6'")
source=source.replace("assert str(exe).startswith('/home/ly/goal_release_fence_20261019/install/')","assert exe.is_relative_to(ROOT/'install')")
anchor=" assert all(v=='86fecc865b6fd6a13a7f0b23f8336b8c4af5c370c95bd877232bc956e6aeba02' for v in report['runtime_identity']['libraries'].values())"
assert source.count(anchor)==1
source=source.replace(anchor," expected=json.loads((ROOT/'result.json').read_text())['libraries']['install/graphnav_planner/lib/libgraphnav_planner_component.so']\n assert all(v==expected and Path(p).is_relative_to(ROOT/'install') for p,v in report['runtime_identity']['libraries'].items())")
(OUT/'probe_recall_dds.py').write_text(source,encoding='utf8',newline='\n')
print('Prepared source-derived diagnostics with explicit runtime identities')
