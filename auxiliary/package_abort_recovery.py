from pathlib import Path
source=Path(__file__).with_name('package_retirement_boundary.py').read_text(encoding='utf8')
source=source.replace('retirement_boundary_20261026','abort_recovery_20261026').replace('retirement_boundary_delta.zip','abort_recovery_delta.zip')
start=source.index('assert set(changed)==');end=source.index('\nwith zipfile.ZipFile',start)
source=source[:start]+"assert set(changed)=={'nav2_adapter/wildos_nav2_adapter/graph_path_bridge.py','nav2_adapter/test/test_abort_recovery_ros.py','nav2_adapter/test/offline_ros_fakes.py'},changed"+source[end:]
source=source.replace("base='v40_observation_heading',target='v42_retirement_boundary'","base='v42_retirement_boundary',target='v43_abort_recovery'")
exec(compile(source,__file__,'exec'))
