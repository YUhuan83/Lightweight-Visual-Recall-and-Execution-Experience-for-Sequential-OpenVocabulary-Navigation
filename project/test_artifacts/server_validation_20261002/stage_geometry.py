from pathlib import Path
import zipfile
root=Path(__file__).resolve().parents[2]
files=[
 'nav2_adapter/wildos_nav2_adapter/vehicle_geometry.py',
 'nav2_adapter/wildos_nav2_adapter/graph_route.py',
 'nav2_adapter/wildos_nav2_adapter/cmd_vel_to_control.py',
 'nav2_adapter/wildos_nav2_adapter/visualization_bridge.py',
 'nav2_adapter/test/test_graph_route.py',
 'nav2_adapter/test/fixtures/town_analytic_join.json',
 'nav2_adapter/test/test_cmd_vel_to_control_ros.py',
 'nav2_adapter/setup.py','nav2_adapter/package.xml',
 'nav2_adapter/config/nav2_official_control.yaml',
 'nav2_adapter/config/beamng_navigation.rviz',
 'nav2_adapter/launch/nav2_official_control.launch.py',
 'nav2_adapter/launch/beamng_rviz.launch.py',
 'official_adapter/beamng_wildos_adapter/node.py',
 'official_adapter/beamng_wildos_adapter/geometry.py',
 'official_adapter/beamng_wildos_adapter/mapping.py',
 'official_adapter/test/test_official_adapter.py',
 'official_adapter/test/test_node_geometry_ros.py',
]
with zipfile.ZipFile(Path(__file__).with_name('geometry_stage.zip'),'w',zipfile.ZIP_DEFLATED) as z:
    for f in files:z.write(root/f,f)
print('Staged',len(files),'files')
