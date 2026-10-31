"""Build a sensor-enabled copy of lane_sample without rebuilding scene assets."""
import json
import xml.etree.ElementTree as ET

from build_world import node, rover, write_xml
from sensor_contract import CONFIG as C, ROOT, bridge_entries, road_height


def build():
    tree = ET.parse(ROOT / 'assets/worlds/lane_sample.sdf')
    world = tree.getroot().find('world')
    world.set('name', 'lane_sensor_sample')
    temporary = ET.Element('world')
    rover(temporary)
    model = temporary.find('model')
    model.find('pose').text = f'-18 0 {road_height() + .45:.12g} 0 0 0'
    base = model.find("link[@name='base_link']")
    for sensor in base.findall('sensor'):
        name = sensor.get('name')
        if name.startswith('camera_'):
            side = name.removeprefix('camera_')
            pose = [*C['camera_position'], 0, 0, C['cameras'][side]]
            topic, frame = f'/sim/camera/{side}', f'camera_{side}_optical_frame'
            sensor.find('camera/horizontal_fov').text = str(C['hfov'])
            for key in ('width', 'height'):
                sensor.find(f'camera/image/{key}').text = str(C[key])
        else:
            pose = [*C[f'{name}_position'], 0, 0, 0]
            topic, frame = f'/sim/{name}', f'{name}_link'
        p = sensor.find('pose')
        if p is None:
            p = node(sensor, 'pose')
        p.text = ' '.join(map(str, pose))
        sensor.find('topic').text = topic
        node(sensor, 'ignition_frame_id', frame)
        node(sensor, 'always_on', 'true')
    drive = model.find('plugin')
    for key, value in dict(topic='/sim/cmd_vel', odom_topic='/sim/wheel_odom',
                           tf_topic='/sim/wheel_tf', frame_id=C['odom_frame'],
                           child_frame_id=C['base_frame'], max_linear_velocity=C['linear_limit'],
                           max_angular_velocity=C['angular_limit']).items():
        drive.find(key).text = str(value)
    world.append(model)
    for system in ('Sensors', 'Imu'):
        plugin = node(world, 'plugin', filename=f'ignition-gazebo-{system.lower()}-system',
                      name=f'ignition::gazebo::systems::{system}')
        if system == 'Sensors':
            node(plugin, 'render_engine', 'ogre2')
    output = ROOT / 'assets/worlds/lane_sensor_sample.sdf'
    write_xml(output, tree.getroot())
    # JSON is YAML 1.2: keeps offline validation entirely in the stdlib.
    (ROOT / 'config/lane_bridge.yaml').write_text(json.dumps(bridge_entries(), indent=2) + '\n', encoding='utf-8')
    print(output)


if __name__ == '__main__':
    build()
