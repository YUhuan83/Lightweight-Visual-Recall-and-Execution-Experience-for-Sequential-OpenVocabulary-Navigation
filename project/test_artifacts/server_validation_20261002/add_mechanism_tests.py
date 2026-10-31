from pathlib import Path
R=Path(r'C:\Users\Yu\Desktop\拟真')
p=R/'nav2_adapter/test/test_graph_path_bridge_routing.py'
s=p.read_text(); t=(R/'test_artifacts/server_validation_20261002/new_bridge_test_methods.txt').read_text()
p.write_text(s.replace('\n\nif __name__ == "__main__":',t+'\n\nif __name__ == "__main__":',1))
p=R/'tools/offline_callback_check.py';s=p.read_text().replace("str(ROOT / 'nav2_adapter/test')]","str(ROOT / 'nav2_adapter/test'), str(ROOT / 'official_adapter')]")
s=s.replace("'test_cmd_vel_to_control_ros'):","'test_cmd_vel_to_control_ros', 'test_goal_confirmation_ros'):")
p.write_text(s)

