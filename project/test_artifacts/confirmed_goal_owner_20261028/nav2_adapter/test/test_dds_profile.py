import copy
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
from wildos_nav2_adapter.dds_profile import environment_errors,local_dds_environment,profile_errors,main

PROFILE=Path(__file__).resolve().parents[1]/'config/fastdds_local_udp.xml'

class LocalDdsTests(unittest.TestCase):
    def setUp(self):self.data=PROFILE.read_bytes()
    def test_frozen_profile_preserves_loopback_without_endpoint_qos_changes(self):
        self.assertEqual(profile_errors(self.data),[])
        source=Path(__file__).resolve().parents[2]/'test_artifacts/tracking_contract_repair_20261014/fastdds_loopback_udp.xml'
        if source.exists():self.assertEqual(self.data,source.read_bytes())
    def test_child_environment_overrides_localhost_and_profile_without_mutating_parent(self):
        inherited={'ROS_DOMAIN_ID':'88','ROS_LOCALHOST_ONLY':'1','CUSTOM':'kept',
                   'FASTRTPS_DEFAULT_PROFILES_FILE':'old.xml'};before=copy.deepcopy(inherited)
        env=local_dds_environment(PROFILE,inherited)
        self.assertEqual(inherited,before);self.assertEqual(env['CUSTOM'],'kept')
        self.assertEqual(env['ROS_DOMAIN_ID'],'88')
        self.assertEqual(environment_errors(env,self.data),[])
    def test_different_rmw_is_not_silently_replaced(self):
        with self.assertRaises(ValueError):local_dds_environment(PROFILE,{'RMW_IMPLEMENTATION':'rmw_cyclonedds_cpp'})
    def test_missing_and_conflicting_environment_is_rejected(self):
        for env in ({},{'ROS_LOCALHOST_ONLY':'1','RMW_IMPLEMENTATION':'rmw_fastrtps_cpp'},
                    {'ROS_LOCALHOST_ONLY':'0','RMW_IMPLEMENTATION':'rmw_cyclonedds_cpp'}):
            with self.subTest(env=env):self.assertTrue(environment_errors(env,self.data))
    def test_bad_xml_and_non_loopback_or_builtin_transport_is_rejected(self):
        for data in (b'bad',self.data.replace(b'127.0.0.1',b'0.0.0.0'),
                     self.data.replace(b'<type>UDPv4</type>',b'<type>SHM</type>'),
                     self.data.replace(b'<useBuiltinTransports>false',b'<useBuiltinTransports>true'),
                     self.data.replace(b'</profiles>',b'<data_writer/></profiles>'),
                     self.data.replace(b'</userTransports>',b'<transport_id>other</transport_id></userTransports>')):
            with self.subTest(data=data[:40]):self.assertTrue(profile_errors(data))
    def test_read_failure_and_semantically_equal_but_unfrozen_bytes_are_rejected(self):
        env=local_dds_environment(PROFILE,{})
        def missing(path):raise OSError('missing')
        self.assertTrue(environment_errors(env,self.data,profile_reader=missing))
        self.assertTrue(environment_errors(env,self.data,profile_reader=lambda p:self.data+b'\n'))
    def test_cli_exec_preserves_argv_and_applies_environment_before_child_ros_initialization(self):
        with patch('sys.argv',['local_dds_run','--profile',str(PROFILE),'--','ros2','launch','a file.launch.py']),\
             patch('os.environ',{'ROS_DOMAIN_ID':'88'}),patch('os.execvpe') as execute:
            main()
        command,argv,env=execute.call_args.args
        self.assertEqual(command,'ros2');self.assertEqual(argv,['ros2','launch','a file.launch.py'])
        self.assertEqual(environment_errors(env,self.data),[])
    def test_missing_command_does_not_exec(self):
        with patch('sys.argv',['local_dds_run','--profile',str(PROFILE)]),patch('os.execvpe') as execute:
            with self.assertRaises(SystemExit):main()
        execute.assert_not_called()

if __name__=='__main__':unittest.main()
