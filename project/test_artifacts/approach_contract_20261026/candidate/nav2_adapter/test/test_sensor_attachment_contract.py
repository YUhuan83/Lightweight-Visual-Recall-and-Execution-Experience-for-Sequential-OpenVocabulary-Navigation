import copy
import importlib.util
from pathlib import Path
import unittest
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from unittest.mock import Mock, patch

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location('sensor_attachment', ROOT/'tools/attach_current_vehicle_sensors.py')
attachment = importlib.util.module_from_spec(spec)
spec.loader.exec_module(attachment)


class SensorAttachmentContractTests(unittest.TestCase):
    def lifetime(self, events=(), tokens=None):
        poller = Mock()
        poller.poll.return_value = list(events)
        tokens = tokens or b'/opt/BeamNG.tech.x64\x00-tport\x0025252\x00'
        with patch.object(attachment.os, 'pidfd_open', return_value=42, create=True), \
             patch.object(attachment.select, 'poll', return_value=poller, create=True), \
             patch.object(attachment.select, 'POLLIN', 1, create=True), \
             patch.object(attachment.Path, 'read_bytes', return_value=tokens):
            life = attachment.EngineLifetime(123, 25252)
        return life, poller

    def test_pinned_lifetime_allows_live_owner_without_sdk_heartbeat(self):
        life, poller = self.lifetime()
        self.assertTrue(life.alive())
        poller.register.assert_called_once_with(42, 1)
        with patch.object(attachment.os, 'close') as close:
            life.close(); life.close()
        close.assert_called_once_with(42)
        self.assertFalse(life.alive())

    def test_old_pidfd_stays_dead_when_pid_or_port_is_reused(self):
        life, poller = self.lifetime()
        poller.poll.return_value = [(42, 1)]
        with patch.object(attachment.Path, 'read_bytes', return_value=b'/new/BeamNG.tech.x64') as read:
            with self.assertRaises(ConnectionError):
                life.require_alive()
            read.assert_not_called()

    def test_multiple_sdk_threads_cannot_poll_same_descriptor_concurrently(self):
        life, poller = self.lifetime()
        inside = threading.Lock()
        def strict_poll(_timeout):
            if not inside.acquire(blocking=False):
                raise RuntimeError('concurrent poll() invocation')
            try:
                time.sleep(.003)  # Allow other SDK threads to try simultaneously.
                return []
            finally:
                inside.release()
        poller.poll.side_effect = strict_poll
        gate = threading.Barrier(8)
        def sdk_request(_):
            gate.wait(timeout=2)
            life.require_alive()
        with ThreadPoolExecutor(max_workers=8) as pool:
            list(pool.map(sdk_request, range(8)))

    def test_dead_or_foreign_engine_is_rejected_and_descriptor_closed(self):
        for events, tokens in (([(42, 1)], None),
                               ((), b'/opt/BeamNG.tech.x64\x00-tport\x0012345\x00'),
                               ((), b'/opt/python\x00-tport\x0025252\x00')):
            with self.subTest(events=events, tokens=tokens), \
                 patch.object(attachment.os, 'close') as close:
                with self.assertRaises((ConnectionError, ValueError)):
                    self.lifetime(events, tokens)
                close.assert_called_once_with(42)

    def test_dead_owner_cannot_poll_remove_or_reconnect_new_engine(self):
        lifetime = Mock()
        class Connection:
            send = Mock(return_value='ack')
        original = Connection.send
        saved = attachment.guard_engine_requests(Connection, lifetime)
        c = Connection()
        self.assertEqual(c.send({'type': 'PollCamera'}), 'ack')
        original.assert_called_once_with(c, {'type': 'PollCamera'})
        lifetime.require_alive.side_effect = ConnectionError('old engine exited')
        for request in ('PollLidar', 'StartVehicleConnection', 'CloseCamera'):
            with self.subTest(request=request), self.assertRaises(ConnectionError):
                c.send({'type': request})
        self.assertEqual(original.call_count, 1)
        self.assertIs(saved, original)

    def test_platform_without_pidfd_fails_before_any_sdk_request(self):
        with patch.object(attachment.os, 'pidfd_open', side_effect=OSError('unsupported'), create=True), \
             patch.object(attachment.Path, 'read_bytes') as read:
            with self.assertRaises(OSError):
                attachment.EngineLifetime(123, 25252)
            read.assert_not_called()

    def test_current_sdk_integral_float_endpoint_is_supported(self):
        self.assertEqual(attachment.current_endpoint(dict(vid='ego', result=36165.0), 'ego'), 36165)

    def test_missing_foreign_or_malformed_endpoint_cannot_use_old_port(self):
        for response in ({}, {'vid': 'other', 'result': 36165},
                         *({'vid': 'ego', 'result': v} for v in
                           (None, True, 0, 65536, 36165.5, '36165', float('nan'), float('inf')))):
            with self.subTest(response=response), self.assertRaises(ValueError):
                attachment.current_endpoint(response, 'ego')

    def rig(self):
        return {'vehicles': [{'name': 'ego', 'wildosBodyCollisionDisabled': True,
                'sensors': [{'name': 'front_cam', 'type': 'Camera.default', 'pos': [0, -1.5, 2.35]},
                            {'name': 'front_lidar', 'type': 'Lidar.default'}]}]}

    def test_only_sensor_spec_is_copied_without_mutating_saved_vehicle(self):
        document = self.rig()
        original = copy.deepcopy(document)
        rig = attachment.rig_for_vehicle(document, 'ego')
        rig[0].pop('name')  # decode_sensors mutates its input.
        rig[0]['pos'][0] = 99
        self.assertEqual(document, original)
        self.assertIsInstance(rig, list)
        self.assertNotIn('wildosBodyCollisionDisabled', rig[0])

    def test_missing_duplicate_vehicle_or_invalid_sensor_identity_is_rejected(self):
        documents = [dict(vehicles=[]), dict(vehicles=self.rig()['vehicles'] * 2)]
        for change in ('duplicate_sensor', 'missing_camera', 'missing_lidar'):
            document = self.rig()
            sensors = document['vehicles'][0]['sensors']
            if change == 'duplicate_sensor':
                sensors[1]['name'] = sensors[0]['name']
            else:
                sensors.pop(0 if change == 'missing_camera' else 1)
            documents.append(document)
        for document in documents:
            with self.subTest(document=document), self.assertRaises(ValueError):
                attachment.rig_for_vehicle(document, 'ego')


if __name__ == '__main__':
    unittest.main()
