"""Transport ownership/ACK contracts; Lua state machine is tested in BeamNG VM."""
import hashlib
from importlib import resources
from types import SimpleNamespace as S
import unittest
from unittest.mock import Mock
from wildos_nav2_adapter.vehicle_control_lease import VehicleControlLease
from wildos_nav2_adapter.direct_vehicle_control import install_direct_steering_control

OWNER = 'a' * 32


class VehicleControlLeaseTests(unittest.TestCase):
    def clients(self):
        source = resources.files('wildos_nav2_adapter').joinpath('vehicle_control_lease.lua').read_text(encoding='utf8')
        sha = hashlib.sha256(source.encode()).hexdigest()
        return S(control=S(queue_lua_command=Mock(return_value=sha))), S(
            queue_lua_command=Mock(return_value='wildos_lease_claimed_v1'))

    def test_invalid_owner_or_timeout_has_no_simulator_mutation(self):
        for value in (True, 0, -1, 2.01, float('nan'), float('inf'), '1'):
            bng, vehicle = self.clients()
            with self.subTest(value=value), self.assertRaises(ValueError):
                VehicleControlLease(bng, vehicle, value, OWNER)
            bng.control.queue_lua_command.assert_not_called()
            vehicle.queue_lua_command.assert_not_called()
        bng, vehicle = self.clients()
        with self.assertRaises(ValueError):
            VehicleControlLease(bng, vehicle, .8, "'); input.event('throttle',1,2)")
        bng.control.queue_lua_command.assert_not_called()

    def test_install_or_claim_failure_never_enables_controls(self):
        bng, vehicle = self.clients()
        bng.control.queue_lua_command.return_value = None
        with self.assertRaises(ConnectionError): VehicleControlLease(bng, vehicle, .8, OWNER)
        vehicle.queue_lua_command.assert_not_called()
        for bad in (None, {}, True, 'wildos_lease_released_v1'):
            bng, vehicle = self.clients(); vehicle.queue_lua_command.return_value = bad
            with self.subTest(bad=bad), self.assertRaises(ConnectionError):
                VehicleControlLease(bng, vehicle, .8, OWNER)

    def test_install_is_pinned_and_claim_does_not_autoload_during_presence_check(self):
        bng, vehicle = self.clients()
        lease = VehicleControlLease(bng, vehicle, .8, OWNER)
        self.assertEqual(lease.owner, OWNER)
        self.assertIn('old == nil or old == source', bng.control.queue_lua_command.call_args.args[0])
        self.assertIn("local loaded=rawget(extensions,'wildosControlLease')", vehicle.queue_lua_command.call_args.args[0])

    def test_control_identity_and_sequence_ack_reject_duplicate_or_foreign_response(self):
        v = S(queue_lua_command=Mock(return_value=f'wildos_direct_control_v2:{OWNER}:1'))
        install_direct_steering_control(v, OWNER)
        v.control(throttle=.25)
        first = v.queue_lua_command.call_args.args[0]
        self.assertLess(first.index('lease.feed('), first.index('input.event('))
        # Old ACK cannot acknowledge the second request.
        with self.assertRaises(ConnectionError): v.control(brake=1.)
        self.assertIn(f"lease.feed('{OWNER}',2)", v.queue_lua_command.call_args.args[0])
        v.queue_lua_command.return_value = f'wildos_direct_control_v2:{"b"*32}:3'
        with self.assertRaises(ConnectionError): v.control(brake=1.)
        v.queue_lua_command.return_value = f'wildos_direct_control_v2:{OWNER}:4'
        v.control(brake=1.)

    def test_release_uses_only_own_token_and_retains_moving_brake_guard(self):
        bng, vehicle = self.clients(); lease = VehicleControlLease(bng, vehicle, .8, OWNER)
        other_stream = S(queue_lua_command=Mock(return_value='wildos_lease_brake_held_v1'))
        self.assertEqual(lease.release(other_stream), 'wildos_lease_brake_held_v1')
        self.assertIn(f"m.release('{OWNER}')", other_stream.queue_lua_command.call_args.args[0])
        other_stream.queue_lua_command.return_value = 'unknown'
        with self.assertRaises(ConnectionError): lease.release(other_stream)


if __name__ == '__main__': unittest.main()
