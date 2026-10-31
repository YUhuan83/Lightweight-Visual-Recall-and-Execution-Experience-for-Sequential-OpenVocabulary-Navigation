import unittest
from types import SimpleNamespace
from unittest.mock import Mock
from wildos_nav2_adapter.direct_vehicle_control import (
    CONTROL_ACK, control_lua, install_direct_steering_control)


class DirectVehicleControlTests(unittest.TestCase):
    def actuator(self):
        vehicle = SimpleNamespace(queue_lua_command=Mock(return_value=CONTROL_ACK))
        install_direct_steering_control(vehicle)
        return vehicle

    def test_one_request_uses_direct_steering_and_preserves_other_sdk_filters(self):
        v = self.actuator()
        v.control(steering=-.561, throttle=.25, brake=0., parkingbrake=0., clutch=0., gear=-1)
        v.queue_lua_command.assert_called_once()
        chunk = v.queue_lua_command.call_args.args[0]
        self.assertTrue(v.queue_lua_command.call_args.kwargs['response'])
        self.assertIn("input.event('steering', -0.561, FILTER_DIRECT)", chunk)
        for name, value in (('throttle', .25), ('brake', 0.), ('parkingbrake', 0.), ('clutch', 0.)):
            self.assertIn(f"input.event('{name}', {value!r}, FILTER_PAD)", chunk)
        self.assertIn('drivetrain.shiftToGear(-1)', chunk)
        self.assertTrue(chunk.startswith('assert('))

    def test_brake_only_does_not_change_gear_or_steering(self):
        chunk = control_lua(brake=1., parkingbrake=1.)
        self.assertNotIn("input.event('steering'", chunk)
        self.assertNotIn('shiftToGear', chunk)
        self.assertIn("input.event('brake', 1.0, FILTER_PAD)", chunk)

    def test_invalid_input_is_rejected_before_any_partial_request(self):
        v = self.actuator()
        for kw in (dict(steering=float('nan')), dict(brake=float('inf')),
                   dict(throttle=-.1), dict(steering=1.01), dict(steering=True),
                   dict(gear=1.5), dict(gear=True), dict(is_adas=True),
                   dict(brake="1); input.event('throttle',1,2)")):
            with self.subTest(kw=kw), self.assertRaises(ValueError):
                v.control(**kw)
        v.queue_lua_command.assert_not_called()

    def test_timeout_and_wrong_ack_are_not_reported_as_success(self):
        for response in (None, True, {}, 'another_request'):
            v = self.actuator(); v.queue_lua_command.return_value = response
            with self.subTest(response=response), self.assertRaises(ConnectionError):
                v.control(brake=1.)
        v = self.actuator(); v.queue_lua_command.side_effect = TimeoutError('late Lua response')
        with self.assertRaises(TimeoutError):
            v.control(throttle=.25)

    def test_instance_installation_does_not_change_a_foreign_client(self):
        foreign = SimpleNamespace(control=Mock())
        v = self.actuator(); v.control(steering=0., gear=0, brake=1.)
        foreign.control.assert_not_called()


if __name__ == '__main__':
    unittest.main()
