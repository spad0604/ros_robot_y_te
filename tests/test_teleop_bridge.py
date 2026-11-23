"""Offline watchdog checks; no ROS master, socket or motor connection."""
import importlib.util
import json
from pathlib import Path
import sys
import threading
import time
import types
import unittest
from unittest.mock import Mock

class Twist:
    def __init__(self):
        self.linear = types.SimpleNamespace(x=0., y=0., z=0.)
        self.angular = types.SimpleNamespace(x=0., y=0., z=0.)

sys.modules['rospy'] = types.ModuleType('rospy')
sys.modules['websocket'] = types.ModuleType('websocket')
sys.modules['geometry_msgs'] = types.ModuleType('geometry_msgs')
sys.modules['geometry_msgs.msg'] = types.SimpleNamespace(Twist=Twist)
sys.modules['std_msgs'] = types.ModuleType('std_msgs')
sys.modules['std_msgs.msg'] = types.SimpleNamespace(String=str)
spec = importlib.util.spec_from_file_location('teleop_bridge', Path(__file__).parents[1] / 'teleop_bridge.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)

class WatchdogChecks(unittest.TestCase):
    def setUp(self):
        self.bridge = module.Bridge.__new__(module.Bridge)
        b = self.bridge
        b.lock = threading.RLock()
        b.state, b.state_at = 'MAPPING', time.monotonic()
        b.auto_at, b.deadline = 0., 0.
        b.command, b.active, b.connected, b.mux_ready = Twist(), False, True, True
        b.buffer = ''
        b.sequence = 0
        b.publisher = Mock()
        b.publisher.get_num_connections.return_value = 1

    def message(self, **changes):
        data = dict(sequence=self.bridge.sequence + 1, x=.15, y=0., yaw=0., expiresAt=time.time()*1000 + 200)
        data.update(changes)
        return 'MESSAGE\ndestination:/user/queue/teleop\n\n' + json.dumps(data) + '\x00'

    def test_valid_then_expired_stops_once(self):
        b = self.bridge
        b.received(None, self.message())
        b.tick(None)
        self.assertEqual(b.publisher.publish.call_args.args[0].linear.x, .15)
        b.deadline = time.monotonic()-1
        b.tick(None)
        self.assertEqual(b.publisher.publish.call_args.args[0].linear.x, 0)
        count = b.publisher.publish.call_count
        b.tick(None)
        self.assertEqual(b.publisher.publish.call_count, count)

    def test_rejects_stale_nonfinite_and_excess_speed(self):
        for changes in ({'expiresAt': 0}, {'x': float('nan')}, {'x': 1}, {'yaw': 2}):
            self.bridge.received(None, self.message(**changes))
            self.assertEqual(self.bridge.deadline, 0)

    def test_mapping_mux_state_age_and_auto_gate(self):
        b = self.bridge
        for field, value in [('state', 'IDLE'), ('mux_ready', False), ('connected', False),
                             ('state_at', time.monotonic()-2), ('auto_at', time.monotonic())]:
            before = getattr(b, field)
            setattr(b, field, value)
            self.assertFalse(b.ready(), field)
            setattr(b, field, before)
        self.assertTrue(b.ready())

    def test_state_change_stops_active_drive(self):
        b = self.bridge
        b.received(None, self.message()); b.tick(None)
        b.state_changed(types.SimpleNamespace(data='IDLE'))
        b.tick(None)
        self.assertEqual(b.publisher.publish.call_args.args[0].linear.x, 0)

    def test_fragmented_frame(self):
        data = self.message()
        self.bridge.received(None, data[:20])
        self.assertEqual(self.bridge.deadline, 0)
        self.bridge.received(None, data[20:])
        self.assertGreater(self.bridge.deadline, time.monotonic())

    def test_old_drive_cannot_replace_newer_stop(self):
        b = self.bridge
        old = self.message(sequence=1)
        b.received(None, self.message(sequence=2, x=0))
        b.received(None, old)
        b.tick(None)
        self.assertEqual(b.command.linear.x, 0)
        self.assertEqual(b.sequence, 2)

    def test_fresh_latched_state_refreshes_only_matching_current_state(self):
        b = self.bridge
        b.state_at = old = time.monotonic() - 10
        module.rospy.wait_for_message = Mock(return_value=types.SimpleNamespace(data='IDLE'))
        b.refresh_state()
        self.assertEqual(b.state_at, old)
        module.rospy.wait_for_message.return_value = types.SimpleNamespace(data='MAPPING')
        b.refresh_state()
        self.assertGreater(b.state_at, old)

    def test_disconnect_stops_active_drive(self):
        b = self.bridge
        b.received(None, self.message()); b.tick(None)
        b.connected = False; b.tick(None)
        self.assertEqual(b.publisher.publish.call_args.args[0].linear.x, 0)

if __name__ == '__main__':
    unittest.main()
