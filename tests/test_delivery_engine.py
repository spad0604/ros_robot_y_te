"""Offline delivery lifecycle checks: fake backend and action gateway, no robot IO."""
import copy
import importlib.util
from pathlib import Path
import unittest

spec = importlib.util.spec_from_file_location('delivery_engine_under_test', Path(__file__).parents[1]/'delivery_engine.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)

class MemoryJournal:
    def __init__(self): self.data = None
    def load(self): return copy.deepcopy(self.data)
    def save(self, data): self.data = copy.deepcopy(data)

class Clock:
    now = 100.
    def __call__(self): return self.now

class Api:
    def __init__(self):
        self.task = dict(id='task-1', mapId='map-1', robotCode='MEDIBOT-01', type='DELIVERY',
                         status='CREATED', dispatchEnabled=True, dropoffX=1.25, dropoffY=-2.5,
                         dropoffYaw=0, cancellationRequested=False, executionSequence=0)
        self.offline = False
        self.results = []
        self.claims = 0
        self.lose_claim_reply = False
    def active(self): return []
    def selection(self): return dict(status='APPLIED', mapId='map-1')
    def pending(self): return [copy.deepcopy(self.task)]
    def get(self, _): return copy.deepcopy(self.task)
    def claim(self, _, execution):
        self.claims += 1
        self.task.update(status='RUNNING', executionId=execution)
        if self.lose_claim_reply: raise OSError('lost reply')
        return self.get(None)
    def heartbeat(self, _, body):
        if self.offline: raise OSError('offline')
        self.task.update(executionSequence=body['sequence'])
        return self.get(None)
    def result(self, _, body):
        if self.offline: raise OSError('offline')
        self.results.append(body)
        self.task['status'] = {'SUCCEEDED':'COMPLETED','FAILED':'FAILED','CANCELLED':'CANCELLED'}[body['outcome']]
        return self.get(None)

class Motion:
    def __init__(self): self.goals, self.cancels, self.is_stopped = [], [], False
    def send(self, goal_id, task, timeout): self.goals.append((goal_id, copy.deepcopy(task), timeout))
    def cancel(self, goal_id): self.cancels.append(goal_id)
    def stopped(self, *_): return self.is_stopped

class Environment:
    available, valid, error, sig = True, True, None, 'map-load-1'
    def ready(self): return self.available, 'WAITING_FOR_LOCALIZATION'
    def active_ok(self, sig): return self.valid and sig == self.sig, 'MAP_CHANGED'
    def signature(self): return self.sig
    def validate_target(self, _): return self.error

class DeliveryChecks(unittest.TestCase):
    def setUp(self):
        self.api, self.motion, self.environment, self.journal, self.clock = Api(), Motion(), Environment(), MemoryJournal(), Clock()
        self.engine = self.new_engine()
    def new_engine(self):
        return module.DeliveryEngine('MEDIBOT-01', self.api, self.motion, self.environment, self.journal, monotonic=self.clock)
    def test_arrival_updates_backend_and_never_duplicates_goal(self):
        self.engine.step(); self.engine.step()
        self.assertEqual(len(self.motion.goals), 1)
        goal_id, task, _ = self.motion.goals[0]
        self.assertEqual((task['dropoffX'],task['dropoffY']), (1.25,-2.5))
        self.engine.result_received(goal_id, dict(outcome='SUCCEEDED',resultCode=0,message='Arrived'))
        self.engine.step(); self.engine.step()
        self.assertEqual(self.api.task['status'], 'COMPLETED')
        self.assertIsNone(self.journal.data)
        self.assertEqual(len(self.motion.goals), 1)
    def test_cancel_targets_only_own_goal_and_waits_for_stop(self):
        self.engine.step()
        self.api.task['cancellationRequested'] = True
        self.engine.step()
        self.assertEqual(self.motion.cancels, [self.motion.goals[0][0]])
        self.assertEqual(self.api.task['status'], 'RUNNING')
        self.motion.is_stopped = True
        self.engine.step(); self.engine.step()
        self.assertEqual(self.api.task['status'], 'CANCELLED')
    def test_watchdog_cancels_without_waiting_for_http(self):
        self.engine.step(); self.api.offline = True; self.clock.now += 3.1
        self.engine.watchdog()
        self.assertEqual(self.motion.cancels[-1], self.motion.goals[0][0])
        self.motion.is_stopped = True
        self.engine.step()
        self.assertEqual(self.journal.data['phase'], 'FINISHING')
        with self.assertRaises(OSError): self.engine.step()
        self.api.offline = False
        restarted = self.new_engine(); restarted.step()
        self.assertEqual(self.api.task['status'], 'FAILED')
        self.assertEqual(len(self.motion.goals), 1)
    def test_restart_cancels_persisted_id_without_replaying(self):
        self.engine.step()
        restarted = self.new_engine()
        restarted.step()
        self.assertEqual(self.motion.cancels[-1], self.motion.goals[0][0])
        self.motion.is_stopped = True
        restarted.step(); restarted.step()
        self.assertEqual(self.api.task['status'], 'FAILED')
        self.assertEqual(len(self.motion.goals), 1)
    def test_waits_for_localization_and_matching_map(self):
        self.environment.available = False
        self.engine.step()
        self.assertEqual(self.api.claims, 0)
        self.environment.available = True
        self.api.task['mapId'] = 'another-map'
        self.engine.step()
        self.assertEqual(self.api.claims, 0)
        self.api.task['mapId'] = 'map-1'
        self.engine.step()
        self.assertEqual(len(self.motion.goals), 1)
    def test_legacy_tasks_not_automatically_dispatched(self):
        self.api.task['dispatchEnabled'] = False
        self.engine.step()
        self.assertEqual(self.api.claims, 0)
    def test_occupied_destination_fails_without_action_goal(self):
        self.environment.error = 'DESTINATION_UNKNOWN_OR_OCCUPIED'
        self.engine.step(); self.engine.step()
        self.assertEqual(self.api.task['status'], 'FAILED')
        self.assertEqual(self.motion.goals, [])
    def test_map_change_cancels_active_goal(self):
        self.engine.step(); self.environment.sig = 'map-load-2'
        self.engine.watchdog()
        self.assertEqual(self.engine.cancel_reason, 'MAP_CHANGED')
        self.assertEqual(self.motion.cancels[-1], self.motion.goals[0][0])
    def test_lost_claim_response_does_not_send_a_goal(self):
        self.api.lose_claim_reply = True
        with self.assertRaises(OSError): self.engine.step()
        self.motion.is_stopped = True
        self.engine.step(); self.engine.step()
        self.assertEqual(self.motion.goals, [])
        self.assertEqual(self.api.task['status'], 'FAILED')
    def test_shutdown_does_not_dispatch_anything_new(self):
        self.engine.shutdown(); self.engine.step()
        self.assertEqual(self.motion.goals, [])

if __name__ == '__main__': unittest.main()
