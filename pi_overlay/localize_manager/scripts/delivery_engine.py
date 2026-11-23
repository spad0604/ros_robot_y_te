#!/usr/bin/env python3
"""Delivery lifecycle, independent of ROS. Never replays an interrupted goal."""
import json
import os
import re
import threading
import time
import uuid
from datetime import datetime


class Journal:
    def __init__(self, path):
        self.path = os.path.abspath(os.path.expanduser(path))

    def load(self):
        try:
            with open(self.path, encoding='utf-8') as source:
                data = json.load(source)
            if data is not None:
                for key in ('task', 'executionId', 'goalId', 'sequence', 'phase'):
                    if key not in data:
                        raise ValueError('Incomplete delivery journal: ' + key)
            return data
        except FileNotFoundError:
            return None

    def save(self, data):
        os.makedirs(os.path.dirname(self.path), exist_ok=True)
        temporary = self.path + '.tmp'
        fd = os.open(temporary, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
        with os.fdopen(fd, 'w', encoding='utf-8') as target:
            json.dump(data, target, allow_nan=False)
            target.flush()
            os.fsync(target.fileno())
        os.replace(temporary, self.path)
        directory = os.open(os.path.dirname(self.path), os.O_RDONLY)
        try:
            os.fsync(directory)
        finally:
            os.close(directory)


class DeliveryEngine:
    def __init__(self, code, api, motion, environment, journal, log=lambda msg: None,
                 network_grace=3., timeout=300., monotonic=time.monotonic):
        self.code, self.api, self.motion, self.environment = code, api, motion, environment
        self.journal, self.log, self.clock = journal, log, monotonic
        self.network_grace, self.timeout = network_grace, timeout
        self.lock = threading.RLock()
        self.job = journal.load()
        self.reconciled = False
        self.stopping = False
        self.last_api = self.clock()
        self.started = self.clock()
        self.last_cancel = -100.
        self.cancel_since = None
        self.cancel_reason = None
        self.feedback = {}
        self.action_result = None
        self.map_signature = None
        self.initialization_failures_reported = set()
        if self.job and self.job['phase'] != 'FINISHING':
            self.job['phase'] = 'RECOVERING'
            self.journal.save(self.job)

    @staticmethod
    def goal_id(code, task_id, execution):
        return 'delivery:' + code + ':' + str(task_id) + ':' + str(execution)

    def _save(self):
        self.journal.save(self.job)

    def _new_job(self, task, execution=None):
        execution = execution or str(uuid.uuid4())
        return {'task': task, 'executionId': execution,
                'goalId': self.goal_id(self.code, task['id'], execution),
                'sequence': int(task.get('executionSequence') or 0), 'phase': 'CLAIMING'}

    def _clear(self):
        with self.lock:
            self.job = None
            self.reconciled = False
            self.cancel_reason = self.cancel_since = self.action_result = None
            self.feedback = {}
            self._save()

    def feedback_received(self, goal_id, feedback):
        with self.lock:
            if self.job and goal_id == self.job['goalId']:
                self.feedback = dict(feedback)

    def result_received(self, goal_id, result):
        with self.lock:
            if self.job and goal_id == self.job['goalId']:
                self.action_result = dict(result)

    def cancel(self, reason):
        with self.lock:
            if not self.job or self.job['phase'] != 'ACTIVE':
                return
            if self.cancel_reason is None:
                self.cancel_reason, self.cancel_since = reason, self.clock()
                self.log('Cancelling delivery ' + str(self.job['task']['id']) + ': ' + reason)
            if self.clock() - self.last_cancel >= .5:
                self.motion.cancel(self.job['goalId'])
                self.last_cancel = self.clock()

    def watchdog(self):
        # ROS timer invokes this separately from blocking HTTP requests.
        with self.lock:
            if not self.job or self.job['phase'] != 'ACTIVE':
                return
            if self.clock() - self.last_api > self.network_grace:
                self.cancel('BACKEND_UNREACHABLE')
            elif self.clock() - self.started > self.timeout + 2:
                self.cancel('GOAL_TIMEOUT')
            else:
                valid, reason = self.environment.active_ok(self.map_signature)
                if not valid:
                    self.cancel(reason)

    def _finish(self, result):
        with self.lock:
            if self.cancel_reason:
                result = dict(result)
                result['outcome'] = 'CANCELLED' if self.cancel_reason == 'OPERATOR_CANCEL' else 'FAILED'
                result['message'] = self.cancel_reason
            self.job['phase'] = 'FINISHING'
            self.job['result'] = result
            self._save()

    def _update(self, result=False):
        with self.lock:
            self.job['sequence'] += 1
            self._save()  # Persist sequence before HTTP; retry cannot resurrect older feedback.
            body = dict(self.job.get('result', {}) if result else self.feedback)
            body.update(executionId=self.job['executionId'], sequence=self.job['sequence'])
            if not result:
                body.setdefault('state', 'DISPATCHING')
                if self.cancel_reason:
                    body.update(state='CANCELLING', message=self.cancel_reason)
            task_id = self.job['task']['id']
        response = self.api.result(task_id, body) if result else self.api.heartbeat(task_id, body)
        self.last_api = self.clock()
        return response

    def _recover(self):
        with self.lock:
            goal_id, task_id = self.job['goalId'], self.job['task']['id']
            if self.cancel_since is None:
                self.cancel_since = self.clock()
            if self.clock() - self.last_cancel >= .5:
                self.motion.cancel(goal_id)
                self.last_cancel = self.clock()
            if not self.motion.stopped(goal_id, self.cancel_since):
                return
        task = self.api.get(task_id)
        if task.get('executionId') != self.job['executionId'] or task.get('status') != 'RUNNING':
            self._clear()
            return
        with self.lock:
            self.job['sequence'] = max(self.job['sequence'], int(task.get('executionSequence') or 0))
            self._finish({'outcome': 'CANCELLED' if task.get('cancellationRequested') else 'FAILED',
                          'message': 'Interrupted execution stopped after executor restart; goal was not replayed.',
                          'resultCode': 2})

    def _reconcile(self):
        active = self.api.active()
        if not active:
            self.reconciled = True
            return True
        task = active[0]
        lease = task.get('executionLeaseUntil')
        expires = datetime.fromisoformat(re.sub(r'(\.\d{6})\d+', r'\1', lease).replace('Z', '+00:00')).timestamp() if lease else 0
        # A different healthy executor may still own this robot. Never steal its goal.
        if not task.get('cancellationRequested') and expires > time.time() - 2:
            return False
        if not task.get('executionId'):
            return False
        with self.lock:
            self.job = self._new_job(task, task['executionId'])
            self.job['phase'] = 'RECOVERING'
            self._save()
        return False

    def step(self):
        if self.stopping: return
        if self.job:
            phase = self.job['phase']
            if phase == 'FINISHING':
                response = self._update(result=True)
                self.log('Delivery result acknowledged: ' + str(response.get('status')))
                self._clear()
                self.reconciled = False
                return
            if phase == 'RECOVERING':
                self._recover()
                return
            if phase == 'ACTIVE':
                self.watchdog()
                with self.lock:
                    result = self.action_result
                    goal_id = self.job['goalId']
                    cancelled = self.cancel_reason is not None
                if result is not None:
                    self._finish(result)
                    return
                if cancelled:
                    self.cancel(self.cancel_reason)
                    if self.motion.stopped(goal_id, self.cancel_since):
                        self._finish({'outcome': 'FAILED', 'message': self.cancel_reason, 'resultCode': 2})
                        return
                response = self._update()
                if response.get('cancellationRequested'):
                    self.cancel('OPERATOR_CANCEL')
                elif response.get('status') != 'RUNNING':
                    self.cancel('SERVER_TASK_TERMINATED')
                return
            # Any uncertain dispatch is reconciled, not sent again.
            self.job['phase'] = 'RECOVERING'
            self._save()
            return
        if not self.reconciled and not self._reconcile():
            return
        selection = self.api.selection()
        if not selection or selection.get('status') != 'APPLIED':
            return
        for task in self.api.pending():
            if (task.get('robotCode') != self.code or task.get('type') != 'DELIVERY'
                    or task.get('status') != 'CREATED' or not task.get('dispatchEnabled')
                    or str(task.get('mapId')) != str(selection.get('mapId'))):
                continue
            reuse_localization = (task.get('executionState') == 'DISPATCHING'
                                  and all(task.get(key) is None for key in
                                          ('initialPoseX', 'initialPoseY', 'initialPoseYaw')))
            if not reuse_localization:
                pose_ready, pose_reason = self.environment.prepare_for_delivery(
                    task, selection.get('mapName'))
                if not pose_ready:
                    if pose_reason in ('INITIAL_POSE_REQUIRED', 'INVALID_INITIAL_POSE',
                                       'POSE_OUTSIDE_MAP', 'INITIAL_POSE_UNKNOWN_OR_OCCUPIED',
                                       'INITIAL_POSE_SUBSCRIBER_UNAVAILABLE'):
                        failure_key = (str(task.get('id')), task.get('initialPoseX'),
                                       task.get('initialPoseY'), task.get('initialPoseYaw'),
                                       task.get('updatedAt'))
                        if failure_key not in self.initialization_failures_reported:
                            self.api.initialization(task['id'], 'INIT_FAILED', pose_reason)
                            self.initialization_failures_reported.add(failure_key)
                    self.log('Preparing delivery: ' + pose_reason)
                    return
            ready, reason = self.environment.ready()
            if not ready:
                self.log('Waiting to dispatch: ' + reason)
                return
            with self.lock:
                self.job = self._new_job(task)
                self._save()  # Intent exists before claim or action publication.
            try:
                response = self.api.claim(task['id'], self.job['executionId'])
            except Exception:
                self.job['phase'] = 'RECOVERING'
                self._save()
                raise
            self.last_api = self.clock()
            with self.lock:
                self.job['task'] = response
                if self.stopping:
                    self.job['phase'] = 'RECOVERING'
                    self.motion.cancel(self.job['goalId'])
                    self._save()
                    return
                self.map_signature = self.environment.signature()
                self.started = self.clock()
                if response.get('cancellationRequested') or response.get('status') != 'RUNNING':
                    self._finish({'outcome': 'CANCELLED', 'message': 'Cancelled before dispatch', 'resultCode': 2})
                    return
                error = self.environment.validate_target(task)
                if error:
                    self._finish({'outcome': 'FAILED', 'message': error, 'resultCode': 4})
                    return
                ready, reason = self.environment.ready()
                if not ready:
                    self._finish({'outcome': 'FAILED', 'message': reason, 'resultCode': 1})
                    return
                if self.environment.signature() != self.map_signature:
                    self._finish({'outcome': 'FAILED', 'message': 'MAP_CHANGED', 'resultCode': 1})
                    return
                self.job['phase'] = 'ACTIVE'
                self._save()  # A crash from this point only cancels this deterministic goal ID.
                self.motion.send(self.job['goalId'], task, self.timeout)
                self.log('Delivery sent to moving_server: ' + str(task['id']))
            return

    def shutdown(self):
        with self.lock:
            self.stopping = True
            if self.job and self.job['phase'] != 'FINISHING':
                self.motion.cancel(self.job['goalId'])
                self.job['phase'] = 'RECOVERING'
                self._save()
