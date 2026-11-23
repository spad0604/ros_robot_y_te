#!/usr/bin/env python3
"""Order execution adapter using ONLY moving_server's standard ROS action protocol."""
import fcntl
import json
import math
import os
import sys
import signal
import subprocess
import threading
import time
from urllib.error import HTTPError
from urllib.parse import quote
from urllib.request import Request, urlopen

import rospy
import tf2_ros
from actionlib_msgs.msg import GoalID, GoalStatusArray
from geometry_msgs.msg import PoseWithCovarianceStamped
from localize_manager.srv import StartLocalization
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
from moving_server_msgs.msg import (MoveCommandActionGoal, MoveCommandActionFeedback,
                                   MoveCommandActionResult, MovingServerState, MoveCommandGoal)
sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from delivery_engine import DeliveryEngine, Journal
from battery_monitor import BatteryMonitor


class RobotApi:
    def __init__(self, code, base, credentials):
        self.code, self.base, self.credentials = code, base.rstrip('/'), os.path.expanduser(credentials)
        self.token = None
        self.root = '/robots/' + quote(code, safe='') + '/deliveries'

    def call(self, path, body=None, authenticate=True):
        if authenticate and not self.token:
            self.login()
        headers = {'Content-Type': 'application/json'}
        if authenticate:
            headers['Authorization'] = 'Bearer ' + self.token
        req = Request(self.base + path, data=None if body is None else json.dumps(body, allow_nan=False).encode(), headers=headers)
        try:
            with urlopen(req, timeout=2) as response:
                return json.load(response).get('data')
        except HTTPError as error:
            if error.code == 401 and authenticate:
                self.token = None
            raise

    def login(self):
        with open(self.credentials, encoding='utf-8') as source:
            saved = json.load(source)
        data = self.call('/auth/login', {k: saved[k] for k in ('username', 'password')}, authenticate=False)
        if data['user']['role'] != 'ROBOT' or data['user']['username'] != self.code:
            raise ValueError('Delivery executor requires matching robot credentials')
        self.token = data['accessToken']

    def pending(self): return self.call(self.root + '/pending') or []
    def active(self): return self.call(self.root + '/active') or []
    def get(self, task): return self.call('/tasks/' + quote(str(task), safe=''))
    def selection(self): return self.call('/robots/' + quote(self.code, safe='') + '/map-selection')
    def claim(self, task, execution): return self.call(self.root + '/' + str(task) + '/claim', {'executionId': execution})
    def initialization(self, task, state, message):
        return self.call(self.root + '/' + str(task) + '/initialization',
                         {'state': state, 'message': message})
    def heartbeat(self, task, body): return self.call(self.root + '/' + str(task) + '/heartbeat', body)
    def result(self, task, body): return self.call(self.root + '/' + str(task) + '/result', body)


def yaw(q):
    return math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


class Motion:
    def __init__(self):
        self.lock = threading.RLock()
        self.engine = None
        self.state = None
        self.status = []
        self.status_at = 0.
        self.follow_motion_status = "OFF"
        self.terminal = set()
        self.frame = rospy.get_param('~global_frame', 'map')
        self.action = rospy.get_param('~action_name', '/moving_server/move_command').rstrip('/')
        self.goal_pub = rospy.Publisher(self.action + '/goal', MoveCommandActionGoal, queue_size=1, latch=False)
        self.cancel_pub = rospy.Publisher(self.action + '/cancel', GoalID, queue_size=1, latch=False)
        self.state_sub = rospy.Subscriber(rospy.get_param('~moving_state_topic', '/moving_server/state'), MovingServerState, self.state_cb)
        rospy.Subscriber(self.action + '/status', GoalStatusArray, self.status_cb)
        self.feedback_sub = rospy.Subscriber(self.action + '/feedback', MoveCommandActionFeedback, self.feedback_cb)
        self.result_sub = rospy.Subscriber(self.action + '/result', MoveCommandActionResult, self.result_cb)
        rospy.Subscriber('/vision/follow_motion_state', String, self.follow_state_cb, queue_size=1)

    def state_cb(self, message):
        with self.lock: self.state = message

    def follow_state_cb(self, message):
        try:
            status = str(json.loads(message.data).get('status', '')).upper()
        except (ValueError, TypeError, AttributeError):
            return
        with self.lock: self.follow_motion_status = status

    def status_cb(self, message):
        with self.lock:
            self.status, self.status_at = message.status_list, time.monotonic()
            for status in self.status:
                if status.status in (2, 3, 4, 5, 8): self.terminal.add(status.goal_id.id)

    def live(self):
        return (self.goal_pub.get_num_connections() > 0 and self.cancel_pub.get_num_connections() > 0
                and self.feedback_sub.get_num_connections() > 0 and self.result_sub.get_num_connections() > 0
                and time.monotonic() - self.status_at < 2)

    def idle(self):
        with self.lock:
            return (self.live() and self.state_sub.get_num_connections() > 0 and self.state is not None
                    and not self.state.active and self.state.state_name == 'IDLE'
                    and self.follow_motion_status not in ('STARTING', 'TARGET_SELECTED', 'FOLLOWING')
                    and not any(s.status in (0, 1, 6, 7) for s in self.status))

    def send(self, goal_id, task, timeout):
        if not self.idle(): raise RuntimeError('moving_server is no longer idle')
        message = MoveCommandActionGoal()
        message.header.stamp = rospy.Time.now()
        message.goal_id.id, message.goal_id.stamp = goal_id, message.header.stamp
        message.goal.mode = MoveCommandGoal.MODE_MOVE_TO_POINT
        message.goal.timeout = timeout
        message.goal.target_pose.header.frame_id = self.frame
        message.goal.target_pose.header.stamp = message.header.stamp
        message.goal.target_pose.pose.position.x = float(task['dropoffX'])
        message.goal.target_pose.pose.position.y = float(task['dropoffY'])
        angle = float(task.get('dropoffYaw') or 0)
        message.goal.target_pose.pose.orientation.z = math.sin(angle/2)
        message.goal.target_pose.pose.orientation.w = math.cos(angle/2)
        self.goal_pub.publish(message)

    def cancel(self, goal_id):
        # A non-empty ID and zero stamp cancel exactly this goal, never all goals.
        self.cancel_pub.publish(GoalID(stamp=rospy.Time(0), id=goal_id))

    def stopped(self, goal_id, since):
        with self.lock:
            if goal_id in self.terminal: return True
            return (self.live() and self.status_at >= since and time.monotonic() - since > 1
                    and not any(s.goal_id.id == goal_id and s.status in (0, 1, 6, 7) for s in self.status))

    def feedback_cb(self, message):
        if not self.engine: return
        feedback = message.feedback
        data = {'state': feedback.state_name, 'message': feedback.message}
        if math.isfinite(feedback.progress): data['progress'] = max(0., min(1., feedback.progress))
        if math.isfinite(feedback.distance_remaining): data['distanceRemaining'] = max(0., feedback.distance_remaining)
        self.engine.feedback_received(message.status.goal_id.id, data)

    def result_cb(self, message):
        if not self.engine: return
        result = message.result
        succeeded = message.status.status == 3 and result.success and result.result_code == 0
        data = {'outcome': 'SUCCEEDED' if succeeded else 'FAILED', 'resultCode': int(result.result_code), 'message': result.message}
        pose = result.final_pose
        if pose.header.frame_id == self.frame:
            values = (pose.pose.position.x, pose.pose.position.y, yaw(pose.pose.orientation))
            if all(math.isfinite(v) for v in values): data.update(zip(('finalX', 'finalY', 'finalYaw'), values))
        self.engine.result_received(message.status.goal_id.id, data)


class Environment:
    def __init__(self, motion):
        self.motion = motion
        self.state, self.grid = '', None
        self.state_sub = rospy.Subscriber('/localize_manager/state', String, self.state_cb)
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_cb, queue_size=1)
        self.base_status, self.base_status_at = [], 0.
        rospy.Subscriber('/move_base/status', GoalStatusArray, self.base_status_cb)
        self.tf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf)
        self.frame = rospy.get_param('~global_frame', 'map')
        self.base = rospy.get_param('~base_frame', 'base_link')
        self.initial_pose_topic = rospy.get_param('~initial_pose_topic', '/initialpose')
        self.localization_service = rospy.get_param(
            '~start_localization_service', '/localize_manager/start_localization')
        self.initial_pose_pub = rospy.Publisher(
            self.initial_pose_topic, PoseWithCovarianceStamped, queue_size=1, latch=False)
        self._bootstrap_key = None
        self._bootstrap_pose = None
        self._localization_requested_for = None
        self._initial_pose_sent = 0
        self._last_initial_pose_sent = 0.
        self._initial_pose_sent_at = rospy.Time()

    def state_cb(self, message): self.state = message.data.strip().upper()
    def base_status_cb(self, message):
        self.base_status, self.base_status_at = message.status_list, time.monotonic()
    def map_cb(self, message): self.grid = message

    def signature(self):
        grid = self.grid
        if grid is None: return None
        i = grid.info
        return (i.map_load_time.to_nsec(), i.width, i.height, i.resolution,
                i.origin.position.x, i.origin.position.y, yaw(i.origin.orientation))

    def active_ok(self, signature):
        if self.state != 'LOCALIZING' or self.state_sub.get_num_connections() == 0:
            return False, 'LOCALIZATION_NOT_ACTIVE'
        if not self.motion.live(): return False, 'MOVING_SERVER_UNAVAILABLE'
        if time.monotonic() - self.base_status_at > 2: return False, 'MOVE_BASE_UNAVAILABLE'
        if self.signature() != signature: return False, 'MAP_CHANGED'
        try:
            transform = self.tf.lookup_transform(self.frame, self.base, rospy.Time(0), rospy.Duration(0))
            age = (rospy.Time.now() - transform.header.stamp).to_sec()
            if age < -.5 or age > 2: return False, 'ROBOT_POSE_STALE'
        except Exception: return False, 'ROBOT_POSE_UNAVAILABLE'
        return True, ''

    def ready(self):
        if self.grid is None or self.map_sub.get_num_connections() == 0: return False, 'WAITING_FOR_MAP'
        if self.grid.header.frame_id != self.frame: return False, 'MAP_FRAME_MISMATCH'
        ok, reason = self.active_ok(self.signature())
        if not ok: return False, reason
        if not self.motion.idle(): return False, 'MOVING_SERVER_BUSY_OR_NOT_READY'
        if any(s.status in (0, 1, 6, 7) for s in self.base_status): return False, 'MOVE_BASE_BUSY'
        return True, ''

    def _cell_value(self, x, y):
        grid = self.grid
        if grid is None or grid.info.resolution <= 0:
            return None, 'WAITING_FOR_MAP'
        info = grid.info
        heading = yaw(info.origin.orientation)
        dx, dy = x - info.origin.position.x, y - info.origin.position.y
        col = math.floor((math.cos(heading)*dx + math.sin(heading)*dy)/info.resolution)
        row = math.floor((-math.sin(heading)*dx + math.cos(heading)*dy)/info.resolution)
        if not 0 <= col < info.width or not 0 <= row < info.height:
            return None, 'POSE_OUTSIDE_MAP'
        return grid.data[row*info.width+col], ''

    def prepare_for_delivery(self, task, map_name):
        """Start localization and seed it with the pose selected in the app."""
        try:
            values = (float(task['initialPoseX']), float(task['initialPoseY']),
                      float(task['initialPoseYaw']))
            if not all(math.isfinite(value) for value in values):
                return False, 'INVALID_INITIAL_POSE'
        except (KeyError, TypeError, ValueError):
            return False, 'INITIAL_POSE_REQUIRED'
        key = str(task.get('id'))
        pose_key = tuple(round(value, 6) for value in values)
        if key != self._bootstrap_key or pose_key != self._bootstrap_pose:
            if key != self._bootstrap_key:
                self._localization_requested_for = None
            self._bootstrap_key, self._bootstrap_pose = key, pose_key
            self._initial_pose_sent = 0
            self._last_initial_pose_sent = 0.
            self._initial_pose_sent_at = rospy.Time()

        if self.state == 'MAPPING':
            return False, 'MAPPING_ACTIVE'
        if not self.motion.idle():
            return False, 'MOVING_SERVER_BUSY_OR_NOT_READY'
        if time.monotonic() - self.base_status_at > 2 or any(
                status.status in (0, 1, 6, 7) for status in self.base_status):
            return False, 'MOVE_BASE_BUSY_OR_NOT_READY'

        if self.state != 'LOCALIZING':
            if not map_name:
                return False, 'SELECTED_MAP_NAME_UNAVAILABLE'
            if self._localization_requested_for != key:
                try:
                    rospy.wait_for_service(self.localization_service, timeout=.2)
                    response = rospy.ServiceProxy(
                        self.localization_service, StartLocalization)(map_name)
                except (rospy.ROSException, rospy.ServiceException):
                    return False, 'LOCALIZATION_SERVICE_UNAVAILABLE'
                if not response.success:
                    return False, 'LOCALIZATION_FAILED: ' + str(response.message)
                self._localization_requested_for = key
            return False, 'LOCALIZATION_STARTING'

        if self.grid is None or self.map_sub.get_num_connections() == 0:
            return False, 'WAITING_FOR_MAP'
        if self.grid.header.frame_id.lstrip('/') != self.frame.lstrip('/'):
            return False, 'MAP_FRAME_MISMATCH'
        cell, error = self._cell_value(values[0], values[1])
        if error:
            return False, error
        if cell is None or cell < 0 or cell >= 65:
            return False, 'INITIAL_POSE_UNKNOWN_OR_OCCUPIED'
        if self.initial_pose_pub.get_num_connections() == 0:
            return False, 'INITIAL_POSE_SUBSCRIBER_UNAVAILABLE'

        now = time.monotonic()
        if self._initial_pose_sent < 3 and now - self._last_initial_pose_sent >= .75:
            message = PoseWithCovarianceStamped()
            message.header.stamp = rospy.Time.now()
            message.header.frame_id = self.frame
            message.pose.pose.position.x = values[0]
            message.pose.pose.position.y = values[1]
            message.pose.pose.orientation.z = math.sin(values[2] / 2.)
            message.pose.pose.orientation.w = math.cos(values[2] / 2.)
            message.pose.covariance[0] = .25
            message.pose.covariance[7] = .25
            message.pose.covariance[35] = .12
            self.initial_pose_pub.publish(message)
            self._initial_pose_sent += 1
            self._last_initial_pose_sent = now
            self._initial_pose_sent_at = message.header.stamp
            return False, 'INITIALIZING_POSE'

        if self._initial_pose_sent < 3:
            return False, 'INITIALIZING_POSE'
        if (rospy.Time.now() - self._initial_pose_sent_at).to_sec() < 1.0:
            return False, 'WAITING_FOR_LOCALIZATION_TF'
        try:
            transform = self.tf.lookup_transform(
                self.frame, self.base, rospy.Time(0), rospy.Duration(0))
            age = (rospy.Time.now() - transform.header.stamp).to_sec()
            estimated = transform.transform.translation
            estimated_yaw = yaw(transform.transform.rotation)
            error_distance = math.hypot(estimated.x - values[0], estimated.y - values[1])
            error_yaw = abs(math.atan2(math.sin(estimated_yaw-values[2]),
                                       math.cos(estimated_yaw-values[2])))
            if age < -.5 or age > 2:
                return False, 'ROBOT_POSE_STALE'
            if error_distance > 1.0 or error_yaw > 1.0:
                return False, 'INITIAL_POSE_NOT_CONFIRMED'
        except Exception:
            return False, 'ROBOT_POSE_UNAVAILABLE'
        return True, ''

    def validate_target(self, task):
        try:
            x, y, angle = float(task['dropoffX']), float(task['dropoffY']), float(task.get('dropoffYaw') or 0)
            if not all(math.isfinite(v) for v in (x, y, angle)): return 'INVALID_DESTINATION'
            info = self.grid.info
            heading = yaw(info.origin.orientation)
            dx, dy = x - info.origin.position.x, y - info.origin.position.y
            col = math.floor((math.cos(heading)*dx + math.sin(heading)*dy)/info.resolution)
            row = math.floor((-math.sin(heading)*dx + math.cos(heading)*dy)/info.resolution)
            if not 0 <= col < info.width or not 0 <= row < info.height: return 'DESTINATION_OUTSIDE_MAP'
            occupancy = self.grid.data[row*info.width+col]
            if occupancy < 0 or occupancy >= 65: return 'DESTINATION_UNKNOWN_OR_OCCUPIED'
        except (TypeError, ValueError, KeyError, ZeroDivisionError, IndexError): return 'INVALID_DESTINATION'
        return None


class ChargeDispatch:
    def __init__(self, api, environment):
        self.api = api
        self.environment = environment
        self.path = '/robots/' + quote(api.code, safe='') + '/charge-command'
        self.process = None
        self.command = None
        self.result = None
        self.next_poll = 0.

    def shutdown(self):
        if self.process and self.process.poll() is None:
            self.process.send_signal(signal.SIGINT)

    def step(self, delivery_active):
        if self.process is not None:
            code = self.process.poll()
            if code is None:
                return True
            self.result = {'success': code == 0,
                           'message': 'Docking completed' if code == 0 else 'Navigation or docking failed; see Pi log'}
            self.process = None
        if self.result is not None:
            try:
                self.api.call(self.path + '/' + self.command + '/result', self.result)
            except HTTPError as error:
                if error.code not in (404, 409):
                    rospy.logwarn_throttle(10, 'Charge result HTTP error: %s', error.code)
                    return False
            except OSError as error:
                rospy.logwarn_throttle(10, 'Charge result unavailable: %s', type(error).__name__)
                return False
            self.result = None
        if delivery_active or time.monotonic() < self.next_poll:
            return False
        self.next_poll = time.monotonic() + 2
        try:
            command = self.api.call(self.path)
        except Exception as error:
            rospy.logwarn_throttle(15, 'Charge API unavailable: %s', type(error).__name__)
            return False
        if not command or command['status'] != 'PENDING':
            return False
        target = command['target']
        selected = self.api.selection()
        ready, reason = self.environment.ready()
        if not ready:
            rospy.loginfo_throttle(10, 'Charge waiting: %s', reason)
            return False
        self.command = command['id']
        self.api.call(self.path + '/' + self.command + '/claim', {})
        if not selected or selected.get('status') != 'APPLIED' or selected.get('mapId') != target['mapId']:
            self.result = {'success': False, 'message': 'Selected robot map does not match charge command'}
            return False
        error = self.environment.validate_target({'dropoffX': target['x'], 'dropoffY': target['y'],
                                                   'dropoffYaw': target['yaw']})
        if error:
            self.result = {'success': False, 'message': error}
            return False
        try:
            self.process = subprocess.Popen([
                'rosrun', 'moving_server', 'move_command_client.py', '--timeout', '120', 'charge',
                str(target['x']), str(target['y']), str(target['yaw']), '--dock-id', target['dockId']])
        except OSError as error:
            self.result = {'success': False, 'message': str(error)}
            return False
        return True


def main():
    os.makedirs(os.path.expanduser('~/.ros'), exist_ok=True)
    singleton = open(os.path.expanduser('~/.ros/delivery_executor.lock'), 'w')
    fcntl.flock(singleton, fcntl.LOCK_EX | fcntl.LOCK_NB)
    rospy.init_node('delivery_executor')
    code = rospy.get_param('~robot_code', 'MEDIBOT-01')
    api = RobotApi(code, rospy.get_param('~api_base_url', 'https://34.172.236.168:8081/api/v1'),
                   rospy.get_param('~credentials_file', '~/.config/medibot/order_listener.json'))
    motion = Motion()
    environment = Environment(motion)
    engine = DeliveryEngine(code, api, motion, environment,
        Journal(rospy.get_param('~journal_file', '~/.ros/delivery_executor.json')),
        log=lambda message: rospy.loginfo_throttle(10, message),
        network_grace=float(rospy.get_param('~network_grace_seconds', 3.)),
        timeout=float(rospy.get_param('~goal_timeout_seconds', 300.)))
    charging = ChargeDispatch(api, environment)
    battery = BatteryMonitor(api)
    rospy.on_shutdown(charging.shutdown)
    motion.engine = engine
    rospy.on_shutdown(engine.shutdown)
    timer = rospy.Timer(rospy.Duration(.1), lambda _: engine.watchdog())
    wake = threading.Event()
    rospy.Subscriber('/robot/orders', String, lambda _: wake.set())
    rospy.loginfo('Delivery executor ready; uses %s; no direct velocity publisher', motion.action)
    motion_lock = open(os.path.expanduser('~/.ros/robot_motion_dispatch.lock'), 'a')
    while not rospy.is_shutdown():
        try:
            if charging.step(engine.job is not None):
                wake.wait(.5)
                continue
            try:
                fcntl.flock(motion_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError:
                wake.wait(.5)
                continue
            try:
                if engine.job is None and battery.blocks_new_delivery():
                    ready, _ = environment.ready()
                    with motion.lock:
                        docked = motion.state is not None and motion.state.message.strip().lower() == 'docked'
                    if ready and not docked:
                        battery.request_charge()
                else:
                    engine.step()
            finally:
                fcntl.flock(motion_lock, fcntl.LOCK_UN)
        except Exception as error:
            rospy.logwarn_throttle(10, 'Delivery API/dispatch unavailable: %s', type(error).__name__)
        wake.wait(.5)
        wake.clear()
    timer.shutdown()


if __name__ == '__main__': main()
