#!/usr/bin/env python3
"""Mapping-only teleoperation through the existing joystick mux input.
Requires websocket-client. Does not start ROS core or change motion configuration.
"""
import json
import fcntl
import math
import os
import threading
import time
from urllib.request import Request, urlopen
import rospy
import websocket
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class Bridge:
    def __init__(self):
        self.code = rospy.get_param('~robot_code', 'MEDIBOT-01')
        self.api = rospy.get_param('~api_base_url', 'https://34.172.236.168:8081/api/v1').rstrip('/')
        self.ws_url = rospy.get_param('~ws_url', 'wss://34.172.236.168:8081/ws')
        self.credentials = os.path.expanduser(rospy.get_param('~credentials_file', '~/.config/medibot/order_listener.json'))
        self.lock = threading.RLock()
        self.state, self.state_at = '', 0.
        self.auto_at = 0.
        self.deadline = 0.
        self.command = Twist()
        self.ws = None
        self.connected = False
        self.last_status = 0.
        self.active = False
        self.buffer = ''
        self.sequence = 0
        self.mux_ready = False
        self.reported_ready = None
        self.publisher = rospy.Publisher('/cmd_vel/joystick', Twist, queue_size=1, latch=False)
        rospy.Subscriber('/localize_manager/state', String, self.state_changed)
        # Any live automatic source blocks manual driving, including zero commands:
        # mux arbitration gives those sources higher priority than joystick.
        for topic in ('/cmd_vel/navi', '/cmd_vel/tracking', '/cmd_vel/docking', '/cmd_vel/emergency'):
            rospy.Subscriber(topic, Twist, self.automatic)
        rospy.on_shutdown(self.shutdown)
        self.timer = rospy.Timer(rospy.Duration(.05), self.tick)
        threading.Thread(target=self.status_loop, daemon=True).start()

    def state_changed(self, message):
        with self.lock:
            self.state, self.state_at = message.data.strip().upper(), time.monotonic()
            if self.state != 'MAPPING':
                self.deadline = 0.

    def refresh_state(self):
        # The installed manager publishes latched state only on transitions.
        # A fresh subscription must obtain that state from a live publisher;
        # an old cached MAPPING value alone must never enable driving.
        snapshot = rospy.wait_for_message('/localize_manager/state', String, timeout=.7)
        with self.lock:
            if snapshot.data.strip().upper() == self.state:
                self.state_at = time.monotonic()

    def automatic(self, _):
        with self.lock:
            self.auto_at = time.monotonic()
            self.deadline = 0.

    def ready(self):
        now = time.monotonic()
        return (self.connected and self.state == 'MAPPING' and now - self.state_at < 1.5
                and now - self.auto_at > .6 and self.mux_ready
                and self.publisher.get_num_connections() > 0)

    def frame(self, command, destination=None, body=None):
        headers = '' if destination is None else 'destination:' + destination + '\ncontent-type:application/json\n'
        data = '' if body is None else json.dumps(body, separators=(',', ':'))
        self.ws.send(command + '\n' + headers + '\n' + data + '\x00')

    def received(self, ws, raw):
        self.buffer += raw
        frames = self.buffer.split('\x00')
        self.buffer = frames.pop()
        for frame in frames:
            frame = frame.lstrip('\n\r')
            if frame.startswith('CONNECTED\n'):
                with self.lock:
                    self.connected = True
                    rospy.loginfo("Teleop WebSocket authenticated and connected")
                    self.sequence = 0
                    self.deadline = 0.
                ws.send('SUBSCRIBE\nid:teleop\ndestination:/user/queue/teleop\nack:auto\n\n\x00')
            elif frame.startswith('ERROR'):
                ws.close()
            elif frame.startswith('MESSAGE\n'):
                try:
                    data = json.loads(frame.split('\n\n', 1)[1])
                    sequence = int(data['sequence'])
                    ttl = (float(data['expiresAt']) - time.time() * 1000) / 1000
                    x, y, yaw = (float(data[k]) for k in ('x', 'y', 'yaw'))
                    with self.lock:
                        if sequence <= self.sequence:
                            continue
                        self.sequence = sequence
                        if (not self.ready() or not 0 < ttl <= .3 or
                                not all(math.isfinite(v) for v in (x, y, yaw)) or
                                math.hypot(x, y) > .20 or abs(yaw) > .40):
                            self.deadline = 0.
                            return
                        self.command = Twist()
                        self.command.linear.x, self.command.linear.y = x, y
                        self.command.angular.z = yaw
                        if x == 0 and y == 0 and yaw == 0:
                            self.deadline = 0.
                            self.publisher.publish(self.command)
                            self.active = False
                        else:
                            # Validate frame age above. Hold a fresh command long
                            # enough to bridge brief gaps in the 10 Hz stream.
                            self.deadline = time.monotonic() + .50
                except (ValueError, KeyError, TypeError):
                    with self.lock:
                        self.deadline = 0.

    def tick(self, _):
        with self.lock:
            ready = self.ready()
            if not ready:
                self.deadline = 0.
            moving = ready and time.monotonic() < self.deadline
            if moving:
                self.publisher.publish(self.command)
                self.active = True
            elif self.active:
                self.publisher.publish(Twist())
                self.active = False

    def status_loop(self):
        while not rospy.is_shutdown():
            try:
                code, _, state = rospy.get_master().getSystemState()
                expected = rospy.get_param('~mux_node', '/cmd_vel_mux')
                self.refresh_state()
                subscribers = dict(state[1]) if code == 1 else {}
                publishers = dict(state[0]) if code == 1 else {}
                with self.lock:
                    self.mux_ready = (expected in subscribers.get('/cmd_vel/joystick', [])
                                      and expected in publishers.get('/cmd_vel', []))
                    ready, connected = self.ready(), self.connected
                    robot_state = self.state if time.monotonic() - self.state_at < 1.5 else 'UNKNOWN'
                if ready != self.reported_ready:
                    rospy.loginfo("Teleop ready=%s state=%s mux=%s", ready, self.state, self.mux_ready)
                    self.reported_ready = ready
                if connected:
                    self.frame('SEND', '/app/robots/' + self.code + '/teleop/status', {'ready': ready, 'state': robot_state})
            except Exception as error:
                rospy.logwarn_throttle(10, 'Teleop readiness check failed: %s', type(error).__name__)
                with self.lock:
                    self.mux_ready = False
                    self.deadline = 0.
            time.sleep(.5)

    def shutdown(self):
        with self.lock:
            if self.active:
                self.publisher.publish(Twist())
            if self.ws:
                self.ws.close()

    def run(self):
        while not rospy.is_shutdown():
            try:
                with open(self.credentials, encoding='utf-8') as source:
                    stored = json.load(source)
                    credentials = {key: stored[key] for key in ('username', 'password')}
                req = Request(self.api + '/auth/login', data=json.dumps(credentials).encode(),
                              headers={'Content-Type': 'application/json'})
                with urlopen(req, timeout=8) as response:
                    token = json.load(response)['data']['accessToken']
                def opened(ws):
                    ws.send('CONNECT\naccept-version:1.2\nheart-beat:0,0\nAuthorization:Bearer ' + token + '\n\n\x00')
                self.ws = websocket.WebSocketApp(
                    self.ws_url, on_open=opened,
                    # Lambda keeps the callback signature compatible with Ubuntu's
                    # websocket-client 0.53 as well as current releases.
                    on_message=lambda ws, raw: self.received(ws, raw),
                    on_error=lambda ws, error: rospy.logwarn_throttle(
                        10, "Teleop WebSocket error: %s", type(error).__name__))
                # The bundled websocket-client sends an Origin by default.
                origin = rospy.get_param("~ws_origin", "https://robotyta-d5872.web.app")
                self.ws.run_forever(ping_interval=10, ping_timeout=5, origin=origin)
            except Exception as error:
                rospy.logwarn_throttle(10, 'Teleop bridge disconnected: %s', type(error).__name__)
            finally:
                with self.lock:
                    self.connected = False
                    self.deadline = 0.
                    self.buffer = ''
            rospy.sleep(2)


if __name__ == '__main__':
    os.makedirs(os.path.expanduser('~/.ros'), exist_ok=True)
    singleton = open(os.path.expanduser('~/.ros/mobile_teleop_bridge.lock'), 'w')
    fcntl.flock(singleton, fcntl.LOCK_EX | fcntl.LOCK_NB)
    websocket.setdefaulttimeout(5)
    rospy.init_node('mobile_teleop_bridge')
    Bridge().run()
