#!/usr/bin/env python3
"""Relay low-rate person crops and Follow Me selection over the authenticated STOMP socket.

This bridge exchanges candidate images, selection commands and motion state.
Follow Me motion is owned by follow_me_motion.py, which uses moving_server.
"""

import fcntl
import json
import os
import threading
import time
from urllib.request import Request, urlopen

import rospy
import websocket
from std_msgs.msg import String


class FollowMeBridge:
    def __init__(self):
        self.code = rospy.get_param("~robot_code", "MEDIBOT-01")
        self.api = rospy.get_param(
            "~api_base_url", "https://34.172.236.168:8081/api/v1"
        ).rstrip("/")
        self.ws_url = rospy.get_param("~ws_url", "wss://34.172.236.168:8081/ws")
        self.credentials = os.path.expanduser(
            rospy.get_param(
                "~credentials_file", "~/.config/medibot/order_listener.json"
            )
        )
        self.lock = threading.RLock()
        self.ws = None
        self.connected = False
        self.buffer = ""
        self.latest_candidates = None
        self.latest_candidates_at = 0.0
        self.latest_state = None
        self.selected_ids = set()
        self.command_pub = rospy.Publisher(
            "/vision/follow_command", String, queue_size=10
        )
        rospy.Subscriber(
            "/vision/person_candidates", String, self.candidates_received, queue_size=1
        )
        rospy.Subscriber(
            "/vision/follow_motion_state", String, self.state_received, queue_size=5
        )

    def send_frame(self, command, destination=None, payload=None):
        with self.lock:
            ws = self.ws
            if not self.connected or ws is None:
                return False
            headers = ""
            if destination:
                headers = "destination:%s\ncontent-type:application/json\n" % destination
            body = "" if payload is None else json.dumps(payload, separators=(",", ":"))
            try:
                ws.send(command + "\n" + headers + "\n" + body + "\x00")
                return True
            except Exception as error:
                rospy.logwarn_throttle(
                    10, "Follow Me WebSocket send failed: %s", type(error).__name__
                )
                return False

    def candidates_received(self, message):
        try:
            event = json.loads(message.data)
            if event.get("robotCode") != self.code:
                return
            candidates = event.get("candidates", [])
            if not isinstance(candidates, list) or len(candidates) > 3:
                return
            ids = {
                str(candidate.get("trackId"))
                for candidate in candidates
                if isinstance(candidate, dict) and candidate.get("trackId")
            }
            with self.lock:
                self.latest_candidates = event
                self.latest_candidates_at = time.monotonic()
                self.selected_ids = ids
            self.send_frame(
                "SEND", "/app/robots/%s/follow/candidates" % self.code, event
            )
        except (ValueError, TypeError, AttributeError) as error:
            rospy.logwarn_throttle(
                10, "Invalid Follow Me candidate frame: %s", type(error).__name__
            )

    def state_received(self, message):
        try:
            event = json.loads(message.data)
            with self.lock:
                self.latest_state = event
            outgoing = dict(event)
            outgoing["robotCode"] = self.code
            self.send_frame("SEND", "/app/robots/%s/follow/state" % self.code, outgoing)
        except (ValueError, TypeError, AttributeError) as error:
            rospy.logwarn_throttle(
                10, "Invalid Follow Me state: %s", type(error).__name__
            )

    def connected_frame(self, ws):
        with self.lock:
            self.connected = True
            candidates = self.latest_candidates
            state = self.latest_state
        rospy.loginfo("Follow Me WebSocket authenticated and connected")
        ws.send(
            "SUBSCRIBE\nid:follow-command\ndestination:/topic/robots/%s/follow/command\nack:auto\n\n\x00"
            % self.code
        )
        if candidates is not None:
            self.send_frame(
                "SEND", "/app/robots/%s/follow/candidates" % self.code, candidates
            )
        if state is not None:
            outgoing = dict(state)
            outgoing["robotCode"] = self.code
            self.send_frame(
                "SEND", "/app/robots/%s/follow/state" % self.code, outgoing
            )

    def received(self, ws, raw):
        with self.lock:
            self.buffer += raw
            frames = self.buffer.split("\x00")
            self.buffer = frames.pop()
        for frame in frames:
            frame = frame.lstrip("\n\r")
            if frame.startswith("CONNECTED\n"):
                self.connected_frame(ws)
            elif frame.startswith("ERROR"):
                rospy.logerr_throttle(10, "Follow Me STOMP connection was rejected")
                ws.close()
            elif frame.startswith("MESSAGE\n"):
                try:
                    payload = json.loads(frame.split("\n\n", 1)[1])
                    action = str(payload.get("action", "")).upper()
                    track_id = str(payload.get("trackId", ""))
                    if action not in ("ENABLE", "START", "STOP"):
                        continue
                    if action == "START":
                        with self.lock:
                            current_ids = set(self.selected_ids)
                            candidates = self.latest_candidates
                        with self.lock:
                            candidates_age = time.monotonic() - self.latest_candidates_at
                        if track_id not in current_ids or candidates_age > 5.0:
                            self.command_pub.publish(
                                String(data=json.dumps({"action": "START", "trackId": ""}))
                            )
                            continue
                    command = {"action": action, "trackId": track_id}
                    self.command_pub.publish(String(data=json.dumps(command)))
                    rospy.loginfo("Follow Me command received: %s", action)
                except (ValueError, TypeError, AttributeError, KeyError):
                    rospy.logwarn_throttle(10, "Ignoring invalid Follow Me command")

    def run(self):
        while not rospy.is_shutdown():
            try:
                with open(self.credentials, encoding="utf-8") as source:
                    stored = json.load(source)
                credentials = {key: stored[key] for key in ("username", "password")}
                request = Request(
                    self.api + "/auth/login",
                    data=json.dumps(credentials).encode("utf-8"),
                    headers={"Content-Type": "application/json"},
                )
                with urlopen(request, timeout=8) as response:
                    token = json.load(response)["data"]["accessToken"]

                def opened(socket):
                    socket.send(
                        "CONNECT\naccept-version:1.2\nheart-beat:0,0\n"
                        "Authorization:Bearer %s\n\n\x00" % token
                    )

                self.ws = websocket.WebSocketApp(
                    self.ws_url,
                    on_open=opened,
                    # websocket-client 0.53 invokes callbacks through the
                    # legacy (ws, message) adapter used by teleop_bridge.
                    on_message=lambda socket, raw: self.received(socket, raw),
                    on_error=lambda socket, error: rospy.logwarn_throttle(
                        10, "Follow Me WebSocket error: %s", type(error).__name__
                    ),
                    on_close=lambda socket, code, reason: rospy.logwarn(
                        "Follow Me WebSocket disconnected"
                    ),
                )
                # The bundled websocket-client sends an Origin by default.
                origin = rospy.get_param("~ws_origin", "https://robotyta-d5872.web.app")
                self.ws.run_forever(ping_interval=10, ping_timeout=5, origin=origin)
            except Exception as error:
                rospy.logwarn_throttle(
                    10, "Follow Me bridge disconnected: %s", type(error).__name__
                )
            finally:
                with self.lock:
                    self.connected = False
                    self.buffer = ""
                    self.ws = None
            rospy.sleep(2)


if __name__ == "__main__":
    os.makedirs(os.path.expanduser("~/.ros"), exist_ok=True)
    singleton = open(os.path.expanduser("~/.ros/follow_me_bridge.lock"), "w")
    fcntl.flock(singleton, fcntl.LOCK_EX | fcntl.LOCK_NB)
    websocket.setdefaulttimeout(5)
    rospy.init_node("follow_me_bridge")
    FollowMeBridge().run()
