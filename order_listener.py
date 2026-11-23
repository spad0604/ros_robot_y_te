#!/usr/bin/env python3
"""Apply selected robot maps and expose new delivery tasks on a ROS topic.

Map selections are applied through localize_manager/start_localization. Order
events are published as JSON only; this node does not call a movement action,
publish velocity commands, or change task status on the API.
"""

import json
import os
import stat
import time
from collections import deque
from urllib.error import HTTPError, URLError
from urllib.parse import quote, urlencode
from urllib.request import Request, urlopen

import rospy
from localize_manager.srv import SaveMap, StartLocalization, StartMapping, Stop
from std_msgs.msg import String


DEFAULT_API_BASE_URL = "https://34.172.236.168:8081/api/v1"
MAX_REMEMBERED_TASKS = 5000


class ApiError(RuntimeError):
    def __init__(self, status):
        super().__init__("API returned HTTP %s" % status)
        self.status = status


class RobotOrderListener:
    def __init__(self):
        self.api_base = str(
            rospy.get_param("~api_base_url", DEFAULT_API_BASE_URL)
        ).rstrip("/")
        self.robot_code = str(rospy.get_param("~robot_code", "MEDIBOT-01"))
        self.poll_interval = max(
            1.0, float(rospy.get_param("~poll_interval_seconds", 2.0))
        )
        self.timeout = max(
            2.0, float(rospy.get_param("~request_timeout_seconds", 8.0))
        )
        self.credentials_file = os.path.expanduser(
            str(
                rospy.get_param(
                    "~credentials_file",
                    "~/.config/medibot/order_listener.json",
                )
            )
        )
        self.state_file = os.path.expanduser(
            str(
                rospy.get_param(
                    "~state_file", "~/.ros/order_listener_seen.json"
                )
            )
        )
        self.orders_topic = str(
            rospy.get_param("~orders_topic", "/robot/orders")
        )
        self.map_selection_service = str(
            rospy.get_param(
                "~map_selection_service",
                "/localize_manager/start_localization",
            )
        )
        self.map_selection_state_file = os.path.expanduser(
            str(
                rospy.get_param(
                    "~map_selection_state_file",
                    "~/.ros/order_listener_map_selection.json",
                )
            )
        )
        self.mapping_command_state_file = os.path.expanduser(
            str(
                rospy.get_param(
                    "~mapping_command_state_file",
                    "~/.ros/order_listener_mapping_command.json",
                )
            )
        )
        self.start_mapping_service = str(
            rospy.get_param("~start_mapping_service", "/localize_manager/start_mapping")
        )
        self.save_map_service = str(
            rospy.get_param("~save_map_service", "/localize_manager/save_map")
        )
        self.stop_service = str(
            rospy.get_param("~stop_service", "/localize_manager/stop")
        )
        self.localize_state_topic = str(
            rospy.get_param("~localize_state_topic", "/localize_manager/state")
        )
        if not self.api_base.startswith(("https://", "http://")):
            raise ValueError("~api_base_url must be an HTTP(S) URL")

        self.username, self.password = self._read_credentials()
        self.access_token = None
        self.refresh_token = None
        self.seen_ids = set()
        self.seen_order = deque()
        self.map_selection_ack = self._load_map_selection_ack()
        self.mapping_command_ack = self._load_mapping_command_ack()
        self.active_map_name = None
        self.active_map_id = None
        self._load_seen_ids()
        self.publisher = rospy.Publisher(
            self.orders_topic, String, queue_size=100, latch=True
        )
        rospy.on_shutdown(self.logout)
        rospy.loginfo(
            "Order listener ready for %s; polling %s every %.1f s; "
            "publishing received orders to %s and listening for map selections",
            self.robot_code,
            self.api_base,
            self.poll_interval,
            self.orders_topic,
        )

    def _read_credentials(self):
        with open(self.credentials_file, "r", encoding="utf-8") as source:
            credentials = json.load(source)
        mode = stat.S_IMODE(os.stat(self.credentials_file).st_mode)
        if mode & 0o077:
            rospy.logwarn(
                "Order listener credentials file should be mode 600: %s",
                self.credentials_file,
            )
        username = str(credentials.get("username", "")).strip()
        password = str(credentials.get("password", ""))
        if not username or not password:
            raise ValueError("Order listener credentials are incomplete")
        return username, password

    def _load_seen_ids(self):
        try:
            with open(self.state_file, "r", encoding="utf-8") as source:
                saved = json.load(source)
            ids = saved.get("seenTaskIds", []) if isinstance(saved, dict) else []
            for task_id in ids[-MAX_REMEMBERED_TASKS:]:
                value = str(task_id)
                self.seen_ids.add(value)
                self.seen_order.append(value)
        except FileNotFoundError:
            return
        except (OSError, ValueError, TypeError) as error:
            rospy.logwarn("Could not read order listener state: %s", error)

    def _save_seen_ids(self):
        directory = os.path.dirname(self.state_file)
        os.makedirs(directory, mode=0o700, exist_ok=True)
        temporary = self.state_file + ".tmp"
        with open(temporary, "w", encoding="utf-8") as target:
            json.dump({"seenTaskIds": list(self.seen_order)}, target)
        os.chmod(temporary, 0o600)
        os.replace(temporary, self.state_file)

    def _load_map_selection_ack(self):
        try:
            with open(self.map_selection_state_file, "r", encoding="utf-8") as source:
                state = json.load(source)
            if isinstance(state, dict) and state.get("requestId"):
                return state
        except FileNotFoundError:
            pass
        except (OSError, ValueError, TypeError) as error:
            rospy.logwarn("Could not read map selection state: %s", error)
        return None

    def _save_map_selection_ack(self, state):
        directory = os.path.dirname(self.map_selection_state_file)
        os.makedirs(directory, mode=0o700, exist_ok=True)
        temporary = self.map_selection_state_file + ".tmp"
        with open(temporary, "w", encoding="utf-8") as target:
            json.dump(state, target, ensure_ascii=False)
        os.chmod(temporary, 0o600)
        os.replace(temporary, self.map_selection_state_file)

    def _load_mapping_command_ack(self):
        try:
            with open(self.mapping_command_state_file, "r", encoding="utf-8") as source:
                state = json.load(source)
            if isinstance(state, dict) and state.get("requestId"):
                return state
        except FileNotFoundError:
            pass
        except (OSError, ValueError, TypeError) as error:
            rospy.logwarn("Could not read mapping command state: %s", error)
        return None

    def _save_mapping_command_ack(self, state):
        directory = os.path.dirname(self.mapping_command_state_file)
        os.makedirs(directory, mode=0o700, exist_ok=True)
        temporary = self.mapping_command_state_file + ".tmp"
        with open(temporary, "w", encoding="utf-8") as target:
            json.dump(state, target, ensure_ascii=False)
        os.chmod(temporary, 0o600)
        os.replace(temporary, self.mapping_command_state_file)

    def _request(self, path, method="GET", body=None, authenticated=True):
        headers = {"Accept": "application/json"}
        data = None
        if body is not None:
            data = json.dumps(body, separators=(",", ":")).encode("utf-8")
            headers["Content-Type"] = "application/json"
        if authenticated and self.access_token:
            headers["Authorization"] = "Bearer " + self.access_token
        request = Request(
            self.api_base + path,
            data=data,
            headers=headers,
            method=method,
        )
        try:
            with urlopen(request, timeout=self.timeout) as response:
                raw = response.read()
                return json.loads(raw.decode("utf-8")) if raw else None
        except HTTPError as error:
            raise ApiError(error.code) from None

    def _authenticated_request(self, path, method="GET", body=None):
        try:
            return self._request(path, method=method, body=body)
        except ApiError as error:
            if error.status != 401:
                raise
            self._refresh_or_login()
            return self._request(path, method=method, body=body)

    def _login(self):
        response = self._request(
            "/auth/login",
            method="POST",
            body={"username": self.username, "password": self.password},
            authenticated=False,
        )
        data = (response or {}).get("data", {})
        self.access_token = data.get("accessToken")
        self.refresh_token = data.get("refreshToken")
        if not self.access_token:
            raise RuntimeError("Robot API login returned no access token")
        rospy.loginfo("Order listener authenticated as robot service account")

    def _refresh_or_login(self):
        if self.refresh_token:
            try:
                response = self._request(
                    "/auth/refresh",
                    method="POST",
                    body={"refreshToken": self.refresh_token},
                    authenticated=False,
                )
                data = (response or {}).get("data", {})
                self.access_token = data.get("accessToken")
                self.refresh_token = data.get("refreshToken")
                if self.access_token:
                    return
            except ApiError as error:
                if error.status not in (400, 401, 403):
                    raise
        self._login()

    def _get_created_tasks(self):
        path = "/tasks?" + urlencode({"status": "CREATED", "size": 100})
        return self._authenticated_request(path)

    def poll_map_selection(self):
        robot_code = quote(self.robot_code, safe="")
        path = "/robots/" + robot_code + "/map-selection"
        response = self._authenticated_request(path) or {}
        selection = response.get("data")
        if not isinstance(selection, dict):
            self.active_map_name = None
            self.active_map_id = None
            return

        request_id = str(selection.get("requestId", ""))
        map_name = str(selection.get("mapName", "")).strip()
        map_id = str(selection.get("mapId") or "").strip()
        status = str(selection.get("status", "")).upper()
        if not request_id or not map_name or not map_id:
            raise ValueError("Map selection endpoint returned an invalid request")
        if status == "APPLIED":
            self.active_map_name = map_name
            self.active_map_id = map_id
            return
        if status == "FAILED":
            self.active_map_name = None
            self.active_map_id = None
            return
        if status != "PENDING":
            return

        saved_ack = self.map_selection_ack
        if (saved_ack is not None
                and saved_ack.get("requestId") == request_id
                and str(saved_ack.get("mapId") or "") == map_id):
            result = saved_ack
        else:
            rospy.loginfo(
                "Applying map selection %s for %s via %s",
                map_name,
                self.robot_code,
                self.map_selection_service,
            )
            rospy.wait_for_service(
                self.map_selection_service,
                timeout=self.timeout,
            )
            start_localization = rospy.ServiceProxy(
                self.map_selection_service,
                StartLocalization,
            )
            response = start_localization(map_name)
            result = {
                "requestId": request_id,
                "mapId": map_id,
                "mapName": map_name,
                "success": bool(response.success),
                "message": str(response.message)[:900],
            }
            self.map_selection_ack = result
            self._save_map_selection_ack(result)

        result_path = path + "/" + quote(request_id, safe="") + "/result"
        self._authenticated_request(
            result_path,
            method="POST",
            body={"success": bool(result.get("success")), "message": str(result.get("message", ""))},
        )
        if result.get("success"):
            self.active_map_name = map_name
            self.active_map_id = map_id
            rospy.loginfo("Localization started with map %s", map_name)
        else:
            self.active_map_name = None
            self.active_map_id = None
            rospy.logerr("Map selection %s failed: %s", map_name, result.get("message", ""))

    def poll_mapping_command(self):
        robot_code = quote(self.robot_code, safe="")
        path = "/robots/" + robot_code + "/mapping-command"
        response = self._authenticated_request(path) or {}
        command = response.get("data")
        if not isinstance(command, dict):
            return

        request_id = str(command.get("requestId", ""))
        action = str(command.get("action", "")).upper()
        map_name = str(command.get("mapName") or "").strip()
        status = str(command.get("status", "")).upper()
        if not request_id or action not in ("START_MAPPING", "SAVE_MAP", "STOP_MAPPING"):
            raise ValueError("Mapping command endpoint returned an invalid command")
        if status != "PENDING":
            if status == "APPLIED" and action in ("START_MAPPING", "SAVE_MAP", "STOP_MAPPING"):
                self.active_map_name = None
                self.active_map_id = None
            return

        saved_ack = self.mapping_command_ack
        if saved_ack is not None and saved_ack.get("requestId") == request_id:
            result = saved_ack
        else:
            rospy.loginfo(
                "Applying mapping command %s for %s",
                action,
                self.robot_code,
            )
            success = False
            message = ""
            if action == "START_MAPPING":
                state_message = rospy.wait_for_message(
                    self.localize_state_topic,
                    String,
                    timeout=self.timeout,
                )
                current_state = state_message.data.strip().upper()
                if current_state == "MAPPING":
                    success = True
                    message = "Mapping already active"
                else:
                    rospy.wait_for_service(self.start_mapping_service, timeout=self.timeout)
                    service = rospy.ServiceProxy(self.start_mapping_service, StartMapping)
                    response = service()
                    success = bool(response.success)
                    message = str(response.message)
            elif action == "SAVE_MAP":
                if not map_name:
                    raise ValueError("SAVE_MAP command has no mapName")
                rospy.wait_for_service(self.save_map_service, timeout=self.timeout)
                service = rospy.ServiceProxy(self.save_map_service, SaveMap)
                response = service(map_name)
                success = bool(response.success)
                message = str(response.message)
            else:
                state_message = rospy.wait_for_message(
                    self.localize_state_topic,
                    String,
                    timeout=self.timeout,
                )
                current_state = state_message.data.strip().upper()
                if current_state != "MAPPING":
                    message = "Refusing to stop: localize manager state is " + (current_state or "unknown")
                else:
                    rospy.wait_for_service(self.stop_service, timeout=self.timeout)
                    service = rospy.ServiceProxy(self.stop_service, Stop)
                    response = service()
                    success = bool(response.success)
                    message = str(response.message)
            result = {
                "requestId": request_id,
                "action": action,
                "mapName": map_name or None,
                "success": success,
                "message": message[:900],
            }
            self.mapping_command_ack = result
            self._save_mapping_command_ack(result)

        result_path = path + "/" + quote(request_id, safe="") + "/result"
        self._authenticated_request(
            result_path,
            method="POST",
            body={
                "success": bool(result.get("success")),
                "message": str(result.get("message", "")),
            },
        )
        if result.get("success") and action in ("START_MAPPING", "STOP_MAPPING"):
            self.active_map_name = None
            self.active_map_id = None
        if result.get("success"):
            rospy.loginfo("Mapping command %s applied: %s", action, result.get("message", ""))
        else:
            rospy.logerr("Mapping command %s failed: %s", action, result.get("message", ""))

    def poll_once(self):
        if not self.access_token:
            self._refresh_or_login()
        response = self._get_created_tasks() or {}
        tasks = response.get("data", [])
        if not isinstance(tasks, list):
            raise ValueError("Task endpoint returned an invalid list")

        new_tasks = [
            task
            for task in tasks
            if isinstance(task, dict)
            and task.get("type") == "DELIVERY"
            and task.get("status") == "CREATED"
            and task.get("id") is not None
            and str(task["id"]) not in self.seen_ids
            and (task.get("robotCode") is None or task.get("robotCode") == self.robot_code)
            and (
                task.get("mapId") is None
                or (
                    task.get("robotCode") == self.robot_code
                    and self.active_map_id is not None
                    and str(task.get("mapId")) == self.active_map_id
                )
            )
        ]
        new_tasks.sort(key=lambda task: str(task.get("createdAt", "")))
        for task in new_tasks:
            task_id = str(task["id"])
            event = {
                "event": "NEW_ORDER",
                "robotCode": self.robot_code,
                "mapName": self.active_map_name,
                "task": task,
            }
            message = String()
            message.data = json.dumps(
                event, ensure_ascii=False, separators=(",", ":")
            )
            self.publisher.publish(message)
            rospy.loginfo(
                "Received delivery order %s; published to %s",
                task_id,
                self.orders_topic,
            )
            self.seen_ids.add(task_id)
            self.seen_order.append(task_id)
            while len(self.seen_order) > MAX_REMEMBERED_TASKS:
                self.seen_ids.discard(self.seen_order.popleft())
            self._save_seen_ids()

    def run(self):
        retry_delay = self.poll_interval
        while not rospy.is_shutdown():
            try:
                if not self.access_token:
                    self._refresh_or_login()
                self.poll_mapping_command()
                self.poll_map_selection()
            except (
                ApiError,
                URLError,
                OSError,
                ValueError,
                RuntimeError,
                rospy.ROSException,
                rospy.ServiceException,
            ) as error:
                rospy.logwarn_throttle(
                    30,
                    "Map selection poll failed; retrying: %s",
                    error,
                )
            try:
                self.poll_once()
                retry_delay = self.poll_interval
            except (ApiError, URLError, OSError, ValueError, RuntimeError) as error:
                rospy.logwarn_throttle(
                    30,
                    "Order listener poll failed; retrying: %s",
                    error,
                )
                retry_delay = min(max(self.poll_interval, retry_delay * 2.0), 30.0)
            try:
                rospy.sleep(retry_delay)
            except rospy.ROSInterruptException:
                return

    def logout(self):
        if not self.refresh_token:
            return
        try:
            self._request(
                "/auth/logout",
                method="POST",
                body={"refreshToken": self.refresh_token},
                authenticated=False,
            )
            self.refresh_token = None
        except (ApiError, URLError, OSError, ValueError):
            pass


def main():
    rospy.init_node("order_listener", anonymous=False)
    try:
        RobotOrderListener().run()
    except (OSError, ValueError) as error:
        rospy.logfatal("Order listener configuration error: %s", error)
        raise


if __name__ == "__main__":
    main()
