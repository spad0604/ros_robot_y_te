"""Monitor discharge energy and request charging through the normal command queue."""
import json
import math
import statistics
import threading
import time
from collections import deque
from urllib.error import HTTPError

import rospy
from std_msgs.msg import Float32, Float32MultiArray, String


class BatteryMonitor:
    def __init__(self, api):
        self.api = api
        self.enabled = rospy.get_param('~auto_charge_enabled', False)
        self.point_name = rospy.get_param('~auto_charge_point_name', '').strip()
        self.low_voltage = float(rospy.get_param('~auto_charge_low_voltage', 10.5))
        self.recovery_voltage = float(rospy.get_param('~auto_charge_recovery_voltage', 0))
        self.low_duration = max(5., float(rospy.get_param('~auto_charge_low_duration', 30.)))
        self.freshness = max(1., float(rospy.get_param('~battery_sample_timeout', 5.)))
        self.configured = (bool(self.point_name) and 0 < self.low_voltage < self.recovery_voltage <= 100)
        if self.enabled and not self.configured:
            rospy.logerr('Auto charge disabled: configure a saved point and pack-specific low/recovery voltages')
        self.lock = threading.Lock()
        self.voltages = deque(maxlen=9)
        self.voltage = None
        self.last_voltage_at = None
        self.low_since = None
        self.last_current_at = None
        self.esp32_current = None
        self.esp32_discharging_sign = int(rospy.get_param('~esp32_discharging_sign', 0))
        self.previous_power = None
        self.watt_hours = None
        self.attempted = False
        self.next_attempt = 0.
        self.status = rospy.Publisher('/robot/energy_state', String, queue_size=1, latch=True)
        self.voltage_sub = rospy.Subscriber(
            '/esp32/sensor_data', Float32MultiArray, self._voltage, queue_size=1)
        current_topic = rospy.get_param('~battery_discharge_current_topic', '').strip()
        if current_topic:
            self.current_sub = rospy.Subscriber(current_topic, Float32, self._current, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(2), self._publish)

    def _voltage(self, message):
        labels = message.layout.dim[0].label.split(',') if message.layout.dim else []
        index = labels.index('battery_voltage') if 'battery_voltage' in labels else 4
        if index >= len(message.data):
            return
        value = float(message.data[index])
        if not 0 < value <= 100:
            return
        now = time.monotonic()
        raw_current = None
        if 'current' in labels:
            index = labels.index('current')
            if index < len(message.data) and math.isfinite(message.data[index]):
                raw_current = float(message.data[index])
        with self.lock:
            self.esp32_current = raw_current
            if self.last_voltage_at is None or now - self.last_voltage_at > self.freshness:
                self.voltages.clear()
                self.low_since = None
            self.last_voltage_at = now
            self.voltages.append(value)
            self.voltage = statistics.median(self.voltages)
            if raw_current is not None and self.esp32_discharging_sign in (-1, 1):
                self._integrate_current(self.esp32_discharging_sign * raw_current, now)
            if not 0 < self.low_voltage <= 100 or len(self.voltages) < 3:
                return
            if self.configured and self.voltage >= self.recovery_voltage:
                self.attempted = False
            if self.voltage > self.low_voltage:
                self.low_since = None
            elif self.low_since is None:
                self.low_since = now

    def _current(self, message):
        amps = float(message.data)
        if not math.isfinite(amps):
            return
        now = time.monotonic()
        with self.lock:
            if self.esp32_discharging_sign not in (-1, 1):
                self._integrate_current(amps, now)

    def _integrate_current(self, amps, now):
        if not self._fresh(now):
            self.previous_power = None
            return
        power = self.voltage * max(0., amps)
        if self.watt_hours is None:
            self.watt_hours = 0.
        if self.previous_power is not None:
            elapsed = now - self.last_current_at
            if 0 < elapsed <= self.freshness:
                self.watt_hours += (self.previous_power + power) * elapsed / 7200.
        self.previous_power = power
        self.last_current_at = now

    def _fresh(self, now):
        return self.last_voltage_at is not None and now - self.last_voltage_at <= self.freshness

    def _low(self, now):
        return bool(self._fresh(now) and self.low_since is not None
                    and now - self.low_since >= self.low_duration)

    def blocks_new_delivery(self):
        with self.lock:
            return self.enabled and self.configured and self._low(time.monotonic())

    def request_charge(self):
        now = time.monotonic()
        with self.lock:
            if not (self.enabled and self.configured and self._low(now)):
                return
            if self.attempted or now < self.next_attempt:
                return
            self.next_attempt = now + 30.
        try:
            path = self.api.root.rsplit('/', 1)[0] + '/charge-command/auto'
            self.api.call(path, {'pointName': self.point_name})
        except (HTTPError, OSError) as error:
            rospy.logwarn_throttle(30, 'Low battery auto-charge request failed: %s', getattr(error, 'code', type(error).__name__))
            return
        with self.lock:
            self.attempted = True
        rospy.logwarn('Low battery: queued return to saved point %s', self.point_name)

    def _publish(self, _event):
        now = time.monotonic()
        with self.lock:
            fresh = self._fresh(now)
            current_fresh = self.last_current_at is not None and now - self.last_current_at <= self.freshness
            payload = {
                'batteryVoltageV': self.voltage if fresh else None,
                'esp32CurrentRaw': self.esp32_current if fresh else None,
                'dischargePowerW': self.previous_power if fresh and current_fresh else None,
                'dischargeEnergyWhSinceStart': self.watt_hours,
                'lowBattery': self._low(now),
                'lowBatteryThresholdV': self.low_voltage,
                'autoChargeEnabled': bool(self.enabled and self.configured),
                'autoChargeRequested': self.attempted,
                'chargePointName': self.point_name,
            }
        self.status.publish(String(data=json.dumps(payload, allow_nan=False)))
