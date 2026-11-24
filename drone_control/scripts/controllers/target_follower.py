#!/usr/bin/env python3
"""
drone_control/scripts/controllers/target_follower.py
Single control owner: converts tracked targets into MAVROS position setpoints.
"""

import math
import time

import rospy
from geometry_msgs.msg import PoseStamped, Twist
from mavros_msgs.msg import PositionTarget, State
from mavros_msgs.srv import CommandBool, SetMode
from std_msgs.msg import String

from drone_control.msg import MissionStatus, SafetyStatus, TrackedTargets


class TargetFollower:
    """Follow the freshest tracked target and own the only control setpoint."""

    def __init__(self):
        rospy.init_node('target_follower', anonymous=False)

        self.rate_hz = float(rospy.get_param('~rate', 20.0))
        self.standoff = float(rospy.get_param('tracking/standoff_distance', 8.0))
        self.altitude = float(rospy.get_param('tracking/follow_altitude', 5.0))
        self.max_speed = float(rospy.get_param('dynamics/constraints/max_velocity', 5.0))
        self.min_alt = float(rospy.get_param('dynamics/safety_margins/min_altitude', 2.0))
        self.max_alt = float(rospy.get_param('dynamics/safety_margins/max_altitude', 50.0))
        self.target_timeout = float(rospy.get_param('~target_timeout', 2.0))
        self.offboard_settle = float(rospy.get_param('~offboard_settle', 2.0))
        self.manual_timeout = float(rospy.get_param('~manual_timeout', 0.6))
        self.max_vertical = float(rospy.get_param('~max_vertical_speed', 2.0))
        # PX4 in this SITL build barely acts on velocity-only setpoints, so
        # joystick commands go through the same position path that flies
        # pursuit: a carrot this many seconds ahead, plus velocity ff.
        self.manual_lookahead = float(
            rospy.get_param('~manual_lookahead', 1.0))
        self.altitude = max(self.min_alt, min(self.altitude, self.max_alt))
        # Last line of defense against a bad vision projection or a joystick
        # held outward: never publish a setpoint beyond the safety fence.
        self.geofence_enabled = bool(
            rospy.get_param('safety/geofence/enabled', True))
        self.geofence_radius = float(
            rospy.get_param('safety/geofence/radius', 100.0))
        self.fence_margin = float(rospy.get_param('~fence_margin', 5.0))
        self._fence_read = 0.0

        self.pose = None
        self.state = None
        self.connected_since = None
        self.target = None
        self.target_stamp = 0.0
        self.emergency = False
        self.mission_active = False
        self.last_mission = 0.0
        self.last_mode_request = 0.0
        self.last_arm_request = 0.0
        self.last_status = 0.0
        self.manual = None
        self.manual_stamp = 0.0
        self.manual_yaw = None
        self.manual_dest = None
        self.last_tick = 0.0

        self.sp_pub = rospy.Publisher(
            'mavros/setpoint_raw/local', PositionTarget, queue_size=10
        )
        self.status_pub = rospy.Publisher(
            'target_follower/status', String, queue_size=10
        )

        rospy.Subscriber('mavros/local_position/pose', PoseStamped, self._on_pose)
        rospy.Subscriber('mavros/state', State, self._on_state)
        rospy.Subscriber('tracked_targets', TrackedTargets, self._on_targets)
        rospy.Subscriber('safety_status', SafetyStatus, self._on_safety)
        rospy.Subscriber('mission_status', MissionStatus, self._on_mission)
        rospy.Subscriber('manual_control', Twist, self._on_manual)

        self.set_mode = rospy.ServiceProxy('mavros/set_mode', SetMode)
        self.arm = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)

        self.timer = rospy.Timer(
            rospy.Duration(1.0 / self.rate_hz), self._tick
        )
        rospy.loginfo(
            "Target follower started: standoff=%.1fm altitude=%.1fm",
            self.standoff, self.altitude
        )

    def _on_pose(self, msg):
        self.pose = msg.pose

    def _on_state(self, msg):
        self.state = msg

    def _on_targets(self, msg):
        if msg.targets:
            self.target = msg.targets[0]
            self.target_stamp = time.time()

    def _on_safety(self, msg):
        self.emergency = msg.emergency_active

    def _on_mission(self, msg):
        self.last_mission = time.time()
        self.mission_active = str(msg.state).lower() in ('searching', 'tracking')

    def _on_manual(self, msg):
        self.manual = msg
        self.manual_stamp = time.time()

    def _manual_active(self, now):
        return (self.manual is not None
                and now - self.manual_stamp <= self.manual_timeout)

    def _current_enu(self):
        if self.pose is None:
            return [0.0, 0.0, self.altitude]
        return [self.pose.position.x, self.pose.position.y, self.pose.position.z]

    def _desired_enu(self, now):
        cur = self._current_enu()
        target_fresh = (
            self.target is not None
            and (now - self.target_stamp) <= self.target_timeout
        )

        if self.emergency or not self.mission_active or not target_fresh:
            dest = [cur[0], cur[1], self.altitude]
        else:
            tx = self.target.position.x
            ty = self.target.position.y
            vx = self.target.velocity.x
            vy = self.target.velocity.y
            speed = math.hypot(vx, vy)
            if speed > 0.2:
                # Trail the target along its heading so the standoff point
                # keeps moving; a radial-only hold parks the drone whenever
                # the target circles inside the standoff ring.
                dest = [
                    tx - vx / speed * self.standoff,
                    ty - vy / speed * self.standoff,
                    self.altitude,
                ]
            else:
                dx, dy = tx - cur[0], ty - cur[1]
                distance = math.hypot(dx, dy)
                if distance > self.standoff and distance > 1e-6:
                    dest = [
                        tx - dx / distance * self.standoff,
                        ty - dy / distance * self.standoff,
                        self.altitude,
                    ]
                else:
                    dest = [cur[0], cur[1], self.altitude]

        # Cap the error fed to PX4's position loop, not the per-tick step:
        # a step-sized carrot only commands XY_P * step (~0.2 m/s) and the
        # aircraft never builds speed.
        reach = self.max_speed
        delta = [dest[i] - cur[i] for i in range(3)]
        norm = math.sqrt(sum(v * v for v in delta))
        if norm > reach:
            dest = [cur[i] + delta[i] / norm * reach for i in range(3)]
        dest[2] = max(self.min_alt, min(dest[2], self.max_alt))
        return dest

    def _publish(self, enu):
        sp = PositionTarget()
        sp.header.stamp = rospy.Time.now()
        sp.header.frame_id = 'map'
        sp.coordinate_frame = 1  # FRAME_LOCAL_NED (fields below are still ENU)
        sp.type_mask = 0b0000111111111000  # position only
        # MAVROS setpoint_raw converts the message ENU->NED itself; sending NED
        # here flips z and makes the aircraft descend into the ground.
        sp.position.x = enu[0]  # east
        sp.position.y = enu[1]  # north
        sp.position.z = enu[2]  # up
        sp.yaw = 0.0
        self.sp_pub.publish(sp)

    def _current_yaw(self):
        if self.pose is None:
            return 0.0
        q = self.pose.orientation
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                          1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def _publish_manual(self, dest, vel, yaw):
        """Joystick setpoint: position carrot only.

        The destination is integrated from the joystick at the commanded
        speed, so it keeps moving while the aircraft chases it. Sending
        velocity alongside position shifts PX4's equilibrium and stalls
        the aircraft, so the velocity field stays ignored.
        """
        sp = PositionTarget()
        sp.header.stamp = rospy.Time.now()
        sp.header.frame_id = 'map'
        sp.coordinate_frame = 1
        sp.type_mask = ((1 << 3) | (1 << 4) | (1 << 5)   # ignore velocity
                        | (1 << 6) | (1 << 7) | (1 << 8)  # ignore accel
                        | (1 << 9) | (1 << 11))         # force, yaw_rate
        sp.position.x = dest[0]
        sp.position.y = dest[1]
        sp.position.z = dest[2]
        sp.yaw = yaw
        self.sp_pub.publish(sp)

    def _clamp_fence(self, dest):
        if not self.geofence_enabled:
            return dest
        limit = max(1.0, self.geofence_radius - self.fence_margin)
        radius = math.hypot(dest[0], dest[1])
        if radius > limit:
            k = limit / radius
            return [dest[0] * k, dest[1] * k, dest[2]]
        return dest

    def _tick(self, _event):
        now = time.time()
        dt = now - (self.last_tick or (now - 0.05))
        dt = min(max(dt, 0.01), 0.2)
        self.last_tick = now
        if now - self._fence_read > 2.0:
            self._fence_read = now
            self.geofence_enabled = bool(rospy.get_param(
                'safety/geofence/enabled', self.geofence_enabled))
            self.geofence_radius = float(rospy.get_param(
                'safety/geofence/radius', self.geofence_radius))
        if now - self.last_mission > 2.5:
            self.mission_active = False

        manual = self._manual_active(now) and not self.emergency
        if manual:
            cur = self._current_enu()
            vx = float(self.manual.linear.x)   # east
            vy = float(self.manual.linear.y)   # north
            vz = float(self.manual.linear.z)   # up
            wz = float(self.manual.angular.z)
            speed = math.hypot(vx, vy)
            if speed > self.max_speed > 0:
                vx = vx / speed * self.max_speed
                vy = vy / speed * self.max_speed
            vz = max(-self.max_vertical, min(vz, self.max_vertical))
            if cur[2] <= self.min_alt + 0.3 and vz < 0:
                vz = 0.0
            if cur[2] >= self.max_alt - 1.0 and vz > 0:
                vz = 0.0
            if self.manual_yaw is None:
                self.manual_yaw = self._current_yaw()
            self.manual_yaw += wz * dt
            look = self.manual_lookahead
            if self.manual_dest is None:
                # Re-anchor on every new manual episode; otherwise the
                # destination walks along with the aircraft and nothing
                # besides PX4's position error ever moves it.
                self.manual_dest = [
                    cur[0] + vx * look,
                    cur[1] + vy * look,
                    max(self.min_alt,
                        min(cur[2] + vz * look, self.max_alt)),
                ]
            else:
                self.manual_dest[0] += vx * dt
                self.manual_dest[1] += vy * dt
                self.manual_dest[2] = max(
                    self.min_alt,
                    min(self.manual_dest[2] + vz * dt, self.max_alt))
            self._publish_manual(self._clamp_fence(self.manual_dest),
                                 [vx, vy, vz], self.manual_yaw)
        else:
            self.manual_yaw = None
            self.manual_dest = None
            self._publish(self._clamp_fence(self._desired_enu(now)))

        if self.state is None or not self.state.connected:
            self.connected_since = None
            return
        if self.connected_since is None:
            self.connected_since = now
        if now - self.last_status >= 1.0:
            self.last_status = now
            self.status_pub.publish(
                f"mode={self.state.mode} armed={self.state.armed} "
                f"emergency={self.emergency} target={self.target is not None} "
                f"manual={manual}"
            )
        if now - self.connected_since < self.offboard_settle:
            return
        if self.emergency or not (self.mission_active or manual):
            return

        if self.state.mode != 'OFFBOARD':
            if now - self.last_mode_request > 2.0:
                self.last_mode_request = now
                try:
                    self.set_mode(0, 'OFFBOARD')
                except (rospy.ServiceException, rospy.ROSException) as exc:
                    rospy.logwarn_throttle(5.0, 'set_mode failed: %s', exc)

        elif not self.state.armed:
            if now - self.last_arm_request > 2.0:
                self.last_arm_request = now
                try:
                    res = self.arm(True)
                    if not res.success:
                        rospy.logwarn_throttle(
                            5.0, 'arm rejected (result=%d)', res.result
                        )
                except (rospy.ServiceException, rospy.ROSException) as exc:
                    rospy.logwarn_throttle(5.0, 'arm failed: %s', exc)


if __name__ == '__main__':
    try:
        TargetFollower()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
