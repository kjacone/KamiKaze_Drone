#!/usr/bin/env python3
"""Fleet control plane for the multi-drone SITL harness.

fleet.yaml is the single source of truth. From it this tool:
  * reconciles one PX4 container per drone (own bridge network and netns,
    `px4 -i` so MAVLink ports don't collide inside the ROS container),
  * renders a namespaced launch (one <group ns="drone_<id>"> per drone)
    plus one mavros config per drone,
  * in `run` mode, supervises roslaunch and serves the config portal.

Subcommands:
  render   reconcile containers and render the launch files
  run      render, start roslaunch, serve the portal, apply changes live
"""

import argparse
import copy
import hashlib
import http.client
import json
import math
import os
import signal
import socket
import subprocess
import sys
import threading
import time
import urllib.parse
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import yaml

FLEET_DIR = os.path.dirname(os.path.abspath(__file__))
CONFIG_PATH = os.path.join(FLEET_DIR, 'fleet.yaml')
GENERATED_DIR = os.path.join(FLEET_DIR, 'generated')
LAUNCH_PATH = os.path.join(GENERATED_DIR, 'fleet.launch')
STATIC_DIR = os.path.join(FLEET_DIR, 'static')
STATIC_FILES = ('three.min.js', 'OrbitControls.js')
HOST_REPO_DIR = os.environ.get('HOST_REPO_DIR') or os.path.dirname(FLEET_DIR)
ROS_CONTAINER = os.environ.get('FLEET_ROS_CONTAINER', 'kamikaze_drone')
DOCKER_SOCK = os.environ.get('DOCKER_SOCK', '/var/run/docker.sock')

LABEL_FLEET = 'kamikaze.fleet'
LABEL_DRONE = 'kamikaze.drone'
LABEL_SPEC = 'kamikaze.spec'

# px4-rc.mavlink reuses the remote port for instances above 9.
MAX_DRONES = 10

# Settle time after a drone's mavros reconnects before auto-resuming it, so
# the service and pose stream are actually up.
AUTO_RESUME_SETTLE = 15.0

# Monitoring stack from docker-compose profiles. Probed by hostname (the
# portal runs on drone_net); links point at the host-mapped ports.
OBSERVABILITY = [
    {'name': 'Grafana', 'host': 'grafana', 'port': 3000,
     'path': '/api/health', 'link': 'http://localhost:3000/d/drone'},
    {'name': 'Prometheus', 'host': 'prometheus', 'port': 9090,
     'path': '/-/healthy', 'link': 'http://localhost:9090/targets'},
    {'name': 'Loki', 'host': 'loki', 'port': 3100,
     'path': '/ready', 'link': 'http://localhost:3100'},
    {'name': 'Tempo', 'host': 'tempo', 'port': 3200,
     'path': '/ready', 'link': 'http://localhost:3200'},
    {'name': 'Performance', 'host': 'performance', 'port': 8000,
     'path': '/metrics', 'link': 'http://localhost:8000/metrics'},
]

NODE_MONITORED = [
    'vehicle_state_monitor',
    'mission_manager',
    'safety_monitor',
    'command_interpreter',
]


# ---------------------------------------------------------------------------
# Docker Engine API over the unix socket (no CLI dependency)
# ---------------------------------------------------------------------------

class UnixHTTPConnection(http.client.HTTPConnection):
    def __init__(self, path, timeout=60):
        super().__init__('docker', timeout=timeout)
        self._path = path

    def connect(self):
        sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        sock.settimeout(self.timeout)
        sock.connect(self._path)
        self.sock = sock


class Docker:
    def __init__(self, path=DOCKER_SOCK):
        self.path = path

    def req(self, method, path, body=None, ok=(200, 201, 204, 304), timeout=60):
        conn = UnixHTTPConnection(self.path, timeout=timeout)
        try:
            data = None
            headers = {}
            if body is not None:
                data = json.dumps(body).encode()
                headers['Content-Type'] = 'application/json'
            conn.request(method, path, body=data, headers=headers)
            resp = conn.getresponse()
            payload = resp.read()
            if resp.status not in ok:
                raise RuntimeError(
                    f'Docker {method} {path} -> {resp.status}: {payload[:300]!r}')
            return json.loads(payload) if payload else None
        finally:
            conn.close()


def _filters(spec):
    return urllib.parse.quote(json.dumps(spec))


def ensure_network(dk, name):
    try:
        return dk.req('GET', f'/networks/{name}')
    except RuntimeError:
        pass
    dk.req('POST', '/networks/create',
           {'Name': name, 'Driver': 'bridge', 'Labels': {LABEL_FLEET: '1'}})
    return dk.req('GET', f'/networks/{name}')


def _network_member(dk, iface_id, container):
    info = dk.req('GET', f'/networks/{iface_id}')
    for member in (info.get('Containers') or {}).values():
        if member.get('Name', '').lstrip('/') == container:
            return member['IPv4Address'].split('/')[0]
    return None


def ensure_ros_attached(dk, net, container):
    """Attach the ROS container to a drone network and return its IP."""
    iface_id = net['Id']
    ip = _network_member(dk, iface_id, container)
    if ip:
        return ip
    dk.req('POST', f'/networks/{iface_id}/connect', {'Container': container})
    ip = _network_member(dk, iface_id, container)
    if not ip:
        raise RuntimeError(f'{container} did not join network {net["Name"]}')
    return ip


def find_container(dk, name):
    found = dk.req('GET', '/containers/json?all=1&filters=' +
                   _filters({'name': [name]}))
    for item in found:
        if item.get('Names') and item['Names'][0].lstrip('/') == name:
            return dk.req('GET', f'/containers/{item["Id"]}/json')
    return None


def _parse_size(value):
    if isinstance(value, int):
        return value
    units = {'k': 1 << 10, 'm': 1 << 20, 'g': 1 << 30}
    text = str(value).strip().lower()
    if text[-1] in units:
        return int(float(text[:-1]) * units[text[-1]])
    return int(text)


GZ_CAM_PORT_BASE = 9200


def _px4_spec(cfg, drone_id, network, ros_ip):
    f = cfg['fleet']
    repo = HOST_REPO_DIR
    post = os.path.join(repo, 'docker', 'px4', '4010_gz_x500_mono_cam.post')
    return {
        'Image': f['px4_image'],
        'Cmd': [ros_ip, ros_ip],
        'Entrypoint': ['/root/start.sh'],
        'Env': [
            f'PX4_INSTANCE={drone_id - 1}',
            'HEADLESS=1',
            f'GZ_CAM_PORT={GZ_CAM_PORT_BASE + drone_id - 1}',
        ],
        'HostConfig': {
            'Binds': [
                f'{os.path.join(repo, "fleet", "px4_entrypoint.sh")}:/root/start.sh:ro',
                f'{os.path.join(repo, "fleet", "gz_shim.sh")}:/usr/local/bin/gz:ro',
                f'{os.path.join(repo, "fleet", "gz_camera_bridge.py")}:/root/gz_camera_bridge.py:ro',
                f'{os.path.join(repo, "fleet", "gz_scenario.py")}:/root/gz_scenario.py:ro',
                f'{os.path.join(repo, "docker", "px4", "mono_cam_model.sdf")}:/root/px4/Tools/simulation/gz/models/mono_cam/model.sdf:ro',
                f'{post}:/root/px4/build/etc/init.d-posix/airframes/4010_gz_x500_mono_cam.post:ro',
            ],
            'ShmSize': _parse_size(f['px4_shm']),
            'NanoCpus': int(float(f['px4_cpus']) * 1e9),
            'Memory': _parse_size(f['px4_memory']),
            'RestartPolicy': {'Name': 'unless-stopped', 'MaximumRetryCount': 0},
            'LogConfig': {
                'Type': 'json-file',
                'Config': {'max-size': '10m', 'max-file': '3'},
            },
            'AutoRemove': False,
        },
        'NetworkingConfig': {'EndpointsConfig': {network: {}}},
    }


def ensure_px4(dk, cfg, drone_id, network, ros_ip):
    name = f'px4_{drone_id}'
    body = _px4_spec(cfg, drone_id, network, ros_ip)
    spec_hash = hashlib.sha1(
        json.dumps(body, sort_keys=True).encode()).hexdigest()
    body['Labels'] = {
        LABEL_FLEET: '1',
        LABEL_DRONE: str(drone_id),
        LABEL_SPEC: spec_hash,
    }

    current = find_container(dk, name)
    if current is not None:
        spec_label = (current['Config'].get('Labels') or {}).get(LABEL_SPEC)
        if spec_label == spec_hash and current['State']['Running']:
            return current
        if current['State']['Running']:
            dk.req('POST', f'/containers/{name}/stop?t=10',
                   ok=(204, 304, 404, 500))
        dk.req('DELETE', f'/containers/{name}?force=1', ok=(204, 404))

    dk.req('POST', f'/containers/create?name={name}&platform=linux/amd64',
           body, timeout=120)
    dk.req('POST', f'/containers/{name}/start', ok=(204, 304, 200))
    return dk.req('GET', f'/containers/{name}/json')


def remove_extra_drones(dk, count):
    listed = dk.req('GET', '/containers/json?all=1&filters=' +
                    _filters({'label': [f'{LABEL_FLEET}=1']}))
    for item in listed:
        labels = item.get('Labels') or {}
        try:
            drone_id = int(labels.get(LABEL_DRONE, '0'))
        except ValueError:
            continue
        if not drone_id or drone_id <= count:
            continue
        name = item['Names'][0].lstrip('/')
        dk.req('POST', f'/containers/{name}/stop?t=10',
               ok=(204, 304, 404, 500))
        dk.req('DELETE', f'/containers/{name}?force=1', ok=(204, 404))
        try:
            dk.req('POST', f'/networks/px4net_{drone_id}/disconnect',
                   {'Container': ROS_CONTAINER, 'Force': True},
                   ok=(200, 204, 403, 404))
        except RuntimeError:
            pass


def reconcile(cfg):
    count = int(cfg['fleet']['count'])
    dk = Docker()
    drones = []
    for drone_id in range(1, count + 1):
        network = f'px4net_{drone_id}'
        net = ensure_network(dk, network)
        ros_ip = ensure_ros_attached(dk, net, ROS_CONTAINER)
        info = ensure_px4(dk, cfg, drone_id, network, ros_ip)
        px4_ip = info['NetworkSettings']['Networks'][network]['IPAddress']
        drones.append({
            'id': drone_id,
            'namespace': f'drone_{drone_id}',
            'network': network,
            'container': f'px4_{drone_id}',
            'px4_ip': px4_ip,
            'ros_ip': ros_ip,
        })
    remove_extra_drones(dk, count)
    return drones


# ---------------------------------------------------------------------------
# Rendering
# ---------------------------------------------------------------------------

def _bool(value):
    return 'true' if value else 'false'


def _mavros_cfg(drone):
    local = 14540 + drone['id'] - 1
    remote = 14580 + drone['id'] - 1
    return (f'fcu_url: "udp://:{local}@{drone["px4_ip"]}:{remote}"\n'
            f'target_system_id: {drone["id"]}\n'
            'target_component_id: 1\n'
            '\n'
            'conn:\n'
            '  timeout: 10.0\n'
            '  heartbeat_rate: 1.0\n')


def _write_if_changed(path, content):
    try:
        with open(path) as handle:
            if handle.read() == content:
                return False
    except FileNotFoundError:
        pass
    with open(path, 'w') as handle:
        handle.write(content)
    return True


def _group_xml(cfg, drone):
    f = cfg['fleet']
    defaults = cfg['defaults']
    drone_id = drone['id']
    mavros_file = os.path.join(GENERATED_DIR, f'mavros_{drone_id}.yaml')

    nodes = [
        f'''    <node pkg="mavros" type="mavros_node" name="mavros"
          output="screen" respawn="true" respawn_delay="2.0">
      <rosparam file="{mavros_file}" command="load"/>
    </node>''',
        '''    <node pkg="drone_control" type="command_interpreter.py"
          name="command_interpreter" output="screen" />''',
        f'''    <node pkg="drone_control" type="mission_manager.py"
          name="mission_manager" output="screen">
      <param name="mission_config" value="$(find drone_control)/config/mission_config.yaml"/>
      <param name="auto_start" value="{_bool(f['auto_start'])}"/>
      <param name="require_mavros" value="true"/>
      <param name="target_timeout" value="2.0"/>
    </node>''',
    ]

    if f['perception_demo']:
        nodes.append(f'''    <node pkg="drone_control" type="gz_camera_bridge.py"
          name="gz_camera_bridge" output="screen" respawn="true">
      <param name="port" value="{GZ_CAM_PORT_BASE + drone_id - 1}"/>
      <param name="horizontal_fov" value="1.74"/>
    </node>

    <node pkg="drone_control" type="camera_calibration.py"
          name="camera_calibration" output="screen">
      <param name="simulation_mode" value="{_bool(f['simulation'])}"/>
    </node>

    <node pkg="drone_control" type="vision_scenario.py"
          name="vision_scenario" output="screen">
      <param name="target_class" value="car"/>
      <param name="decoy_class" value="person"/>
    </node>

    <node pkg="drone_control" type="sensor_fusion.py"
          name="sensor_fusion" output="screen">
      <param name="publish_rate" value="20"/>
      <param name="simulation_mode" value="{_bool(f['simulation'])}"/>
    </node>

    <node pkg="drone_control" type="target_tracking_controller.py"
          name="target_tracking_controller" output="screen">
      <param name="simulation_mode" value="{_bool(f['simulation'])}"/>
    </node>

    <node pkg="drone_control" type="target_follower.py"
          name="target_follower" output="screen">
      <param name="target_timeout" value="2.0"/>
    </node>''')
    else:
        nodes.append(f'''    <node pkg="drone_control" type="simulated_target.py"
          name="simulated_target" output="screen">
      <param name="scenario" value="default"/>
    </node>

    <node pkg="drone_control" type="target_follower.py"
          name="target_follower" output="screen">
      <param name="target_timeout" value="2.0"/>
    </node>''')

    nodes.append(f'''    <node pkg="drone_control" type="vehicle_state_monitor.py"
          name="vehicle_state_monitor" output="screen">
      <param name="simulation_mode" value="{_bool(f['simulation'])}"/>
    </node>''')

    if f['enable_safety']:
        nodes.append(f'''    <node pkg="drone_control" type="safety_monitor.py"
          name="safety_monitor" output="screen">
      <param name="simulation_mode" value="{_bool(f['simulation'])}"/>
    </node>''')

    nodes.append('''    <node pkg="drone_control" type="system_health_monitor.py"
          name="system_health_monitor" output="screen">
      <param name="monitoring_interval" value="5.0"/>
      <param name="critical_threshold_cpu" value="90.0"/>
      <param name="critical_threshold_memory" value="90.0"/>
      <param name="warning_threshold_cpu" value="70.0"/>
      <param name="warning_threshold_memory" value="70.0"/>
      <param name="enable_network_monitoring" value="true"/>
      <param name="enable_disk_monitoring" value="true"/>
      <param name="enable_process_monitoring" value="true"/>
    </node>''')

    monitored = [n for n in NODE_MONITORED
                 if f['enable_safety'] or n != 'safety_monitor']
    monitored_xml = ', '.join(f"'{name}'" for name in monitored)
    nodes.append(f'''    <node pkg="drone_control" type="health_checker.py"
          name="health_checker" output="screen">
      <param name="check_interval" value="5.0"/>
      <param name="timeout_threshold" value="10.0"/>
      <param name="restart_on_failure" value="false"/>
      <param name="max_restarts" value="0"/>
      <rosparam param="monitored_nodes">[{monitored_xml}]</rosparam>
    </node>''')

    params = [
        f'<param name="use_simulation" value="{_bool(f["simulation"])}"/>',
        '<param name="test_mode" value="false"/>',
        f'<param name="fleet/drone_id" value="{drone_id}"/>',
        f'<param name="tracking/standoff_distance" value="{defaults["standoff_distance"]}"/>',
        f'<param name="tracking/follow_altitude" value="{defaults["follow_altitude"]}"/>',
        f'<param name="tracking/engagement_distance" value="{defaults["engagement_distance"]}"/>',
        f'<param name="dynamics/constraints/max_velocity" value="{defaults["max_velocity"]}"/>',
        f'<param name="safety/geofence/radius" value="{defaults["geofence_radius"]}"/>',
        f'<param name="safety/geofence/height" value="{defaults["geofence_height"]}"/>',
    ]
    param_xml = '\n'.join(f'    {line}' for line in params)

    body = '\n\n'.join(nodes)
    return f'''<group ns="drone_{drone_id}">
    <rosparam file="$(find drone_control)/config/target_params.yaml" command="load"/>
    <rosparam file="$(find drone_control)/config/camera_calib.yaml" command="load" ns="camera_calib"/>
    <rosparam file="$(find drone_control)/config/flight_control.yaml" command="load"/>
    <rosparam file="$(find drone_control)/config/kalman_filter.yaml" command="load" ns="kalman_filter"/>
    <rosparam file="$(find drone_control)/config/system_params.yaml" command="load"/>
{param_xml}

{body}
</group>'''


def render(cfg, drones):
    """Render launch + mavros configs; return True if anything changed."""
    os.makedirs(GENERATED_DIR, exist_ok=True)
    changed = False
    for drone in drones:
        changed |= _write_if_changed(
            os.path.join(GENERATED_DIR, f'mavros_{drone["id"]}.yaml'),
            _mavros_cfg(drone))
    groups = '\n\n'.join(_group_xml(cfg, drone) for drone in drones)
    launch = '<?xml version="1.0"?>\n<launch>\n' + groups + '\n</launch>\n'
    changed |= _write_if_changed(LAUNCH_PATH, launch)
    return changed


# ---------------------------------------------------------------------------
# Config handling
# ---------------------------------------------------------------------------

def load_config():
    with open(CONFIG_PATH) as handle:
        return yaml.safe_load(handle)


def save_config(cfg):
    with open(CONFIG_PATH, 'w') as handle:
        yaml.safe_dump(cfg, handle, sort_keys=False, default_flow_style=False)


def _merge_config(base, updates):
    cfg = copy.deepcopy(base)
    fleet_updates = updates.get('fleet') or {}
    for key in ('simulation', 'auto_start', 'auto_resume', 'perception_demo',
                'enable_safety', 'enable_diagnostics'):
        if key in fleet_updates:
            cfg['fleet'][key] = bool(fleet_updates[key])
    if 'count' in fleet_updates:
        count = int(fleet_updates['count'])
        if not 1 <= count <= MAX_DRONES:
            raise ValueError(f'count must be 1..{MAX_DRONES}')
        cfg['fleet']['count'] = count
        if count > 3:
            print(f'[fleet] warning: count={count} exceeds what this host can '
                  'simulate reliably; expect estimator glitches', file=sys.stderr)
    if 'px4_cpus' in fleet_updates:
        px4_cpus = float(fleet_updates['px4_cpus'])
        if not math.isfinite(px4_cpus) or not 0.25 <= px4_cpus <= 4.0:
            raise ValueError('px4_cpus must be between 0.25 and 4.0')
        cfg['fleet']['px4_cpus'] = px4_cpus

    default_updates = updates.get('defaults') or {}
    for key in cfg['defaults']:
        if key in default_updates:
            value = float(default_updates[key])
            if not math.isfinite(value) or value < 0:
                raise ValueError(f'{key} must be a non-negative number')
            cfg['defaults'][key] = value
    return cfg


# ---------------------------------------------------------------------------
# Status sampling for the portal
# ---------------------------------------------------------------------------

class RosCache:
    """Latest /drone_<id>/mavros state and pose, cached in-process.

    Spawning `rostopic echo` per sample starved the fleet under load, so the
    portal keeps its own subscriptions instead.
    """

    def __init__(self):
        self.lock = threading.Lock()
        self.states = {}
        self.poses = {}
        self.safety = {}
        self.missions = {}
        self.velocities = {}
        self.targets = {}
        self.camera_msgs = {}
        self.detections = {}
        self.det_stamps = {}
        self.cam_stamps = {}
        self.subscribed = set()
        self.camera_ready = set()
        self.scenario_pubs = {}
        self.tracking_pubs = {}
        self.manual_pubs = {}
        self.overridden = set()
        self.ready = False

    def _on_state(self, msg, namespace):
        with self.lock:
            self.states[namespace] = msg

    def _on_pose(self, msg, namespace):
        with self.lock:
            self.poses[namespace] = msg

    def _on_safety(self, msg, namespace):
        with self.lock:
            self.safety[namespace] = msg

    def _on_mission(self, msg, namespace):
        with self.lock:
            self.missions[namespace] = msg

    def _on_velocity(self, msg, namespace):
        with self.lock:
            self.velocities[namespace] = msg

    def _on_targets(self, msg, namespace):
        with self.lock:
            self.targets[namespace] = msg

    def _on_camera(self, msg, namespace):
        with self.lock:
            self.camera_msgs[namespace] = msg
            self.cam_stamps[namespace] = time.time()

    def _on_detections(self, msg, namespace):
        with self.lock:
            self.detections[namespace] = msg
            self.det_stamps[namespace] = time.time()

    def ensure(self, namespaces):
        if not self.ready:
            import rospy
            rospy.init_node('fleet_portal', anonymous=True,
                            disable_signals=True)
            self.ready = True
        if not self.subscribed:
            import rospy
            from drone_control.msg import (MissionStatus, SafetyStatus,
                                           TrackedTargets)
            from geometry_msgs.msg import PoseStamped, TwistStamped
            from mavros_msgs.msg import State
            self._rospy = rospy
            self._types = (State, PoseStamped, TwistStamped, SafetyStatus,
                           MissionStatus, TrackedTargets)
        rospy = self._rospy
        (State, PoseStamped, TwistStamped, SafetyStatus, MissionStatus,
         TrackedTargets) = self._types
        for namespace in namespaces:
            if namespace in self.subscribed:
                continue
            rospy.Subscriber(f'/{namespace}/mavros/state', State,
                             self._on_state, callback_args=namespace,
                             queue_size=1)
            rospy.Subscriber(f'/{namespace}/mavros/local_position/pose',
                             PoseStamped, self._on_pose,
                             callback_args=namespace, queue_size=1)
            rospy.Subscriber(
                f'/{namespace}/mavros/local_position/velocity_local',
                TwistStamped, self._on_velocity,
                callback_args=namespace, queue_size=1)
            rospy.Subscriber(f'/{namespace}/safety_status', SafetyStatus,
                             self._on_safety, callback_args=namespace,
                             queue_size=1)
            rospy.Subscriber(f'/{namespace}/mission_status', MissionStatus,
                             self._on_mission, callback_args=namespace,
                             queue_size=1)
            rospy.Subscriber(f'/{namespace}/tracked_targets', TrackedTargets,
                             self._on_targets, callback_args=namespace,
                             queue_size=1)
            # Create the joystick publisher up front: a lazy publisher only
            # links to the follower seconds after its first message, which
            # would swallow the start of every manual-control session.
            from geometry_msgs.msg import Twist
            if namespace not in self.manual_pubs:
                self.manual_pubs[namespace] = rospy.Publisher(
                    f'/{namespace}/manual_control', Twist, queue_size=1)
            self.subscribed.add(namespace)

    def ensure_camera(self, namespace):
        """Subscribe to camera + YOLO outputs on first live-view request."""
        if namespace in self.camera_ready:
            return
        import rospy
        if not self.ready:
            rospy.init_node('fleet_portal', anonymous=True,
                            disable_signals=True)
            self.ready = True
        from drone_control.msg import DetectedObjects
        from sensor_msgs.msg import Image
        rospy.Subscriber(f'/{namespace}/camera/color/image_raw', Image,
                         self._on_camera, callback_args=namespace,
                         queue_size=1)
        rospy.Subscriber(f'/{namespace}/detected_objects', DetectedObjects,
                         self._on_detections, callback_args=namespace,
                         queue_size=1)
        self.camera_ready.add(namespace)

    def _publish_string(self, namespaces, topic, value, store):
        """Publish a lazily-created String publisher per namespace.

        A freshly created rospy.Publisher is not yet connected to its
        subscribers; give registration a moment so the first message is
        not silently dropped.
        """
        if not self.ready:
            import rospy
            rospy.init_node('fleet_portal', anonymous=True,
                            disable_signals=True)
            self.ready = True
        import rospy
        from std_msgs.msg import String
        for namespace in namespaces:
            pub = store.get(namespace)
            if pub is None:
                pub = rospy.Publisher(f'/{namespace}/{topic}', String,
                                      queue_size=1)
                store[namespace] = pub
                time.sleep(0.2)
            pub.publish(String(data=value))

    def publish_scenario(self, namespaces, mode):
        """Tell each drone's vision front end what to report as its target."""
        self._publish_string(namespaces, 'scenario_mode', mode,
                             self.scenario_pubs)

    def publish_tracking_command(self, namespaces, command):
        """Operator override for the tracker: hold (drop + ignore) / engage.

        `override` bypasses the class check so the operator can confirm a
        misjudged photo by hand and have the drone pursue what it sees.
        """
        self._publish_string(namespaces, 'tracking_command', command,
                             self.tracking_pubs)

    def publish_manual(self, namespace, vx, vy, vz, yaw_rate):
        """Stream one joystick velocity command (ENU m/s, yaw rate rad/s)."""
        if not self.ready:
            import rospy
            rospy.init_node('fleet_portal', anonymous=True,
                            disable_signals=True)
            self.ready = True
        import rospy
        from geometry_msgs.msg import Twist
        pub = self.manual_pubs.get(namespace)
        if pub is None:
            pub = rospy.Publisher(f'/{namespace}/manual_control', Twist,
                                  queue_size=1)
            self.manual_pubs[namespace] = pub
            time.sleep(0.15)
        msg = Twist()
        msg.linear.x = float(vx)
        msg.linear.y = float(vy)
        msg.linear.z = float(vz)
        msg.angular.z = float(yaw_rate)
        pub.publish(msg)

    def frame_rgb(self, namespace):
        """Latest camera frame as a BGR ndarray, or None."""
        with self.lock:
            msg = self.camera_msgs.get(namespace)
        if msg is None:
            return None
        try:
            import cv2
            import numpy as np
            buf = np.frombuffer(msg.data, dtype=np.uint8)
            if msg.encoding == 'rgb8':
                return cv2.cvtColor(
                    buf.reshape(msg.height, msg.width, 3),
                    cv2.COLOR_RGB2BGR)
            if msg.encoding == 'bgr8':
                return buf.reshape(msg.height, msg.width, 3).copy()
            if msg.encoding in ('mono8', '8UC1'):
                return cv2.cvtColor(buf.reshape(msg.height, msg.width),
                                    cv2.COLOR_GRAY2BGR)
        except Exception:
            return None
        return None

    def frame(self, namespace):
        """Latest camera frame as JPEG bytes, or None."""
        arr = self.frame_rgb(namespace)
        if arr is None:
            return None
        try:
            import cv2
            ok, enc = cv2.imencode('.jpg', arr,
                                   [cv2.IMWRITE_JPEG_QUALITY, 70])
            return enc.tobytes() if ok else None
        except Exception:
            return None

    @staticmethod
    def _region_colour(crop):
        """Classify the dominant look of a detection crop.

        Mean RGB over the whole box washes out on small/oblique props where
        background leaks in, so classify per pixel: a car is "red" when a
        solid share of pixels are clearly red, "grey" when most are low
        saturation.
        """
        import numpy as np
        b = crop[:, :, 0].astype(np.float32)
        g = crop[:, :, 1].astype(np.float32)
        r = crop[:, :, 2].astype(np.float32)
        red = (r > 85) & (r > 1.3 * g) & (r > 1.3 * b)
        hi = np.maximum(np.maximum(r, g), b)
        lo = np.minimum(np.minimum(r, g), b)
        grey = (hi - lo < 24) & (lo > 45) & (hi < 215)
        if red.mean() >= 0.18:
            return 'red'
        if grey.mean() >= 0.5:
            return 'grey'
        return 'other'

    def analyse(self, namespaces, standoff):
        """Judge each drone's latest photo + detections against the mission.

        This reads actual pixels: the dominant colour of the detected
        region is compared with the annotated class, and the tracker range
        decides whether to keep closing or hold standoff.
        """
        now = time.time()
        overridden = set(self.overridden)
        with self.lock:
            detections = {
                ns: (self.detections.get(ns)
                     if now - self.det_stamps.get(ns, 0.0) <= 1.5
                     else None)
                for ns in namespaces}
            targets = {ns: self.targets.get(ns) for ns in namespaces}
            missions = {ns: self.missions.get(ns) for ns in namespaces}

        results = []
        for namespace in namespaces:
            entry = {'namespace': namespace,
                     'id': int(namespace.split('_')[-1]),
                     'mission': str(getattr(missions.get(namespace),
                                            'state', '') or '')}
            det = detections.get(namespace)
            objects = list(getattr(det, 'objects', []) or [])
            if not objects:
                entry.update(
                    verdict='NO_OBJECT', recommendation='hold',
                    overridden=namespace in overridden,
                    reason='Nothing in the latest frame — hold and keep '
                           'scanning.')
                results.append(entry)
                continue

            best = max(objects, key=lambda o: o.confidence)
            arr = self.frame_rgb(namespace)
            colour = 'unknown'
            if arr is not None and best.bbox.width > 2 and best.bbox.height > 2:
                height, width = arr.shape[:2]
                x1 = max(int(best.bbox.x), 0)
                y1 = max(int(best.bbox.y), 0)
                x2 = min(int(best.bbox.x + best.bbox.width), width - 1)
                y2 = min(int(best.bbox.y + best.bbox.height), height - 1)
                if x2 > x1 + 1 and y2 > y1 + 1:
                    mx = max(1, int((x2 - x1) * 0.15))
                    my = max(1, int((y2 - y1) * 0.15))
                    colour = self._region_colour(
                        arr[y1 + my:y2 - my, x1 + mx:x2 - mx])

            tracked = targets.get(namespace)
            tracked_list = list(getattr(tracked, 'targets', []) or [])
            rng = float(tracked_list[0].distance) if tracked_list else None

            entry.update(cls=best.class_name,
                         confidence=round(float(best.confidence), 2),
                         colour=colour, range=rng,
                         overridden=namespace in overridden)
            is_target = best.class_name in ('car', 'truck', 'bus')
            if is_target and colour in ('red', 'unknown'):
                if rng is not None and rng <= standoff * 0.95:
                    entry.update(
                        verdict='TARGET_CONFIRMED', recommendation='hold',
                        reason=f'Photo matches the target ({colour} vehicle) '
                               f'and range is {rng:.1f} m — at standoff, hold.')
                else:
                    where = (f'{rng:.1f} m out' if rng is not None
                             else 'not yet ranged')
                    entry.update(
                        verdict='TARGET_CONFIRMED', recommendation='pursue',
                        reason=f'Photo matches the target ({colour} vehicle, '
                               f'{where}) — pursue to standoff.')
            elif is_target:
                entry.update(
                    verdict='SUSPECT', recommendation='hold',
                    reason=f'Classifier says {best.class_name} but the '
                           f'image region is {colour} — unverified, hold.')
            else:
                entry.update(
                    verdict='NOT_TARGET', recommendation='hold',
                    reason=f"The object is a {best.class_name}, not this "
                           f"mission's target — reject and hold.")
            if (entry.get('overridden')
                    and entry['verdict'] in ('NOT_TARGET', 'SUSPECT')):
                entry.update(
                    recommendation='pursue',
                    reason=f"Operator override active: the classifier says "
                           f"'{best.class_name}' but the crew confirmed the "
                           f"photo — pursuing anyway.")
            results.append(entry)
        return results

    def rows(self, namespaces):
        rows = []
        with self.lock:
            for namespace in namespaces:
                row = {'id': int(namespace.split('_')[-1]),
                       'namespace': namespace}
                state = self.states.get(namespace)
                if state is not None:
                    row['connected'] = bool(state.connected)
                    row['armed'] = bool(state.armed)
                    row['mode'] = state.mode
                pose = self.poses.get(namespace)
                if pose is not None:
                    position = pose.pose.position
                    quat = pose.pose.orientation
                    row['altitude'] = position.z
                    row['x'] = position.x
                    row['y'] = position.y
                    row['heading'] = math.atan2(
                        2 * (quat.w * quat.z + quat.x * quat.y),
                        1 - 2 * (quat.y * quat.y + quat.z * quat.z))
                velocity = self.velocities.get(namespace)
                if velocity is not None:
                    linear = velocity.twist.linear
                    row['speed'] = math.hypot(linear.x, linear.y)
                targets = self.targets.get(namespace)
                if targets is not None and targets.targets:
                    target = targets.targets[0]
                    row['target'] = {
                        'x': target.position.x,
                        'y': target.position.y,
                        'z': target.position.z,
                        'confidence': target.confidence,
                    }
                detections = self.detections.get(namespace)
                if detections is not None:
                    row['detections'] = [
                        {'class': obj.class_name,
                         'confidence': round(obj.confidence, 2),
                         'bbox': [round(obj.bbox.x, 1),
                                  round(obj.bbox.y, 1),
                                  round(obj.bbox.width, 1),
                                  round(obj.bbox.height, 1)]}
                        for obj in detections.objects[:8]]
                camera = self.camera_msgs.get(namespace)
                if camera is not None:
                    row['camera'] = {'width': camera.width,
                                     'height': camera.height}
                safety = self.safety.get(namespace)
                if safety is not None:
                    row['safe'] = bool(safety.is_safe)
                    row['emergency'] = bool(safety.emergency_active)
                mission = self.missions.get(namespace)
                if mission is not None:
                    row['mission'] = str(mission.state)
                rows.append(row)
        return rows


# ---------------------------------------------------------------------------
# Supervisor + portal
# ---------------------------------------------------------------------------

class Supervisor:
    def __init__(self):
        self.lock = threading.Lock()
        self.proc = None
        self.cfg = load_config()
        self.ros_cache = RosCache()
        self.status = {'drones': [], 'updated': 0.0}
        self.flight = {'drones': [], 'updated': 0.0}
        self.trails = {}
        self.last_error = None
        self._reset_lock = threading.Lock()
        self._seen_connected = {}
        self._resume_at = {}
        self._prev = {}
        self._events = deque(maxlen=60)
        self._events_lock = threading.Lock()
        self.observability = [dict(svc, up=False, detail='not probed')
                              for svc in OBSERVABILITY]

    def event(self, kind, text):
        with self._events_lock:
            self._events.appendleft({'ts': time.time(), 'kind': kind,
                                     'text': text})
        print(f'[fleet] {kind}: {text}', file=sys.stderr, flush=True)

    def events_snapshot(self):
        with self._events_lock:
            return list(self._events)

    def _probe_observability(self):
        result = []
        for svc in OBSERVABILITY:
            entry = {'name': svc['name'], 'link': svc['link'],
                     'host': svc['host'], 'port': svc['port']}
            try:
                conn = http.client.HTTPConnection(svc['host'], svc['port'],
                                                  timeout=2)
                conn.request('GET', svc['path'])
                resp = conn.getresponse()
                resp.read()
                conn.close()
                entry['up'] = resp.status < 500
                entry['detail'] = f'HTTP {resp.status}'
            except Exception as exc:
                entry['up'] = False
                entry['detail'] = type(exc).__name__
            result.append(entry)
        self.observability = result

    def probe_loop(self):
        while True:
            try:
                self._probe_observability()
            except Exception as exc:
                self.last_error = str(exc)
            time.sleep(10)

    def start_ros(self):
        self.proc = subprocess.Popen(['roslaunch', LAUNCH_PATH],
                                     preexec_fn=os.setsid)

    def stop_ros(self):
        if self.proc is None:
            return
        if self.proc.poll() is None:
            try:
                os.killpg(os.getpgid(self.proc.pid), signal.SIGINT)
            except ProcessLookupError:
                pass
            try:
                self.proc.wait(timeout=12)
            except subprocess.TimeoutExpired:
                os.killpg(os.getpgid(self.proc.pid), signal.SIGKILL)
                self.proc.wait(timeout=5)
        self.proc = None

    def boot(self):
        drones = reconcile(self.cfg)
        render(self.cfg, drones)
        self._probe_observability()
        self.start_ros()

    def apply(self, updates):
        with self.lock:
            cfg = _merge_config(self.cfg, updates)
            save_config(cfg)
            drones = reconcile(cfg)
            changed = render(cfg, drones)
            running = self.proc is not None and self.proc.poll() is None
            restarted = False
            if changed or not running:
                self.stop_ros()
                self.start_ros()
                restarted = True
            self.cfg = cfg
            self.event('config', f'fleet applied: count={len(drones)}'
                       + (', ROS restarted' if restarted else ''))
            return {'drones': len(drones), 'restarted': restarted}

    def watch_ros(self):
        while True:
            time.sleep(5)
            with self.lock:
                if self.proc is not None and self.proc.poll() is not None:
                    self.start_ros()

    def reset_fleet(self):
        """Clear latched safety emergencies, then resume every mission."""
        namespaces = [f'drone_{i}' for i in
                      range(1, int(self.cfg['fleet']['count']) + 1)]
        self.ros_cache.ensure(namespaces)
        return {ns: self._reset_drone(ns) for ns in namespaces}

    def _reset_drone(self, ns, source='portal'):
        import rospy
        from drone_control.msg import Command
        from std_srvs.srv import Trigger

        step = {'cleared': False, 'resumed': False, 'error': None}
        with self._reset_lock:
            try:
                rospy.wait_for_service(f'/{ns}/safety/emergency_clear',
                                       timeout=5)
                resp = rospy.ServiceProxy(
                    f'/{ns}/safety/emergency_clear', Trigger)()
                step['cleared'] = bool(resp.success)
            except Exception as exc:
                step['error'] = str(exc)
            try:
                pub = rospy.Publisher(f'/{ns}/drone_control/command', Command,
                                      queue_size=1)
                time.sleep(0.5)
                for _ in range(3):
                    pub.publish(Command(command='resume', source=source,
                                        data='', priority=1,
                                        timestamp=rospy.Time.now()))
                    time.sleep(0.3)
                pub.unregister()
                step['resumed'] = True
            except Exception as exc:
                step['error'] = str(exc)
        return step

    def _watch_reconnects(self, rows):
        """Auto-resume a drone whose mavros came back after a sim restart."""
        if not self.cfg['fleet'].get('auto_resume', True):
            return
        now = time.time()
        for row in rows:
            ns = row['namespace']
            connected = row.get('connected')
            if connected is None:
                continue
            previous = self._seen_connected.get(ns)
            self._seen_connected[ns] = connected
            if (connected and previous is False
                    and now >= self._resume_at.get(ns, 0.0)):
                self._resume_at[ns] = now + 60.0
                threading.Thread(target=self._auto_resume, args=(ns,),
                                 daemon=True).start()

    def _track_changes(self, rows):
        """Turn per-drone state edges into the portal's event feed."""
        for row in rows:
            ns = row['namespace']
            cur = {key: row.get(key)
                   for key in ('connected', 'emergency', 'mission')}
            prev = self._prev.get(ns)
            if prev is not None:
                if (prev['connected'] is not None
                        and cur['connected'] != prev['connected']):
                    self.event('link', f'{ns} '
                               + ('reconnected' if cur['connected']
                                  else 'disconnected'))
                if (prev['emergency'] is not None
                        and cur['emergency'] != prev['emergency']):
                    self.event('safety', f'{ns} emergency '
                               + ('latched' if cur['emergency']
                                  else 'cleared'))
                if (prev['mission'] is not None
                        and cur['mission'] != prev['mission']):
                    self.event('mission', f'{ns} mission '
                               f'{prev["mission"]}→{cur["mission"]}')
            self._prev[ns] = cur

    def _auto_resume(self, ns):
        time.sleep(AUTO_RESUME_SETTLE)
        if not self.cfg['fleet'].get('auto_resume', True):
            return
        with self.ros_cache.lock:
            state = self.ros_cache.states.get(ns)
        if state is None or not state.connected:
            return
        step = self._reset_drone(ns, source='auto_resume')
        self.event('recovery', f'{ns} auto-resumed after sim restart '
                   f'(cleared={step["cleared"]}, resumed={step["resumed"]})')

    def _flight_snapshot(self, rows):
        keep = ('id', 'namespace', 'x', 'y', 'altitude', 'heading', 'speed',
                'target', 'detections', 'camera', 'connected', 'armed',
                'mission', 'safe', 'emergency')
        drones = []
        for row in rows:
            entry = {key: row[key] for key in keep if key in row}
            namespace = row['namespace']
            if 'x' in row:
                trail = self.trails.setdefault(
                    namespace, deque(maxlen=300))
                trail.append([round(row['x'], 2), round(row['y'], 2),
                              round(row.get('altitude', 0.0), 2)])
                entry['trail'] = list(trail)
            else:
                entry['trail'] = []
            drones.append(entry)
        self.flight = {
            'updated': time.time(),
            'geofence_radius': self.cfg['defaults']['geofence_radius'],
            'perception': bool(self.cfg['fleet'].get('perception_demo')),
            'drones': drones,
        }

    def sample_loop(self):
        while True:
            try:
                namespaces = [f'drone_{i}' for i in
                              range(1, int(self.cfg['fleet']['count']) + 1)]
                self.ros_cache.ensure(namespaces)
                rows = self.ros_cache.rows(namespaces)
                self.status = {'drones': rows, 'updated': time.time()}
                self._flight_snapshot(rows)
                self._track_changes(rows)
                self._watch_reconnects(rows)
            except Exception as exc:
                self.last_error = str(exc)
            time.sleep(1)


INDEX_HTML = '''<!doctype html>
<html><head><meta charset="utf-8"><title>KamiKaze Fleet</title>
<style>
body{font-family:system-ui,sans-serif;margin:0;background:#0f1216;color:#e6e9ee}
main{max-width:960px;margin:0 auto;padding:24px}
h1{font-size:20px;margin:0 0 16px}
section{background:#171c22;border:1px solid #2a323c;border-radius:10px;padding:16px;margin-bottom:16px}
label{display:inline-block;margin:6px 12px 6px 0;font-size:13px}
input[type=number]{width:88px;background:#0f1216;color:#e6e9ee;border:1px solid #38414d;border-radius:6px;padding:5px}
input[type=checkbox]{margin-right:4px}
button{background:#2563eb;color:#fff;border:0;border-radius:6px;padding:9px 18px;font-size:14px;cursor:pointer}
table{width:100%;border-collapse:collapse;font-size:13px}
th,td{text-align:left;padding:7px 8px;border-bottom:1px solid #2a323c}
.ok{color:#34d399}.bad{color:#f87171}.muted{color:#8b96a5}
.grid{display:grid;grid-template-columns:repeat(auto-fill,minmax(180px,1fr));gap:10px}
.svc{display:flex;flex-direction:column;gap:4px;background:#11161c;border:1px solid #2a323c;border-radius:8px;padding:10px 12px;text-decoration:none;color:#e6e9ee}
.svc:hover{border-color:#3b82f6}
.dot{width:8px;height:8px;border-radius:50%;display:inline-block;margin-right:6px}
.dot.up{background:#34d399}.dot.down{background:#f87171}
.small{font-size:11px}
#events{list-style:none;margin:0;padding:0;font-size:13px;max-height:220px;overflow:auto}
#events li{padding:5px 0;border-bottom:1px solid #222a33}
.k-safety{color:#f87171}.k-recovery{color:#34d399}.k-link{color:#fbbf24}.k-mission{color:#67e8f9}.k-config{color:#c4b5fd}
#msg{margin-left:12px;font-size:13px}
.hdr{display:flex;align-items:center;gap:14px;flex-wrap:wrap;margin-bottom:16px}
.chip{background:#1f2630;border:1px solid #2f3945;border-radius:999px;padding:3px 10px;font-size:12px}
.chip.warn{border-color:#7f1d1d;color:#fca5a5}
#map,#scene{width:100%;height:430px;display:block;background:#0b0e12;border:1px solid #2a323c;border-radius:8px;touch-action:none}
#vtools{display:flex;gap:8px;align-items:center;flex-wrap:wrap;margin-bottom:8px}
.vt{background:#1f2630;border:1px solid #38414d;color:#c9d2dd;border-radius:6px;padding:5px 12px;font-size:12px;cursor:pointer}
.vt.on{background:#2563eb;border-color:#2563eb;color:#fff}
#followSel{background:#0f1216;color:#e6e9ee;border:1px solid #38414d;border-radius:6px;padding:5px 8px;font-size:12px}
#flbar{display:flex;gap:14px;align-items:center;flex-wrap:wrap;margin-top:8px;font-size:12px;color:#8b96a5}
#flbar .key{display:inline-flex;align-items:center;gap:5px}
.sw{width:10px;height:10px;border-radius:50%;display:inline-block}
#camgrid{display:grid;grid-template-columns:repeat(auto-fill,minmax(260px,1fr));gap:12px;margin-top:10px}
.camtile{background:#11161c;border:1px solid #2a323c;border-radius:8px;padding:8px}
.camtile .cap{font-size:12px;color:#8b96a5;margin-bottom:6px;display:flex;justify-content:space-between}
.cambody{position:relative;background:#0b0e12;border-radius:6px;overflow:hidden;min-height:110px}
.cambody img{width:100%;display:block}
.cambody canvas{position:absolute;inset:0;width:100%;height:100%}
.cambody .none{position:absolute;inset:0;display:flex;align-items:center;justify-content:center;color:#8b96a5;font-size:12px;text-align:center;padding:10px}
@media (max-width:600px){#map,#scene{height:320px}}
.tabs{display:flex;gap:6px;margin:0 0 16px}
.tab{background:#1f2630;border:1px solid #38414d;color:#c9d2dd;border-radius:8px;padding:8px 16px;font-size:13px;cursor:pointer}
.tab.on{background:#2563eb;border-color:#2563eb;color:#fff}
.vbadge{display:inline-block;border-radius:999px;padding:2px 10px;font-size:11px;border:1px solid;white-space:nowrap}
.vbadge.ok{color:#34d399;border-color:#065f46}
.vbadge.warn{color:#fca5a5;border-color:#7f1d1d}
.vbadge.amb{color:#fbbf24;border-color:#78350f}
.vbadge.mut{color:#8b96a5;border-color:#2f3945}
#mgcards{grid-template-columns:repeat(auto-fill,minmax(220px,1fr))}
.mgcard{background:#11161c;border:1px solid #2a323c;border-radius:8px;padding:10px}
.mgcard b{font-size:13px}
.mgthumb{width:100%;border-radius:6px;display:block;background:#0b0e12;margin-top:8px;min-height:60px}
#mres table td{vertical-align:top;font-size:12px}
.mgscope{display:flex;gap:6px;flex-wrap:wrap}
</style></head><body><main>
<div class="hdr">
<h1>KamiKaze Fleet Control</h1>
<span id="summary"></span>
</div>
<div class="tabs">
<button class="tab on" data-tab="dash">Dashboard</button>
<button class="tab" data-tab="mgmt">Management</button>
</div>
<div id="dash">

<section>
<h3 style="margin-top:0;font-size:15px">Drones</h3>
<table><thead><tr><th>#</th><th>Connected</th><th>Armed</th><th>Mode</th><th>Altitude</th><th>Safety</th><th>Mission</th></tr></thead>
<tbody id="rows"></tbody></table>
<p class="muted" id="age" style="font-size:12px"></p>
<div style="margin-top:12px"><button id="reset">Clear emergencies &amp; resume all</button><span id="rmsg" class="muted" style="margin-left:12px;font-size:13px"></span></div>
</section>

<section>
<h3 style="margin-top:0;font-size:15px">Live flight view <span class="muted small" id="flage"></span></h3>
<div id="vtools">
<button id="btn3d" class="vt on">3D playground</button>
<button id="btn2d" class="vt">2D map</button>
<label class="muted small">Camera
<select id="followSel"><option value="">Orbit</option></select></label>
<span id="camang" style="display:none">
<button class="vt camang" data-angle="chase">Chase</button>
<button class="vt camang" data-angle="pov">Nose POV</button>
<button class="vt camang" data-angle="top">Top</button>
</span>
<span class="muted small" id="f3dnote">click a drone to follow · wheel zoom</span>
</div>
<canvas id="scene"></canvas>
<canvas id="map" style="display:none"></canvas>
<div id="flbar"></div>
<div id="camwrap" style="display:none">
<h4 style="margin:16px 0 0;font-size:14px">Camera / vision check</h4>
<span id="scen" style="display:flex;gap:6px;margin:6px 0;flex-wrap:wrap">
<button class="vt scen" data-mode="target">Target car</button>
<button class="vt scen" data-mode="decoy">Bystander decoy</button>
<button class="vt scen" data-mode="none">Clear</button>
<span class="muted small" id="scenmsg" style="align-self:center"></span>
</span>
<div id="camgrid"></div>
</div>
</section>

<section>
<h3 style="margin-top:0;font-size:15px">Observability</h3>
<div class="grid" id="obs"></div>
<p class="muted small" id="obsnote" style="margin:10px 0 0"></p>
</section>

<section>
<h3 style="margin-top:0;font-size:15px">Recent events</h3>
<ul id="events"></ul>
</section>

<section>
<h3 style="margin-top:0;font-size:15px">Fleet configuration</h3>
<label>Drones <input type="number" id="count" min="1" max="10" step="1"></label>
<label>Standoff (m) <input type="number" id="standoff_distance" step="0.5"></label>
<label>Altitude (m) <input type="number" id="follow_altitude" step="0.5"></label>
<label>Max speed (m/s) <input type="number" id="max_velocity" step="0.5"></label>
<label>Geofence radius (m) <input type="number" id="geofence_radius" step="5"></label>
<label>Geofence height (m) <input type="number" id="geofence_height" step="5"></label>
<label>PX4 CPU each <input type="number" id="px4_cpus" min="0.25" max="4" step="0.1"></label>
<div style="margin-top:8px">
<label><input type="checkbox" id="auto_start">Auto-start missions</label>
<label><input type="checkbox" id="auto_resume">Auto-resume after sim recovery</label>
<label><input type="checkbox" id="simulation">Simulation</label>
<label><input type="checkbox" id="enable_safety">Safety monitor</label>
<label><input type="checkbox" id="perception_demo">Camera/YOLO pipeline</label>
</div>
<div style="margin-top:12px"><button id="apply">Apply</button><span id="msg"></span></div>
</section>
</div><!-- #dash -->

<section id="tab-mgmt" style="display:none">
<h3 style="margin-top:0;font-size:15px">Fleet management</h3>
<p class="muted small" style="margin-top:0">Pick a scope, analyse the drones' live camera views, then issue instructions. Drones carry out the order immediately.</p>

<div style="margin:10px 0">
<div class="mgscope" id="mgscopemode">
<button class="vt on" data-smode="all">All drones</button>
<button class="vt" data-smode="many">Select several</button>
<button class="vt" data-smode="one">Single drone</button>
</div>
<div id="mgscope" class="mgscope" style="margin-top:10px"></div>
<span class="muted small" id="mgscopesum"></span>
</div>

<div style="display:flex;gap:8px;flex-wrap:wrap;margin:14px 0">
<button id="mgAnalyse">Analyse photos</button>
<button id="mgPursue" class="vt">Instruct: pursue target</button>
<button id="mgHold" class="vt">Instruct: hold position</button>
<button id="mgOverride" class="vt">Override: it IS the target</button>
<button id="mgResume" class="vt">Clear emergencies &amp; resume</button>
<span class="muted small" id="mgmsg" style="align-self:center"></span>
</div>

<div id="mgmanual" style="background:#11161c;border:1px solid #2a323c;border-radius:8px;padding:10px 12px;margin-bottom:16px">
<h4 style="margin:0 0 8px;font-size:14px">Manual control <span class="muted small" id="mgmPad"></span></h4>
<div style="display:flex;gap:10px;align-items:center;flex-wrap:wrap">
<label class="muted small">Drone <select id="mgmSel"></select></label>
<button id="mgmTake" class="vt">Take control</button>
<span class="muted small" id="mgmState">idle</span>
</div>
<div id="mgmHints" class="muted small" style="display:none;margin-top:8px">
W/S forward-back · A/D strafe · R/F up-down · Q/E turn · Space or Release hands control back.
Gamepad supported: left stick moves, right stick vertical = altitude, right stick horizontal = yaw.
</div>
</div>

<div class="grid" id="mgcards"></div>

<div id="mres" style="margin-top:16px"></div>
</section>

</main>
<script src="/static/three.min.js"></script>
<script src="/static/OrbitControls.js"></script>
<script>
const ids=['count','standoff_distance','follow_altitude','max_velocity','geofence_radius','geofence_height'];
const fleetNums=['px4_cpus'];
const flags=['auto_start','auto_resume','simulation','enable_safety','perception_demo'];
let loaded=false,cfgNow=null;
function fmt(v){return v===undefined||v===null?'-':v;}
function esc(s){const d=document.createElement('div');d.textContent=s;return d.innerHTML;}
async function refresh(){
  try{
    const r=await fetch('/api/status');const j=await r.json();
    cfgNow=j.config;
    if(!loaded){const f=j.config.fleet,d=j.config.defaults;
      for(const k of ids){document.getElementById(k).value=k==='count'?f.count:d[k];}
      for(const k of fleetNums){document.getElementById(k).value=f[k];}
      for(const k of flags){document.getElementById(k).checked=!!f[k];}
      loaded=true;}
    const conn=j.drones.filter(d=>d.connected).length;
    const em=j.drones.filter(d=>d.emergency).length;
    const armed=j.drones.filter(d=>d.armed).length;
    document.getElementById('summary').innerHTML=
      `<span class="chip">${conn}/${j.drones.length} connected</span>`+
      `<span class="chip ${em?'warn':''}">${em} emergency</span>`+
      `<span class="chip">${armed} armed</span>`;
    document.getElementById('rows').innerHTML=j.drones.map(d=>{
      const connCell=d.connected===undefined?'<span class="muted">waiting</span>':(d.connected?'<span class="ok">yes</span>':'<span class="bad">no</span>');
      let safety='-';
      if(d.emergency) safety='<span class="bad">EMERGENCY</span>';
      else if(d.safe===false) safety='<span class="bad">unsafe</span>';
      else if(d.safe===true) safety='<span class="ok">safe</span>';
      const mission=d.mission?esc(d.mission):'-';
      return `<tr><td>${d.id}</td><td>${connCell}</td><td>${fmt(d.armed)}</td>
        <td>${esc(fmt(d.mode))}</td><td>${d.altitude!==undefined?Number(d.altitude).toFixed(1):'-'}</td>
        <td>${safety}</td><td>${mission}</td></tr>`;}).join('');
    document.getElementById('obs').innerHTML=(j.observability||[]).map(o=>
      `<a class="svc" href="${o.link}" target="_blank" rel="noopener">
        <span><span class="dot ${o.up?'up':'down'}"></span><b>${o.name}</b></span>
        <span class="muted small">${esc(o.detail||'')} · ${o.host}:${o.port}</span></a>`).join('');
    document.getElementById('obsnote').textContent=(j.observability||[]).every(o=>!o.up)
      ? 'Monitoring stack not running — start it with: COMPOSE_PROFILES=monitoring docker compose up -d'
      : '';
    const ev=(j.events||[]).slice(0,15);
    document.getElementById('events').innerHTML=ev.length?ev.map(e=>
      `<li><span class="muted">${new Date(e.ts*1000).toLocaleTimeString()}</span> `+
      `<span class="k-${e.kind}">${e.kind}</span> ${esc(e.text)}</li>`).join('')
      : '<li class="muted">no events yet</li>';
    document.getElementById('age').textContent='Updated '+new Date(j.updated*1000).toLocaleTimeString();
  }catch(e){document.getElementById('age').textContent='Status unavailable: '+e;}
}
document.getElementById('apply').onclick=async()=>{
  const msg=document.getElementById('msg');msg.textContent='Applying...';
  const body={fleet:{},defaults:{}};
  for(const k of ids){const v=Number(document.getElementById(k).value);if(k==='count')body.fleet.count=v;else body.defaults[k]=v;}
  for(const k of fleetNums){body.fleet[k]=Number(document.getElementById(k).value);}
  for(const k of flags){body.fleet[k]=document.getElementById(k).checked;}
  const r=await fetch('/api/apply',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)});
  const j=await r.json();
  msg.textContent=j.ok?`Applied: ${j.drones} drones${j.restarted?', ROS stack restarted':''}`:('Error: '+j.error);
  setTimeout(refresh,1000);
};
/* ---- live flight view ---- */
const WCOL=['#60a5fa','#fb923c','#34d399','#c084fc','#f472b6','#facc15'];
let viewMode='3d';
const map=document.getElementById('map'),mctx=map.getContext('2d');
let flight=null,fitC={x:0,y:0},wantS=40,dispC={x:0,y:0},dispS=40;
let mapZoom=1,mapPan={x:0,y:0},drag=null,camIds='',camTick=0;
const camTiles={};
function mapSize(){const r=map.getBoundingClientRect(),d=window.devicePixelRatio||1;
  const w=Math.round(r.width*d),h=Math.round(r.height*d);
  if(map.width!==w||map.height!==h){map.width=w;map.height=h;}}
window.addEventListener('resize',mapSize);
map.addEventListener('wheel',e=>{e.preventDefault();
  mapZoom=Math.min(12,Math.max(0.35,mapZoom*(e.deltaY<0?1.15:1/1.15)));},{passive:false});
map.addEventListener('pointerdown',e=>{drag={sx:e.clientX,sy:e.clientY,px:mapPan.x,py:mapPan.y};
  map.setPointerCapture(e.pointerId);});
map.addEventListener('pointermove',e=>{if(!drag)return;
  const k=dispS/Math.min(map.clientWidth||1,map.clientHeight||1);
  mapPan.x=drag.px-(e.clientX-drag.sx)*k;mapPan.y=drag.py+(e.clientY-drag.sy)*k;});
map.addEventListener('pointerup',()=>{drag=null;});
map.addEventListener('dblclick',()=>{mapZoom=1;mapPan={x:0,y:0};});
function computeFit(){
  const pts=[];
  for(const dr of (flight?flight.drones:[])){
    if(dr.x!==undefined)pts.push([dr.x,dr.y]);
    if(dr.target)pts.push([dr.target.x,dr.target.y]);
    const tr=dr.trail||[];for(let i=0;i<tr.length;i+=6)pts.push(tr[i]);
  }
  if(!pts.length){fitC={x:0,y:0};wantS=40;return;}
  let x0=1e9,x1=-1e9,y0=1e9,y1=-1e9;
  for(const p of pts){x0=Math.min(x0,p[0]);x1=Math.max(x1,p[0]);y0=Math.min(y0,p[1]);y1=Math.max(y1,p[1]);}
  fitC={x:(x0+x1)/2,y:(y0+y1)/2};
  const w=map.clientWidth||600,h=map.clientHeight||400,m=Math.min(w,h);
  wantS=Math.max((x1-x0)*m/w,(y1-y0)*m/h,24)*1.35;
}
function drawMap(){
  if(viewMode!=='2d')return;
  mapSize();computeFit();
  const w=map.width,h=map.height,d=window.devicePixelRatio||1;
  if(!w)return;
  const tc={x:fitC.x+mapPan.x,y:fitC.y+mapPan.y},ts=wantS/mapZoom;
  dispC.x+=(tc.x-dispC.x)*0.15;dispC.y+=(tc.y-dispC.y)*0.15;dispS+=(ts-dispS)*0.15;
  const s=Math.min(w,h)/dispS,X=x=>w/2+(x-dispC.x)*s,Y=y=>h/2-(y-dispC.y)*s;
  mctx.fillStyle='#0b0e12';mctx.fillRect(0,0,w,h);
  const steps=[1,2,5,10,20,25,50,100,200,500];
  const step=steps.find(v=>v*s>=60*d)||500;
  const x0=dispC.x-w/(2*s),x1=dispC.x+w/(2*s),y0=dispC.y-h/(2*s),y1=dispC.y+h/(2*s);
  mctx.strokeStyle='#1b222b';mctx.lineWidth=1;mctx.font=(10*d)+'px system-ui';mctx.fillStyle='#5b6675';
  for(let gx=Math.ceil(x0/step)*step;gx<=x1;gx+=step){const px=X(gx);
    mctx.beginPath();mctx.moveTo(px,0);mctx.lineTo(px,h);mctx.stroke();
    mctx.fillText(gx.toFixed(0)+'m',px+3*d,h-4*d);}
  for(let gy=Math.ceil(y0/step)*step;gy<=y1;gy+=step){const py=Y(gy);
    mctx.beginPath();mctx.moveTo(0,py);mctx.lineTo(w,py);mctx.stroke();
    mctx.fillText(gy.toFixed(0)+'m',3*d,py-3*d);}
  mctx.fillStyle='#8b96a5';mctx.font=(11*d)+'px system-ui';
  mctx.fillText('N ↑   E →',10*d,16*d);
  if(flight&&flight.geofence_radius>0){
    const g=flight.geofence_radius;
    if(g*s<Math.max(w,h)*1.6){mctx.beginPath();mctx.arc(X(0),Y(0),g*s,0,Math.PI*2);
      mctx.setLineDash([6*d,5*d]);mctx.strokeStyle='#7f1d1d';mctx.stroke();mctx.setLineDash([]);}
  }
  const drones=(flight?flight.drones:[]);
  if(!drones.length){mctx.fillStyle='#5b6675';mctx.font=(13*d)+'px system-ui';
    mctx.fillText('waiting for telemetry…',12*d,h/2);}
  for(const dr of drones){
    const col=WCOL[(dr.id-1)%WCOL.length],tr=dr.trail||[];
    if(tr.length>1){mctx.beginPath();mctx.strokeStyle=col;mctx.globalAlpha=0.45;mctx.lineWidth=1.5*d;
      tr.forEach((p,j)=>{const px=X(p[0]),py=Y(p[1]);j?mctx.lineTo(px,py):mctx.moveTo(px,py);});
      mctx.stroke();mctx.globalAlpha=1;}
  }
  const seen={};
  for(const dr of drones){
    if(!dr.target)continue;
    const k=dr.target.x.toFixed(1)+','+dr.target.y.toFixed(1);
    if(seen[k])continue;seen[k]=1;
    const tx=X(dr.target.x),ty=Y(dr.target.y),r=7*d;
    mctx.beginPath();mctx.moveTo(tx,ty-r);mctx.lineTo(tx+r,ty);mctx.lineTo(tx,ty+r);mctx.lineTo(tx-r,ty);
    mctx.closePath();mctx.strokeStyle='#fbbf24';mctx.lineWidth=2*d;mctx.stroke();
    mctx.fillStyle='#8b96a5';mctx.font=(10*d)+'px system-ui';mctx.fillText('TGT',tx+r+3*d,ty+3*d);
  }
  for(const dr of drones){
    if(dr.x===undefined)continue;
    const col=dr.connected===false?'#6b7280':WCOL[(dr.id-1)%WCOL.length];
    const px=X(dr.x),py=Y(dr.y),hd=dr.heading||0;
    if(dr.emergency){mctx.beginPath();mctx.arc(px,py,12*d,0,Math.PI*2);mctx.strokeStyle='#f87171';mctx.lineWidth=2*d;mctx.stroke();}
    mctx.beginPath();mctx.moveTo(px,py);mctx.lineTo(px+Math.cos(hd)*14*d,py-Math.sin(hd)*14*d);
    mctx.strokeStyle=col;mctx.lineWidth=2*d;mctx.stroke();
    mctx.beginPath();mctx.arc(px,py,6*d,0,Math.PI*2);mctx.fillStyle=col;mctx.fill();
    mctx.strokeStyle='#e6e9ee';mctx.lineWidth=1.5*d;mctx.stroke();
    mctx.fillStyle='#c9d2dd';mctx.font=(11*d)+'px system-ui';
    const label='D'+dr.id+(dr.altitude!==undefined?' '+Number(dr.altitude).toFixed(1)+'m':'');
    mctx.fillText(label,px+9*d,py+16*d);
  }
}
function ensureCamTiles(drones){
  const ids=drones.map(dr=>dr.id).join(',');
  if(ids===camIds)return;
  camIds=ids;
  const grid=document.getElementById('camgrid');grid.innerHTML='';
  for(const k of Object.keys(camTiles))delete camTiles[k];
  for(const dr of drones){
    const tile=document.createElement('div');tile.className='camtile';
    const cap=document.createElement('div');cap.className='cap';
    cap.innerHTML='<span>'+esc(dr.namespace||('drone_'+dr.id))+'</span><span class="camage muted"></span>';
    const body=document.createElement('div');body.className='cambody';
    const img=document.createElement('img');img.alt='';
    const cv=document.createElement('canvas');
    const none=document.createElement('div');none.className='none';none.textContent='waiting for frames…';
    img.addEventListener('load',()=>{none.style.display='none';});
    img.addEventListener('error',()=>{none.style.display='flex';
      none.textContent='no camera stream — enable Camera/YOLO pipeline and Apply';});
    body.appendChild(img);body.appendChild(cv);body.appendChild(none);
    tile.appendChild(cap);tile.appendChild(body);grid.appendChild(tile);
    camTiles[dr.id]={img:img,canvas:cv,none:none,age:cap.querySelector('.camage')};
  }
}
function renderCams(){
  const wrap=document.getElementById('camwrap');
  if(!flight||!flight.perception){wrap.style.display='none';return;}
  wrap.style.display='';
  const drones=flight.drones||[];
  ensureCamTiles(drones);camTick++;
  for(const dr of drones){
    const tile=camTiles[dr.id];if(!tile)continue;
    const cw=(dr.camera&&dr.camera.width)||640,ch=(dr.camera&&dr.camera.height)||480;
    if(tile.canvas.width!==cw||tile.canvas.height!==ch){tile.canvas.width=cw;tile.canvas.height=ch;}
    const c=tile.canvas.getContext('2d');c.clearRect(0,0,cw,ch);c.font='14px system-ui';
    for(const b of (dr.detections||[])){
      c.strokeStyle='#34d399';c.lineWidth=2;c.strokeRect(b.bbox[0],b.bbox[1],b.bbox[2],b.bbox[3]);
      c.fillStyle='#34d399';c.fillText(b.class+' '+b.confidence,Math.max(2,b.bbox[0]),Math.max(14,b.bbox[1]-4));
    }
    if(camTick%7===0)tile.img.src='/api/camera/'+dr.id+'?t='+Date.now();
    tile.age.textContent=dr.mission||'';
  }
}
function renderLegend(){
  const el=document.getElementById('flbar');const drones=(flight?flight.drones:[]);
  const sig=drones.map(dr=>dr.id).join(',');
  if(el.dataset.sig===sig)return;
  el.dataset.sig=sig;
  el.innerHTML=drones.map(dr=>'<span class="key"><span class="sw" style="background:'
    +WCOL[(dr.id-1)%WCOL.length]+'"></span>D'+dr.id+'</span>').join('')
    +'<span class="key"><span class="sw" style="background:#fbbf24"></span>target (amber diamond)</span>'
    +'<span class="muted">drag to orbit · double-click reset</span>';
}
/* ---- three.js playground ---- */
const sceneEl=document.getElementById('scene');
const followSel=document.getElementById('followSel');
let scene3=null,camera3=null,renderer3=null,controls3=null,threeReady=false;
let drone3={},target3=null,geofence3=null,geoR=0;
let followId=null,followZoom=18,lastFrame=performance.now();
const camangEl=document.getElementById('camang');
const camAngleBtns=Array.prototype.slice.call(document.querySelectorAll('.camang'));
const camAngleByDrone={};let camAngle='chase';
function syncCamang(){
  camangEl.style.display=(followId&&viewMode==='3d')?'flex':'none';
  if(controls3)controls3.maxPolarAngle=(followId&&camAngle==='pov')?Math.PI:Math.PI*0.49;
  for(const b of camAngleBtns)b.classList.toggle('on',!!followId&&b.dataset.angle===camAngle);
}
for(const b of camAngleBtns)b.addEventListener('click',()=>{
  if(!followId)return;
  camAngle=b.dataset.angle;camAngleByDrone[followId]=camAngle;
  followZoom=18;syncCamang();
});
let scenMode='target';
const scenBtns=[...document.querySelectorAll('.scen')];
function syncScen(){
  for(const b of scenBtns)b.classList.toggle('on',b.dataset.mode===scenMode);
}
for(const b of scenBtns)b.addEventListener('click',async()=>{
  scenMode=b.dataset.mode;syncScen();
  const sm=document.getElementById('scenmsg');sm.textContent='applying...';
  try{
    const r=await fetch('/api/scenario',{method:'POST',
      headers:{'Content-Type':'application/json'},
      body:JSON.stringify({mode:scenMode})});
    const j=await r.json();
    sm.textContent=j.ok?('scenario: '+scenMode):('error: '+j.error);
  }catch(e){sm.textContent='error: '+e;}
});
syncScen();
function enuV(x,y,z){return new THREE.Vector3(x,z||0,-y);}
function makeLabel(text,color,width){
  const cv=document.createElement('canvas');cv.width=256;cv.height=80;
  const g=cv.getContext('2d');
  g.fillStyle='rgba(11,14,18,0.72)';
  g.beginPath();g.roundRect?g.roundRect(8,8,240,64,16):g.rect(8,8,240,64);g.fill();
  g.font='bold 44px system-ui,sans-serif';g.textAlign='center';g.textBaseline='middle';
  g.fillStyle=color;g.fillText(text,128,42);
  const tex=new THREE.CanvasTexture(cv);tex.minFilter=THREE.LinearFilter;
  const sp=new THREE.Sprite(new THREE.SpriteMaterial({map:tex,transparent:true,depthTest:false}));
  const w=width||4.2;sp.scale.set(w,w*80/256,1);
  return sp;
}
function droneModel(color){
  const g=new THREE.Group();
  const bodyMat=new THREE.MeshLambertMaterial({color:color});
  const armMat=new THREE.MeshLambertMaterial({color:new THREE.Color(color).multiplyScalar(0.55)});
  g.add(new THREE.Mesh(new THREE.BoxGeometry(1.1,0.32,0.8),bodyMat));
  for(const rot of [Math.PI/4,-Math.PI/4]){
    const arm=new THREE.Mesh(new THREE.BoxGeometry(2.3,0.07,0.13),armMat);
    arm.rotation.y=rot;g.add(arm);
  }
  const rotors=[],rotorMat=new THREE.MeshLambertMaterial({color:0x14181f});
  for(const p of [[1,1],[1,-1],[-1,1],[-1,-1]]){
    const r=new THREE.Mesh(new THREE.CylinderGeometry(0.42,0.42,0.06,18),rotorMat);
    r.position.set(p[0]*0.81,0.09,p[1]*0.81);g.add(r);rotors.push(r);
  }
  const nose=new THREE.Mesh(new THREE.ConeGeometry(0.16,0.45,10),
    new THREE.MeshLambertMaterial({color:0xffffff}));
  nose.rotation.z=-Math.PI/2;nose.position.set(0.72,0.05,0);g.add(nose);
  const beacon=new THREE.Mesh(new THREE.SphereGeometry(0.12,10,10),
    new THREE.MeshBasicMaterial({color:color}));
  beacon.position.set(0,0.3,0);g.add(beacon);
  g.scale.setScalar(1.7);g.userData.rotors=rotors;
  return g;
}
function resize3D(){if(!renderer3)return;
  const w=sceneEl.clientWidth||600,h=sceneEl.clientHeight||400;
  renderer3.setPixelRatio(window.devicePixelRatio||1);
  renderer3.setSize(w,h,false);
  camera3.aspect=w/h;camera3.updateProjectionMatrix();}
function init3D(){
  if(threeReady)return;threeReady=true;
  scene3=new THREE.Scene();
  scene3.background=new THREE.Color(0x0b0e12);
  scene3.fog=new THREE.Fog(0x0b0e12,200,480);
  camera3=new THREE.PerspectiveCamera(55,sceneEl.clientWidth/(sceneEl.clientHeight||1),0.1,2000);
  camera3.position.set(26,17,26);
  renderer3=new THREE.WebGLRenderer({canvas:sceneEl,antialias:true});
  scene3.add(new THREE.HemisphereLight(0xbfd4ff,0x20242a,0.95));
  const sun=new THREE.DirectionalLight(0xffffff,0.85);
  sun.position.set(60,90,30);scene3.add(sun);
  const ground=new THREE.Mesh(new THREE.PlaneGeometry(600,600),
    new THREE.MeshLambertMaterial({color:0x131a22}));
  ground.rotation.x=-Math.PI/2;scene3.add(ground);
  const grid=new THREE.GridHelper(600,120,0x2a3644,0x1b242e);
  grid.position.y=0.02;scene3.add(grid);
  const rim=[];
  for(let i=0;i<=160;i++){const a=i/160*Math.PI*2;rim.push(new THREE.Vector3(Math.cos(a),0.06,Math.sin(a)));}
  geofence3=new THREE.Line(new THREE.BufferGeometry().setFromPoints(rim),
    new THREE.LineDashedMaterial({color:0x7f1d1d,dashSize:2.5,gapSize:1.8}));
  geofence3.visible=false;geofence3.computeLineDistances();scene3.add(geofence3);
  target3=new THREE.Group();
  const oct=new THREE.Mesh(new THREE.OctahedronGeometry(1.6,0),
    new THREE.MeshLambertMaterial({color:0xfbbf24,emissive:0x7a5b00}));
  target3.add(oct);
  const tLabel=makeLabel('TARGET','#fbbf24',6.6);
  tLabel.position.set(0,3.9,0);target3.add(tLabel);
  target3.visible=false;scene3.add(target3);
  controls3=new THREE.OrbitControls(camera3,sceneEl);
  controls3.enableDamping=true;controls3.dampingFactor=0.08;
  controls3.maxPolarAngle=Math.PI*0.49;
  controls3.target.set(8,3,0);
  resize3D();
  window.addEventListener('resize',resize3D);
  sceneEl.addEventListener('wheel',e=>{
    if(followId){e.preventDefault();e.stopPropagation();
      followZoom=Math.min(70,Math.max(4,followZoom*(e.deltaY<0?1.1:0.9)));}
  },{passive:false});
  let down=null;
  sceneEl.addEventListener('dblclick',()=>{
    setFollow(null);followSel.value='';
    camera3.position.set(26,17,26);controls3.target.set(8,3,0);
  });
  sceneEl.addEventListener('pointerdown',e=>{down={x:e.clientX,y:e.clientY};});
  sceneEl.addEventListener('pointerup',e=>{
    if(!down)return;const moved=Math.hypot(e.clientX-down.x,e.clientY-down.y);down=null;
    if(moved>4||!camera3)return;
    const r=sceneEl.getBoundingClientRect();
    const m=new THREE.Vector2(((e.clientX-r.left)/r.width)*2-1,-((e.clientY-r.top)/r.height)*2+1);
    const ray=new THREE.Raycaster();ray.setFromCamera(m,camera3);
    let hit=null;
    for(const id in drone3){
      const ix=ray.intersectObject(drone3[id].group,true);
      if(ix.length&&(!hit||ix[0].distance<hit.d))hit={id:Number(id),d:ix[0].distance};
    }
    if(hit){followSel.value=String(hit.id);setFollow(hit.id);}
  });
  requestAnimationFrame(animate3D);
}
function angDiff(a,b){let d=(a-b+Math.PI)%(Math.PI*2);if(d<0)d+=Math.PI*2;return d-Math.PI;}
function setFollow(id){
  followId=id||null;
  if(controls3)controls3.enabled=!followId;
  if(followId){followZoom=18;camAngle=camAngleByDrone[followId]||'chase';}
  syncCamang();
}
followSel.addEventListener('change',()=>setFollow(followSel.value?Number(followSel.value):null));
function update3D(){
  if(!threeReady||!flight)return;
  const ids=flight.drones.map(dr=>dr.id);
  for(const dr of flight.drones){
    let o=drone3[dr.id];
    if(!o){
      const g=droneModel(WCOL[(dr.id-1)%WCOL.length]);
      scene3.add(g);
      const col=WCOL[(dr.id-1)%WCOL.length];
      const pole=new THREE.Line(
        new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(0,0,0),new THREE.Vector3(0,0,0)]),
        new THREE.LineBasicMaterial({color:col,transparent:true,opacity:0.3}));
      scene3.add(pole);
      const line=new THREE.Line(new THREE.BufferGeometry(),
        new THREE.LineBasicMaterial({color:col,transparent:true,opacity:0.65}));
      scene3.add(line);
      const label=makeLabel('D'+dr.id,col,3.2);
      scene3.add(label);
      o={group:g,pos:new THREE.Vector3(0,0,0),tp:null,heading:0,tphi:0,
         rotors:g.userData.rotors,pole:pole,line:line,label:label,has:false};
      drone3[dr.id]=o;
    }
    if(dr.x!==undefined){
      const tp=enuV(dr.x,dr.y,dr.altitude||0);
      if(!o.has){o.pos.copy(tp);o.has=true;}
      o.tp=tp;o.tphi=dr.heading||0;
    }
    const tr=dr.trail||[];
    const pts=tr.map(p=>enuV(p[0],p[1],p[2]!==undefined?p[2]:1.5));
    o.line.geometry.dispose();
    o.line.geometry=new THREE.BufferGeometry().setFromPoints(pts);
  }
  for(const id in drone3){
    if(ids.indexOf(Number(id))<0){
      scene3.remove(drone3[id].group);scene3.remove(drone3[id].pole);
      scene3.remove(drone3[id].line);scene3.remove(drone3[id].label);
      delete drone3[id];
    }
  }
  const sig=ids.join(',');
  if(followSel.dataset.sig!==sig){
    followSel.dataset.sig=sig;
    followSel.innerHTML='<option value="">Orbit</option>'
      +ids.map(i=>'<option value="'+i+'">Follow D'+i+'</option>').join('');
    if(followId&&ids.indexOf(followId)<0){setFollow(null);followSel.value='';}
  }
  const anyT=flight.drones.find(dr=>dr.target);
  if(anyT){target3.visible=true;
    target3.position.set(anyT.target.x,anyT.target.z||2,-anyT.target.y);}
  else target3.visible=false;
  const R=flight.geofence_radius||0;
  if(geofence3){geofence3.visible=R>0;
    if(R>0&&R!==geoR){geoR=R;geofence3.scale.set(R,1,R);geofence3.computeLineDistances();}}
}
function animate3D(now){
  requestAnimationFrame(animate3D);
  if(viewMode!=='3d'||!threeReady)return;
  const dt=Math.min(0.1,(now-lastFrame)/1000);lastFrame=now;
  for(const id in drone3){
    const o=drone3[id];
    if(!o.tp)continue;
    o.pos.lerp(o.tp,1-Math.exp(-dt*8));
    o.heading+=angDiff(o.tphi,o.heading)*(1-Math.exp(-dt*8));
    o.group.position.copy(o.pos);
    o.group.rotation.y=o.heading;
    o.label.position.set(o.pos.x,o.pos.y+3.1,o.pos.z);
    for(const r of o.rotors)r.rotation.y+=dt*45;
    o.pole.geometry.setFromPoints([new THREE.Vector3(o.pos.x,0.05,o.pos.z),o.pos.clone()]);
  }
  if(target3&&target3.visible){
    target3.rotation.y+=dt*0.9;
    target3.position.y+=Math.sin(now/700)*0.004;
  }
  if(followId&&drone3[followId]){
    const o=drone3[followId];
    const fwd=new THREE.Vector3(Math.cos(o.heading),0,-Math.sin(o.heading));
    const up=new THREE.Vector3(0,1,0);
    let want,look,kp,tp;
    if(camAngle==='pov'){
      want=o.pos.clone().addScaledVector(fwd,2.6).addScaledVector(up,0.18);
      look=o.pos.clone().addScaledVector(fwd,40).addScaledVector(up,0.18);
      kp=1-Math.exp(-dt*12);tp=kp;
    }else if(camAngle==='top'){
      want=o.pos.clone().addScaledVector(up,followZoom).add(new THREE.Vector3(0.001,0,0.001));
      look=o.pos.clone();
      kp=1-Math.exp(-dt*6);tp=1-Math.exp(-dt*7);
    }else{
      want=o.pos.clone().addScaledVector(fwd,-followZoom)
        .addScaledVector(up,followZoom*0.4);
      look=o.pos.clone().addScaledVector(fwd,3);
      kp=1-Math.exp(-dt*5);tp=1-Math.exp(-dt*7);
    }
    camera3.position.lerp(want,kp);
    controls3.target.lerp(look,tp);
  }
  controls3.update();
  renderer3.render(scene3,camera3);
}
document.getElementById('btn3d').onclick=()=>{
  viewMode='3d';
  sceneEl.style.display='block';map.style.display='none';
  document.getElementById('btn3d').classList.add('on');
  document.getElementById('btn2d').classList.remove('on');
  document.getElementById('f3dnote').style.display='';
  init3D();resize3D();syncCamang();
};
document.getElementById('btn2d').onclick=()=>{
  viewMode='2d';
  sceneEl.style.display='none';map.style.display='block';
  document.getElementById('btn2d').classList.add('on');
  document.getElementById('btn3d').classList.remove('on');
  document.getElementById('f3dnote').style.display='none';
  mapSize();syncCamang();
};
async function pollFlight(){
  try{const r=await fetch('/api/flight');flight=await r.json();
    document.getElementById('flage').textContent=flight.updated
      ?('updated '+new Date(flight.updated*1000).toLocaleTimeString()):'';
    renderCams();renderLegend();update3D();
    if(document.getElementById('tab-mgmt').style.display!=='none'){
      renderMgScope();renderMgCards();
    }
  }catch(e){document.getElementById('flage').textContent='flight view unavailable';}
}
pollFlight();setInterval(pollFlight,300);
init3D();
setInterval(drawMap,100);

/* ---- management tab ---- */
let mgMode='all',mgSel=new Set(),mgThumbTick=0;
const tabBtns=[...document.querySelectorAll('.tab')];
function setTab(name){
  for(const b of tabBtns)b.classList.toggle('on',b.dataset.tab===name);
  document.getElementById('dash').style.display=name==='dash'?'':'none';
  document.getElementById('tab-mgmt').style.display=name==='mgmt'?'':'none';
  if(name==='mgmt'){renderMgScope(true);renderMgCards();mgmSyncSel();}
  else{if(mgmTimer)mgmRelease();resize3D();mapSize();}
}
for(const b of tabBtns)b.addEventListener('click',()=>setTab(b.dataset.tab));
const smBtns=[...document.querySelectorAll('#mgscopemode .vt')];
function mgAllIds(){
  return (flight&&flight.drones?flight.drones:[]).map(d=>d.id).sort((a,b)=>a-b);
}
function mgScopeIds(){
  const all=mgAllIds();
  if(mgMode==='all')return all;
  const sel=[...mgSel].sort((a,b)=>a-b);
  return sel.length?sel:all;
}
function syncMgMode(){
  for(const b of smBtns)b.classList.toggle('on',b.dataset.smode===mgMode);
  const chips=document.getElementById('mgscope');
  chips.style.display=mgMode==='all'?'none':'flex';
  for(const c of chips.querySelectorAll('button'))
    c.classList.toggle('on',mgSel.has(+c.dataset.id));
  const ids=mgScopeIds();
  document.getElementById('mgscopesum').textContent=ids.length
    ?'orders apply to: '+ids.map(i=>'Drone '+i).join(', '):'no drones yet';
}
function renderMgScope(force){
  const chips=document.getElementById('mgscope'),ids=mgAllIds();
  const sig=ids.join(',');
  if(force||chips.dataset.sig!==sig){
    chips.dataset.sig=sig;
    chips.innerHTML=ids.map(id=>
      `<button class="vt" data-id="${id}">Drone ${id}</button>`).join('');
    for(const c of chips.querySelectorAll('button'))c.addEventListener('click',()=>{
      const id=+c.dataset.id;
      if(mgMode==='one'){mgSel=new Set([id]);}
      else{if(mgMode==='all'){mgMode='many';mgSel=new Set(ids);}
        mgSel.has(id)?mgSel.delete(id):mgSel.add(id);}
      syncMgMode();
    });
    if(ids.length&&!mgSel.size){mgMode='all';}
  }
  syncMgMode();
  mgmSyncSel();
}
for(const b of smBtns)b.addEventListener('click',()=>{
  mgMode=b.dataset.smode;
  const ids=mgAllIds();
  if(mgMode==='all')mgSel=new Set(ids);
  else if(mgMode==='many'&&!mgSel.size)mgSel=new Set(ids);
  else if(mgMode==='one'){const first=mgSel.values().next().value;mgSel=new Set([first||ids[0]].filter(Boolean));}
  syncMgMode();
});
function mgMeta(id){
  const d=(flight&&flight.drones||[]).find(x=>x.id===id)||{};
  const t=d.target;
  // A snapshot older than a few seconds cannot prove a link is down;
  // claim "offline" only from fresh telemetry.
  const fresh=!!flight&&(Date.now()/1000-(flight.updated||0))<5;
  let range=null;
  if(t){
    if(t.distance!==undefined)range=Number(t.distance);
    else if(d.x!==undefined)range=Math.hypot(t.x-d.x,t.y-d.y);
  }
  return {d:d,mission:d.mission||'',emergency:!!d.emergency,
    stale:!fresh,connected:fresh&&d.connected!==false,range:range};
}
function renderMgCards(){
  const ids=mgAllIds(),grid=document.getElementById('mgcards');
  const sig=ids.join(',');
  if(grid.dataset.sig!==sig){
    grid.dataset.sig=sig;grid.dataset.first='';
    grid.innerHTML=ids.map(id=>`<div class="mgcard" data-id="${id}">
      <div style="display:flex;justify-content:space-between;align-items:center">
        <b>Drone ${id}</b><span class="muted small" data-role="mission"></span></div>
      <img class="mgthumb" data-role="thumb" alt="">
      <div class="muted small" data-role="meta" style="margin-top:6px"></div></div>`).join('');
  }
  if(document.getElementById('tab-mgmt').style.display==='none')return;
  if(++mgThumbTick%10===0||!grid.dataset.first){
    grid.dataset.first='1';
    for(const img of grid.querySelectorAll('img[data-role=thumb]'))
      img.src='/api/camera/'+img.closest('.mgcard').dataset.id+'?t='+Date.now();
  }
  for(const card of grid.querySelectorAll('.mgcard')){
    const m=mgMeta(+card.dataset.id),bits=[];
    card.querySelector('[data-role=mission]').textContent=m.mission||'-';
    if(m.stale)bits.push('telemetry stale');
    else if(!m.connected)bits.push('offline');
    if(m.emergency)bits.push('EMERGENCY');
    bits.push(m.range!==null?'range '+m.range.toFixed(1)+' m'
      :(m.mission==='searching'?'holding / scanning':'no target'));
    card.querySelector('[data-role=meta]').textContent=bits.join(' · ');
  }
}
const VERDICT={'TARGET_CONFIRMED':['ok','Target confirmed'],
  'SUSPECT':['amb','Suspect'],'NOT_TARGET':['warn','Not my target'],
  'NO_OBJECT':['mut','No object']};
async function mgPost(path,body){
  const r=await fetch(path,{method:'POST',
    headers:{'Content-Type':'application/json'},body:JSON.stringify(body)});
  return r.json();
}
async function mgInstruct(ids,action,label){
  const msg=document.getElementById('mgmsg');
  if(!ids.length){msg.textContent='no drones selected';return;}
  msg.textContent=(label||action)+' → '+ids.map(i=>'D'+i).join(', ')+'…';
  try{
    const j=await mgPost('/api/manage',{drones:ids,action:action});
    msg.textContent=j.ok?((label||action)+' sent to '+j.drones.join(', '))
      :('error: '+j.error);
  }catch(e){msg.textContent='error: '+e;}
}
document.getElementById('mgPursue').onclick=()=>mgInstruct(mgScopeIds(),'pursue','Pursue');
document.getElementById('mgHold').onclick=()=>mgInstruct(mgScopeIds(),'hold','Hold');
document.getElementById('mgOverride').onclick=()=>mgInstruct(mgScopeIds(),
  'override','Override (pursue what the camera sees)');
document.getElementById('mgResume').onclick=async()=>{
  const msg=document.getElementById('mgmsg');msg.textContent='Resetting...';
  try{const r=await fetch('/api/reset',{method:'POST'});const j=await r.json();
    msg.textContent=j.ok?'emergencies cleared, missions resumed':'error: '+j.error;
  }catch(e){msg.textContent='error: '+e;}
};
/* ---- manual joystick / keyboard control ---- */
let mgmId=null,mgmTimer=null,mgmPadOn=false;
const mgmKeys=new Set();
const mgmKeyMap={KeyW:'n',KeyS:'s',KeyA:'w',KeyD:'e',
  KeyR:'u',KeyF:'d',KeyQ:'l',KeyE:'r'};
function mgmSyncSel(){
  const sel=document.getElementById('mgmSel');if(!sel)return;
  const ids=mgAllIds(),sig=ids.join(',');
  if(sel.dataset.sig!==sig){
    sel.dataset.sig=sig;
    sel.innerHTML=ids.map(id=>`<option value="${id}">Drone ${id}</option>`).join('');
  }
}
function mgmAxes(){
  let vx=0,vy=0,vz=0,wz=0;
  for(const k of mgmKeys){
    if(k==='n')vy+=1;else if(k==='s')vy-=1;else if(k==='e')vx+=1;
    else if(k==='w')vx-=1;else if(k==='u')vz+=1;else if(k==='d')vz-=1;
    else if(k==='l')wz+=0.8;else if(k==='r')wz-=0.8;
  }
  if(navigator.getGamepads){
    const pad=navigator.getGamepads()[0];
    if(pad&&pad.axes.length>=4){
      const dz=v=>Math.abs(v)<0.15?0:v;
      mgmPadOn=true;
      vx+=dz(pad.axes[0]);
      vy+=-dz(pad.axes[1]);
      vz+=-dz(pad.axes[3]);
      wz+=-dz(pad.axes[2])*0.9;
    }
  }
  const sp=Math.hypot(vx,vy);
  if(sp>1){vx/=sp;vy/=sp;}
  const vmax=cfgNow?Math.min(4,cfgNow.defaults.max_velocity):2;
  return {vx:vx*vmax,vy:vy*vmax,
    vz:Math.max(-1.5,Math.min(1.5,vz)),
    yaw:Math.max(-1,Math.min(1,wz))};
}
async function mgmTick(){
  if(mgmId===null)return;
  const a=mgmAxes(),st=document.getElementById('mgmState');
  st.textContent=`D${mgmId} → vx ${a.vx.toFixed(1)} vy ${a.vy.toFixed(1)} `
    +`vz ${a.vz.toFixed(1)} yaw ${a.yaw.toFixed(1)}`
    +(mgmPadOn?' · gamepad':'');
  try{
    await fetch('/api/manual',{method:'POST',
      headers:{'Content-Type':'application/json'},
      body:JSON.stringify({drone:mgmId,vx:a.vx,vy:a.vy,vz:a.vz,yaw:a.yaw})});
  }catch(e){st.textContent='manual send failed: '+e;}
}
function mgmRelease(){
  if(mgmTimer){clearInterval(mgmTimer);mgmTimer=null;}
  if(mgmId!==null){
    fetch('/api/manual',{method:'POST',
      headers:{'Content-Type':'application/json'},
      body:JSON.stringify({drone:mgmId,vx:0,vy:0,vz:0,yaw:0})}).catch(()=>{});
  }
  mgmId=null;mgmKeys.clear();
  const btn=document.getElementById('mgmTake'),sel=document.getElementById('mgmSel');
  btn.textContent='Take control';btn.classList.remove('on');sel.disabled=false;
  document.getElementById('mgmHints').style.display='none';
  document.getElementById('mgmState').textContent='released — drone holds';
}
document.getElementById('mgmTake').onclick=()=>{
  if(mgmTimer){mgmRelease();return;}
  const ids=mgAllIds();
  mgmId=+document.getElementById('mgmSel').value||ids[0];
  if(!mgmId){document.getElementById('mgmState').textContent='no drones';return;}
  const btn=document.getElementById('mgmTake');
  btn.textContent='Release control';btn.classList.add('on');
  document.getElementById('mgmSel').disabled=true;
  document.getElementById('mgmHints').style.display='';
  document.getElementById('mgmState').textContent='taking control of D'+mgmId+'…';
  mgmTimer=setInterval(mgmTick,120);
  mgmTick();
};
window.addEventListener('keydown',e=>{
  const k=mgmKeyMap[e.code];
  if(!k||!mgmTimer)return;
  if(e.target&&/INPUT|SELECT|TEXTAREA/.test(e.target.tagName))return;
  e.preventDefault();mgmKeys.add(k);
});
window.addEventListener('keyup',e=>{
  const k=mgmKeyMap[e.code];if(k)mgmKeys.delete(k);
});
window.addEventListener('keydown',e=>{
  if(e.code==='Space'&&mgmTimer){e.preventDefault();mgmRelease();}
});
window.addEventListener('gamepadconnected',()=>{
  mgmPadOn=true;document.getElementById('mgmPad').textContent='gamepad connected';
});
window.addEventListener('gamepaddisconnected',()=>{
  mgmPadOn=false;document.getElementById('mgmPad').textContent='';
});
document.getElementById('mgAnalyse').onclick=async()=>{
  const msg=document.getElementById('mgmsg'),ids=mgScopeIds();
  if(!ids.length){msg.textContent='no drones selected';return;}
  msg.textContent='analysing '+ids.map(i=>'D'+i).join(', ')+'…';
  try{
    const j=await mgPost('/api/analyse',{drones:ids});
    if(!j.ok){msg.textContent='error: '+j.error;return;}
    msg.textContent='analysed '+j.results.length+' drone(s)';
    renderMgResults(j.results);
  }catch(e){msg.textContent='error: '+e;}
};
function renderMgResults(res){
  const el=document.getElementById('mres');
  el.innerHTML='<h4 style="margin:0 0 8px;font-size:14px">Photo analysis</h4><table><thead><tr>'
    +'<th>Drone</th><th>Mission</th><th>Verdict &amp; reasoning</th><th>Seen</th><th>Range</th><th>Order</th><th></th></tr></thead><tbody>'
    +res.map(r=>{
      const v=VERDICT[r.verdict]||['mut',r.verdict];
      const seen=(r.cls?esc(r.cls):'-')
        +(r.colour&&r.colour!=='unknown'?' / '+esc(r.colour):'')
        +(r.confidence!==undefined?' · '+Math.round(r.confidence*100)+'%':'');
      const range=r.range!==null&&r.range!==undefined?r.range.toFixed(1)+' m':'—';
      return `<tr><td><b>D${r.id}</b></td><td>${esc(r.mission||'-')}</td>`
        +`<td><span class="vbadge ${v[0]}">${v[1]}</span>`
        +(r.overridden?' <span class="vbadge amb">operator override</span>':'')
        +`<div class="muted small">${esc(r.reason||'')}</div></td>`
        +`<td>${seen}</td><td>${range}</td><td>${r.recommendation}</td>`
        +`<td><button class="vt mgapply" data-id="${r.id}" data-act="${r.recommendation}">Do: ${r.recommendation}</button>`
        +((r.verdict==='NOT_TARGET'||r.verdict==='SUSPECT')
          ?` <button class="vt mgoverride" data-id="${r.id}" title="The classifier got this wrong — pursue what the camera sees">It IS the target</button>`
          :'')
        +`</td></tr>`;
    }).join('')+'</tbody></table>'
    +'<div style="margin-top:8px"><button id="mgApplyAll" class="vt">Apply each recommendation</button></div>';
  for(const b of el.querySelectorAll('.mgapply'))
    b.addEventListener('click',()=>mgInstruct([+b.dataset.id],b.dataset.act));
  for(const b of el.querySelectorAll('.mgoverride'))
    b.addEventListener('click',()=>mgInstruct([+b.dataset.id],'override',
      'Override (pursue what the camera sees)'));
  document.getElementById('mgApplyAll').onclick=()=>{
    const pursue=res.filter(r=>r.recommendation==='pursue').map(r=>r.id);
    const hold=res.filter(r=>r.recommendation!=='pursue').map(r=>r.id);
    if(pursue.length)mgInstruct(pursue,'pursue','Pursue');
    if(hold.length)setTimeout(()=>mgInstruct(hold,'hold','Hold'),900);
  };
}
if(flight)renderMgScope(true);

refresh();setInterval(refresh,4000);
document.getElementById('reset').onclick=async()=>{
  const msg=document.getElementById('rmsg');msg.textContent='Resetting...';
  try{
    const r=await fetch('/api/reset',{method:'POST'});
    const j=await r.json();
    if(!j.ok){msg.textContent='Error: '+j.error;return;}
    msg.textContent=Object.entries(j.drones).map(([ns,s])=>`${ns}: ${s.error?('error '+s.error):(s.cleared?'cleared':'not latched')}${s.resumed?' + resumed':''}`).join(' | ');
  }catch(e){msg.textContent='Error: '+e;}
};
</script></body></html>'''


def make_handler(supervisor):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def _send(self, code, body, ctype='application/json'):
            data = body.encode() if isinstance(body, str) else body
            self.send_response(code)
            self.send_header('Content-Type', ctype)
            # Telemetry must never be served from the browser cache: a stale
            # snapshot right after a reconnect shows drones as offline.
            self.send_header('Cache-Control', 'no-store, no-cache, must-revalidate')
            self.send_header('Pragma', 'no-cache')
            self.send_header('Content-Length', str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        @staticmethod
        def _scope(supervisor, body):
            count = int(supervisor.cfg['fleet']['count'])
            ids = body.get('drones')
            if not isinstance(ids, list) or not ids:
                raise ValueError('drones must be a non-empty list of ids')
            namespaces = []
            for raw in ids:
                drone_id = int(raw)
                if not 1 <= drone_id <= count:
                    raise ValueError(f'drone id out of range: {drone_id}')
                namespaces.append(f'drone_{drone_id}')
            return namespaces

        def do_GET(self):
            if self.path == '/':
                self._send(200, INDEX_HTML, 'text/html')
            elif self.path == '/api/status':
                self._send(200, json.dumps(
                    {'config': supervisor.cfg, **supervisor.status,
                     'observability': supervisor.observability,
                     'events': supervisor.events_snapshot()}))
            elif self.path == '/api/flight':
                self._send(200, json.dumps(supervisor.flight))
            elif self.path.startswith('/static/'):
                name = self.path.split('/static/', 1)[1].split('?', 1)[0]
                if name not in STATIC_FILES:
                    self._send(404, '{}')
                    return
                try:
                    with open(os.path.join(STATIC_DIR, name), 'rb') as handle:
                        data = handle.read()
                except FileNotFoundError:
                    self._send(404, '{}')
                    return
                self._send(200, data, 'application/javascript')
            elif self.path.startswith('/api/camera/'):
                tail = self.path.split('/api/camera/', 1)[1]
                tail = tail.split('?', 1)[0].split('.', 1)[0]
                if not tail.isdigit():
                    self._send(404, '{}')
                    return
                namespace = f'drone_{int(tail)}'
                if not supervisor.cfg['fleet'].get('perception_demo'):
                    self._send(409, json.dumps(
                        {'error': 'perception demo disabled'}))
                    return
                supervisor.ros_cache.ensure_camera(namespace)
                jpeg = supervisor.ros_cache.frame(namespace)
                if jpeg is None:
                    self._send(503, json.dumps(
                        {'error': 'no camera frame yet'}))
                else:
                    self._send(200, jpeg, 'image/jpeg')
            else:
                self._send(404, '{}')

        def do_POST(self):
            if self.path == '/api/reset':
                try:
                    results = supervisor.reset_fleet()
                    supervisor.ros_cache.overridden.clear()
                    supervisor.event('config', 'manual reset: '
                                     + ', '.join(sorted(results)))
                    self._send(200, json.dumps(
                        {'ok': True, 'drones': results}))
                except Exception as exc:
                    self._send(500, json.dumps(
                        {'ok': False, 'error': str(exc)}))
                return
            if self.path == '/api/scenario':
                length = int(self.headers.get('Content-Length') or 0)
                try:
                    body = json.loads(self.rfile.read(length) or '{}')
                    mode = str(body.get('mode', '')).lower()
                    if mode not in ('target', 'decoy', 'none'):
                        raise ValueError('mode must be target/decoy/none')
                    rows = supervisor.status.get('drones') or []
                    namespaces = [row['namespace'] for row in rows]
                    supervisor.ros_cache.publish_scenario(namespaces, mode)
                    supervisor.event('vision', f'scenario {mode} sent to '
                                     + ', '.join(namespaces))
                    self._send(200, json.dumps({'ok': True, 'mode': mode}))
                except Exception as exc:
                    self._send(400, json.dumps(
                        {'ok': False, 'error': str(exc)}))
                return
            if self.path in ('/api/analyse', '/api/manage'):
                length = int(self.headers.get('Content-Length') or 0)
                try:
                    body = json.loads(self.rfile.read(length) or '{}')
                    namespaces = self._scope(supervisor, body)
                    if self.path == '/api/analyse':
                        for namespace in namespaces:
                            supervisor.ros_cache.ensure_camera(namespace)
                        results = supervisor.ros_cache.analyse(
                            namespaces,
                            float(supervisor.cfg['defaults']
                                  ['standoff_distance']))
                        self._send(200, json.dumps(
                            {'ok': True, 'results': results}))
                    else:
                        action = str(body.get('action', '')).lower()
                        if action not in ('pursue', 'hold', 'override'):
                            raise ValueError(
                                'action must be pursue/hold/override')
                        if action == 'pursue':
                            supervisor.ros_cache.publish_scenario(
                                namespaces, 'target')
                            supervisor.ros_cache.publish_tracking_command(
                                namespaces, 'engage')
                            supervisor.ros_cache.overridden.difference_update(
                                namespaces)
                        elif action == 'hold':
                            supervisor.ros_cache.publish_scenario(
                                namespaces, 'none')
                            supervisor.ros_cache.publish_tracking_command(
                                namespaces, 'hold')
                            supervisor.ros_cache.overridden.difference_update(
                                namespaces)
                        else:
                            supervisor.ros_cache.overridden.update(
                                namespaces)
                            # Keep the scenario as-is: pursue whatever the
                            # camera currently reports, class check bypassed.
                            supervisor.ros_cache.publish_tracking_command(
                                namespaces, 'override')
                        supervisor.event(
                            'vision', f'instruction {action} -> '
                            + ', '.join(namespaces))
                        self._send(200, json.dumps(
                            {'ok': True, 'action': action,
                             'drones': namespaces}))
                except Exception as exc:
                    self._send(400, json.dumps(
                        {'ok': False, 'error': str(exc)}))
                return
            if self.path == '/api/manual':
                length = int(self.headers.get('Content-Length') or 0)
                try:
                    body = json.loads(self.rfile.read(length) or '{}')
                    count = int(supervisor.cfg['fleet']['count'])
                    drone_id = int(body.get('drone'))
                    if not 1 <= drone_id <= count:
                        raise ValueError(
                            f'drone id out of range: {drone_id}')
                    max_v = float(
                        supervisor.cfg['defaults']['max_velocity'])
                    vx = max(-max_v, min(float(body.get('vx', 0.0)), max_v))
                    vy = max(-max_v, min(float(body.get('vy', 0.0)), max_v))
                    vz = max(-2.0, min(float(body.get('vz', 0.0)), 2.0))
                    wz = max(-1.0, min(float(body.get('yaw', 0.0)), 1.0))
                    supervisor.ros_cache.publish_manual(
                        f'drone_{drone_id}', vx, vy, vz, wz)
                    self._send(200, json.dumps({'ok': True}))
                except Exception as exc:
                    self._send(400, json.dumps(
                        {'ok': False, 'error': str(exc)}))
                return
            if self.path != '/api/apply':
                self._send(404, '{}')
                return
            length = int(self.headers.get('Content-Length') or 0)
            try:
                updates = json.loads(self.rfile.read(length) or '{}')
                result = supervisor.apply(updates)
                if result.get('restarted'):
                    supervisor.ros_cache.overridden.clear()
                self._send(200, json.dumps({'ok': True, **result}))
            except Exception as exc:
                self._send(400, json.dumps({'ok': False, 'error': str(exc)}))
    return Handler


def run_portal(supervisor):
    port = int(os.environ.get('FLEET_PORTAL_PORT') or
               supervisor.cfg['fleet'].get('portal_port', 8090))
    server = ThreadingHTTPServer(('0.0.0.0', port), make_handler(supervisor))
    print(f'[fleet] portal listening on :{port}', flush=True)
    server.serve_forever()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command', choices=['render', 'run'])
    args = parser.parse_args()

    if args.command == 'render':
        cfg = load_config()
        drones = reconcile(cfg)
        changed = render(cfg, drones)
        for drone in drones:
            print(f'drone {drone["id"]}: {drone["namespace"]} '
                  f'px4={drone["container"]} ip={drone["px4_ip"]}')
        print('rendered (changed)' if changed else 'rendered (no changes)')
        return

    supervisor = Supervisor()
    supervisor.boot()
    threading.Thread(target=supervisor.watch_ros, daemon=True).start()
    threading.Thread(target=supervisor.sample_loop, daemon=True).start()
    threading.Thread(target=supervisor.probe_loop, daemon=True).start()

    def _shutdown(signum, _frame):
        supervisor.stop_ros()
        sys.exit(0)

    signal.signal(signal.SIGTERM, _shutdown)
    signal.signal(signal.SIGINT, _shutdown)
    run_portal(supervisor)


if __name__ == '__main__':
    main()
