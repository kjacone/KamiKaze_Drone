#!/usr/bin/env python3
"""Spawn the two static scenario props into the running gz world.

target_vehicle: red car-sized box 12 m east of the spawn pad.
bystander:      grey pedestrian-sized cylinder 12 m east, 5 m north.

The ROS-side vision_scenario node projects these same geometry constants,
so bboxes published as detections line up with what the camera renders.
"""

import sys
import time

from gz.msgs10.entity_factory_pb2 import EntityFactory
from gz.msgs10.boolean_pb2 import Boolean
from gz.transport13 import Node

WORLD = 'default'

PROPS = {
    'scenario_target_vehicle': '''
<sdf version="1.9"><model name="scenario_target_vehicle">
  <static>true</static><pose>12 0 0.75 0 0 0</pose>
  <link name="body">
    <visual name="shell">
      <geometry><box><size>4.6 1.8 1.5</size></box></geometry>
      <material><ambient>0.75 0.08 0.08 1</ambient>
        <diffuse>0.85 0.1 0.1 1</diffuse></material>
    </visual>
    <visual name="cabin">
      <pose>-0.2 0 0.95 0 0 0</pose>
      <geometry><box><size>2.2 1.6 0.9</size></box></geometry>
      <material><ambient>0.1 0.1 0.15 1</ambient>
        <diffuse>0.15 0.15 0.2 1</diffuse></material>
    </visual>
  </link>
</model></sdf>''',
    'scenario_bystander': '''
<sdf version="1.9"><model name="scenario_bystander">
  <static>true</static><pose>12 5 0.9 0 0 0</pose>
  <link name="body">
    <visual name="torso">
      <geometry><cylinder><radius>0.35</radius><length>1.8</length></cylinder></geometry>
      <material><ambient>0.6 0.6 0.62 1</ambient>
        <diffuse>0.7 0.7 0.72 1</diffuse></material>
    </visual>
  </link>
</model></sdf>''',
}


def spawn(node, name, sdf):
    req = EntityFactory()
    req.name = name
    req.sdf = sdf
    req.allow_renaming = False
    ok, _resp = node.request(f'/world/{WORLD}/create', req,
                             EntityFactory, Boolean, 3000)
    return ok


def main():
    node = Node()
    for _attempt in range(60):
        results = [spawn(node, name, sdf) for name, sdf in PROPS.items()]
        if all(results):
            print('gz_scenario: props spawned', flush=True)
            return 0
        time.sleep(2)
    print('gz_scenario: failed to spawn props', flush=True)
    return 1


if __name__ == '__main__':
    sys.exit(main())
