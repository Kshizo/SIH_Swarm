#!/usr/bin/env python3
"""Generate the sensor-equipped drone models used by the maze_survivors world.

The flight-dynamics half of the model (rotors, lift-drag, ArduPilotPlugin) is
taken from ardupilot_gazebo's ``iris_with_ardupilot``, which is LGPL-3.0.  We do
not vendor a copy of it into this repository; instead this script reads the
installed model and splices in:

  * a 2D lidar on ``laser_link``   -> <prefix>/lidar/scan
  * an RGB camera on ``camera_link``  -> <prefix>/camera/image
  * a depth camera on ``camera_link`` -> <prefix>/camera/depth_image

and rewrites the model name plus the ArduPilot FDM port so that two drones can
share one Gazebo instance.  Generated models land in ``sim/models/`` and are
git-ignored.

Sensor placement matches what the ROS nodes already assume:
  * laser_link at (0, 0, 0.05) w.r.t. base_link            (drone_mapping/urdf/drone.urdf)
  * camera at (0.10, 0, 0.03), pitched 0.4109 rad down     (ssd_lite_ros/person_detector.py)
  * camera hfov 1.03 rad                                   (ssd_lite_ros/person_detector.py)
"""

import argparse
import os
import re
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
SIM_DIR = HERE.parent

# Geometry the ROS stack assumes. Keep in sync with drone.urdf / person_detector.py.
LASER_POSE = (0.0, 0.0, 0.05, 0.0, 0.0, 0.0)
CAMERA_POSE = (0.10, 0.0, 0.03, 0.0, 0.4109, 0.0)
CAMERA_HFOV = 1.03
CAMERA_W, CAMERA_H = 640, 480

SENSOR_BLOCK = """
    <!-- ================= Sensor payload (added by gen_drone_models.py) ================= -->

    <link name="laser_link">
      <pose>{lx} {ly} {lz} {lr} {lp} {lyaw}</pose>
      <inertial>
        <mass>0.05</mass>
        <inertia>
          <ixx>1e-05</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>1e-05</iyy><iyz>0</iyz><izz>1e-05</izz>
        </inertia>
      </inertial>
      <visual name="laser_visual">
        <geometry><cylinder><radius>0.03</radius><length>0.03</length></cylinder></geometry>
        <material>
          <ambient>0.1 0.1 0.1 1</ambient>
          <diffuse>0.2 0.2 0.2 1</diffuse>
        </material>
      </visual>
      <sensor name="lidar" type="gpu_lidar">
        <pose>0 0 0 0 0 0</pose>
        <topic>{prefix}/lidar/scan</topic>
        <gz_frame_id>laser_link</gz_frame_id>
        <update_rate>20</update_rate>
        <always_on>1</always_on>
        <visualize>true</visualize>
        <lidar>
          <scan>
            <horizontal>
              <samples>360</samples>
              <resolution>1</resolution>
              <min_angle>-3.141592653589793</min_angle>
              <max_angle>3.141592653589793</max_angle>
            </horizontal>
            <vertical>
              <samples>1</samples>
              <resolution>1</resolution>
              <min_angle>0</min_angle>
              <max_angle>0</max_angle>
            </vertical>
          </scan>
          <range>
            <min>0.20</min>
            <max>12.0</max>
            <resolution>0.01</resolution>
          </range>
          <noise>
            <type>gaussian</type>
            <mean>0.0</mean>
            <stddev>0.01</stddev>
          </noise>
        </lidar>
      </sensor>
    </link>

    <joint name="laser_joint" type="fixed">
      <parent>iris_with_standoffs::base_link</parent>
      <child>laser_link</child>
    </joint>

    <link name="camera_link">
      <pose>{cx} {cy} {cz} {cr} {cp} {cyaw}</pose>
      <inertial>
        <mass>0.05</mass>
        <inertia>
          <ixx>1e-05</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>1e-05</iyy><iyz>0</iyz><izz>1e-05</izz>
        </inertia>
      </inertial>
      <visual name="camera_visual">
        <geometry><box><size>0.03 0.05 0.03</size></box></geometry>
        <material>
          <ambient>0.05 0.05 0.05 1</ambient>
          <diffuse>0.1 0.1 0.1 1</diffuse>
        </material>
      </visual>
      <sensor name="rgb_camera" type="camera">
        <pose>0 0 0 0 0 0</pose>
        <topic>{prefix}/camera/image</topic>
        <gz_frame_id>camera_link</gz_frame_id>
        <update_rate>15</update_rate>
        <always_on>1</always_on>
        <camera>
          <horizontal_fov>{hfov}</horizontal_fov>
          <image>
            <width>{width}</width>
            <height>{height}</height>
            <format>R8G8B8</format>
          </image>
          <clip><near>0.1</near><far>50.0</far></clip>
          <optical_frame_id>camera_link</optical_frame_id>
        </camera>
      </sensor>
      <!-- Same pose / FOV / resolution as the RGB camera: person_detector.py indexes
           the depth image with RGB pixel coordinates, so the two must be aligned. -->
      <sensor name="depth_camera" type="depth_camera">
        <pose>0 0 0 0 0 0</pose>
        <topic>{prefix}/camera/depth_image</topic>
        <gz_frame_id>camera_link</gz_frame_id>
        <update_rate>15</update_rate>
        <always_on>1</always_on>
        <camera>
          <horizontal_fov>{hfov}</horizontal_fov>
          <image>
            <width>{width}</width>
            <height>{height}</height>
            <format>R_FLOAT32</format>
          </image>
          <clip><near>0.1</near><far>50.0</far></clip>
          <optical_frame_id>camera_link</optical_frame_id>
        </camera>
      </sensor>
    </link>

    <joint name="camera_joint" type="fixed">
      <parent>iris_with_standoffs::base_link</parent>
      <child>camera_link</child>
    </joint>
"""

MODEL_CONFIG = """<?xml version="1.0"?>
<model>
  <name>{name}</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <description>
    iris_with_ardupilot (ardupilot_gazebo, LGPL-3.0) plus a 2D lidar and an
    RGB-D camera. Generated by sim/tools/gen_drone_models.py - do not edit.
    Publishes on {prefix}/... and talks to ArduPilot SITL on FDM port {port}.
  </description>
</model>
"""

PROVENANCE = """<!--
  GENERATED FILE - do not edit. Regenerate with:
      python3 sim/tools/gen_drone_models.py

  Derived from ardupilot_gazebo's `iris_with_ardupilot` model, which is
  licensed LGPL-3.0 (https://github.com/ArduPilot/ardupilot_gazebo).
  Only the sensor payload below and the model name / FDM port are ours.
-->
"""


def find_source_model(explicit=None):
    candidates = []
    if explicit:
        candidates.append(Path(explicit))
    for entry in os.environ.get('GZ_SIM_RESOURCE_PATH', '').split(':'):
        if entry:
            candidates.append(Path(entry) / 'iris_with_ardupilot')
    candidates.append(Path.home() / 'ardupilot_gazebo' / 'models' / 'iris_with_ardupilot')
    for candidate in candidates:
        model = candidate / 'model.sdf'
        if model.is_file():
            return model
    raise SystemExit(
        'Could not find the ardupilot_gazebo `iris_with_ardupilot` model.\n'
        'Install https://github.com/ArduPilot/ardupilot_gazebo and either set\n'
        'GZ_SIM_RESOURCE_PATH to its models/ directory or pass --source.')


def build_model(source_sdf, name, prefix, fdm_port):
    text = source_sdf.read_text()

    text, count = re.subn(r'<model name="iris_with_ardupilot">',
                          f'<model name="{name}">', text, count=1)
    if count != 1:
        raise SystemExit(f'Unexpected model name in {source_sdf}')

    text, count = re.subn(r'<fdm_port_in>\s*\d+\s*</fdm_port_in>',
                          f'<fdm_port_in>{fdm_port}</fdm_port_in>', text, count=1)
    if count != 1:
        raise SystemExit(f'Could not find <fdm_port_in> in {source_sdf}')

    sensors = SENSOR_BLOCK.format(
        prefix=prefix,
        lx=LASER_POSE[0], ly=LASER_POSE[1], lz=LASER_POSE[2],
        lr=LASER_POSE[3], lp=LASER_POSE[4], lyaw=LASER_POSE[5],
        cx=CAMERA_POSE[0], cy=CAMERA_POSE[1], cz=CAMERA_POSE[2],
        cr=CAMERA_POSE[3], cp=CAMERA_POSE[4], cyaw=CAMERA_POSE[5],
        hfov=CAMERA_HFOV, width=CAMERA_W, height=CAMERA_H)

    marker = text.rindex('</model>')
    text = text[:marker] + sensors + '\n  ' + text[marker:]

    # Put the provenance note right after the XML declaration.
    lines = text.splitlines(keepends=True)
    return lines[0] + PROVENANCE + ''.join(lines[1:])


DRONES = [
    # (model directory / name, topic prefix, ArduPilot SITL instance)
    ('iris_with_sensors', '/drone1', 0),
    ('iris_with_sensors_2', '/drone2', 1),
]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', help='path to the iris_with_ardupilot model directory')
    parser.add_argument('--out', default=str(SIM_DIR / 'models'),
                        help='output models directory (default: sim/models)')
    args = parser.parse_args()

    source_sdf = find_source_model(args.source)
    out_root = Path(args.out)

    for name, prefix, instance in DRONES:
        fdm_port = 9002 + 10 * instance
        target = out_root / name
        target.mkdir(parents=True, exist_ok=True)
        (target / 'model.sdf').write_text(build_model(source_sdf, name, prefix, fdm_port))
        (target / 'model.config').write_text(
            MODEL_CONFIG.format(name=name, prefix=prefix, port=fdm_port))
        print(f'wrote {target}  (topics {prefix}/..., FDM port {fdm_port}, SITL -I{instance})')

    print(f'source: {source_sdf}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
