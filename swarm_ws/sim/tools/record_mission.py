#!/usr/bin/env python3
"""Record a swarm mission to an mp4, straight from the ROS graph.

Each drone runs on its own ROS_DOMAIN_ID, so a single node cannot see both. This
opens one rclpy context per domain and spins them side by side, which is what
lets both drones' camera views land in the same video.

Six panels:

    overhead camera   |  drone 1 detector view  |  drone 1 SLAM map
    mission summary   |  drone 2 detector view  |  drone 2 SLAM map

Nothing here touches the desktop, so it works headless and captures only the
simulation - no X server, no ffmpeg, no screen recorder.

    python3 sim/tools/record_mission.py --out mission.mp4 --drones 2

Recording stops once every drone has disarmed after landing, or after
--duration seconds, whichever comes first.
"""

import argparse
import math
import os
import threading
import time

import cv2
import numpy as np
import rclpy
import rclpy.executors
import tf2_ros
from ardupilot_msgs.msg import Status
from cv_bridge import CvBridge
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.context import Context
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from visualization_msgs.msg import MarkerArray

PANEL = 640                     # every panel is PANEL x PANEL
BAR = 58
FPS = 10

# Palette, matched to the project's pages so stills from either read as one set.
BG = (17, 18, 15)
INK = (231, 235, 231)
MUTED = (154, 160, 151)
ACCENT = (68, 165, 239)         # BGR: hi-vis amber
OK = (168, 192, 82)             # BGR: teal
FREE = (46, 50, 44)
OCCUPIED = (206, 212, 205)
TRAIL = (120, 140, 110)
HAZARD = (60, 158, 237)         # BGR: amber, narrowed passage
BLOCKED = (80, 69, 217)         # BGR: red, no route

MODES = {0: 'STABILIZE', 2: 'ALT_HOLD', 4: 'GUIDED', 5: 'LOITER', 6: 'RTL', 9: 'LAND'}


def put(img, text, org, scale=0.5, color=INK, weight=1):
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, color, weight, cv2.LINE_AA)


def placeholder(label):
    panel = np.full((PANEL, PANEL, 3), BG, np.uint8)
    put(panel, label, (24, PANEL // 2), 0.5, MUTED)
    return panel


def fit(image, size=PANEL):
    """Letterbox an image into a square panel without distorting it."""
    height, width = image.shape[:2]
    scale = min(size / width, size / height)
    resized = cv2.resize(image, (max(1, int(width * scale)), max(1, int(height * scale))),
                         interpolation=cv2.INTER_AREA)
    panel = np.full((size, size, 3), BG, np.uint8)
    y = (size - resized.shape[0]) // 2
    x = (size - resized.shape[1]) // 2
    panel[y:y + resized.shape[0], x:x + resized.shape[1]] = resized
    return panel


def label_panel(panel, text):
    cv2.rectangle(panel, (0, 0), (PANEL, 30), BG, -1)
    put(panel, text, (16, 21), 0.48, MUTED)
    return panel


def crop_to_known(canvas, grid, info, margin_m=1.5):
    """Crop to the explored region, squared off, with a little margin.

    slam_toolbox grows the grid well past the explored area; without this the
    map is a sliver of pixels in the middle of the panel.
    """
    known = grid >= 0
    if not known.any():
        return canvas

    rows = np.where(np.any(known, axis=1))[0]
    cols = np.where(np.any(known, axis=0))[0]
    height = canvas.shape[0]
    # The canvas was flipped vertically, so grid rows count from the bottom.
    top, bottom = height - 1 - rows[-1], height - 1 - rows[0]
    left, right = cols[0], cols[-1]

    margin = int(margin_m / info.resolution)
    top, bottom, left, right = top - margin, bottom + margin, left - margin, right + margin

    side = max(bottom - top, right - left)
    cy, cx = (top + bottom) // 2, (left + right) // 2
    top, bottom = cy - side // 2, cy + side // 2
    left, right = cx - side // 2, cx + side // 2

    # Pad rather than clamp, so a crop at the edge keeps its aspect ratio.
    pad_top, pad_left = max(0, -top), max(0, -left)
    pad_bottom = max(0, bottom - canvas.shape[0])
    pad_right = max(0, right - canvas.shape[1])
    if pad_top or pad_left or pad_bottom or pad_right:
        canvas = cv2.copyMakeBorder(canvas, pad_top, pad_bottom, pad_left, pad_right,
                                    cv2.BORDER_CONSTANT, value=BG)
        top, bottom = top + pad_top, bottom + pad_top
        left, right = left + pad_left, right + pad_left

    return canvas[top:bottom, left:right]


class DroneFeed:
    """Everything filmed from one drone, on that drone's own ROS domain."""

    def __init__(self, drone_id, want_observer, launch_pose=(0.0, 0.0, 0.0)):
        self.drone_id = drone_id
        self.launch_pose = launch_pose
        self.bridge = CvBridge()
        self.detection = None
        self.observer = None
        self.map_msg = None
        self.path = []
        self.pins = []
        self.mission_pins = []
        self.hazards = []
        self.status = None
        self.trail = []
        self.survivors = 0
        self.armed_once = False
        self.disarmed_at = None
        self.lock = threading.Lock()

        self.context = Context()
        rclpy.init(context=self.context, domain_id=drone_id)
        self.node = rclpy.create_node('mission_recorder_d%d' % drone_id, context=self.context)
        self.tf_buffer = tf2_ros.Buffer(node=self.node)
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.node)

        sensor = qos_profile_sensor_data
        self.node.create_subscription(Image, '/detection/image', self.on_detection, 10)
        self.node.create_subscription(OccupancyGrid, '/map', self.on_map, 10)
        self.node.create_subscription(Path, '/exploration/planned_path', self.on_path, 10)
        self.node.create_subscription(MarkerArray, '/person_map_pins', self.on_pins, 10)
        self.node.create_subscription(MarkerArray, '/hazard_markers', self.on_hazards, 10)
        self.node.create_subscription(Status, '/ap/status', self.on_status, sensor)
        if want_observer:
            # The overhead camera is bridged into drone 1's domain only.
            self.node.create_subscription(Image, '/observer/image', self.on_observer, sensor)

        self.executor = rclpy.executors.SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()

    def to_mission_frame(self, x, y):
        """This drone's map frame -> the shared mission frame."""
        ox, oy, yaw = self.launch_pose
        cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
        return (ox + cos_yaw * x - sin_yaw * y,
                oy + sin_yaw * x + cos_yaw * y)

    # ------------------------------------------------------------ callbacks
    def on_detection(self, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception:
            return
        with self.lock:
            self.detection = image

    def on_observer(self, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception:
            return
        with self.lock:
            self.observer = image

    def on_map(self, msg):
        with self.lock:
            self.map_msg = msg

    def on_path(self, msg):
        with self.lock:
            self.path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]

    def on_pins(self, msg):
        pins = [(m.pose.position.x, m.pose.position.y)
                for m in msg.markers if m.action != 2]
        if not pins:
            return
        with self.lock:
            self.pins = pins
            self.survivors = max(self.survivors, len(pins))
            self.mission_pins = [self.to_mission_frame(x, y) for x, y in pins]

    def on_hazards(self, msg):
        hazards = [(m.pose.position.x, m.pose.position.y, m.ns)
                   for m in msg.markers
                   if m.action == 0 and not m.ns.endswith('_label')]
        with self.lock:
            self.hazards = hazards

    def on_status(self, msg):
        with self.lock:
            self.status = msg
            if msg.armed:
                self.armed_once = True
                self.disarmed_at = None
            elif self.armed_once and self.disarmed_at is None:
                self.disarmed_at = time.time()

    # --------------------------------------------------------------- state
    def pose(self):
        try:
            tf = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        except Exception:
            return None
        t, q = tf.transform.translation, tf.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y ** 2 + q.z ** 2))
        return t.x, t.y, yaw

    def state_text(self):
        with self.lock:
            status = self.status
            armed_once = self.armed_once
        if status is None:
            return 'no /ap/status', MUTED
        if status.armed:
            return 'ARMED  ' + MODES.get(status.mode, 'mode ' + str(status.mode)), OK
        if armed_once:
            return 'landed, disarmed', MUTED
        return 'disarmed', MUTED

    def finished(self):
        with self.lock:
            return self.disarmed_at is not None and time.time() - self.disarmed_at > 5.0

    # -------------------------------------------------------------- panels
    def detection_panel(self):
        with self.lock:
            image = None if self.detection is None else self.detection.copy()
        if image is None:
            return label_panel(placeholder('waiting for /detection/image'),
                               'drone %d camera' % self.drone_id)
        return label_panel(fit(image), 'drone %d camera + detector' % self.drone_id)

    def observer_panel(self):
        with self.lock:
            image = None if self.observer is None else self.observer.copy()
        if image is None:
            return label_panel(placeholder('waiting for /observer/image'), 'overhead')
        return label_panel(fit(image), 'overhead')

    def map_panel(self):
        with self.lock:
            msg = self.map_msg
            path = list(self.path)
            pins = list(self.pins)
            hazards = list(self.hazards)
        if msg is None:
            return label_panel(placeholder('waiting for /map'),
                               'drone %d SLAM map' % self.drone_id)

        info = msg.info
        grid = np.asarray(msg.data, np.int8).reshape(info.height, info.width)
        canvas = np.full((info.height, info.width, 3), BG, np.uint8)
        canvas[grid == 0] = FREE
        canvas[grid > 50] = OCCUPIED
        canvas = cv2.flip(canvas, 0)            # map y up -> image y down

        def to_px(x, y):
            col = (x - info.origin.position.x) / info.resolution
            row = info.height - (y - info.origin.position.y) / info.resolution
            return int(round(col)), int(round(row))

        for i in range(1, len(path)):
            cv2.line(canvas, to_px(*path[i - 1]), to_px(*path[i]), OK, 1, cv2.LINE_AA)

        pose = self.pose()
        if pose is not None:
            x, y, _ = pose
            if not self.trail or math.hypot(x - self.trail[-1][0], y - self.trail[-1][1]) > 0.1:
                self.trail.append((x, y))

        for i in range(1, len(self.trail)):
            cv2.line(canvas, to_px(*self.trail[i - 1]), to_px(*self.trail[i]),
                     TRAIL, 1, cv2.LINE_AA)

        # Hazards first, so a survivor pin on top of one stays readable.
        for hx, hy, kind in hazards:
            centre = to_px(hx, hy)
            colour = HAZARD if kind == 'constriction' else BLOCKED
            cv2.drawMarker(canvas, centre, colour, cv2.MARKER_TRIANGLE_UP, 14, 2)

        for pin in pins:
            centre = to_px(*pin)
            cv2.circle(canvas, centre, 6, ACCENT, -1, cv2.LINE_AA)
            cv2.circle(canvas, centre, 11, ACCENT, 1, cv2.LINE_AA)

        if pose is not None:
            x, y, yaw = pose
            body = to_px(x, y)
            nose = to_px(x + 0.7 * math.cos(yaw), y + 0.7 * math.sin(yaw))
            cv2.circle(canvas, body, 5, OK, -1, cv2.LINE_AA)
            cv2.line(canvas, body, nose, OK, 2, cv2.LINE_AA)

        canvas = crop_to_known(canvas, grid, info)
        panel = fit(canvas)
        px = int(5.0 / info.resolution * min(PANEL / canvas.shape[1], PANEL / canvas.shape[0]))
        if 20 < px < PANEL - 120:
            cv2.line(panel, (20, PANEL - 24), (20 + px, PANEL - 24), MUTED, 2)
            put(panel, '5 m', (20 + px + 8, PANEL - 19), 0.42, MUTED)
        return label_panel(panel, 'drone %d SLAM map' % self.drone_id)

    def close(self):
        try:
            self.executor.shutdown()
            self.node.destroy_node()
            rclpy.try_shutdown(context=self.context)
        except Exception:
            pass


def merged_survivors(feeds, radius=3.0):
    """Unique survivors across the team.

    Summing each drone's pin count double-counts anyone both drones saw, so the
    pins are brought into the shared mission frame and clustered - the same
    reconciliation the ground station does.
    """
    clusters = []
    for feed in feeds:
        with feed.lock:
            pins = list(feed.mission_pins)
        for x, y in pins:
            if not any(math.hypot(x - cx, y - cy) <= radius for cx, cy in clusters):
                clusters.append((x, y))
    return clusters


def summary_panel(feeds, started):
    panel = np.full((PANEL, PANEL, 3), BG, np.uint8)
    put(panel, 'SIH_Swarm', (24, 58), 0.95, INK, 2)
    put(panel, 'maze_survivors  /  GPS-denied search', (24, 86), 0.45, MUTED)
    cv2.line(panel, (24, 108), (PANEL - 24, 108), (40, 44, 38), 1)

    y = 152
    for feed in feeds:
        state, colour = feed.state_text()
        with feed.lock:
            survivors = feed.survivors
        put(panel, 'drone %d' % feed.drone_id, (24, y), 0.6, INK)
        put(panel, 'ROS_DOMAIN_ID=%d' % feed.drone_id, (160, y), 0.42, MUTED)
        put(panel, state, (24, y + 26), 0.5, colour)
        with feed.lock:
            hazard_count = len(feed.hazards)
        put(panel, 'survivors pinned: %d' % survivors, (24, y + 50), 0.5, ACCENT)
        put(panel, 'hazards mapped: %d' % hazard_count, (24, y + 72), 0.5, HAZARD)
        y += 118

    total = len(merged_survivors(feeds))
    raw = sum(f.survivors for f in feeds)
    cv2.line(panel, (24, y + 4), (PANEL - 24, y + 4), (40, 44, 38), 1)
    put(panel, 'pinned %d of 4' % total, (24, y + 44), 0.62, ACCENT)
    if raw != total:
        put(panel, '(%d raw pins, %d merged)' % (raw, raw - total + 1 if total else raw),
            (170, y + 44), 0.42, MUTED)
    put(panel, 't+%d s' % int(time.time() - started), (24, y + 74), 0.5, MUTED)
    put(panel, 'lidar SLAM + SSD-Lite on RGB-D', (24, PANEL - 54), 0.42, MUTED)
    put(panel, 'ArduPilot SITL over native DDS', (24, PANEL - 30), 0.42, MUTED)
    return panel


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', default='mission.mp4')
    parser.add_argument('--live', action='store_true',
                        help='also show the composite in a window, for screen capture')
    parser.add_argument('--no-file', action='store_true',
                        help='live view only, write no mp4')
    parser.add_argument('--drones', type=int, default=2)
    parser.add_argument('--launch-poses', default='-9.6,-9.6,0 9.6,9.6,3.14159',
                        help='x,y,yaw per drone, so pins can be merged across drones')
    parser.add_argument('--duration', type=float, default=1800.0,
                        help='hard stop, in seconds of wall clock')
    args = parser.parse_args()

    poses = [tuple(float(v) for v in chunk.split(','))
             for chunk in args.launch_poses.split()]
    while len(poses) < args.drones:
        poses.append((0.0, 0.0, 0.0))
    feeds = [DroneFeed(d, want_observer=(d == 1), launch_pose=poses[d - 1])
             for d in range(1, args.drones + 1)]
    overhead = feeds[0]

    width = PANEL * 3
    height = PANEL * len(feeds) + BAR
    writer = None
    if not args.no_file:
        writer = cv2.VideoWriter(args.out, cv2.VideoWriter_fourcc(*'mp4v'),
                                 FPS, (width, height))
        if not writer.isOpened():
            raise SystemExit('could not open ' + args.out + ' for writing')
        print('recording to ' + os.path.abspath(args.out), flush=True)

    window = 'SIH_Swarm - live mission view'
    if args.live or args.no_file:
        # Fit the composite to the screen; it is 1920x1338 at full size.
        cv2.namedWindow(window, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(window, 1600, int(1600 * height / width))
        print('live view open - press q in the window to close', flush=True)

    started = time.time()
    frames = 0
    period = 1.0 / FPS
    try:
        while True:
            tick = time.time()

            rows = []
            for index, feed in enumerate(feeds):
                left = (overhead.observer_panel() if index == 0
                        else summary_panel(feeds, started))
                rows.append(np.hstack([left, feed.detection_panel(), feed.map_panel()]))
            frame = np.vstack(rows)

            bar = np.full((BAR, width, 3), BG, np.uint8)
            cv2.line(bar, (0, 0), (width, 0), (40, 44, 38), 1)
            put(bar, 'SIH_Swarm  maze_survivors', (24, 37), 0.58, INK)
            for index, feed in enumerate(feeds):
                state, colour = feed.state_text()
                put(bar, 'drone %d  %s' % (feed.drone_id, state),
                    (430 + index * 340, 37), 0.5, colour)
            put(bar, 'survivors: %d   hazards: %d'
                % (len(merged_survivors(feeds)),
                   sum(len(f.hazards) for f in feeds)),
                (width - 520, 37), 0.55, ACCENT)
            put(bar, 't+%d s' % int(time.time() - started), (width - 170, 37), 0.55, MUTED)
            composite = np.vstack([frame, bar])
            if writer is not None:
                writer.write(composite)
            if args.live or args.no_file:
                cv2.imshow(window, composite)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    print('live view closed', flush=True)
                    break
            frames += 1

            if frames % (FPS * 30) == 0:
                print('%d frames (%.0f s of video)' % (frames, frames / FPS), flush=True)

            if all(feed.finished() for feed in feeds):
                print('all drones landed and disarmed', flush=True)
                break
            if time.time() - started > args.duration:
                print('duration reached', flush=True)
                break

            slack = period - (time.time() - tick)
            if slack > 0:
                time.sleep(slack)
    except KeyboardInterrupt:
        pass
    finally:
        if writer is not None:
            writer.release()
            print('wrote %s: %d frames, %.1f s'
                  % (args.out, frames, frames / FPS), flush=True)
        if args.live or args.no_file:
            cv2.destroyAllWindows()
        for feed in feeds:
            feed.close()


if __name__ == '__main__':
    main()
