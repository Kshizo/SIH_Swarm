#!/usr/bin/env python3
"""Generate the `maze_survivors` Gazebo world used by the SIH_Swarm stack.

The world is a walled maze with survivors (Rescue Randy, from Gazebo Fuel)
standing in dead ends, plus the two sensor-equipped drones.  It is generated
rather than hand-written so the layout stays verifiable: the maze is a seeded
perfect maze with a few extra loops knocked through, the generator flood-fills
to prove every free cell is reachable, and survivors are placed automatically at
the dead ends furthest from the drone start cells.

    python3 sim/tools/gen_world.py [--seed 7] [--survivors 4]
"""

import argparse
import math
import random
from collections import deque
from pathlib import Path

HERE = Path(__file__).resolve().parent
SIM_DIR = HERE.parent

CELL = 2.4           # metres per maze cell (corridor width)
WALL_HEIGHT = 2.5
CELLS = 5            # maze is CELLS x CELLS rooms -> (2*CELLS+1) grid squares

# Grid coordinates (col, row) of the two drone start squares.
START_A = (1, 1)
START_B = (2 * CELLS - 1, 2 * CELLS - 1)

SURVIVOR_URI = 'https://fuel.gazebosim.org/1.0/OpenRobotics/models/Rescue Randy'

# Collapsed debris narrows a corridor without sealing it. The gap left is wide
# enough to fly through and narrow enough that the hazard mapper flags it, which
# is the case a responder actually cares about: passable, but not safely.
DEBRIS_GAP = 1.8
DEBRIS_HEIGHT = 1.4


def carve_maze(seed):
    """Recursive-backtracker perfect maze on a (2n+1) grid; True == wall."""
    size = 2 * CELLS + 1
    grid = [[True] * size for _ in range(size)]
    rng = random.Random(seed)

    def cell_square(cx, cy):
        return 2 * cx + 1, 2 * cy + 1

    visited = set()
    stack = [(0, 0)]
    visited.add((0, 0))
    col, row = cell_square(0, 0)
    grid[row][col] = False

    while stack:
        cx, cy = stack[-1]
        neighbours = []
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nx, ny = cx + dx, cy + dy
            if 0 <= nx < CELLS and 0 <= ny < CELLS and (nx, ny) not in visited:
                neighbours.append((nx, ny, dx, dy))
        if not neighbours:
            stack.pop()
            continue
        nx, ny, dx, dy = rng.choice(neighbours)
        col, row = cell_square(cx, cy)
        grid[row + dy][col + dx] = False          # knock out the shared wall
        col, row = cell_square(nx, ny)
        grid[row][col] = False
        visited.add((nx, ny))
        stack.append((nx, ny))

    # A perfect maze is all dead ends and no loops, which makes exploration a
    # sequence of backtracks. Open a few interior walls so there are cycles.
    interior = [
        (c, r)
        for r in range(1, size - 1)
        for c in range(1, size - 1)
        if grid[r][c] and (c % 2 == 0) != (r % 2 == 0)
    ]
    rng.shuffle(interior)
    for col, row in interior[:max(1, len(interior) // 6)]:
        grid[row][col] = False

    return grid


def free_neighbours(grid, col, row):
    out = []
    for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
        nc, nr = col + dx, row + dy
        if 0 <= nr < len(grid) and 0 <= nc < len(grid[0]) and not grid[nr][nc]:
            out.append((nc, nr))
    return out


def flood(grid, start):
    seen = {start}
    queue = deque([(start, 0)])
    distance = {start: 0}
    while queue:
        (col, row), dist = queue.popleft()
        for neighbour in free_neighbours(grid, col, row):
            if neighbour not in seen:
                seen.add(neighbour)
                distance[neighbour] = dist + 1
                queue.append((neighbour, dist + 1))
    return distance


def place_survivors(grid, distance_a, distance_b, count):
    """Spread `count` survivors out as far as the maze allows."""
    for separation in range(8, 1, -1):
        try:
            return pick_squares(grid, distance_a, distance_b, count, separation)
        except ValueError:
            continue
    raise SystemExit(f'could not place {count} survivors; try another --seed')


def pick_squares(grid, distance_a, distance_b, count, min_separation):
    """Dead ends first, then whatever is furthest from both drone starts.

    Survivors have to be spread out: the detector confirms a survivor only once
    it has held a stable 3D position, and two mannequins in one corridor would be
    merged into a single map pin.
    """
    def cost(square):
        return -min(distance_a[square], distance_b[square])

    reachable = [s for s in distance_a if s not in (START_A, START_B)]
    dead_ends = [s for s in reachable if len(free_neighbours(grid, *s)) == 1]
    others = [s for s in reachable if s not in dead_ends]
    candidates = sorted(dead_ends, key=cost) + sorted(others, key=cost)

    chosen = []
    for square in candidates:
        if len(chosen) == count:
            break
        far_enough = all(
            abs(square[0] - other[0]) + abs(square[1] - other[1]) >= min_separation
            for other in chosen + [START_A, START_B])
        if far_enough:
            chosen.append(square)
    if len(chosen) < count:
        raise ValueError(min_separation)
    return chosen



def place_debris(grid, distance_a, distance_b, keep_clear, count, rng):
    """Pick straight corridor squares to partly block.

    Only squares with exactly two free neighbours facing each other qualify -
    a junction would be blocked rather than narrowed - and nothing goes next to
    a survivor or a launch point, so the debris changes the difficulty of the
    search without changing what there is to find.
    """
    chosen = []
    candidates = [s for s in distance_a if s not in (START_A, START_B)]
    rng.shuffle(candidates)
    for square in candidates:
        if len(chosen) == count:
            break
        neighbours = free_neighbours(grid, *square)
        if len(neighbours) != 2:
            continue
        (ax, ay), (bx, by) = neighbours
        horizontal = ay == by == square[1]
        vertical = ax == bx == square[0]
        if not (horizontal or vertical):
            continue
        if any(abs(square[0] - k[0]) + abs(square[1] - k[1]) <= 1 for k in keep_clear):
            continue
        if any(abs(square[0] - c[0][0]) + abs(square[1] - c[0][1]) <= 2 for c in chosen):
            continue
        chosen.append((square, 'horizontal' if horizontal else 'vertical'))
    return chosen


def wall_runs(grid):
    """Merge horizontal runs of wall squares into single boxes."""
    runs = []
    for row, cells in enumerate(grid):
        col = 0
        while col < len(cells):
            if not cells[col]:
                col += 1
                continue
            start = col
            while col < len(cells) and cells[col]:
                col += 1
            runs.append((start, col - 1, row))
    return runs


def world_xy(col, row, size):
    """Grid square -> world metres, with the maze centred on the origin."""
    offset = (size - 1) / 2.0
    return (col - offset) * CELL, (row - offset) * CELL


WORLD_HEADER = """<?xml version="1.0" ?>
<!--
  GENERATED FILE - regenerate with: python3 sim/tools/gen_world.py
  A {size}x{size} maze ({extent:.1f} m square, {cell:.1f} m corridors) with {survivors}
  survivors in dead ends and two RGB-D + lidar drones.
-->
<sdf version="1.9">
  <world name="maze_survivors">
    <physics name="1ms" type="ignore">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>

    <scene>
      <ambient>0.6 0.6 0.6 1</ambient>
      <background>0.75 0.8 0.85</background>
      <grid>false</grid>
    </scene>

    <!-- SITL's default home. vio_to_ardupilot sends this as the EKF origin. -->
    <spherical_coordinates>
      <latitude_deg>-35.363262</latitude_deg>
      <longitude_deg>149.165237</longitude_deg>
      <elevation>584</elevation>
      <heading_deg>0</heading_deg>
      <surface_model>EARTH_WGS84</surface_model>
    </spherical_coordinates>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 20 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.3 0.3 0.3 1</specular>
      <direction>-0.4 0.2 -0.9</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>200 200</size></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>200 200</size></plane></geometry>
          <material>
            <ambient>0.3 0.3 0.32 1</ambient>
            <diffuse>0.45 0.45 0.48 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
"""

WALL_TEMPLATE = """
    <model name="wall_{index}">
      <static>true</static>
      <pose>{x:.3f} {y:.3f} {z:.3f} 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box></geometry>
          <material>
            <ambient>0.35 0.33 0.30 1</ambient>
            <diffuse>0.62 0.58 0.52 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
"""

SURVIVOR_TEMPLATE = """
    <!-- survivor {index} at maze cell {cell} -->
    <include>
      <uri>{uri}</uri>
      <name>survivor_{index}</name>
      <pose>{x:.3f} {y:.3f} 0 0 0 {yaw:.3f}</pose>
      <static>true</static>
    </include>
"""

OBSERVER_TEMPLATE = """
    <!-- Fixed overhead camera, for recording the mission from outside.
         Looks straight down with image up = world +Y, image right = world +X. -->
    <model name="observer">
      <static>true</static>
      <pose>0 0 {height:.1f} 0 1.5707963 1.5707963</pose>
      <link name="link">
        <sensor name="observer_camera" type="camera">
          <topic>/observer/image</topic>
          <gz_frame_id>observer</gz_frame_id>
          <update_rate>10</update_rate>
          <always_on>1</always_on>
          <camera>
            <horizontal_fov>{hfov:.4f}</horizontal_fov>
            <image>
              <width>720</width>
              <height>720</height>
              <format>R8G8B8</format>
            </image>
            <clip><near>0.5</near><far>200.0</far></clip>
          </camera>
        </sensor>
      </link>
    </model>
"""

DEBRIS_TEMPLATE = """
    <!-- debris {index}: narrows the corridor at {cell} to {gap:.1f} m -->
    <model name="debris_{index}">
      <static>true</static>
      <pose>{x:.3f} {y:.3f} {z:.3f} 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box></geometry>
          <material>
            <ambient>0.28 0.24 0.20 1</ambient>
            <diffuse>0.46 0.40 0.33 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
"""

DRONE_TEMPLATE = """
    <include>
      <uri>model://{model}</uri>
      <name>{name}</name>
      <pose>{x:.3f} {y:.3f} 0.195 0 0 {yaw:.3f}</pose>
    </include>
"""

WORLD_FOOTER = """
  </world>
</sdf>
"""


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seed', type=int, default=7)
    parser.add_argument('--debris', type=int, default=3,
                        help='corridor squares to partly block with debris')
    parser.add_argument('--survivors', type=int, default=4,
                        help='must match target_survivor_count in beta_stack.launch.py')
    parser.add_argument('--out', default=str(SIM_DIR / 'worlds' / 'maze_survivors.sdf'))
    args = parser.parse_args()

    grid = carve_maze(args.seed)
    size = len(grid)

    for name, square in (('drone 1', START_A), ('drone 2', START_B)):
        if grid[square[1]][square[0]]:
            raise SystemExit(f'{name} start square {square} is a wall')

    distance = flood(grid, START_A)
    free_count = sum(1 for row in grid for cell in row if not cell)
    if len(distance) != free_count:
        raise SystemExit(f'maze is not fully connected '
                         f'({len(distance)}/{free_count} squares reachable from {START_A})')
    if START_B not in distance:
        raise SystemExit('the two drone start squares are not connected')

    distance_b = flood(grid, START_B)
    chosen = place_survivors(grid, distance, distance_b, args.survivors)

    rng = random.Random(args.seed)
    parts = [WORLD_HEADER.format(size=size, extent=size * CELL,
                                 cell=CELL, survivors=args.survivors)]

    # Every wall square is a full CELL x CELL block, so a corridor is exactly
    # CELL wide wherever the map says it is open. Runs along a row are merged
    # into one box purely to keep the model count down.
    index = 0
    for col_start, col_end, row in wall_runs(grid):
        x0, y = world_xy(col_start, row, size)
        x1, _ = world_xy(col_end, row, size)
        parts.append(WALL_TEMPLATE.format(
            index=index, x=(x0 + x1) / 2.0, y=y, z=WALL_HEIGHT / 2.0,
            sx=(x1 - x0) + CELL, sy=CELL, sz=WALL_HEIGHT))
        index += 1

    for number, square in enumerate(chosen, start=1):
        x, y = world_xy(square[0], square[1], size)
        parts.append(SURVIVOR_TEMPLATE.format(
            index=number, cell=square, uri=SURVIVOR_URI,
            x=x, y=y, yaw=rng.uniform(-3.14, 3.14)))

    debris = place_debris(grid, distance, distance_b,
                          chosen + [START_A, START_B], args.debris, rng)
    for number, (square, orientation) in enumerate(debris, start=1):
        x, y = world_xy(square[0], square[1], size)
        block = CELL - DEBRIS_GAP
        # Push the pile against one side, leaving the gap on the other.
        offset = (CELL - block) / 2.0
        if orientation == 'horizontal':      # corridor runs east-west
            parts.append(DEBRIS_TEMPLATE.format(
                index=number, cell=square, gap=DEBRIS_GAP,
                x=x, y=y + offset, z=DEBRIS_HEIGHT / 2.0,
                sx=CELL * 0.45, sy=block, sz=DEBRIS_HEIGHT))
        else:                                # corridor runs north-south
            parts.append(DEBRIS_TEMPLATE.format(
                index=number, cell=square, gap=DEBRIS_GAP,
                x=x + offset, y=y, z=DEBRIS_HEIGHT / 2.0,
                sx=block, sy=CELL * 0.45, sz=DEBRIS_HEIGHT))

    # Frame the whole maze with a little margin to spare.
    observer_height = 30.0
    parts.append(OBSERVER_TEMPLATE.format(
        height=observer_height,
        hfov=2.0 * math.atan((size * CELL * 0.56) / observer_height)))

    for model, name, square, yaw in (
            ('iris_with_sensors', 'drone1', START_A, 0.0),
            ('iris_with_sensors_2', 'drone2', START_B, 3.141592653589793)):
        x, y = world_xy(square[0], square[1], size)
        parts.append(DRONE_TEMPLATE.format(model=model, name=name, x=x, y=y, yaw=yaw))

    parts.append(WORLD_FOOTER)
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(''.join(parts))

    print(f'wrote {out}')
    print(f'  maze      : {size}x{size} squares, {size * CELL:.1f} m across, '
          f'{CELL:.1f} m corridors, seed {args.seed}')
    print(f'  walls     : {index} boxes, all {free_count} free squares reachable')
    print(f'  debris    : {len(debris)} piles, leaving {DEBRIS_GAP:.1f} m gaps')
    print(f'  survivors : ' + ', '.join(
        f'{n}@{world_xy(s[0], s[1], size)[0]:.1f},{world_xy(s[0], s[1], size)[1]:.1f}'
        for n, s in enumerate(chosen, start=1)))
    for name, square, in (('drone1', START_A), ('drone2', START_B)):
        x, y = world_xy(square[0], square[1], size)
        print(f'  {name}    : x={x:.1f} y={y:.1f}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
