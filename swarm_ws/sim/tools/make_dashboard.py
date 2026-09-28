#!/usr/bin/env python3
"""Build the command-centre dashboard from a finished mission.

The ground station writes survivors.json at the end of every run. This turns it
into a single self-contained HTML page a dispatcher can open - no server, no
network, no build step - showing where the survivors are, how they are ranked
for dispatch, which hazards block access, and where everything sits on the map.

    python3 sim/tools/make_dashboard.py sim/logs/<run>/survivors.json \\
        --out docs/dashboard.html

The mission data is written into the page rather than fetched, because a page
opened from disk cannot fetch a sibling file.
"""

import argparse
import json
import math
from pathlib import Path

HERE = Path(__file__).resolve().parent
EARTH_RADIUS_M = 6378137.0


def to_latlon(x, y, datum):
    lat = datum['lat'] + math.degrees(y / EARTH_RADIUS_M)
    lon = datum['lon'] + math.degrees(
        x / (EARTH_RADIUS_M * math.cos(math.radians(datum['lat']))))
    return lat, lon


def triage(survivors, hazards, obstruction_radius=4.0):
    """Same rule the situation report uses: access first, then confidence."""
    ranked = []
    for s in survivors:
        near = sorted(((math.hypot(s['x'] - h['x'], s['y'] - h['y']), h) for h in hazards),
                      key=lambda pair: pair[0])
        blocking = [p for p in near if p[0] <= obstruction_radius]
        if blocking:
            priority, reason = 1, 'access obstructed'
        elif len(s['seen_by']) < 2:
            priority, reason = 2, 'single-drone detection, needs verification'
        else:
            priority, reason = 3, 'confirmed by two drones, access clear'
        ranked.append({
            'survivor': s, 'priority': priority, 'reason': reason,
            'nearest': ({'kind': near[0][1]['kind'], 'distance': round(near[0][0], 1)}
                        if near else None),
            'blocking': [{'kind': h['kind'], 'distance': round(d, 1)} for d, h in blocking],
        })
    ranked.sort(key=lambda r: (r['priority'], -len(r['survivor']['seen_by'])))
    return ranked


# The walls of the generated world, so the dashboard can draw the building.
# [centre x, centre y, size x, size y] in metres.
WALLS = [
    [0.0, -12.0, 26.4, 2.4], [-12.0, -9.6, 2.4, 2.4], [-7.2, -9.6, 2.4, 2.4],
    [12.0, -9.6, 2.4, 2.4], [-12.0, -7.2, 2.4, 2.4], [-4.8, -7.2, 7.2, 2.4],
    [7.2, -7.2, 12.0, 2.4], [-12.0, -4.8, 2.4, 2.4], [-2.4, -4.8, 2.4, 2.4],
    [12.0, -4.8, 2.4, 2.4], [-12.0, -2.4, 2.4, 2.4], [-7.2, -2.4, 2.4, 2.4],
    [-2.4, -2.4, 2.4, 2.4], [4.8, -2.4, 7.2, 2.4], [12.0, -2.4, 2.4, 2.4],
    [-12.0, 0.0, 2.4, 2.4], [-7.2, 0.0, 2.4, 2.4], [-2.4, 0.0, 2.4, 2.4],
    [2.4, 0.0, 2.4, 2.4], [12.0, 0.0, 2.4, 2.4], [-12.0, 2.4, 2.4, 2.4],
    [-7.2, 2.4, 2.4, 2.4], [0.0, 2.4, 7.2, 2.4], [7.2, 2.4, 2.4, 2.4],
    [12.0, 2.4, 2.4, 2.4], [-12.0, 4.8, 2.4, 2.4], [-7.2, 4.8, 2.4, 2.4],
    [7.2, 4.8, 2.4, 2.4], [12.0, 4.8, 2.4, 2.4], [-12.0, 7.2, 2.4, 2.4],
    [-4.8, 7.2, 7.2, 2.4], [4.8, 7.2, 7.2, 2.4], [12.0, 7.2, 2.4, 2.4],
    [-12.0, 9.6, 2.4, 2.4], [12.0, 9.6, 2.4, 2.4], [0.0, 12.0, 26.4, 2.4],
]

TEMPLATE = (HERE / 'dashboard_template.html').read_text()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('report', help='survivors.json from a finished mission')
    parser.add_argument('--out', default='docs/dashboard.html')
    parser.add_argument('--mission', default='', help='label for the run')
    args = parser.parse_args()

    data = json.loads(Path(args.report).read_text())
    datum = data.get('datum', {'lat': -35.363262, 'lon': 149.165237, 'alt': 584})

    for item in data['survivors'] + data.get('hazards', []):
        if 'lat' not in item:
            item['lat'], item['lon'] = to_latlon(item['x'], item['y'], datum)

    payload = {
        'mission': args.mission or Path(args.report).parent.name,
        'datum': datum,
        'expected': data.get('expected', 4),
        'found': data.get('found', len(data['survivors'])),
        'elapsed_s': data.get('elapsed_s', 0),
        'per_drone_raw_pins': data.get('per_drone_raw_pins', {}),
        'ranked': triage(data['survivors'], data.get('hazards', [])),
        'hazards': data.get('hazards', []),
        'walls': WALLS,
    }

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(TEMPLATE.replace('/*MISSION_DATA*/null',
                                    json.dumps(payload, separators=(',', ':'))))
    print('wrote %s  (%d survivors, %d hazards)'
          % (out, len(data['survivors']), len(data.get('hazards', []))))


if __name__ == '__main__':
    main()
