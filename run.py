#!/usr/bin/env python3
"""Follow one lap of either Town04 route."""

import csv
import math
from datetime import datetime
from pathlib import Path

import numpy as np

if __package__:
    from . import simulation as tracking
else:
    import simulation as tracking

PROJECT = Path(__file__).resolve().parent
DEFAULT_MAP = 'Town04_Opt'
CASES = ('forward', 'clockwise')
START = (23, -2, 240.0)
FINISH_LANE = -3
PREFERRED_LINKS = {(23, -2): (1421, -1), (50, 6): (1162, 6)}
RETURN_LANE_CHANGES = ((4, 5, 150.0, 105.0), (5, 6, 95.0, 50.0))
CLOCKWISE_START = (8, 1, 22.0)
CLOCKWISE_MIDPOINT = (16, -1, 20.0)
CLOCKWISE_LINKS = {
    (8, 1): (1225, 1),
    (51, 1): (241, 1),
    (16, -1): (955, -1),
    (52, -1): (1141, -1),
}


def route_coordinate(waypoint):
    pose = waypoint.transform
    return pose.location.x, -pose.location.y, -math.radians(pose.rotation.yaw)


def bezier(points, t):
    degree = len(points) - 1
    return sum(math.comb(degree, i) * ((1 - t)**(degree - i) * t**i)[:, None] * point
               for i, point in enumerate(points))


def lane4_guide(road_map):
    # Fit to the midpoint of the two painted turn guides in Town04 junction 1368.
    # The handles share the endpoint tangents to join the lane centerline smoothly.
    first_s, last_s = 12.0, 50.1
    start = route_coordinate(road_map.get_waypoint_xodr(1421, -1, first_s))
    end = route_coordinate(road_map.get_waypoint_xodr(1421, -1, last_s))
    a, b = np.array(start[:2]), np.array(end[:2])
    forward = np.array([math.cos(start[2]), math.sin(start[2])])
    exit_forward = np.array([math.cos(end[2]), math.sin(end[2])])
    controls = np.array([a, a + 13.6 * forward, a + 13.7 * forward,
                         b - 15.3 * exit_forward, b - 2.8 * exit_forward, b])
    t = np.linspace(0, 1, 401)
    points = bezier(controls, t)
    dx, dy = bezier(np.diff(controls, axis=0), t).T
    yaw = np.unwrap(np.arctan2(dy, dx))
    distance = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(points, axis=0), axis=1))]
    stations = first_s + (last_s - first_s) * distance / distance[-1]

    def coordinate(waypoint):
        if (waypoint.road_id, waypoint.lane_id) == (1421, -1) and first_s < waypoint.s < last_s:
            return tuple(float(np.interp(waypoint.s, stations, values))
                         for values in (points[:, 0], points[:, 1], yaw))
        return route_coordinate(waypoint)

    return coordinate


def lane_key(waypoint):
    return waypoint.road_id, waypoint.section_id, waypoint.lane_id


def connected_waypoints(start, end, preferred_links):
    waypoints = [start]
    end_key = lane_key(end)
    direction = 1 if end.lane_id < 0 else -1
    visited = {lane_key(start)}
    while True:
        current = waypoints[-1]
        candidates = current.next(0.5)
        if not candidates:
            raise ValueError(f'Lane {lane_key(current)} ends before {end_key}.')
        preferred = preferred_links.get((current.road_id, current.lane_id))
        following = min(candidates, key=lambda wp: (
            preferred is not None and (wp.road_id, wp.lane_id) != preferred,
            abs(math.remainder(wp.transform.rotation.yaw - current.transform.rotation.yaw, 360)),
            wp.road_id, wp.section_id, wp.lane_id,
        ))
        key = lane_key(following)
        if key == end_key and direction * (following.s - end.s) >= 0:
            waypoints.append(end)
            return waypoints
        if key != lane_key(current):
            if key in visited:
                raise ValueError(f'The selected lane connections do not reach {end_key} at s={end.s:g}.')
            visited.add(key)
        waypoints.append(following)


def lane_change(road_map, source_lane, target_lane, first_s, last_s):
    # Road 50 has broken separators between lanes 4, 5 and 6 in these intervals.
    stations = np.linspace(first_s, last_s, math.ceil((first_s - last_s) / 0.5) + 1)
    source = [road_map.get_waypoint_xodr(50, source_lane, float(s)) for s in stations]
    target = [road_map.get_waypoint_xodr(50, target_lane, float(s)) for s in stations]
    a = np.array([route_coordinate(wp) for wp in source])
    b = np.array([route_coordinate(wp) for wp in target])
    t = np.linspace(0, 1, len(stations))
    blend = (t**3 * (10 - 15 * t + 6 * t**2))[:, None]
    xy = a[:, :2] + blend * (b[:, :2] - a[:, :2])
    dx, dy = np.gradient(xy, axis=0).T
    yaw = np.arctan2(dy, dx)
    yaw[0], yaw[-1] = a[0, 2], b[-1, 2]
    route = [tuple(point) for point in np.column_stack((xy, yaw)).tolist()]
    return route, source[:-1] + target[-1:]


def forward_lap(road_map):
    start = road_map.get_waypoint_xodr(*START)
    finish = road_map.get_waypoint_xodr(start.road_id, FINISH_LANE, start.s)
    guide = lane4_guide(road_map)
    route, waypoints = [guide(start)], [start]

    for source, target, first_s, last_s in RETURN_LANE_CHANGES:
        change, change_waypoints = lane_change(road_map, source, target, first_s, last_s)
        approach = connected_waypoints(waypoints[-1], change_waypoints[0], PREFERRED_LINKS)[1:]
        waypoints.extend(approach)
        route.extend(map(guide, approach))
        waypoints.extend(change_waypoints[1:])
        route.extend(change[1:])

    returning = connected_waypoints(waypoints[-1], finish, PREFERRED_LINKS)[1:]
    waypoints.extend(returning)
    route.extend(map(guide, returning))

    return route, waypoints


def clockwise_lap(road_map):
    start = road_map.get_waypoint_xodr(*CLOCKWISE_START)
    midpoint = road_map.get_waypoint_xodr(*CLOCKWISE_MIDPOINT)
    waypoints = connected_waypoints(start, midpoint, CLOCKWISE_LINKS)
    waypoints.extend(connected_waypoints(midpoint, start, CLOCKWISE_LINKS)[1:])
    return [route_coordinate(waypoint) for waypoint in waypoints], waypoints


def lap_route(road_map, case):
    return clockwise_lap(road_map) if case == 'clockwise' else forward_lap(road_map)


def main():
    parser = tracking.trial_parser(__doc__)
    parser.add_argument('--case', choices=CASES, default='forward')
    parser.add_argument('--use-current-map', action='store_true', help='use the loaded Town04 or Town04_Opt')
    args = parser.parse_args()
    if not math.isfinite(args.speed) or args.speed <= 0:
        parser.error('--speed must be positive')
    case = args.case

    def load_scene(client):
        world = client.get_world() if args.use_current_map else client.load_world(DEFAULT_MAP)
        road_map = world.get_map()
        map_name = road_map.name.rsplit('/', 1)[-1]
        if map_name.removesuffix('_Opt') != 'Town04':
            raise ValueError(f'Load Town04 or Town04_Opt first. The current map is {map_name}.')
        route, waypoints = lap_route(road_map, case)
        route_path = PROJECT / 'routes' / f'{map_name}_{case}.csv'
        route_path.parent.mkdir(parents=True, exist_ok=True)
        with route_path.open('w', newline='') as file:
            csv.writer(file, lineterminator='\n').writerows(route)
        distance = sum(math.dist(a[:2], b[:2]) for a, b in zip(route, route[1:]))
        print(f'{map_name} | {case} | {distance:.2f} m | {len(route)} waypoints: {route_path}', flush=True)
        return world, route_path, route, waypoints

    map_label = 'current' if args.use_current_map else DEFAULT_MAP
    output = args.output or (PROJECT / 'results' /
        f'{map_label}_{case}_{args.controller}_{args.speed:g}kmh_{datetime.now():%Y%m%d_%H%M%S}.csv')
    return tracking.run_trial(args, output, load_scene)


if __name__ == '__main__':
    raise SystemExit(main())
