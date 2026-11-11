"""Connect the ROS1 path followers to a driving lane in CARLA."""

import argparse
import csv
import math
import shutil
import subprocess
import time
from contextlib import contextmanager
from pathlib import Path
from queue import Empty, Queue
from tempfile import TemporaryDirectory
from uuid import uuid4

import carla
import numpy as np
import roslibpy

if __package__:
    from .progress import STOP_DISTANCE, RouteProgress
else:
    from progress import STOP_DISTANCE, RouteProgress


LAUNCH_FILE = Path(__file__).with_name('tracking.launch')
STEP = 0.05
STEP_NS = round(STEP * 1_000_000_000)
STOP_TOLERANCE = 1.0  # m from the rear axle to the route endpoint
TRACKING_TOPICS = {'/clock', '/base_waypoints', '/final_waypoints', '/smart/rear_pose', '/smart/cmd_vel'}


def wait_for_idle_ros(ros):
    # rosbridge can keep publishers registered for several seconds after a trial.
    deadline = time.monotonic() + 15
    while occupied := TRACKING_TOPICS & set(ros.get_topics()):
        if time.monotonic() >= deadline:
            raise RuntimeError(f'Stop the other tracking session first. Active topics: {sorted(occupied)}')
        time.sleep(0.5)


@contextmanager
def ros_controllers(args, wheelbase, route_path):
    docker = ['docker', 'exec', args.container]
    token = f'/tmp/ugv-carla-{uuid4().hex}'
    pidfile = f'{token}/launch.pid'
    process = None
    try:
        with TemporaryDirectory(prefix='ugv-lap-') as directory:
            staging = Path(directory)
            package = staging / 'ros_waypoints'
            shutil.copytree(LAUNCH_FILE.with_name('ros_waypoints'), package)
            shutil.copyfile(LAUNCH_FILE.with_name('progress.py'), package / 'progress.py')
            shutil.copyfile(route_path, staging / 'route.csv')
            subprocess.run(['docker', 'cp', directory, f'{args.container}:{token}'], check=True)
        with LAUNCH_FILE.open() as launch:
            process = subprocess.Popen([
                'docker', 'exec', '-i', args.container, 'bash', '-c',
                'source /opt/ros/noetic/setup.bash && source /root/car_ws/devel/setup.bash '
                '&& export ROS_PACKAGE_PATH="$1:$ROS_PACKAGE_PATH" '
                '&& exec roslaunch --wait --pid="$2" - "${@:3}"', 'bash', token, pidfile,
                f'controller:={args.controller}', f'wheelbase:={wheelbase:.6f}',
                f'route:={token}/route.csv', f'speed:={args.speed}',
            ], stdin=launch)
        yield process
    finally:
        if process is not None and process.poll() is None:
            subprocess.run(docker + ['bash', '-c',
                                     'if [ -s "$1" ]; then kill -INT "$(cat "$1")"; fi',
                                     'bash', pidfile], check=False)
            process.wait(timeout=15)
        subprocess.run(docker + ['rm', '-rf', token], check=False)


def axle_geometry(vehicle):
    transform = vehicle.get_transform()
    yaw = math.radians(transform.rotation.yaw)
    origin = transform.location
    # CARLA 0.9.16 reports the wheel positions in world coordinates in centimetres.
    wheels = vehicle.get_physics_control().wheels
    longitudinal = sorted((wheel.position.x / 100 - origin.x) * math.cos(yaw)
                          + (wheel.position.y / 100 - origin.y) * math.sin(yaw)
                          for wheel in wheels)
    rear_left, rear_right, front_left, front_right = longitudinal
    rear = (rear_left + rear_right) / 2
    front = (front_left + front_right) / 2
    return rear, front - rear


def spawn_vehicle(world, blueprint_id, waypoint, yaw):
    blueprint = world.get_blueprint_library().find(blueprint_id)
    transform = waypoint.transform
    transform.rotation.yaw = -math.degrees(yaw)
    transform.location.z += 0.4
    vehicle = world.spawn_actor(blueprint, transform)
    try:
        vehicle.apply_control(carla.VehicleControl(brake=1.0))
        world.tick()
        rear_offset, wheelbase = axle_geometry(vehicle)
        forward = transform.get_forward_vector()
        transform.location.x -= rear_offset * forward.x
        transform.location.y -= rear_offset * forward.y
        transform.location.z -= rear_offset * forward.z
        vehicle.set_transform(transform)
        for _ in range(20):
            world.tick()
        return vehicle, rear_offset, wheelbase
    except BaseException:
        vehicle.destroy()
        raise


def show_route(world, route, waypoints, speed):
    points = [carla.Location(x=x, y=-y, z=wp.transform.location.z + 0.08)
              for (x, y, _), wp in zip(route, waypoints)]
    distance = sum(a.distance(b) for a, b in zip(points, points[1:]))
    lifetime = 2 * distance / (speed / 3.6) + 30
    for start, end in zip(points, points[1:]):
        world.debug.draw_line(start, end, thickness=0.08, color=carla.Color(0, 220, 255),
                              life_time=lifetime)


def update_follow_camera(spectator, vehicle_transform):
    location = vehicle_transform.location
    heading = vehicle_transform.rotation.yaw
    yaw = math.radians(heading)
    camera_location = carla.Location(x=location.x - 6.0 * math.cos(yaw),
                                     y=location.y - 6.0 * math.sin(yaw),
                                     z=location.z + 2.8)
    spectator.set_transform(carla.Transform(camera_location, carla.Rotation(pitch=-15, yaw=heading)))


def follow(args, ros, client, world, vehicle, rear_offset, route, process, writer, waypoints):
    pose_topic = roslibpy.Topic(ros, '/smart/rear_pose', 'geometry_msgs/PoseStamped')
    velocity_topic = roslibpy.Topic(ros, '/smart/velocity', 'geometry_msgs/TwistStamped')
    clock_topic = roslibpy.Topic(ros, '/clock', 'rosgraph_msgs/Clock')
    command_topic = roslibpy.Topic(ros, '/smart/cmd_vel', 'geometry_msgs/Twist')
    commands = Queue()
    command_topic.subscribe(commands.put)
    progress = RouteProgress(np.asarray(route)[:, :2])
    distance = progress.stations[-1]
    cruise_speed = args.speed / 3.6
    max_frames = math.ceil((2 * distance / cruise_speed + 30) / STEP)
    start_frame = last_command_frame = stopped_frame = None
    wall_start = previous_position = None
    travelled = 0.0
    target_speed = steer = 0.0
    braking = True
    brake_command = carla.command.ApplyVehicleControl(vehicle.id, carla.VehicleControl(brake=1.0))
    startup_deadline = time.monotonic() + 15
    lane_widths = np.array([waypoint.lane_width for waypoint in waypoints])
    spectator = None if args.no_rendering else world.get_spectator()
    next_tick = time.monotonic()

    try:
        for frame in range(1, max_frames + 1):
            if process.poll() is not None or not ros.is_connected:
                raise RuntimeError('The ROS launch or WebSocket connection ended during the trial.')
            while not commands.empty():
                commands.get_nowait()
            if not args.no_rendering:
                delay = next_tick - time.monotonic()
                if delay > 0:
                    time.sleep(delay)
                next_tick = time.monotonic() + STEP
            world.tick()
            transform = vehicle.get_transform()
            if spectator is not None:
                update_follow_camera(spectator, transform)
            velocity = vehicle.get_velocity()
            yaw = -math.radians(transform.rotation.yaw)
            x = transform.location.x + rear_offset * math.cos(yaw)
            y = -transform.location.y + rear_offset * math.sin(yaw)
            speed = math.hypot(velocity.x, velocity.y)
            station, error, index, ratio = progress.update(x, y)
            seconds, nanoseconds = divmod(frame * STEP_NS, 1_000_000_000)
            stamp = {'secs': seconds, 'nsecs': nanoseconds}
            header = {'stamp': stamp, 'frame_id': 'world'}
            pose_topic.publish(roslibpy.Message({'header': header, 'pose': {
                'position': {'x': x, 'y': y, 'z': 0.0},
                'orientation': {'x': 0.0, 'y': 0.0, 'z': math.sin(yaw / 2), 'w': math.cos(yaw / 2)},
            }}))
            velocity_topic.publish(roslibpy.Message({'header': header, 'twist': {
                'linear': {'x': velocity.x, 'y': -velocity.y, 'z': velocity.z},
                'angular': {'x': 0.0, 'y': 0.0, 'z': 0.0},
            }}))
            clock_topic.publish(roslibpy.Message({'clock': stamp}))

            try:
                command = commands.get(timeout=0.12)
            except Empty:
                if last_command_frame is None or frame - last_command_frame >= 4:
                    target_speed = steer = 0.0
            else:
                last_command_frame = frame
                target_speed = command['linear']['x']
                steer = command['angular']['z']
                if start_frame is None and target_speed > 0.0:
                    start_frame = frame
                    wall_start = time.monotonic()

            if distance - station <= STOP_DISTANCE:
                # Keep braking at the end of the lap even if a delayed command arrives.
                target_speed = steer = 0.0

            if target_speed > 0.0:
                vehicle.apply_ackermann_control(carla.VehicleAckermannControl(
                    speed=target_speed, steer=-steer))
                braking = False
            elif not braking:
                # Send the brake directly to bypass the cached control from before Ackermann mode.
                response, = client.apply_batch_sync([brake_command], False)
                if response.has_error():
                    raise RuntimeError(response.error)
                braking = True
            if start_frame is None:
                if time.monotonic() > startup_deadline:
                    raise RuntimeError('No positive speed command received on /smart/cmd_vel.')
                continue

            width = lane_widths[index] + ratio * (lane_widths[index + 1] - lane_widths[index])
            outside = error > width / 2
            writer.writerow(((frame - start_frame) * STEP, x, y, speed, target_speed, steer,
                             int(outside), station, error))
            if previous_position is not None:
                travelled += math.hypot(x - previous_position[0], y - previous_position[1])
            previous_position = (x, y)
            if (frame - start_frame) % 200 == 0:
                print(f'{(frame - start_frame) * STEP:.1f} s simulated | '
                      f'{time.monotonic() - wall_start:.1f} s real | '
                      f'{speed * 3.6:.2f} km/h | {travelled:.1f} m travelled', flush=True)
            remaining = math.hypot(x - route[-1][0], y - route[-1][1])
            if target_speed < 0.05 and speed < 0.1 and frame - last_command_frame < 4:
                if stopped_frame is None:
                    stopped_frame = frame
                if (frame - stopped_frame) * STEP >= 1.0:
                    print(f'Stopped after {(frame - start_frame) * STEP:.2f} s, '
                          f'{remaining:.3f} m from the endpoint.', flush=True)
                    print(f'Real elapsed time {time.monotonic() - wall_start:.2f} s, '
                          f'distance travelled {travelled:.2f} m.', flush=True)
                    return remaining <= STOP_TOLERANCE and distance - station <= STOP_TOLERANCE
            else:
                stopped_frame = None
        print(f'Trial timed out {remaining:.3f} m from the endpoint.', flush=True)
        return False
    finally:
        command_topic.unsubscribe()
        for topic in (clock_topic, velocity_topic, pose_topic):
            topic.unadvertise()


def trial_parser(description):
    parser = argparse.ArgumentParser(description=description)
    parser.add_argument('--controller', choices=('pure_pursuit', 'stanley'), required=True)
    parser.add_argument('--speed', type=float, default=10.0, help='target speed in km/h')
    parser.add_argument('--carla-port', type=int, default=2000)
    parser.add_argument('--ros-port', type=int, default=9090)
    parser.add_argument('--container', default='ros1_car')
    parser.add_argument('--vehicle', default='vehicle.audi.a2', help='CARLA four-wheel blueprint')
    parser.add_argument('--output', type=Path, help='trajectory CSV file')
    parser.add_argument('--no-rendering', action='store_true')
    return parser


def run_trial(args, output, load_scene):
    output.parent.mkdir(parents=True, exist_ok=True)
    ros = roslibpy.Ros(host='127.0.0.1', port=args.ros_port)
    ros.run(timeout=8)
    try:
        wait_for_idle_ros(ros)
        old_sim_time = ros.get_param('/use_sim_time') if '/use_sim_time' in ros.get_params() else None
        client = carla.Client('127.0.0.1', args.carla_port)
        client.set_timeout(30.0)
        world, route_path, route, waypoints = load_scene(client)
        start_waypoint = waypoints[0]
        original_settings = world.get_settings()
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = STEP
        settings.no_rendering_mode = args.no_rendering
        world.apply_settings(settings)
        vehicle = None
        try:
            vehicle, rear_offset, wheelbase = spawn_vehicle(world, args.vehicle, start_waypoint, route[0][2])
            if not args.no_rendering:
                show_route(world, route, waypoints, args.speed)
            print(f'{args.vehicle}: wheelbase {wheelbase:.3f} m, rear offset {rear_offset:.3f} m', flush=True)
            ros.set_param('/use_sim_time', True)
            with output.open('x', newline='') as track, ros_controllers(args, wheelbase, route_path) as process:
                writer = csv.writer(track)
                writer.writerow(('time', 'x', 'y', 'speed', 'target_speed', 'steering',
                                 'outside_lane', 'progress_m', 'cross_track_m'))
                completed = follow(args, ros, client, world, vehicle, rear_offset, route, process, writer, waypoints)
            print(f'Trajectory: {output.resolve()}', flush=True)
        finally:
            try:
                if vehicle is not None:
                    vehicle.destroy()
                world.apply_settings(original_settings)
            finally:
                if old_sim_time is None:
                    if '/use_sim_time' in ros.get_params():
                        ros.delete_param('/use_sim_time')
                else:
                    ros.set_param('/use_sim_time', old_sim_time)
    finally:
        ros.terminate()
    return 0 if completed else 1
