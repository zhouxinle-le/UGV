"""Run one isolated ROS1/Gazebo trial inside the prepared Docker container."""

import argparse
import csv
import math
import os
import signal
import socket
import subprocess
import time
from pathlib import Path

import numpy as np

from evaluate import evaluate
from progress import RouteProgress


STEP = 0.05
REAR_OFFSET = 0.945  # Same rear-axle offset as vehicle_pose_and_velocity_updater.py.
FIELDS = ('route', 'controller', 'target_speed_kmh', 'repeat', 'completed', 'outside_lane',
          'elapsed_s', 'endpoint_error_m', 'rmse_m', 'max_error_m', 'mean_speed_kmh',
          'steering_rate_rms_deg_s')
TRACK_FIELDS = ('time', 'x', 'y', 'speed', 'target_speed', 'steering',
                'outside_lane', 'progress_m', 'cross_track_m')


def isolated_environment():
    with socket.socket() as ros_socket, socket.socket() as gazebo_socket:
        ros_socket.bind(('127.0.0.1', 0))
        gazebo_socket.bind(('127.0.0.1', 0))
        ros_port = ros_socket.getsockname()[1]
        gazebo_port = gazebo_socket.getsockname()[1]
    os.environ.pop('ROS_HOSTNAME', None)
    os.environ.update(ROS_MASTER_URI=f'http://127.0.0.1:{ros_port}',
                      GAZEBO_MASTER_URI=f'http://127.0.0.1:{gazebo_port}', ROS_IP='127.0.0.1')
    return ros_port


def collect(args, route, launch, x, y, yaw):
    import rospy
    from gazebo_msgs.msg import ModelState
    from gazebo_msgs.srv import GetModelState, SetModelState
    from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
    from std_srvs.srv import Empty
    from tf.transformations import quaternion_from_euler

    # Wait on the master before registering the recorder with rospy.
    master = rospy.get_master()
    deadline = time.monotonic() + 25
    while True:
        try:
            master.getPid()
            break
        except (OSError, rospy.ROSException):
            if launch.poll() is not None or time.monotonic() >= deadline:
                raise RuntimeError('Isolated ROS master did not start.')
            time.sleep(.15)
    rospy.init_node('gazebo_trial', disable_signals=True)

    state = {}

    def on_command(message):
        state['command'] = rospy.Time.now().to_sec(), message

    subscriptions = [
        rospy.Subscriber('/smart/rear_pose', PoseStamped, lambda msg: state.update(pose=msg), queue_size=1),
        rospy.Subscriber('/smart/velocity', TwistStamped, lambda msg: state.update(velocity=msg), queue_size=1),
        rospy.Subscriber('/gazebo_trial/command', Twist, on_command, queue_size=1),
    ]
    drive = rospy.Publisher('/gazebo_trial/drive', Twist, queue_size=1)
    try:
        for service in ('get_model_state', 'set_model_state', 'pause_physics', 'unpause_physics'):
            rospy.wait_for_service(f'/gazebo/{service}', timeout=25)
        deadline = time.monotonic() + 25
        while len(state) < 3 or drive.get_num_connections() == 0:
            if launch.poll() is not None or time.monotonic() >= deadline:
                raise RuntimeError('Gazebo did not publish pose, velocity and control messages.')
            time.sleep(.02)

        pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        get_model = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        set_model = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        drive.publish(Twist())
        pause()
        try:
            current = get_model('smart', 'world')
            if not current.success:
                raise RuntimeError(current.status_message)
            reset = ModelState(model_name='smart', reference_frame='world')
            reset.pose.position.x, reset.pose.position.y = x, y
            reset.pose.position.z = current.pose.position.z
            (reset.pose.orientation.x, reset.pose.orientation.y,
             reset.pose.orientation.z, reset.pose.orientation.w) = quaternion_from_euler(0, 0, yaw)
            reply = set_model(reset)
            if not reply.success:
                raise RuntimeError(reply.status_message)
            reset_stamp = rospy.Time.now()
            state.clear()
        finally:
            unpause()

        progress = RouteProgress(route[:, :2])
        length = float(progress.stations[-1])
        deadline = time.monotonic() + 10
        while (len(state) < 3 or state['pose'].header.stamp <= reset_stamp):
            if launch.poll() is not None or time.monotonic() >= deadline:
                raise RuntimeError('No fresh ROS state after resetting the vehicle.')
            time.sleep(.01)
        started = rospy.Time.now().to_sec()
        next_sample = started
        sim_limit = 2 * length / (args.speed / 3.6) + 30
        last_clock = started
        clock_deadline = time.monotonic() + 10
        stopped_since = None
        samples = []
        print(f'Collecting {args.controller} at {args.speed} km/h over {length:.1f} m.', flush=True)
        with args.trajectory.open('w', newline='') as stream:
            writer = csv.writer(stream)
            writer.writerow(TRACK_FIELDS)
            while not rospy.is_shutdown() and launch.poll() is None:
                now = rospy.Time.now().to_sec()
                if now != last_clock:
                    last_clock, clock_deadline = now, time.monotonic() + 10
                if time.monotonic() >= clock_deadline:
                    raise RuntimeError('Gazebo clock stopped during the experiment.')
                if now < next_sample:
                    time.sleep(.004)
                    continue
                next_sample = started + (int((now - started) / STEP + 1e-6) + 1) * STEP
                position = state['pose'].pose.position
                velocity = state['velocity'].twist.linear
                speed = math.hypot(velocity.x, velocity.y)
                command_time, command = state['command']
                if now - command_time > .2:
                    command = Twist()
                drive.publish(command)
                station, error, _, _ = progress.update(position.x, position.y)
                elapsed = now - started
                row = (elapsed, position.x, position.y, speed, command.linear.x,
                       command.angular.z, int(error > args.half_width), station, error)
                writer.writerow(row)
                samples.append(row)
                near_end = length - station <= 1.0
                if near_end and speed < .1 and command.linear.x < .05:
                    if stopped_since is None:
                        stopped_since = elapsed
                    if elapsed - stopped_since >= 1.0:
                        break
                else:
                    stopped_since = None
                if elapsed >= sim_limit:
                    break
            else:
                raise RuntimeError('Gazebo or its ROS nodes exited during the experiment.')
        return np.asarray(samples)
    finally:
        drive.publish(Twist())
        for subscription in subscriptions:
            subscription.unregister()
        rospy.signal_shutdown('Trial finished')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--route', type=Path, required=True)
    parser.add_argument('--route-name', required=True)
    parser.add_argument('--controller', choices=('pure_pursuit', 'stanley'), required=True)
    parser.add_argument('--speed', type=int, required=True)
    parser.add_argument('--repeat', type=int, required=True)
    parser.add_argument('--half-width', type=float, required=True)
    parser.add_argument('--trajectory', type=Path, required=True)
    parser.add_argument('--summary', type=Path, required=True)
    args = parser.parse_args()

    route = np.loadtxt(args.route, delimiter=',', ndmin=2)
    steps = np.linalg.norm(np.diff(route[:, :2], axis=0), axis=1)
    if route.shape[1] != 3 or not np.isfinite(route).all() or (steps < 1e-6).any() or steps.sum() <= 20:
        parser.error('Expected finite x,y,yaw waypoints without repeated adjacent positions and a route longer than 20 m.')
    rear_x, rear_y, yaw = route[0]
    x, y = rear_x + REAR_OFFSET * math.cos(yaw), rear_y + REAR_OFFSET * math.sin(yaw)
    ros_port = isolated_environment()
    command = ['roslaunch', '--port', str(ros_port), str(Path(__file__).with_name('trial.launch')),
               f'controller:={args.controller}', f'route:={args.route}', f'speed:={args.speed}',
               f'x:={x}', f'y:={y}', f'yaw:={yaw}']
    launch = subprocess.Popen(command, start_new_session=True)
    try:
        samples = collect(args, route, launch, x, y, yaw)
    finally:
        if launch.poll() is None:
            os.killpg(launch.pid, signal.SIGINT)
            try:
                launch.wait(timeout=18)
            except subprocess.TimeoutExpired:
                os.killpg(launch.pid, signal.SIGKILL)
                launch.wait()
    metrics = evaluate(route[:, :2], samples, trim=10.0)
    row = dict(route=args.route_name, controller=args.controller, target_speed_kmh=args.speed,
               repeat=args.repeat, outside_lane=bool(samples[:, 6].any()), **metrics)
    with args.summary.open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=FIELDS)
        writer.writeheader()
        writer.writerow(row)
    print(f"completed={metrics['completed']}, outside_lane={row['outside_lane']}, RMSE={metrics['rmse_m']}", flush=True)


if __name__ == '__main__':
    main()
