import math
import os
from pathlib import Path
import random
import subprocess
import threading
import time

import pytest
import yaml

# Set these before initializing either participant.
os.environ['ROS_DOMAIN_ID'] = str(random.SystemRandom().randrange(100, 200))
os.environ['ROS_LOCALHOST_ONLY'] = '1'
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'

import rclpy
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_prefix
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTrajectoryControllerState
from hardware_manager.action import Execute
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rosgraph_msgs.msg import Clock

G = Execute.Goal
R = Execute.Result
F = Execute.Feedback
CONFIG = Path(__file__).resolve().parents[1] / 'config' / 'takeshi'


def wait_for(predicate, timeout=5.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.01)
    assert predicate(), 'condition did not become true'


def direct(component, positions, duration=0.0):
    return G(type=G.DIRECT, component=component, positions=positions, duration=duration)


def predefined(name):
    return G(type=G.PREDEFINED, motion_name=name)


class FakeController:
    def __init__(self, node, settings, clock_message=None):
        self.node = node
        self.joints = settings['joints']
        self.clock_message = clock_message
        self.position = {joint: 0.0 for joint in self.joints}
        self.goals = []
        self.cancel_requests = 0
        self.release = threading.Event()
        self.release.set()
        self.publish = True
        self.ignore_cancel = False
        self.fail = False
        self.legacy_state = False
        self.accept_delay = 0.0
        self.accept_started = threading.Event()
        self.group = ReentrantCallbackGroup()
        self.publisher = node.create_publisher(
            JointTrajectoryControllerState, settings['state_topic'], 10)
        self.timer = node.create_timer(0.025, self.state, callback_group=self.group)
        self.server = ActionServer(
            node, FollowJointTrajectory, settings['action'], self.execute,
            goal_callback=self.accept, cancel_callback=self.cancel,
            callback_group=self.group)

    def state(self):
        if not self.publish:
            return
        msg = JointTrajectoryControllerState()
        msg.header.stamp = (self.clock_message().clock if self.clock_message else
                            self.node.get_clock().now().to_msg())
        msg.joint_names = self.joints[::-1]
        actual = [self.position[joint] for joint in msg.joint_names]
        desired = [value + 0.125 for value in actual]
        if self.legacy_state:
            msg.actual.positions = actual
            msg.desired.positions = desired
        else:
            msg.feedback.positions = actual
            msg.reference.positions = desired
        self.publisher.publish(msg)

    def accept(self, _):
        self.accept_started.set()
        time.sleep(self.accept_delay)
        return GoalResponse.ACCEPT

    def cancel(self, _):
        self.cancel_requests += 1
        return CancelResponse.REJECT if self.ignore_cancel else CancelResponse.ACCEPT

    def execute(self, goal):
        trajectory = goal.request.trajectory
        self.goals.append(trajectory)
        result = FollowJointTrajectory.Result()
        while not self.release.wait(0.01):
            if goal.is_cancel_requested:
                goal.canceled()
                return result
        if goal.is_cancel_requested:
            goal.canceled()
        elif self.fail:
            result.error_code = result.PATH_TOLERANCE_VIOLATED
            goal.abort()
        else:
            self.position.update(zip(trajectory.joint_names, trajectory.points[-1].positions))
            self.state()
            goal.succeed()
        return result


class Rig:
    def __init__(self, tmp_path, sim_time=False, missing_acceleration=False):
        hardware = yaml.safe_load((CONFIG / 'hardware.yaml').read_text())
        hardware_path = CONFIG / 'hardware.yaml'
        if missing_acceleration:
            hardware['joints']['head_pan_joint']['acceleration'] = None
            hardware_path = tmp_path / 'hardware.yaml'
            hardware_path.write_text(yaml.safe_dump(hardware))
        motions = yaml.safe_load((CONFIG / 'motions.yaml').read_text())
        motions['motions'].update({
            'compound': {'waypoints': [
                {'time': 1.5, 'positions': {'HEAD': [0.1, 0.0], 'TORSO': [0.05]}},
                {'time': 3.0, 'positions': {'LEFT_ARM': [-0.1, 0.0, 0.0, 0.0]}},
            ]},
            'invalid_compound': {'waypoints': [
                {'time': 1.0, 'positions': {'HEAD': [0.1, 0.0]}},
                {'time': 2.0, 'positions': {'TORSO': [10.0]}},
            ]},
        })
        motions_path = tmp_path / 'motions.yaml'
        motions_path.write_text(yaml.safe_dump(motions))
        rclpy.init()
        self.node = rclpy.create_node('hardware_manager_test')
        self.executor = MultiThreadedExecutor(num_threads=6)
        self.executor.add_node(self.node)
        self.sim_seconds = 10
        if sim_time:
            publisher = self.node.create_publisher(Clock, '/clock', 10)
            self.node.create_timer(0.02, lambda: publisher.publish(self.clock_message()))
        self.head = FakeController(self.node, hardware['controllers']['head'],
                                   self.clock_message if sim_time else None)
        self.arm = FakeController(self.node, hardware['controllers']['arm'],
                                  self.clock_message if sim_time else None)
        self.client = ActionClient(self.node, Execute, '/hardware/execute')
        self.spin = threading.Thread(target=self.executor.spin)
        self.spin.start()
        executable = (Path(get_package_prefix('hardware_manager')) / 'lib' /
                      'hardware_manager' / 'hardware_manager_node')
        self.log = (tmp_path / 'manager.log').open('w+')
        self.process = subprocess.Popen([
            str(executable), '--ros-args',
            '-p', f'hardware_file:={hardware_path}',
            '-p', f'motions_file:={motions_path}',
            '-p', 'state_timeout:=0.25',
            '-p', 'execution_margin:=0.1',
            '-p', 'execution_margin_factor:=0.0',
            '-p', 'cancel_timeout:=0.3',
            '-p', 'goal_response_timeout:=0.5',
            '-p', 'queue_timeout:=0.8',
            '-p', f'use_sim_time:={str(sim_time).lower()}',
        ], stdout=self.log, stderr=subprocess.STDOUT)

    def clock_message(self):
        message = Clock()
        message.clock.sec = self.sim_seconds
        return message

    def close(self):
        self.head.release.set()
        self.arm.release.set()
        if self.process.poll() is None:
            self.process.terminate()
        try:
            self.process.wait(timeout=5)
        finally:
            if self.process.poll() is None:
                self.process.kill()
                self.process.wait()
            self.executor.shutdown()
            self.spin.join()
            self.node.destroy_node()
            rclpy.shutdown()
            self.log.seek(0)
            print(self.log.read())
            self.log.close()

    def ready(self):
        assert self.client.wait_for_server(timeout_sec=5)
        wait_for(lambda: self.head.publisher.get_subscription_count() == 1 and
                 self.arm.publisher.get_subscription_count() == 1)
        time.sleep(0.1)

    def send(self, request):
        feedback = []
        future = self.client.send_goal_async(
            request, feedback_callback=lambda msg: feedback.append(msg.feedback))
        wait_for(future.done)
        goal = future.result()
        assert goal.accepted
        return goal, feedback

    @staticmethod
    def result(goal, expected, status=None):
        future = goal.get_result_async()
        wait_for(future.done)
        wrapped = future.result()
        assert wrapped.result.result_code == expected, wrapped.result.result_message
        if status is None:
            status = {
                R.SUCCESS: GoalStatus.STATUS_SUCCEEDED,
                R.SUCCESS_WITH_WARNING: GoalStatus.STATUS_SUCCEEDED,
                R.CANCELED: GoalStatus.STATUS_CANCELED,
            }.get(expected, GoalStatus.STATUS_ABORTED)
        assert wrapped.status == status
        return wrapped.result

    @staticmethod
    def cancel(goal):
        future = goal.cancel_goal_async()
        wait_for(future.done)
        assert future.result().goals_canceling


@pytest.fixture
def rig(tmp_path):
    instance = Rig(tmp_path)
    try:
        instance.ready()
        yield instance
    finally:
        instance.close()


def test_initialization_and_valid_requests(rig):
    assert not rig.head.goals and not rig.arm.goals
    goal, _ = rig.send(direct(G.HEAD, [0.25, 0.0]))
    result = rig.result(goal, R.SUCCESS)
    assert result.joint_names == ['head_pan_joint', 'head_tilt_joint']
    assert list(result.final_positions) == list(result.target_positions) == [0.25, 0.0]
    assert list(result.position_errors) == [0.0, 0.0]
    assert result.planned_duration == pytest.approx(math.sqrt(10.0 * 0.25 / math.sqrt(3.0)))
    goal, _ = rig.send(predefined('head_center'))
    assert rig.result(goal, R.SUCCESS).planned_duration == pytest.approx(2.0)
    goal, _ = rig.send(predefined('compound'))
    result = rig.result(goal, R.SUCCESS)
    assert len(rig.arm.goals) == 1 and len(rig.head.goals) == 3
    assert len(rig.arm.goals[-1].points) == 3
    assert list(rig.arm.goals[-1].points[-1].positions[:2]) == [0.05, -0.1]
    assert rig.arm.goals[-1].points[-1].time_from_start == (
        rig.head.goals[-1].points[-1].time_from_start)
    assert result.final_positions == result.target_positions
    assert rig.arm.goals[-1].header.stamp == rig.head.goals[-1].header.stamp
    assert rig.arm.goals[-1].header.stamp.sec > 0


def test_torso_uses_operating_speed_limit(rig):
    goal, _ = rig.send(direct(G.TORSO, [0.3], 1.0))
    result = rig.result(goal, R.SUCCESS_WITH_WARNING)
    assert result.warnings == ['duration_adjusted']
    assert result.planned_duration == pytest.approx(1.875 * 0.3 / 0.15)


@pytest.mark.parametrize('target,duration,warnings', [
    ([0.2, 0.0], 0.01, {'duration_adjusted'}),
    ([2.0, 0.0], 0.0, {'position_clamped'}),
    ([2.0, 0.0], 0.01, {'position_clamped', 'duration_adjusted'}),
])
def test_adjustments(rig, target, duration, warnings):
    goal, feedback = rig.send(direct(G.HEAD, target, duration))
    result = rig.result(goal, R.SUCCESS_WITH_WARNING)
    assert set(result.warnings) == warnings
    assert list(result.target_positions) == [min(target[0], 1.74), 0.0]
    if duration:
        wait_for(lambda: any('Duration adjusted' in msg.status_message for msg in feedback))


@pytest.mark.parametrize('command,code', [
    (direct(G.TORSO, [1.0]), R.JOINT_LIMIT_VIOLATION),
    (direct(G.LEFT_ARM, [1.0, 0.0, 0.0, 0.0]), R.JOINT_LIMIT_VIOLATION),
    (direct(G.RIGHT_ARM, [0.0]), R.COMPONENT_NOT_AVAILABLE),
    (direct(G.HEAD, [float('nan'), 0.0]), R.INVALID_GOAL),
    (direct(G.HEAD, [0.0, 0.0], -0.1), R.INVALID_GOAL),
    (predefined('unknown'), R.MOTION_NOT_FOUND),
    (predefined('invalid_compound'), R.JOINT_LIMIT_VIOLATION),
])
def test_validation(rig, command, code):
    goal, _ = rig.send(command)
    result = rig.result(goal, code)
    assert not result.final_positions
    assert not rig.head.goals and not rig.arm.goals


def test_missing_and_stale_state(rig):
    rig.arm.publish = False
    time.sleep(0.33)
    goal, _ = rig.send(direct(G.TORSO, [0.1]))
    assert not rig.result(goal, R.STATE_UNAVAILABLE).final_positions
    assert not rig.arm.goals


def test_same_component_preemption_and_invalid_replacement(rig):
    rig.head.release.clear()
    previous, _ = rig.send(direct(G.HEAD, [0.2, 0.0], 3.0))
    wait_for(lambda: len(rig.head.goals) == 1)
    invalid, _ = rig.send(direct(G.HEAD, [0.2]))
    rig.result(invalid, R.INVALID_GOAL)
    assert rig.head.cancel_requests == 0
    replacement, _ = rig.send(direct(G.HEAD, [0.3, 0.0], 3.0))
    rig.result(previous, R.PREEMPTED)
    wait_for(lambda: len(rig.head.goals) == 2)
    assert rig.head.cancel_requests == 1
    rig.head.release.set()
    rig.result(replacement, R.SUCCESS)


def test_independent_controllers_and_feedback(rig):
    rig.head.release.clear()
    rig.arm.release.clear()
    head, head_feedback = rig.send(direct(G.HEAD, [0.1, 0.0], 3.0))
    arm, arm_feedback = rig.send(direct(G.TORSO, [0.1], 3.0))
    wait_for(lambda: len(rig.head.goals) == len(rig.arm.goals) == 1)

    def running(feedback):
        return [msg for msg in feedback if msg.status_code == F.EXECUTING and msg.position_errors]

    wait_for(lambda: running(head_feedback) and running(arm_feedback))
    assert running(head_feedback)[-1].joint_names == rig.head.joints
    assert running(arm_feedback)[-1].joint_names == rig.arm.joints
    assert running(head_feedback)[-1].position_errors == pytest.approx([0.125] * 2)
    rig.head.release.set()
    rig.result(head, R.SUCCESS)
    rig.cancel(arm)
    rig.result(arm, R.CANCELED)
    assert rig.arm.cancel_requests == 1 and rig.head.cancel_requests == 0


def test_shared_controller_queue_and_current_hold(rig):
    rig.arm.release.clear()
    torso, _ = rig.send(direct(G.TORSO, [0.15], 3.0))
    wait_for(lambda: len(rig.arm.goals) == 1)
    arm, feedback = rig.send(direct(G.LEFT_ARM, [-0.2, 0.0, 0.0, 0.0], 3.0))
    wait_for(lambda: any(msg.status_code == F.QUEUED for msg in feedback))
    assert all(msg.progress == 0.0 for msg in feedback)
    assert len(rig.arm.goals) == 1
    rig.arm.release.set()
    rig.result(torso, R.SUCCESS)
    result = rig.result(arm, R.SUCCESS)
    assert rig.arm.goals[-1].points[0].positions[0] == 0.15
    assert result.target_positions[0] == result.final_positions[0] == 0.15


def test_queued_replacement_cancellation_and_timeout(rig):
    rig.arm.release.clear()
    arm, _ = rig.send(direct(G.LEFT_ARM, [-0.1, 0.0, 0.0, 0.0], 3.0))
    wait_for(lambda: len(rig.arm.goals) == 1)
    first, _ = rig.send(direct(G.TORSO, [0.1], 3.0))
    second, _ = rig.send(direct(G.TORSO, [0.2], 3.0))
    rig.result(first, R.PREEMPTED)
    rig.cancel(second)
    assert rig.result(second, R.CANCELED).execution_time == 0.0
    timed, _ = rig.send(direct(G.TORSO, [0.25], 3.0))
    assert rig.result(timed, R.RESOURCE_BUSY).execution_time == 0.0
    assert len(rig.arm.goals) == 1 and rig.arm.cancel_requests == 0
    rig.arm.release.set()
    rig.result(arm, R.SUCCESS)


def test_compound_failure_cancels_other_native_action(rig):
    rig.head.release.clear()
    rig.arm.fail = True
    goal, _ = rig.send(predefined('compound'))
    rig.result(goal, R.CONTROL_FAILED)
    assert rig.head.cancel_requests == 1
    assert len(rig.head.goals) == len(rig.arm.goals) == 1


def test_cancel_while_waiting_for_native_acceptance(rig):
    rig.head.accept_delay = 0.2
    rig.head.release.clear()
    goal, _ = rig.send(direct(G.HEAD, [0.1, 0.0], 3.0))
    wait_for(rig.head.accept_started.is_set)
    rig.cancel(goal)
    rig.result(goal, R.CANCELED)
    assert rig.head.cancel_requests == 1


def test_execution_timeout_and_unknown_quarantine(rig):
    rig.head.release.clear()
    rig.head.ignore_cancel = True
    goal, _ = rig.send(direct(G.HEAD, [0.0, 0.0], 0.15))
    result = rig.result(goal, R.TIMEOUT)
    assert 'unconfirmed' in result.result_message
    assert rig.head.cancel_requests == 1
    replacement, _ = rig.send(direct(G.HEAD, [0.1, 0.0], 3.0))
    rig.result(replacement, R.CONTROLLER_NOT_AVAILABLE)
    independent, _ = rig.send(direct(G.TORSO, [0.1], 3.0))
    rig.result(independent, R.SUCCESS)
    assert len(rig.head.goals) == 1
    rig.head.release.set()
    # A late terminal result unlocks only its own physical controller.
    time.sleep(0.15)
    replacement, _ = rig.send(direct(G.HEAD, [0.1, 0.0], 3.0))
    rig.result(replacement, R.SUCCESS)


def test_execution_timeout_with_confirmed_cancel(rig):
    rig.head.release.clear()
    goal, _ = rig.send(direct(G.HEAD, [0.0, 0.0], 0.15))
    result = rig.result(goal, R.TIMEOUT)
    assert 'unconfirmed' not in result.result_message
    assert rig.head.cancel_requests == 1


def test_incomplete_profile_blocks_only_affected_controller(tmp_path):
    rig = Rig(tmp_path, missing_acceleration=True)
    try:
        rig.ready()
        goal, _ = rig.send(predefined('head_center'))
        result = rig.result(goal, R.COMPONENT_NOT_AVAILABLE)
        assert 'acceleration limit' in result.result_message
        assert not rig.head.goals and not rig.arm.goals
        goal, _ = rig.send(direct(G.TORSO, [0.0]))
        rig.result(goal, R.SUCCESS)
    finally:
        rig.close()


@pytest.mark.skipif(not hasattr(JointTrajectoryControllerState(), 'actual'),
                    reason='control_msgs has no legacy state fields')
def test_legacy_controller_state(rig):
    rig.head.legacy_state = True
    goal, _ = rig.send(direct(G.HEAD, [0.1, 0.0], 1.0))
    assert list(rig.result(goal, R.SUCCESS).final_positions) == [0.1, 0.0]


def test_compound_waits_for_all_controllers_before_dispatch(rig):
    rig.arm.release.clear()
    torso, _ = rig.send(direct(G.TORSO, [0.0], 3.0))
    wait_for(lambda: len(rig.arm.goals) == 1)
    # The compound conflicts logically with the active torso and preempts it.
    rig.arm.ignore_cancel = True
    compound, feedback = rig.send(predefined('compound'))
    wait_for(lambda: any(msg.status_code == F.QUEUED for msg in feedback))
    assert not rig.head.goals
    wait_for(lambda: rig.arm.cancel_requests == 1)
    rig.arm.release.set()
    rig.result(torso, R.PREEMPTED)
    rig.result(compound, R.SUCCESS)
    assert len(rig.head.goals) == 1 and len(rig.arm.goals) == 2


def test_preemption_of_compound_stops_all_native_actions(rig):
    rig.head.release.clear()
    rig.arm.release.clear()
    original, _ = rig.send(predefined('compound'))
    wait_for(lambda: len(rig.head.goals) == len(rig.arm.goals) == 1)
    replacement, _ = rig.send(direct(G.HEAD, [0.1, 0.0], 3.0))
    rig.result(original, R.PREEMPTED)
    assert rig.head.cancel_requests == rig.arm.cancel_requests == 1
    wait_for(lambda: len(rig.head.goals) == 2)
    assert len(rig.arm.goals) == 1
    rig.head.release.set()
    rig.result(replacement, R.SUCCESS)


def test_native_acceptance_timeout_cancels_late_goal(rig):
    rig.head.accept_delay = 0.65
    rig.head.release.clear()
    goal, _ = rig.send(direct(G.HEAD, [0.1, 0.0], 3.0))
    result = rig.result(goal, R.CONTROLLER_NOT_AVAILABLE)
    assert 'acceptance timeout' in result.result_message
    assert rig.head.cancel_requests == 1


def test_unconfirmed_client_cancel_is_aborted(rig):
    rig.head.release.clear()
    rig.head.ignore_cancel = True
    goal, _ = rig.send(direct(G.HEAD, [0.1, 0.0], 3.0))
    wait_for(lambda: len(rig.head.goals) == 1)
    rig.cancel(goal)
    assert 'unconfirmed' in rig.result(goal, R.CONTROL_FAILED).result_message


def test_execution_timeout_uses_ros_clock(tmp_path):
    rig = Rig(tmp_path, sim_time=True)
    try:
        rig.ready()
        rig.head.release.clear()
        goal, feedback = rig.send(direct(G.HEAD, [0.0, 0.0], 0.15))
        result = goal.get_result_async()
        wait_for(lambda: len(rig.head.goals) == 1)
        time.sleep(0.35)
        assert not result.done() and rig.head.cancel_requests == 0
        assert feedback and all(msg.progress == 0.0 for msg in feedback)
        rig.sim_seconds += 1
        wait_for(result.done)
        assert result.result().result.result_code == R.TIMEOUT
        assert result.result().status == GoalStatus.STATUS_ABORTED
        assert rig.head.cancel_requests == 1
    finally:
        rig.close()


def test_shutdown_cancels_active_goal_and_drains_queue(rig):
    rig.arm.release.clear()
    torso, _ = rig.send(direct(G.TORSO, [0.1], 3.0))
    wait_for(lambda: len(rig.arm.goals) == 1)
    arm, feedback = rig.send(direct(G.LEFT_ARM, [-0.1, 0.0, 0.0, 0.0], 3.0))
    wait_for(lambda: any(msg.status_code == F.QUEUED for msg in feedback))
    active_result = torso.get_result_async()
    queued_result = arm.get_result_async()
    time.sleep(0.1)
    rig.process.terminate()
    wait_for(lambda: active_result.done() and queued_result.done())
    assert active_result.result().status == queued_result.result().status == GoalStatus.STATUS_ABORTED
    assert 'shutting down' in active_result.result().result.result_message
    assert queued_result.result().result.execution_time == 0.0
    assert len(rig.arm.goals) == 1 and rig.arm.cancel_requests == 1
    assert rig.process.wait(timeout=3) == 0
