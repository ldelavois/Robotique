import argparse
import json
import math
import numpy as np
import pybullet as p
import pybullet_data
import sys
from time import sleep, time
import control as ctrl

class Simulation:
    def __init__(self, robot_name, dt = 0.01):
        self.robot_name = robot_name
        # PyBullet initialization
        self.physicsClient = p.connect(p.GUI)
        p.setGravity(0, 0, -9.81)

        # Loading ground
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.loadURDF("plane.urdf")
        # Loading target
        self.tool_target_id = p.loadURDF("resources/tool_target.urdf", [0,0,0], useFixedBase=True)
        self.tool_pos_id = p.loadURDF("resources/tool_position.urdf", [0,0,0], useFixedBase=True)
        # Loading robot
        self.robot = p.loadURDF("resources/{:}_robot.urdf".format(robot_name),useFixedBase=True)
        self.nb_joints = p.getNumJoints(self.robot)
        self.target = np.array([0.0]* self.nb_joints)

        # Time management
        p.setRealTimeSimulation(0)
        self.t = 0
        self.dt = dt
        p.setPhysicsEngineParameter(fixedTimeStep=dt)
        self.last_tick = None

    def updateStatus(self):
        self.joints = np.array([0.0] * self.nb_joints)
        for i in range(self.nb_joints):
            self.joints[i] = p.getJointState(self.robot, i)[0]

    def setWheelSpeed(self,wheel_control):
        self.wheel_control = wheel_control;

    def setJointTarget(self, target):
        self.target = target

    def tick(self):
        for i in range(self.nb_joints):
            ctrl_mode = p.POSITION_CONTROL
            p.setJointMotorControl2(self.robot, i, ctrl_mode, self.target[i])

        # Make sure that time spent is not too high
        now = time()
        if not self.last_tick is None:
            tick_time = now - self.last_tick
            sleep_time = self.dt - tick_time
            if sleep_time > 0:
                sleep(sleep_time)
            else:
                print("Time budget exceeded: {:}".format(tick_time), file=sys.stderr)
        self.last_tick = time()
        self.t += self.dt
        p.stepSimulation()


    def setTargetPos(self, pos):
        p.resetBasePositionAndOrientation(self.tool_target_id, pos, [0,0,0,1])

    def setToolPos(self, pos):
        p.resetBasePositionAndOrientation(self.tool_pos_id, pos, [0,0,0,1])

    def drawPoint(self, pos, color):
        for dim in range(3):
            offset = [0.0,0,0]
            offset[dim] = 0.02
            start = pos - offset
            end = pos + offset
            p.addUserDebugLine(start, end, color, 3, self.dt)

    def operationalToWorld(self, pos):
        #TODO only valid for RTRobot
        if self.robot_name == "rt":
            return np.concatenate((pos, [1.05]))
        elif self.robot_name == "rrr":
            return pos
        raise RuntimeError("Unknown robot name {:}".format(robot_name))

if __name__ == "__main__":
    # Reading arguments
    parser = argparse.ArgumentParser()
    commandType = parser.add_mutually_exclusive_group(required=True)
    commandType.add_argument("--joint-space", action="store_true",
                             help="Target trajectory is provided in joint space directly")
    commandType.add_argument("--analytical-mgi", action="store_true",
                             help="Target trajectory is provided in operational space, "
                             "analyticalMGI provides the joint target")
    commandType.add_argument("--jacobian-inverse", action="store_true",
                             help="Target trajectory is provided in operational space, "
                             "jacobian inverse method is used to provide the joint target")
    commandType.add_argument("--jacobian-transposed", action="store_true",
                             help="Target trajectory is provided in operational space, "
                             "jacobian transposed method is used to provide the joint target")
    parser.add_argument("--target", type=float, nargs="+", required=True,
                        help="Fixed target, either in operational space or joint space")
    parser.add_argument("--robot", type=str, default="rt",
                        help="The name of the robot to be used: rt, rrr")

    parser.add_argument("--duration",type=float, default = -1,
                        help= "Duration of the simulation [s]")
    args = parser.parse_args()
    # Loading target
    target = np.array(args.target)
    # Converter
    robot_model = None
    if args.robot == "rt":
        robot_model = ctrl.RTRobot()
    elif args.robot == "rrr":
        robot_model = ctrl.RRRRobot()

    # Launching simulation
    simulation = Simulation(args.robot)
    simulation.updateStatus()
    last_target = np.array([0,0,0])
    while args.duration < 0 or simulation.t < args.duration:
        joint_target = None
        if args.joint_space:
            joint_target = target
        if args.analytical_mgi:
            joint_target, nb_sols = robot_model.computeAnalyticalMGI(target)
        if args.jacobian_inverse:
            joint_target = ctrl.searchJacInv(robot_model, simulation.joints, target)
        if args.jacobian_transposed:
            joint_target = ctrl.searchJacTransposed(robot_model, simulation.joints, target)
        if not joint_target is None:
            simulation.setJointTarget(joint_target)
        simulation.tick()
        simulation.updateStatus()
        last_target = target

        tool_pos = robot_model.computeMGD(simulation.joints)

        if not args.joint_space:
            target_in_world = simulation.operationalToWorld(target)
            simulation.setTargetPos(target_in_world)
        if tool_pos is not None:
            tool_pos = simulation.operationalToWorld(tool_pos)
            simulation.setToolPos(tool_pos)
