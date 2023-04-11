import os
import sys
cwd = os.getcwd()
sys.path.append(cwd)
import time
import math
import copy
from collections import OrderedDict

from ruamel.yaml import YAML
import pybullet as p
import numpy as np
import csv
np.set_printoptions(precision=3)

from util import pybullet_util
from util import util
from util import liegroup
from util import robot_kinematics

import cv2 as cv

## Configs
DT = 0.01

PRINT_ROBOT_INFO = False
VIDEO_RECORD = False

INITIAL_POS_WORLD_TO_BASEJOINT = [0, 0, 2.5 - 1.365 - 0.11]
INITIAL_QUAT_WORLD_TO_BASEJOINT = [0., 0., 0., 1.]

import argparse
parser = argparse.ArgumentParser()
parser.add_argument("--file", type=str)
parser.add_argument("--num", type=int)
parser.add_argument("--map", type=str)
parser.add_argument("--tex", type=str)
args = parser.parse_args()

## Parse file
file = args.file
num = args.num
mapfileinput = args.map
texfileinput = args.tex
vis_idx = 0
if file is not None:
    if num is not None:
        tempfile = file.split('.')[0] + "0" + ".yaml"
        with open(tempfile, 'r') as stream:
            data = YAML().load(stream)
            vis_time = data["trajectory"]["time"]
            vis_base_lin = np.array(data["trajectory"]["base_lin"])
            vis_base_ang = np.array(data["trajectory"]["base_ang"])
            vis_ee_motion_lin = dict()
            vis_ee_motion_ang = dict()
            vis_ee_wrench_lin = dict()
            vis_ee_wrench_ang = dict()
            for ee in range(2):
                vis_ee_motion_lin[ee] = np.array(
                    data["trajectory"]["ee_motion_lin"][ee])
                vis_ee_motion_ang[ee] = np.array(
                    data["trajectory"]["ee_motion_ang"][ee])
                vis_ee_wrench_lin[ee] = np.array(
                    data["trajectory"]["ee_wrench_lin"][ee])
                vis_ee_wrench_ang[ee] = np.array(
                    data["trajectory"]["ee_wrench_ang"][ee])
        if num > 1:
            for i in range(1, num):
                tempfile = file.split('.')[0] + str(i) + ".yaml"
                with open(tempfile, 'r') as stream:
                    data = YAML().load(stream)
                    vis_time = np.append(vis_time, data["trajectory"]["time"], axis=0)
                    vis_base_lin = np.append(vis_base_lin, np.array(data["trajectory"]["base_lin"]), axis=0)
                    vis_base_ang = np.append(vis_base_ang, np.array(data["trajectory"]["base_ang"]), axis=0)
                    for ee in range(2):
                        vis_ee_motion_lin[ee] = np.append(vis_ee_motion_lin[ee], np.array(
                            data["trajectory"]["ee_motion_lin"][ee]), axis=0)
                        vis_ee_motion_ang[ee] = np.append(vis_ee_motion_ang[ee], np.array(
                            data["trajectory"]["ee_motion_ang"][ee]), axis=0)
                        vis_ee_wrench_lin[ee] = np.append(vis_ee_wrench_lin[ee], np.array(
                            data["trajectory"]["ee_wrench_lin"][ee]), axis=0)
                        vis_ee_wrench_ang[ee] = np.append(vis_ee_wrench_ang[ee], np.array(
                            data["trajectory"]["ee_wrench_ang"][ee]), axis=0)
    else:
        with open(file, 'r') as stream:
            data = YAML().load(stream)
            vis_time = data["trajectory"]["time"]
            vis_base_lin = np.array(data["trajectory"]["base_lin"])
            vis_base_ang = np.array(data["trajectory"]["base_ang"])
            vis_ee_motion_lin = dict()
            vis_ee_motion_ang = dict()
            vis_ee_wrench_lin = dict()
            vis_ee_wrench_ang = dict()
            for ee in range(2):
                vis_ee_motion_lin[ee] = np.array(
                    data["trajectory"]["ee_motion_lin"][ee])
                vis_ee_motion_ang[ee] = np.array(
                    data["trajectory"]["ee_motion_ang"][ee])
                vis_ee_wrench_lin[ee] = np.array(
                    data["trajectory"]["ee_wrench_lin"][ee])
                vis_ee_wrench_ang[ee] = np.array(
                    data["trajectory"]["ee_wrench_ang"][ee])


def set_initial_config(robot, joint_id):
    p.resetJointState(robot, joint_id["leftHipPitch"], -0.6, 0.)
    p.resetJointState(robot, joint_id["rightHipPitch"], -0.6, 0.)
    p.resetJointState(robot, joint_id["leftKneePitch"], 1.2, 0.)
    p.resetJointState(robot, joint_id["rightKneePitch"], 1.2, 0.)
    p.resetJointState(robot, joint_id["leftAnklePitch"], -0.6, 0.)
    p.resetJointState(robot, joint_id["rightAnklePitch"], -0.6, 0.)
    p.resetJointState(robot, joint_id["rightShoulderPitch"], 0.2, 0.)
    p.resetJointState(robot, joint_id["leftShoulderPitch"], 0.2, 0.)
    p.resetJointState(robot, joint_id["leftShoulderRoll"], -1.1, 0.)
    p.resetJointState(robot, joint_id["rightShoulderRoll"], 1.1, 0.)
    p.resetJointState(robot, joint_id["rightElbowPitch"], 1.57, 0.)
    p.resetJointState(robot, joint_id["leftElbowPitch"], -1.57, 0.)


if __name__ == "__main__":

    # Environment Setup
    p.connect(p.GUI)
    cdist = 2
    cyaw = 300
    cpitch = -30
    p.resetDebugVisualizerCamera(cameraDistance=cdist,
                                 cameraYaw=cyaw,
                                 cameraPitch=cpitch,
                                 cameraTargetPosition=[1, 0.5, 1.5])
    p.setGravity(0, 0, -9.81)
    p.setPhysicsEngineParameter(fixedTimeStep=DT, numSubSteps=1)
    if VIDEO_RECORD:
        if not os.path.exists('video'):
            os.makedirs('video')
        for f in os.listdir('video'):
            if f == 'valkyrie_kin.mp4':
                os.remove('video/' + f)
        p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4,
                            "video/valkyrie_kin.mp4")

    # Create Robot, Ground
    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
    robot = p.loadURDF(cwd + "/robot_model/valkyrie/valkyrie.urdf",
                       INITIAL_POS_WORLD_TO_BASEJOINT,
                       INITIAL_QUAT_WORLD_TO_BASEJOINT)
    if file == "data/valkyrie_block.yaml":
        block = p.loadURDF(cwd + "/robot_model/ground/block.urdf",
                           [0, 0, 0.15],
                           useFixedBase=True)
        # Ground Plane
        p.loadURDF(cwd + "/robot_model/ground/plane.urdf", [0, 0, 0])
    elif file == "data/valkyrie_stair.yaml":
        stair = p.loadURDF(cwd + "/robot_model/ground/stair.urdf",
                           [0.2, 0, 0.],
                           useFixedBase=True)
        # Ground Plane
        p.loadURDF(cwd + "/robot_model/ground/plane.urdf", [0, 0, 0])
    elif file == "data/valkyrie_slope.yaml":
        gap = p.loadURDF(cwd + "/robot_model/ground/slope.urdf",
                         [0.325, 0, -0.125],
                         useFixedBase=True)
        # Ground Plane
        p.loadURDF(cwd + "/robot_model/ground/plane.urdf", [0, 0, 0])
    elif file[0:21] == "data/valkyrie_terrain":
        mapfile = cwd + "/" + mapfileinput
        texturefile = cwd + "/" + texfileinput
        heightfile = mapfile + ".txt"
        heightfield = np.genfromtxt(heightfile, delimiter=',')
        numHeightfieldRows = 100
        numHeightfieldColumns = 100
        scale = heightfield[1,0]
        heightfieldData = heightfield[:,2]
        zeroH = np.amax(heightfieldData)/2
        terrainShape = p.createCollisionShape(shapeType = p.GEOM_HEIGHTFIELD, heightfieldTextureScaling=1, meshScale=[scale,scale,1], heightfieldData=heightfieldData, numHeightfieldRows=numHeightfieldRows, numHeightfieldColumns=numHeightfieldColumns)
        terrain  = p.createMultiBody(0, terrainShape, basePosition=[scale*50,scale*50,zeroH])
        if texfileinput is not None:
            textureId = p.loadTexture(texturefile)
        p.changeVisualShape(terrain, -1, textureUniqueId = textureId, rgbaColor=[1,0.5,0.25,1])

    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)

    # Robot Configuration : 0 << Left Foot, 1 << Right Foot
    nq, nv, na, joint_id, link_id, pos_basejoint_to_basecom, rot_basejoint_to_basecom = pybullet_util.get_robot_config(
        robot, INITIAL_POS_WORLD_TO_BASEJOINT, INITIAL_QUAT_WORLD_TO_BASEJOINT)

    joint_screws_in_ee_at_home, ee_SE3_at_home = dict(), dict()
    open_chain_joints, base_link, ee_link = dict(), dict(), dict()
    base_link[0] = 'pelvis'
    ee_link[0] = 'leftCOP_Frame'
    open_chain_joints[0] = [
        'leftHipYaw', 'leftHipRoll', 'leftHipPitch', 'leftKneePitch',
        'leftAnklePitch', 'leftAnkleRoll'
    ]
    base_link[1] = 'pelvis'
    ee_link[1] = 'rightCOP_Frame'
    open_chain_joints[1] = [
        'rightHipYaw', 'rightHipRoll', 'rightHipPitch', 'rightKneePitch',
        'rightAnklePitch', 'rightAnkleRoll'
    ]

    for ee in range(2):
        joint_screws_in_ee_at_home[ee], ee_SE3_at_home[
            ee] = pybullet_util.get_kinematics_config(robot, joint_id, link_id,
                                                      open_chain_joints[ee],
                                                      base_link[ee],
                                                      ee_link[ee])

    # Initial Config
    set_initial_config(robot, joint_id)

    # Joint Friction
    pybullet_util.set_joint_friction(robot, joint_id, 0)

    # Run Sim
    t = 0
    dt = DT
    count = 0

    nominal_sensor_data = pybullet_util.get_sensor_data(
        robot, joint_id, link_id, pos_basejoint_to_basecom,
        rot_basejoint_to_basecom)
    base_pos = np.copy(nominal_sensor_data['base_com_pos'])
    base_quat = np.copy(nominal_sensor_data['base_com_quat'])
    joint_pos = copy.deepcopy(nominal_sensor_data['joint_pos'])
    while (1):

        # Get SensorData
        sensor_data = pybullet_util.get_sensor_data(robot, joint_id, link_id,
                                                    pos_basejoint_to_basecom,
                                                    rot_basejoint_to_basecom)

        # Parse data
        vis_t = vis_time[vis_idx]
        vis_base_lin_pos = vis_base_lin[vis_idx, 0:3]
        vis_base_ang_pos = vis_base_ang[vis_idx, 0:3]
        vis_ee_motion_lin_pos = dict()
        vis_ee_motion_ang_pos = dict()
        vis_ee_wrench_lin_pos = dict()
        vis_ee_wrench_ang_pos = dict()
        for ee in range(2):
            vis_ee_motion_lin_pos[ee] = vis_ee_motion_lin[ee][vis_idx, 0:3]
            vis_ee_motion_ang_pos[ee] = vis_ee_motion_ang[ee][vis_idx, 0:3]
            vis_ee_wrench_lin_pos[ee] = vis_ee_wrench_lin[ee][vis_idx, 0:3]
            vis_ee_wrench_ang_pos[ee] = vis_ee_wrench_ang[ee][vis_idx, 0:3]

        # Solve Inverse Kinematics
        base_pos = np.copy(vis_base_lin_pos)
        base_quat = np.copy(
            util.rot_to_quat(util.euler_to_rot(vis_base_ang_pos)))
        for ee in range(2):
            q_guess = np.array([
                nominal_sensor_data['joint_pos'][j_name]
                for j_name in open_chain_joints[ee]
            ])
            T_w_base = liegroup.RpToTrans(util.euler_to_rot(vis_base_ang_pos),
                                          vis_base_lin_pos)
            T_w_ee = liegroup.RpToTrans(
                util.euler_to_rot(vis_ee_motion_ang_pos[ee]),
                vis_ee_motion_lin_pos[ee])
            des_T = np.dot(liegroup.TransInv(T_w_base), T_w_ee)  # << T_base_ee
            q_sol, done = robot_kinematics.IKinBody(
                joint_screws_in_ee_at_home[ee], ee_SE3_at_home[ee], des_T,
                q_guess)
            for j_id, j_name in enumerate(open_chain_joints[ee]):
                joint_pos[j_name] = q_sol[j_id]

            if not done:
                print("====================================")
                print("Sovling inverse kinematics for ee-{} at time {}".format(
                    ee, vis_t))
                print("success: {}".format(done))
                print("T_w_base")
                print(T_w_base)
                print("T_w_ee")
                print(T_w_ee)
                print("q_guess")
                print(q_guess)
                print("q_sol")
                print(q_sol)
                __import__('ipdb').set_trace()
                print("====================================")

        # Handle timings
        if vis_idx == len(vis_time) - 1:
            vis_idx = 0
        else:
            vis_idx += 1

        # Visualize config
        pybullet_util.set_config(robot, joint_id, link_id, base_pos, base_quat,
                                 joint_pos)
        
        p.resetDebugVisualizerCamera( cameraDistance=cdist, cameraYaw=cyaw, cameraPitch=cpitch, cameraTargetPosition=base_pos)

        keys = p.getKeyboardEvents()
        #Keys to change camera
        if keys.get(100):  #D
            cyaw+=1
        if keys.get(97):   #A
            cyaw-=1
        if keys.get(99):   #C
            cpitch+=1
        if keys.get(102):  #F
            cpitch-=1
        if keys.get(122):  #Z
            cdist+=.01
        if keys.get(120):  #X
            cdist-=.01

        # Disable forward step
        # p.stepSimulation()

        time.sleep(dt)
        t += dt
        count += 1
