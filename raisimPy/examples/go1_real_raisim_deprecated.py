import os
import numpy as np
import raisimpy as raisim
import time
import pdb

import json

from robot_interface_go1 import RobotInterfaceGo1  # pytype: disable=import-error | amarco: # From motion_imitation/motion_imitation/robots/a1_robot.py
"""
'robot_interface_go1' is the Pyhton library that wraps the c++ unitree_legged_sdk to be callable from python.
It was originally developed by google, for A1, and accessible at the motion_imitation github repo. We modified it for Go1.
It needs to be compiled in every machine. The python bindings are found at: /Users/alonrot/work/code_projects_WIP/motion_imitation/third_party/unitree_legged_sdk
Once compiled, the library address needs to be included in the python path:
export PYTHONPATH=$PYTHONPATH:/Users/alonrot/work/code_projects_WIP/motion_imitation/third_party/unitree_legged_sdk/build

I can't compile it on mac because for compiling it, I need to link against unitree_legged_sdk. However, unitree_legged_sdk is a 
pre-compiled library provided by Unitree and such pre-compilation is provided only for linux.
"""


def ramp_from_pos1_to_pos2(tt,t_offset,t_duration,pos1_joint,pos2_joint):


    tt_ela = tt - t_offset

    alpha = tt_ela / t_duration

    pos_des = pos1_joint * (1. - alpha) + alpha*pos2_joint

    return pos_des


def get_trajectory_from_file(tt,joints_pos_resting):

    # motion_file = "./dog_pace.txt"
    motion_file = "./inplace_steps.txt"
    with open(motion_file, "r") as f:
        motion_json = json.load(f)
    frames_vec = np.array(motion_json["Frames"])

    joint_positions = frames_vec[:,-12::]

    deltaT_frames = 0.01667

    # We need to interpolate between frames because we're assuming a 0.001 loop time
    quo, rem = np.divmod(tt/3.0,deltaT_frames)

    alpha = rem/deltaT_frames # [0,1]

    ind = int(quo) % frames_vec.shape[0]
    ind_next = (int(quo)+1) % frames_vec.shape[0]

    # print("ind:",ind)
    # print("ind_next:",ind_next)
    # print("alpha:",alpha)

    des_pos = joint_positions[ind,:] * (1. - alpha) + joint_positions[ind_next,:]*alpha

    des_joint_position_full = joints_pos_resting.copy()
    des_joint_position_full[-12::] = des_pos

    return des_joint_position_full



def test():

    motorcmd = np.zeros(2,dtype=np.float32)

    # Construct floor:
    # https://raisim.com/sections/HeightMap_example_png.html

    raisim.World.setLicenseFile(os.path.dirname(os.path.abspath(__file__)) + "/../../rsc/activation.raisim")

    # go1_urdf_file = "/Users/alonrot/work/code_projects_WIP/unitree_pybullet_modif/data/go1_description/urdf/go1.urdf"
    # go1_urdf_file = "/Users/alonrot/work/code_projects_WIP/unitree_pybullet_modif/data/go1_description/urdf/go1_no_gazebo.urdf"
    go1_urdf_file = "/home/ubuntu/mounted_home/work/code_projects_WIP/unitree_mujoco/data/go1/urdf/go1.urdf"

    world = raisim.World()
    world.setTimeStep(0.001)
    server = raisim.RaisimServer(world)
    ground = world.addGround()

    go1_nominal_joint_config = np.array([0.0, -1.5245, 0.3435, 1.0, 0.0, 0.0, 1.0, # (com [Y(green), X(red), Z(blue)], quaternion [1,i,j,k]],...
                                        0.0136, 0.7304, -1.4505,  # FR [hip lateral, hip, knee]
                                        -0.0118, 0.7317, -1.4437, # FL [hip lateral, hip, knee]
                                        0.0105, 0.6590, -1.3903, # RR [hip lateral, hip, knee]
                                        -0.0102, 0.6563, -1.3944]) # RL [hip lateral, hip, knee]
    go1_nominal_joint_config[2] += 1.0 # Robot hanging

    # # Visual object (without physics):
    go1_visual = server.addVisualArticulatedSystem("go1",go1_urdf_file) # Add a visual capsule without physics. See src/world.cpp
    go1_visual.setGeneralizedCoordinate(go1_nominal_joint_config)
    go1_visual.setColor(0.0,0.9,0.0,0.2)

    print("Launching Raisim server...")
    server.launchServer(8080)

    time.sleep(2.0)

    motorcmd = np.zeros(60,dtype=np.float32)

    interface_real_go1 = RobotInterfaceGo1()

    Nsteps = 1000
    pos_curr_joint = go1_nominal_joint_config.copy()
    for ii in range(Nsteps):

        interface_real_go1.send_command(motorcmd)
        interface_real_go1.collect_observations()

        joint_pos_curr = interface_real_go1.get_joint_pos_curr()
        joint_vel_curr = interface_real_go1.get_joint_vel_curr()

        pos_curr_joint[7::] = joint_pos_curr

        go1_visual.setGeneralizedCoordinate(pos_curr_joint)

        time.sleep(0.002)


        pdb.set_trace()


    server.killServer()


if __name__ == "__main__":

    test()


