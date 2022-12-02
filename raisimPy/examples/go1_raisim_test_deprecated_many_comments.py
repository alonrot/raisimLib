import os
import numpy as np
import raisimpy as raisim
import time
import pdb

import json

from robot_interface import RobotInterface  # pytype: disable=import-error | amarco: # From motion_imitation/motion_imitation/robots/a1_robot.py
"""
'robot_interface' is the Pyhton library that wraps the c++ unitree_legged_sdk to be callable from python.
It was originally developed by google, and accessible at the motion_imitation github repo.
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

    # Construct floor:
    # https://raisim.com/sections/HeightMap_example_png.html

    raisim.World.setLicenseFile(os.path.dirname(os.path.abspath(__file__)) + "/../../rsc/activation.raisim")

    """
    In general, about urdf files: the corresponding meshes can be either .stl or .dae. 
        * STL is computationally light, but unfortunately, the ones I found contain no texture, so it isn't great for visualization.
        * DAE is a more dense mesh, but for some reason the rear joints are mislabeled as front and viceversa
    """
    # http://wiki.ros.org/urdf/XML/joint
    # go1_urdf_file = "/Users/alonrot/work/code_projects_WIP/unitree_pybullet_modif/data/go1_description/urdf/go1.urdf"
    # go1_urdf_file = "/Users/alonrot/work/code_projects_WIP/unitree_pybullet_modif/data/go1_description/urdf/go1_no_gazebo.urdf"
    go1_urdf_file = "/Users/alonrot/work/code_projects_WIP/unitree_mujoco/data/go1/urdf/go1.urdf"
    # go1 = world_->addArticulatedSystem("/Users/alonrot/work/code_projects_WIP/unitree_mujoco/data/go1/urdf/go1.urdf");

    world = raisim.World()
    world.setTimeStep(0.001)
    server = raisim.RaisimServer(world)
    # ground = world.addGround()
    go1 = world.addArticulatedSystem(go1_urdf_file)
    # pdb.set_trace()
    go1.setName("go1")
    # go1_nominal_joint_config = np.array([0.0, -1.5, 0.54, 1.0, 0.0, 0.0, 1.0, # (com [Y(green), X(red), Z(blue)], quaternion [1,i,j,k]],...
    #                                     0.01, 0.4, -1.3, -0.01, 0.4, -1.3, 0.01, -0.4, 1.3, -0.01, -0.4, 1.3]) # ...joint_positions [hip, thigh, knee]x4)


    # go1_nominal_joint_config = np.array([0.0, -1.5, 0.54, 1.0, 0.0, 0.0, 1.0, # (com [Y(green), X(red), Z(blue)], quaternion [1,i,j,k]],...
    #                                     0.01, 0.7, -1.4,  # FR [hip lateral, hip, knee]
    #                                     -0.01, 0.7, -1.4, # FL [hip lateral, hip, knee]
    #                                     0.01, 0.7, -1.3, # RR [hip lateral, hip, knee]
    #                                     -0.01, 0.7, -1.3]) # RL [hip lateral, hip, knee]

    go1_nominal_joint_config = np.array([0.0, -1.5245, 0.3435, 1.0, 0.0, 0.0, 1.0, # (com [Y(green), X(red), Z(blue)], quaternion [1,i,j,k]],...
                                        0.0136, 0.7304, -1.4505,  # FR [hip lateral, hip, knee]
                                        -0.0118, 0.7317, -1.4437, # FL [hip lateral, hip, knee]
                                        0.0105, 0.6590, -1.3903, # RR [hip lateral, hip, knee]
                                        -0.0102, 0.6563, -1.3944]) # RL [hip lateral, hip, knee]


    # # // INCORRECT posture:
    # go1_nominal_joint_config = np.array([0.1454, -1.4882, 0.3408, 1.0, 0.0, 0.0, 1.0, # (com [Y(green), X(red), Z(blue)], quaternion [1,i,j,k]],...
    #                                     0.0273, 0.3974, -1.3512,  # FR [hip lateral, hip, knee]
    #                                     -0.0371, 0.4368, -1.3878, # FL [hip lateral, hip, knee]
    #                                     0.0218, -0.4503, 1.3713, # RR [hip lateral, hip, knee]
    #                                     -0.0141, -0.3949, 1.347]) # RL [hip lateral, hip, knee]



    # // INCORRECT posture:
    # gc_init_ << 0.1454, -1.4882, 0.3408, 
    # 1.0, 0.0, 0.0, 0.0, 
    # 0.0273, 0.3974, -1.3512, // FR
    # -0.0371, 0.4368, -1.3878, 
    # 0.0218, -0.4503, 1.3713, 
    # -0.0141, -0.3949, 1.347; // go1



    # # middle
    # go1_nominal_joint_config = np.array([0.0, -1.5, 0.50, 1.0, 0.0, 0.0, 1.0,
    # -0.07549, -0.26493, -0.80317, -0.11877, 0.35467, -0.59078, -0.20209, -0.14052, -0.72329, -0.16125, 0.51232, -0.79071])

    # # initial
    # go1_walking_starting = np.array([0.0, -1.5, 0.45, 1.0, 0.0, 0.0, 1.0,
    #     -0.12721, 0.07675, -0.95545, -0.25301, 0.18682, -1.14403, -0.19362, 0.14030, -0.77823, -0.09528, 0.05437, -0.97596])


    # go1_nominal_joint_config = go1_walking_starting.copy()

    go1.setGeneralizedCoordinate(go1_nominal_joint_config)
    # go1.setPdGains(200*np.ones([18]), np.ones([18]))
    go1.setPdGains(50*np.ones([18]), np.ones([18]))
    go1.setPdTarget(go1_nominal_joint_config, np.zeros([18]))

    # # Visual object (without physics):
    # go1_visual = server.addVisualArticulatedSystem("go1",go1_urdf_file) # Add a visual capsule without physics. See src/world.cpp
    # go1_visual.setGeneralizedCoordinate(go1_nominal_joint_config)
    # go1_visual.setColor(0.0,0.9,0.0,0.2)

    # NOTE: How does stepSimulation() in pybullet take care of updating the body position and orientation? -> getLinkState()

    print("Launching Raisim server...")
    server.launchServer(8080)

    time.sleep(2.0)
    world.integrate1()

    ### get dynamic properties
    # Mass matrix:
    mass_matrix = go1.getMassMatrix()
    # Non-linear term (gravity+coriolis):
    non_linearities = go1.getNonlinearities([0,0,-9.81])
    # Jacobians:
    jaco_foot_lh_linear = go1.getDenseFrameJacobian("LF_ADAPTER_TO_FOOT")
    jaco_foot_lh_angular = go1.getDenseFrameRotationalJacobian("LF_ADAPTER_TO_FOOT")

    
    omega_ramp = 1.5
    ampl = 0.15
    id_thighs = [8,11,14,17]
    id_knees = [9,12,15,18]
    t_init = 2.0 # Seconds
    def joints_target_pos_ramp(t):
        joints_target_curr = np.copy(joints_pos_resting)
        joints_target_curr[id_thighs] += ampl*np.sin(omega_ramp*(t-t_init))
        joints_target_curr[id_knees] -= ampl*np.sin(omega_ramp*(t-t_init))
        return joints_target_curr


    t_ramp = t_init + 3.0
    t_walk = t_ramp + 2.0

    time_ela = 0.0
    time_start = time.time_ns()
    Nsteps_plot = 2500
    joints_target_pos_vec = np.zeros(Nsteps_plot)
    time_ela_vec = np.zeros(Nsteps_plot)
    print("Starting while() loop...")
    print("Waiting {} seconds ...".format(t_init))
    first_time = True
    while(True):

        # Set target position:
        if first_time: 
            print("Starting movement...")
            joints_pos_resting = go1.getGeneralizedCoordinate()
            joints_target_pos = joints_pos_resting.copy()
            first_time = False

        if time_ela >= t_init:

            # joints_target_pos = ramp_from_pos1_to_pos2(time_ela,t_offset=t_init,t_duration=t_ramp-t_init,pos1_joint=joints_pos_resting,pos2_joint=go1_walking_starting)
            # joints_target_pos = get_trajectory_from_file(time_ela-t_init,joints_target_pos)

            joints_target_pos = joints_target_pos_ramp(time_ela)


        elif time_ela >= t_ramp:
            pass

            # joints_target_pos = joints_target_pos_ramp(time_ela)
            # joints_target_pos = joints_target_pos_ramp(time_ela)

            # joints_target_pos = get_trajectory_from_file(time_ela-t_ramp,joints_target_pos_last)

            # joints_target_pos = get_trajectory_from_file(0.01667*20,joints_pos_resting)

        go1.setPdTarget(joints_target_pos, np.zeros([18]))
        # go1.setPTarget(joints_target_pos) # Not part of the Python bindings

        # Read the position from the simulated robot and copy it to the visualization object:
        position_curr_go1 = go1.getGeneralizedCoordinate()
        position_curr_go1[1] -= 0.5 # Shift the position a bit for better visualization
        # go1_visual.setGeneralizedCoordinate(position_curr_go1)


        time.sleep(0.001)
        world.integrate()

        # Update elapsed time:
        time_ela = (time.time_ns() - time_start) / 1000000000. # Seconds


    server.killServer()


    # Cannot be tested on my mac because unitree_legged_sdk doesn't compile on my mac, due to bad linking of libraries (see above, at the imports section)
    # Use motion_imitation/motion_imitation/robots/a1_robot.py as inspiration
    # self._robot_interface = RobotInterface()
    # self._robot_interface.send_command(np.zeros(60, dtype=np.float32))
    # state = self._robot_interface.receive_observation()
    # self._robot_interface.send_command(command)





if __name__ == "__main__":

    test()


