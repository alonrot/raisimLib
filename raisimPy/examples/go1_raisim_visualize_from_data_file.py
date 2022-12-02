import os
import numpy as np
import raisimpy as raisim
import time
import pdb
from unitree_legged_sdk_python_tools.utils.data_parsing import read_cvs_file


def main():

    # TODO: can we move this to the import section?
    raisim.World.setLicenseFile(os.path.dirname(os.path.abspath(__file__)) + "/../../rsc/activation.raisim")

    path2data = "/Users/alonrot/work/code_projects_WIP/unitree_legged_sdk_from_inside_robot/examples"

    folder_name = "2022_11_22_16_43_76"; # from laptop
    # folder_name = "2022_11_22_16_46_34"; # inside the robot

    data, file_names, joints_names = read_cvs_file(path2data,folder_name)

    time_stamp = data[0,:,0]
    joint_pos_curr = data[0,:,1::]
    joint_pos_des = data[1,:,1::]


    """
    TODO:

    Overlay both, the current and the desired joint positions
    """

    # go1_urdf_file = "/home/ubuntu/mounted_home/work/code_projects_WIP/unitree_mujoco/data/go1/urdf/go1.urdf"
    go1_urdf_file = "/Users/alonrot/work/code_projects_WIP/unitree_mujoco/data/go1/urdf/go1.urdf"

    world = raisim.World()
    world.setTimeStep(0.001)
    server = raisim.RaisimServer(world)
    ground = world.addGround()

    go1_nominal_joint_config = np.array([0.0, -1.5245, 0.3435, 1.0, 0.0, 0.0, 1.0, # (com [Y(green), X(red), Z(blue)], quaternion [1,i,j,k]],...
                                        0.0136, 0.7304, -1.4505,  # FR [hip lateral, hip, knee]
                                        -0.0118, 0.7317, -1.4437, # FL [hip lateral, hip, knee]
                                        0.0105, 0.6590, -1.3903, # RR [hip lateral, hip, knee]
                                        -0.0102, 0.6563, -1.3944]) # RL [hip lateral, hip, knee]
    go1_nominal_joint_config[2] += 0.3 # Robot hanging

    # # Visual object (without physics):
    go1_visual_curr = server.addVisualArticulatedSystem("go1_curr",go1_urdf_file) # Add a visual capsule without physics. See src/world.cpp
    go1_visual_curr.setGeneralizedCoordinate(go1_nominal_joint_config)
    # go1_visual_curr.setColor(0.0,0.9,0.0,0.2) # [R,G,B,transparency]


    go1_visual_des = server.addVisualArticulatedSystem("go1_des",go1_urdf_file) # Add a visual capsule without physics. See src/world.cpp
    go1_visual_des.setGeneralizedCoordinate(go1_nominal_joint_config)
    go1_visual_des.setColor(0.0,0.0,0.9,0.3) # [R,G,B,transparency]


    print("Launching Raisim server...")
    server.launchServer(8080)

    time.sleep(2.0)

    Nsteps = len(time_stamp)
    pos_curr_joint = go1_nominal_joint_config.copy()
    pos_des_joint = go1_nominal_joint_config.copy()
    time_sleep_vec = np.diff(time_stamp,prepend=0.0)
    # pdb.set_trace()
    cc = 0
    while True:


        # Update current position with readings:
        pos_curr_joint[7::] = joint_pos_curr[cc,:]

        # Update desired position:
        pos_des_joint[7::] = joint_pos_des[cc,:]

        # Set current position:
        go1_visual_curr.setGeneralizedCoordinate(pos_curr_joint)
        go1_visual_des.setGeneralizedCoordinate(pos_des_joint)

        time.sleep(time_sleep_vec[cc])

        cc = ( cc + 1 ) % Nsteps


    server.killServer()


if __name__ == "__main__":

    main()


