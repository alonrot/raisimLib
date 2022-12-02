from ruamel.yaml import YAML, dump, RoundTripDumper
from raisimGymTorch.env.RaisimGymVecEnv import RaisimGymVecEnv as VecEnv
import torch
from raisimGymTorch.env.bin import rsg_go1 # This import fails unless 'import torch' has been called previously, either in this script or in other imported modules
import os
import math
import time
import torch
import argparse
import pdb
import numpy as np


def test():

	# configuration
	parser = argparse.ArgumentParser()
	parser.add_argument('-w', '--weight', help='trained weight path', type=str, default='')
	args = parser.parse_args()

	# directories
	task_path = os.path.dirname(os.path.realpath(__file__))

	# config
	cfg = YAML().load(open(task_path + "/cfg.yaml", 'r'))

	# create environment from the configuration file
	cfg['environment']['num_envs'] = 1

	env = VecEnv(rsg_go1.RaisimGymEnv("dummy", dump(cfg['environment'], Dumper=RoundTripDumper)), cfg['environment'])
	# env = VecEnv(rsg_go1.RaisimGymEnv("dummy", dump(cfg['environment'], Dumper=RoundTripDumper)), cfg['environment'])
	# env = VecEnv(rsg_anymal.RaisimGymEnv("dummy", dump(cfg['environment'], Dumper=RoundTripDumper)), cfg['environment'])
	env.reset()

	# pdb.set_trace()

	# env.load_scaling(weight_dir, int(iteration_number))
	env.turn_on_visualization()
	action = torch.zeros([1, env.num_acts],dtype=torch.float32) # go1
	# amarco: NOTE: env.num_acts is number of actuators. In case of g1 it equals the number of joints, i.e., 12

	joints_pos_init_env = env.observe(update_statistics=False)
	joints_pos_resting = joints_pos_init_env[0,4:16] # [12,]

	assert cfg['environment']['control_dt'] >= cfg['environment']['simulation_dt']

	"""
	SOLVED QUESTIONS:
	1) What are the actions in this environment?
		* We can see how the actions interplay with the controller and joint positions in the step() function implemented in raisimGymTorch/env/envs/rsg_go1/Environment.hpp
		* At the moment, the desired joint positions are computed as q_des = action_mean + action_std*action, where 
			action_mean = q_init (initial generalized coordinates (hardcoded in Environment.hpp))
			action_std = 1.0 (hardcoded in Environment.hpp)
		* Right now we're doing position control using q_des as a target
		* So, action is a delta position variable
		* When 'action' is zero, the position controller keeps the robot in the same position
	2) Where is the wrapper rsg_go1 defined?
		* It is defined at raisimGymTorch/env/raisim_gym.cpp
		* The wrapper gets constructed by calling rsg_go1.RaisimGymEnv(...)
	3) Does env.step() invoke the integration step of the contact solver?
		* Yes; the step() function we're using is implemented in raisimGymTorch/env/envs/rsg_go1/Environment.hpp; therein, world_->integrate(); is called
	4) How and when is rsg_go1 generated?
		* We need to compile the entire package raisimGymTorch by doing:
			cd /Users/alonrot/work/code_projects_WIP/raisimLib/raisimGymTorch
			pip install -e .
			Then, the targets are generated
	5) How does the program know where to find the meshes? We're only passing the URDF file at training time (runner.py)
		* The path to the meshes is defined inside the urdf file as a relative import
	6) What's the difference between simulation_dt: 0.0025 and control_dt: 0.01 in the cfg file?
		* Every simulation_dt seconds, the simulator calls world_->integrate() inside the step() function, defined in raisimGymTorch/env/envs/rsg_go1/Environment.hpp
		* Every control_dt seconds, we change the desired target joint position, hence changing the control input itself
		* In step(), inside raisimGymTorch/env/envs/rsg_go1/Environment.hpp, the function world_->integrate() is called int(control_dt_ / simulation_dt_ + 1e-10) times.
		* So, in a nutshell, the duration of each time step is dictated by simulation_dt. Then, if we want the control input to change every 20 steps, 
		  we need to set control_dt=20*simulation_dt_
		* It is implicitly assumed that control_dt>=simulation_dt_
	7) How do we select the time.sleep(XXX) input? Do we actually need to? Why do we need to call world_->setTimeStep(dt) then?
		*   Calling world_->setTimeStep(dt) is necessary to update the time step in the numerical solution to the differential equations. We simply need it.
			In order for the visualization to be as close as possible to reality, we need to wait a wall clock time amount equal to dt. So then,
			we should we calling time.sleep(dt-loop_processing_time)
	8) We compile modifications to raisimGymTorch/env/envs/rsg_go1/Environment.hpp using the alias comprai, defined in ~/.zshrc
	9) Does anymal start on the floor?
		* Yes, its initial height is pretty much adjusted so that it starts on the floor
	10) Can't run ./runner.py or ./test_raisim_gym_go1.py because `from raisimGymTorch.env.bin import rsg_go1` fails:  Symbol not found: ___kmpc_dispatch_init_4
		* Solved by fixing the imports
	11) At the moment, we apply a new action every control_dt/simulation_dt steps. We only visualize every time a new action is commanded, but we don't see ant of the steps that
		occur inbetween actions. I tried to make that possible by mofifying the step() funciton in Environment.hpp and exposing to Python the elemental functions
		that make it possible. However, it's quite complicated, mainly because in order to expose them in python, they need to also by declared/implemented in 
		VectorizedEnvironment.hpp and RaisimGymEnv.hpp, or create a whole new python package using pybind11
	12) num_threads in the config file isn't used anywhere in the code; why? What is it for?
		* It's used in raisimGymTorch/env/VectorizedEnvironment.hpp to paralelize the training using OMP
	13) How are the 'done/dones' calculated? What is the terminal state and where is it defined?
		* It's defined in Environment.hpp at isTerminalState(), then wrapped in VectorizedEnvironment.hpp, which is where the dones are being calculated
	14) Try to get the id's of every joint using getFrameIdxByName() from raisim/mac/include/raisim/object/ArticulatedSystem/ArticulatedSystem.hpp
		* I tried using `size_t id1 = go1->getFrameIdxByName("FR_hip_joint");` and `std::cout << "id1: " + std::to_string(id1) + "\n";`
		* However, the number of names in the URDF is much much larger than the number of available joints, and it's hard to infer the name of
			the joint that corresponds to each index of the vector of desired joint positions
	15) How can we expose the std that multiplies the actions to be a cfg parameter?
		* I think the cfg file is NOT accessible from Environment.hpp


	TODO:
	4) Have a way to handle nan torque during training
	7) Read the PPO paper and understand what's going on; also understand the google collab plots
	8) Have an easy, systematic way of visualizing the joint space trajectories that is also compatible with the real robot. What tools are avaialble out there?
		8.1) See if we can use google colab to plot stuff; probably...
		8.2) What about raisim itself?
	"""


	omega_ramp = 1.5
	ampl = 0.20
	fac = 7
	id_thighs = [8-fac,11-fac,14-fac,17-fac]
	id_knees = [9-fac,12-fac,15-fac,18-fac]
	def joints_target_pos_ramp(tt,t_init,joints_pos_resting):
		joints_target_curr = np.copy(joints_pos_resting)
		joints_target_curr[id_thighs] += ampl*np.sin(omega_ramp*(tt-t_init))
		joints_target_curr[id_knees] -= ampl*np.sin(omega_ramp*(tt-t_init))
		return joints_target_curr

	t_hanging = 1.0
	print("Hanging for {0:f} seconds ...".format(t_hanging))
	time.sleep(t_hanging)

	time_ela = 0.0
	Nsteps = 10000 ## 10 secs
	action_np = np.zeros((1,env.num_acts),dtype=np.float32)
	time_ela_perf_vec = np.zeros(Nsteps)
	time_sleep = int(cfg['environment']['control_dt'] / cfg['environment']['simulation_dt'] + 1e-10) * cfg['environment']['simulation_dt']
	t_init = 2.0 # Seconds
	time_start = time.time_ns()
	print("Starting movement...")
	for tt in range(Nsteps):

		time_start_perf = time.time_ns() 

		if time_ela >= t_init:
			joints_target_curr = joints_target_pos_ramp(time_ela,t_init,joints_pos_resting)
			action_np[0,:] = joints_target_curr - joints_pos_resting

			# time_sleep = 0.5

		obs = env.observe(update_statistics=False) # [ body height [1], body orientation [3], joint angles [12], bodyLinearVel_ [3], bodyAngularVel_ [3], joint velocity [12] ]
		reward_ll, dones = env.step(action_np) # amarco
		# reward_ll, dones = env.step(action.cpu().detach().numpy()) # original

		# This variable measures the computational time of all the heavy computation of the loop; everything that happens before calling time.sleep()
		# To have a more realistic "real time" visualization, we subtract it from the sleeping time
		time_ela_perf_vec[tt] = (time.time_ns() - time_start_perf) / 1000000000. # Seconds
		assert time_ela_perf_vec[tt] <= time_sleep
		time.sleep(time_sleep - time_ela_perf_vec[tt]) 	# raisim::MSLEEP() # If in c++ we would use raisim::MSLEEP(), defined in /Users/alonrot/work/code_projects_WIP/raisimLib/raisim/mac/include/raisim/helper.hpp
														# MSLEEP receives the time in miliseconds

		# This variable is a global time counter that we use to compute the sinusoidal reference in joints_target_pos_ramp()
		# We don't strictly need it. We could make the sines inside joints_target_pos_ramp() simply depend on the current time step
		time_ela = (time.time_ns() - time_start) / 1000000000. # Seconds



	print("time_ela_perf: {0:1.10f} ({1:1.10f})".format(np.mean(time_ela_perf_vec),np.std(time_ela_perf_vec)))

	env.turn_off_visualization()
	env.reset()
	print("Finished at the maximum visualization steps")


if __name__ == "__main__":

	test()


