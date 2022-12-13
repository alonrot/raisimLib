from ruamel.yaml import YAML, dump, RoundTripDumper
import torch
from raisimGymTorch.env.bin import rsg_go1 # This import fails unless 'import torch' has been called previously, either in this script or in other imported modules
from raisimGymTorch.env.RaisimGymVecEnv import RaisimGymVecEnv as VecEnv
import raisimGymTorch.algo.ppo.module as ppo_module
import os
import math
import time
import argparse

# amarco:
import pdb
# python raisimGymTorch/env/envs/rsg_go1/tester.py --weight data/go1_locomotion/2022-11-11-08-31-33/full_800.pt
# python raisimGymTorch/env/envs/rsg_go1/tester.py --weight data/go1_locomotion/2022-11-13-21-49-05/full_500.pt #not good

# python raisimGymTorch/env/envs/rsg_go1/tester.py --weight data/go1_locomotion/2022-11-13-22-21-43/full_400.pt

# configuration
parser = argparse.ArgumentParser()
parser.add_argument('-w', '--weight', help='trained weight path', type=str, default='')
args = parser.parse_args()

# directories
task_path = os.path.dirname(os.path.realpath(__file__))
home_path = task_path + "/../../../../.."

# config
cfg = YAML().load(open(task_path + "/cfg.yaml", 'r'))

# create environment from the configuration file
cfg['environment']['num_envs'] = 1

env = VecEnv(rsg_go1.RaisimGymEnv(home_path + "/rsc", dump(cfg['environment'], Dumper=RoundTripDumper)), cfg['environment'])



# shortcuts
ob_dim = env.num_obs
act_dim = env.num_acts
weight_path = args.weight
iteration_number = weight_path.rsplit('/', 1)[1].split('_', 1)[1].rsplit('.', 1)[0]
weight_dir = weight_path.rsplit('/', 1)[0] + '/'

if weight_path == "":
    raise ValueError("Can't find trained weight, please provide a trained weight with --weight switch\n")


# integration_steps = int(cfg['environment']['control_dt'] / cfg['environment']['simulation_dt'] + 1e-10)
# print("integration_steps: {0:d}".format(integration_steps))

print("Loaded weight from {}\n".format(weight_path))
start = time.time()
env.reset()
reward_ll_sum = 0
done_sum = 0
average_dones = 0.
n_steps = math.floor(cfg['environment']['max_time'] / cfg['environment']['control_dt'])
total_steps = n_steps * 1
start_step_id = 0

# pdb.set_trace()
# weight_path = "./raisimGymTorch/env/envs/rsg_go1/isaac_gym/policy_1.pt"

print("Visualizing and evaluating the policy: ", weight_path)
loaded_graph = ppo_module.MLP(cfg['architecture']['policy_net'], torch.nn.LeakyReLU, ob_dim, act_dim)
loaded_graph.load_state_dict(torch.load(weight_path)['actor_architecture_state_dict'])

# Override:
# weight_dir =



env.load_scaling(weight_dir, int(iteration_number))
env.turn_on_visualization()



# using_single_integration_step = False # This is only True if we hardcode integration_steps_ = 1; in Environment.hpp

# if using_single_integration_step:
#     integration_steps = 1
# else:
#     integration_steps = int(cfg['environment']['control_dt'] / cfg['environment']['simulation_dt'] + 1e-10)
#     time_sleep = integration_steps * cfg['environment']['simulation_dt']


# integration_steps = env.wrapper.get_nr_of_times_integration_is_called_inside_step_for_the_same_applied_action()
time_sleep = cfg['environment']['control_dt']
# time_sleep = cfg['environment']['simulation_dt']

time2sleep_for_slow_visualization = 0.05
# time2sleep_for_slow_visualization = 0.0

max_steps = 1000000
# max_steps = 1000 ## 10 secs
# max_steps = 600
# dones = False
for step in range(max_steps):
    time.sleep(time2sleep_for_slow_visualization)
    obs = env.observe(False)
    print("obs",obs)
    action_ll = loaded_graph.architecture(torch.from_numpy(obs).cpu())

    # """
    # This is a proxy for the main functionalities of the step() function, in Environment.hpp
    # NOTE:   The code below is missing many of the functionalities of env.step(), which wraps the low-level step() by nesting it with 3 layers
    #         For example, dones is never computed here and reward_ll is not returned properly
    # """
    # env.wrapper.setPdTarget_from_commanded_action(action_ll.cpu().detach().numpy())
    # for ii in range(integration_steps):
    #     env.wrapper.step_integrate_once()
    #     time.sleep(time_sleep)    
    # env.wrapper.updateObservation()
    # reward_ll = env.wrapper.get_latest_rewards()


    # if using_single_integration_step:
    #     for ii in range(integration_steps):
    #         reward_ll, dones = env.step(action_ll.cpu().detach().numpy())
    #         time.sleep(time_sleep)
    # else:

    # for ii in range(5):
    #     reward_ll, dones = env.step(action_ll.cpu().detach().numpy())
    #     time.sleep(time_sleep)

    reward_ll, dones = env.step(action_ll.cpu().detach().numpy())
    time.sleep(time_sleep)
    

    reward_ll_sum = reward_ll_sum + reward_ll[0]
    if dones or step == max_steps - 1:
        print('----------------------------------------------------')
        print('{:<40} {:>6}'.format("average ll reward: ", '{:0.10f}'.format(reward_ll_sum / (step + 1 - start_step_id))))
        print('{:<40} {:>6}'.format("time elapsed [sec]: ", '{:6.4f}'.format((step + 1 - start_step_id) * 0.01)))
        print('----------------------------------------------------\n')
        start_step_id = step + 1
        reward_ll_sum = 0.0

env.turn_off_visualization()
env.reset()
print("Finished at the maximum visualization steps")
