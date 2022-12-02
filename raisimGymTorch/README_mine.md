### amarco: Compilation/Running issues
Note: it was super hard to compile this, and haven't got it to run yet
1) `fatal error: 'omp.h' file not found` -> The solution was to modify the `CMakeLists.txt`, following these instructions [M1 Mac error compiling raisimGymTorch · Issue #221 · raisimTech/raisimLib · GitHub](https://github.com/raisimTech/raisimLib/issues/221)
2) Still not able to run `/Users/alonrot/work/code_projects_WIP/raisimLib/raisimGymTorch/raisimGymTorch/stable_baselines3/anymal.py` due to relative import errors.
3) Following this README.md, I executed `runner.py` but it doesn’t work due to `TypeError: Descriptors cannot not be created directly.`. I believe this is because I’m using Python 3.9
4) Couldn't run `./raisimGymTorch/stable_baselines3/anymal.py` (missing modules)
5) protobuf problem:
	```
	TypeError: Descriptors cannot not be created directly.
	If this call came from a _pb2.py file, your generated code is out of date and must be regenerated with protoc >= 3.19.0.
	If you cannot immediately regenerate your protos, some other possible workarounds are:
	 1. Downgrade the protobuf package to 3.20.x or lower.
	 2. Set PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python (but this will use pure-Python parsing and will be much slower).
	```
	Solved it with option 2, `export PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python`
	Didn't try option 1.


## raisim_env_anymal

### How to use this repo
There is nothing to compile here. This only provides a header file for an environment definition. Read the instruction of raisimGym. 

### Dependencies
- raisimgym




### Run
1. Compile raisimgym: ```python setup develop```
	[amarco NOTE:] for me, it worked better to run `pip install -e .`
	[amarco NOTE:] and if using the above, it should be `python setup.py develop`

2. run runner.py of the task (for anymal example): ```cd raisimGymTorch/env/envs/rsg_anymal && python ./runner.py```

### Test policy
1. Compile raisimgym: ```python setup develop```
2. run tester.py of the task with policy (for anymal example): ``` python raisimGymTorch/env/envs/rsg_anymal/tester.py --weight data/roughTerrain/FOLDER_NAME/full_XXX.pt```

### Retrain policy
1. run runner.py of the task with policy (for anymal example): ``` python raisimGymTorch/env/envs/rsg_anymal/runner.py --mode retrain --weight data/roughTerrain/FOLDER_NAME/full_XXX.pt```

### Debugging
1. Compile raisimgym with debug symbols: ```python setup develop --Debug```. This compiles <YOUR_APP_NAME>_debug_app
2. Run it with Valgrind. I strongly recommend using Clion for 