# License - Steps
1. Free academic license: https://raisim.com/sections/License.html
2. They take 1/2 dyas to reply.
2. You can use the same key for multiple machines

# Installation steps
1. Create anaconda environment with Python 3.9:
```bash
conda create -n env_raisim python=3.9
```
1.5 Add the following packages:
```bash
pip install numpy
````

2. The instructions on the webpage (https://raisim.com/sections/Installation.html) didn't work out fo the box for me. Here are some troubleshooting:
 * Use the command `cmake .. -DCMAKE_INSTALL_PREFIX=$LOCAL_INSTALL -DRAISIM_EXAMPLE=ON -DRAISIM_PY=ON -DPYTHON_EXECUTABLE=$(python3 -c "import sys; print(sys.executable)")`
	but replace `$LOCAL_INSTALL` with the directory where you wanna dump the installation. I placed it inside the raisimLib folder, specifically:
	```bash
	cd <path/to/raisimLib>
	mkdir build_mac
	mkdir build_install_mac
	cd <path/to/raisimLib>/build_mac
	```
	* On Mac:
		`cmake .. -DCMAKE_INSTALL_PREFIX=/Users/alonrot/work/code_projects_WIP/raisimLib/build_install_mac -DRAISIM_EXAMPLE=ON -DRAISIM_PY=ON -DPYTHON_EXECUTABLE=$(python3 -c "import sys; print(sys.executable)")`
	* On Linux:
		`cmake .. -DCMAKE_INSTALL_PREFIX=/home/ubuntu/mounted_home/work/code_projects_WIP/raisimLib/build_install_ubuntu_vm -DRAISIM_EXAMPLE=ON -DRAISIM_PY=ON -DPYTHON_EXECUTABLE=$(python3 -c "import sys; print(sys.executable)")`
 * With version 1.1.5 `make install -j4` worked
 * With version 1.1.0 `make install -j4` didn't work on my mac and I had to modify 2 files:
		* `/Users/alonrot/work/code_projects_WIP/raisimLib/raisimPy/src/world.cpp` -> commented out lines 928-941
		* `/Users/alonrot/work/code_projects_WIP/raisimLib/examples/CMakeLists.txt` -> commented out line 77; basically, the target sensors.cpp was giving compilation problems
 * The global paths need to be entered in `~/.zshrc` using full paths. 
 	* On Mac:
	 	* The `DYLD_LIBRARY_PATH` line on the webpage is missing a `$`:
			* Use: `export DYLD_LIBRARY_PATH=$DYLD_LIBRARY_PATH:/Users/alonrot/work/code_projects_WIP/raisimLib/build_install/lib`
		* The `PYTHONPATH` line on the webpage assumes that the libraries are under `<...>/lib` but that wasn't my case with version 1.1.5. I had to do:
			* `export PYTHONPATH=$PYTHONPATH:/Users/alonrot/Desktop/tmp/raisimLib/build_install/lib/python3.9/site-packages`
	* On Ubuntu 22.05
		```
		export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/ubuntu/mounted_home/work/code_projects_WIP/raisimLib/build_install/lib
		export PYTHONPATH=$PYTHONPATH:/home/ubuntu/mounted_home/work/code_projects_WIP/raisimLib/build_install/lib/python3.9/site-packages
		```





# Usage
1. Run <path/to/raisimLib>/raisimUnity/<OS>/RaiSimUnity
2. Run the python code
```bash
cd /Users/alonrot/Desktop/tmp/raisimLib/raisimPy/examples
python robots.py
````

# Issues
* If the `RaiSimUnity` can't be opened due to permissions issues, allow it by using `chmod -R 777 RaiSimUnity.app`
* Is after clicking on `Connect` I get `Connection error` message, just ignore it and still run the python scripts


# NOTES
* The A1 URDF that RaiSim uses is exactly the same URDF used in `unitree_pybullet_modif`; I checked that with `diff`







