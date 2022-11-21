# RTD PyBullet
Running RTD ([Autonomous Reachability-based Manipulator Trajectory Design](https://arxiv.org/abs/2002.01591)) in various platforms and simulate in pybullet, and create rendered animation.

<table>
  <tr>
    <td><img src="data/gif/kinova_obstacle_avoidance.gif?raw=true" width="400"></td>
    <td><img src="data/gif/fetch_force_closure.gif?raw=true" width="400"></td>
  </tr>
</table>

## Installation
It would be a good idea to install the python code in a virtual environment using using either [`conda`](https://docs.conda.io/en/latest/) or [`venv`](https://docs.python.org/3/library/venv.html).

This installation assumes that you have correctly compiled [`armour`](https://github.com/roahmlab/armtd-dev)'s *dependencies*, which includes [`ipopt`](https://coin-or.github.io/Ipopt/).

### External Dependencies
[`rtd-pybullet`](https://github.com/roahmlab/rtd-pybullet) has a number of dependencies.  To generate nice visuals, you'll need Blender:

- [`blender`](https://www.blender.org/)
- [`pybullet-blender-recorder`](https://github.com/huy-ha/pybullet-blender-recorder) 

There are also a number of python dependencies that can be installed by 

     pip install -r requirements.txt

### Included as submodules
- [`zonopy`](https://github.com/roahmlab/zonopy)
- [`armour`](https://github.com/roahmlab/armtd-dev/tree/1719161629de9820625ad52bc8e42b7a01a6543d)

### Install the Blender plugin on Ubuntu
Make sure Blender is installed first. Then install the blender plugin:
    
    sudo snap install blender --classic

Look at the Blender documentation for installing plugins on Windows or MacOS.

### Install `rtd-pybullet`
    git clone https://github.com/roahmlab/rtd-pybullet
    cd rtd-pybullet
    git submodule update --init --recursive
    pip install -e .

### Install `zonopy`
    cd rtd-pybullet/zonopy
    pip install -e .

### Build `ARMOUR`
    cd armtd-dev/cuda-dev/PZsparse-Bernstein/build
    cmake ..
    make
We need to add the path to `PYTHONPATH` so that python can find the pybind module created by CMake.

    export PYTHONPATH=$PYTHONPATH:$PWD

## Examples
### ARMOUR examples
Go to `scripts/` and run `test_bullet_planner.py` to see what happens.
### Zonopy examples
Go to `scripts/` and run `test_zonopy.py` to see what happens.

## Rendering trajectory and zonotopes (this needs update)
### I. Running zonopy simulation and record data
Zonopy environment will generate an obstacle avoidance task, where a Kinova arm starts from a random position moves towards a random goal positions, with random obstacles around. Run `scripts/test_zonopy.py` to see what happens.

### II. Save the reachable sets as mesh files
In order to visualize the reachable sets in Pybullet and in Blender, it has to be saved as mesh files so that they can be imported. Use MATLAB to run `zonotope/FO2stl_zonopy.m` will save those reachable sets as convex set in the format `.stl`. Don't forget to change folder.

### III. Convert mesh files to urdf
Since the *pybullet_blender_recorder* works the best with `.urdf` format, we can convert those files to `.urdf` first. This process is done using [pymeshlab](https://pymeshlab.readthedocs.io/en/latest/) and [object2urdf](https://github.com/harvard-microrobotics/object2urdf). These packages make things easy. Running `utils/stl2obj2urdf.py` will do the job.

### IV. Running the pre-computed trajectories in Pybullet
The pre-computed trajectories are discrete time and are assumed to be perfect tracking. We can track the trajectories using Pybullet with much smaller time step and real physics. The Pybullet environment is set up in [here](https://github.com/Wangbaiyue007/rtd-pybullet/blob/master/bullet/bulletRtdEnv.py). Run `scripts/bullet_zonopy_obstacle_avoidance.py` to see the Pybullet simulation. In the mean time, the motions of the arm and the reachable sets are saved as `.pkl` files.

### V. Render animation using Blender
With all the `.pkl` files, it is easy to do the rendering by importing them to Blender. Check out the instructions [here](https://github.com/huy-ha/pybullet-blender-recorder).

# TODO
- [ ] Improve the installation instructions
- [ ] Remove ARMOUR and zonopy as submodules
