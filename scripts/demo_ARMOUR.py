import numpy as np
import os
from bullet.bulletRtdEnv import bulletRtdEnv
from bullet.bulletPlot import plotTraj
import pybullet as pbt

# remove previous data
dir_list = os.listdir("../data/ARMOUR/reachsets/")
for file in dir_list:
    os.remove("../data/ARMOUR/reachsets/"+file)
breakpoint()

# initialize environment
urdf_path="../assets"
useGUI=True
useGravity=True
useRobot=True
useTorqueControl=True
planner='armour'
record=True
blender=True

# key positions
qpos = np.array([[0.28159142, 0.6, 3.6350845 , 4.73481646, 2.55565072, 0.83405794, 2.05711487],
       [4.56258737, 0.4, 3.39192523, 4.20470761, 2.86698255, 0.41219441, 6.05075981]])
qpos = (qpos + np.pi) % (2 * np.pi) - np.pi # wrap to pi

# generate obstacles
obs_pos = np.array([[0.32, 0.1, 0.125]])
obs_size = np.array([[0.3, 0.3, 0.25]])
rack_pos = np.array([[-0.3, 0., 0.5], [0.8, 0, 1], [1, -0.7, 0], [1, 0.81, 0]])
rack_size = np.array([[0.01, 2, 2], [2, 2, 0.01], [2, 0.01, 2], [2, 0.01, 2]])

obs_pos_i = np.append([obs_pos[0]], rack_pos, axis=0)
obs_size_i = np.append([obs_size[0]], rack_size, axis=0)
btEnv = bulletRtdEnv(urdf_path=urdf_path, zonopyGUI=useGUI, useGravity=useGravity, \
    useRobot=useRobot, useTorqueControl=useTorqueControl, planner=planner, record=record, blender_record=blender,\
    q0=qpos[0], qgoal=qpos[1], obs_pos=obs_pos_i, obs_size=obs_size_i)

# simulate tracking from q0 to qgoal
btEnv.simulate(qpos[1])

# plotting
plotTraj(btEnv)

# save data
btEnv.dump_traj('../data/trajectories/ARMOUR_1208.npz')
btEnv.dump_video('../data/trajectories/ARMOUR_1208.pkl')

breakpoint()
btEnv.Disconnect()
