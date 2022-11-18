from sys import path_importer_cache
from bullet.bulletRtdEnv import bulletRtdEnv
from recorder.pyBulletSimRecorder import PyBulletRecorder
import numpy as np
import pybullet as pbt
import matplotlib.pyplot as plt
import math as m
import time
import os

# Joint data
joint_pos = np.genfromtxt('../data/ARMTD_matlab/joint_pos.csv', delimiter=',').T
joint_vel = np.genfromtxt('../data/ARMTD_matlab/joint_vel.csv', delimiter=',').T
joint_acc = np.genfromtxt('../data/ARMTD_matlab/joint_acc.csv', delimiter=',').T

# Obstacles data
obstacle_info = np.genfromtxt('../data/ARMTD_matlab/matlab_obstacles.csv', delimiter=',')
obstacle_pos = obstacle_info[::4]
obstacle_scale = np.zeros((int(np.size(obstacle_info, 0)/4), 3))
index = 0
for i in range(np.size(obstacle_info, 0)):
    if i % 4 == 1:
        obstacle_scale[index] = np.array([obstacle_info[i, 0], obstacle_info[i+1, 1], obstacle_info[i+2, 2]]) * 2
        index += 1

# Zonotope data
path_to_zono = '../assets/zonotope/meshes_matlab/'
files_zono = os.listdir(path_to_zono)

# Initialize simulation
num_step = 6
timestep = 0.001
bulletGUI = True
zonopyGUI = True
planner = 'zonopy'
blender_record = True
fetch_env = bulletRtdEnv(urdf_path="../assets", bulletGUI=bulletGUI, zonopyGUI=zonopyGUI, \
    planner=planner, blender_record=blender_record, q0=joint_pos[0], obs_pos=obstacle_pos, obs_size=obstacle_scale, \
    timestep=timestep, control_gain=10000)

# initialize renderer
recorder_zono = PyBulletRecorder()
recorder_zono_slc = PyBulletRecorder()

q_record = np.zeros([np.size(joint_pos, 0), 7])
q_des_record = np.zeros([np.size(joint_pos, 0), 7])
t_record = np.arange(0, 0.01*np.size(joint_pos, 0), 0.01)
for step in range(num_step):

    # create zonotope visual
    zonoIds_slc = []
    zonoIds = []
    for file in files_zono:
        if 'slc_step'+str(step+1)+'_' in file and '.obj' in file and '.urdf' not in file:
            zonoId_slc = fetch_env.create_visual(path_to_zono+file)
            recorder_zono_slc.register_object(zonoId_slc, path_to_zono+file+'.urdf')
            zonoIds_slc.append(zonoId_slc)
        if 'step'+str(step+1)+'_' in file and '.obj' in file and '.urdf' not in file and 'slc' not in file:
            zonoId = fetch_env.create_visual(path_to_zono+file)
            recorder_zono.register_object(zonoId, path_to_zono+file+'.urdf')
            zonoIds.append(zonoId)

    # control loop
    dataIndex = int(50 * step)
    fetch_env.step(joint_acc[dataIndex-1])
    
    # dump recording and reset
    recorder_zono.save("../data/pkl/matlab_zono_step"+str(step+1)+".pkl")
    recorder_zono_slc.save("../data/pkl/matlab_zono_step"+str(step+1)+"_slc.pkl")
    recorder_zono.reset()
    recorder_zono_slc.reset()

    # remove zonotope visual
    for Id in zonoIds:
        fetch_env.remove_body(Id)
        recorder_zono.unregister_object(Id)
    for Id in zonoIds_slc:
        fetch_env.remove_body(Id)
        recorder_zono_slc.unregister_object(Id)

plt.plot(t_record[:-1], fetch_env.qpos_des_record[:-1], 'r--', label='q_desired')
plt.plot(t_record[:-1], fetch_env.qpos_record[:-1], 'b-', label='q')
plt.xlabel('time/s')
plt.ylabel('position/rad')
plt.legend()
plt.show()

fetch_env.Disconnect()