from sys import path_importer_cache
from bullet.bulletRtdEnv import bulletRtdEnv
from pybullet_blender_recorder.pyBulletSimRecorder import PyBulletRecorder
import numpy as np
import pybullet as pbt
import matplotlib.pyplot as plt
import math as m
import time
import os

num_step = 26
timestep = 0.001
fetch_env = bulletRtdEnv(urdf_path="../assets/fetch/fetch_arm_new_dumbbell.urdf", timestep=timestep, useGravity=True, useRobot=True)

# Joint data
joint_pos = np.genfromtxt('../data/ARMTD_matlab/joint_pos.csv', delimiter=',').T
joint_vel = np.genfromtxt('../data/ARMTD_matlab/joint_vel.csv', delimiter=',').T

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
path_to_zono = '../zonotope/meshes_matlab/'
files_zono = os.listdir(path_to_zono)

# initialize obstacle positions and robot positions
for i in range(np.size(obstacle_pos, 0)):
    fetch_env.create_visual("../assets/objects/cube.obj", scale=obstacle_scale[i], pos=obstacle_pos[i])
fetch_env.forwardkinematics(joint_pos[0])

# initialize renderer
recorder = PyBulletRecorder()
recorder_zono = PyBulletRecorder()
# register objects to recorder
for i in range(len(fetch_env.EnvId)):
    recorder.register_object(fetch_env.EnvId[i], fetch_env.path[i])
    break

qpos = joint_pos[0]
qvel = np.zeros(7)
for step in range(num_step):

    # create zonotope visual
    zonoIds = []
    for file in files_zono:
        if 'slc_step'+str(step+1)+'_' in file and '.obj' in file and '.urdf' not in file:
            zonoId = fetch_env.create_visual(path_to_zono+file)
            recorder_zono.register_object(zonoId, path_to_zono+file+'.urdf')
            zonoIds.append(zonoId)

    # control loop
    dataIndex = int(50 * step)
    for t in range(int(0.5/timestep)):
        if t % 10 == 0: 
            dataIndex += 1
            qacc_d = (joint_vel[dataIndex]-joint_vel[dataIndex-1])/0.01
        qpos, qvel = fetch_env.get_joint_traj(qpos, qvel, qacc_d)
        torque = fetch_env.inversedynamics(qpos, qvel, qacc_d)
        fetch_env.torque_control(torque)
        pbt.stepSimulation()
        time.sleep(timestep)
        # record the video at 50 fps
        if t%20 == 0: 
            recorder.add_keyframe()
            recorder_zono.add_keyframe()
    breakpoint()
    # dump recording and reset
    recorder_zono.save("../data/pkl/matlab_step"+str(step+1)+".pkl")
    recorder_zono.reset()

    # remove zonotope visual
    for Id in zonoIds:
        fetch_env.remove_body(Id)
        recorder_zono.unregister_object(Id)

recorder.save("../data/pkl/matlab_fetch.pkl")
fetch_env.Disconnect()