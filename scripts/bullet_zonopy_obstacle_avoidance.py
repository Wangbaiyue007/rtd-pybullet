from sys import path_importer_cache
from bullet.bulletRtdEnv import bulletRtdEnv
from pybullet_blender_recorder.pyBulletSimRecorder import PyBulletRecorder
import numpy as np
import pybullet as pbt
import matplotlib.pyplot as plt
import math as m
import time
import os

num_step = 30
timestep = 0.001
kinova_env = bulletRtdEnv(urdf_path="../assets/kinova_gen3_7dof/kinova_with_robotiq_85.urdf", timestep=timestep, useGravity=True, useRobot=True)
joint_pos = np.genfromtxt('../zonotope/data/zonopy_qpos.csv', delimiter=',')
joint_acc = np.genfromtxt('../zonotope/data/zonopy_qacc.csv', delimiter=',')
obstacle_pos = np.genfromtxt('../zonotope/data/zonopy_obstacles.csv', delimiter=',')
path_to_zono = '../zonotope/meshes/'
files_zono = os.listdir(path_to_zono)

# initialize obstacle positions and robot positions
for i in range(np.size(obstacle_pos, 0)):
    kinova_env.load("../assets/objects/cube_small_zero.urdf", obstacle_pos[i])
kinova_env.forwardkinematics(joint_pos[0])

# initialize renderer
recorder = PyBulletRecorder()
for i in range(len(kinova_env.EnvId)):
    recorder.register_object(kinova_env.EnvId[i], kinova_env.path[i])

qpos = joint_pos[0]
qvel = np.zeros(7)
for step in range(num_step):
    # create zonotope visual
    zonoIds = []
    for file in files_zono:
        if 'step'+str(step+1)+'_' in file and '.obj' in file and '.urdf' not in file:
            zonoId = kinova_env.create_visual(path_to_zono+file)
            recorder.register_object(zonoId, path_to_zono+file+'.urdf')
            zonoIds.append(zonoId)

    # control loop
    for t in range(int(0.5/timestep)):
        qpos, qvel = kinova_env.get_joint_traj(qpos, qvel, joint_acc[step])
        torque = kinova_env.inversedynamics(qpos, qvel, joint_acc[step])
        kinova_env.torque_control(torque)
        pbt.stepSimulation()
        time.sleep(timestep)

    # remove zonotope visual
    for Id in zonoIds:
        kinova_env.remove_body(Id)
        recorder.unregister_object(zonoId)

breakpoint()
recorder.save("../data/zonopy.pkl")
kinova_env.Disconnect()