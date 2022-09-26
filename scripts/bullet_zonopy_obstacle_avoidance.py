from sys import path_importer_cache
from bullet.bulletRtdEnv import bulletRtdEnv
from recorder.pyBulletSimRecorder import PyBulletRecorder
import numpy as np
import pybullet as pbt
import matplotlib.pyplot as plt
import math as m
import time
import os

num_step = 30
timestep = 0.001
kinova_env = bulletRtdEnv(urdf_path="../assets/kinova_gen3_7dof/kinova_with_robotiq_85.urdf", timestep=timestep, useGravity=True, useRobot=True)
joint_pos = np.genfromtxt('../data/ARMTD_zonopy/zonopy_qpos.csv', delimiter=',')
joint_acc = np.genfromtxt('../data/ARMTD_zonopy/zonopy_qacc.csv', delimiter=',')
obstacle_pos = np.genfromtxt('../data/ARMTD_zonopy/zonopy_obstacles.csv', delimiter=',')
path_to_zono = '../zonotope/meshes/'
files_zono = os.listdir(path_to_zono)

# initialize obstacle positions and robot positions
for i in range(np.size(obstacle_pos, 0)):
    kinova_env.load("../assets/objects/cube_small_zero.urdf", obstacle_pos[i])
kinova_env.forwardkinematics(joint_pos[0])

# initialize renderer
recorder = PyBulletRecorder()
recorder_zono = PyBulletRecorder()
# register objects to recorder
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
            recorder_zono.register_object(zonoId, path_to_zono+file+'.urdf')
            zonoIds.append(zonoId)

    # control loop
    for t in range(int(0.5/timestep)):
        qpos, qvel = kinova_env.get_joint_traj(qpos, qvel, joint_acc[step])
        torque = kinova_env.inversedynamics(qpos, qvel, joint_acc[step])
        kinova_env.torque_control(torque)
        pbt.stepSimulation()
        time.sleep(timestep)
        # record the video at 50 fps
        if t%20 == 0: 
            recorder.add_keyframe()
            recorder_zono.add_keyframe()

    # dump recording and reset
    recorder_zono.save("../data/pkl/zonopy_step"+str(step+1)+".pkl")
    recorder_zono.reset()

    # remove zonotope visual
    for Id in zonoIds:
        kinova_env.remove_body(Id)
        recorder_zono.unregister_object(Id)

recorder.save("../data/pkl/zonopy_kinova.pkl")
kinova_env.Disconnect()