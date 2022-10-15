from sys import path_importer_cache
from bullet.bulletRtdEnv import bulletRtdEnv
from recorder.pyBulletSimRecorder import PyBulletRecorder
import numpy as np
import pybullet as pbt
import matplotlib.pyplot as plt
import math as m
import time
import os
## check

# simulation timestep, should be 0.01 seconds
timestep = 0.0001
control_gain = 100000
# loading URDF
kinova_env = bulletRtdEnv(urdf_path="assets\kinova_w_tray\CC_Kinova_combined_wo_cube.urdf", control_gain=control_gain, timestep=timestep, useGravity=True, useRobot=True)
# loading data for failure
# joint_pos = np.genfromtxt('data\ARMTD_force\ARMTD_Force_w_Kinova_Failure_pos.csv', delimiter=',')
# joint_vel = np.genfromtxt('data\ARMTD_force\ARMTD_Force_w_Kinova_Failure_vel.csv', delimiter=',')
# joint_acc = np.genfromtxt('data\ARMTD_force\ARMTD_Force_w_Kinova_Failure_accel.csv', delimiter=',')
# loading data for success
joint_pos = np.genfromtxt('data\ARMTD_force\ARMTD_Force_w_Kinova_Success_pos.csv', delimiter=',')
joint_vel = np.genfromtxt('data\ARMTD_force\ARMTD_Force_w_Kinova_Success_vel.csv', delimiter=',')
joint_acc = np.genfromtxt('data\ARMTD_force\ARMTD_Force_w_Kinova_Success_accel.csv', delimiter=',')
# loading object to place on the tray
kinova_env.load("../assets/objects/cube_small.urdf", pos=[-1.127055022922495, -0.024827098925242, 0.32], ori=[0, 0, 0], scale=0.05) # center of the tray
#obstacle_pos = np.genfromtxt('../data/ARMTD_zonopy/zonopy_obstacles.csv', delimiter=',')
#path_to_zono = '../zonotope/meshes/'
#files_zono = os.listdir(path_to_zono)

print("Loaded Data \n")

# initialize obstacle positions and robot positions
#for i in range(np.size(obstacle_pos, 0)):
#    kinova_env.load("../assets/objects/cube_small_zero.urdf", obstacle_pos[i])
kinova_env.forwardkinematics(joint_pos[0]) # 1 if NaN

print(joint_pos[0])
print("Initialized Positions \n")

# initialize renderer
# recorder = PyBulletRecorder()
# recorder_zono = PyBulletRecorder()
# # register objects to recorder
# for i in range(len(kinova_env.EnvId)):
#     recorder.register_object(kinova_env.EnvId[i], kinova_env.path[i])

breakpoint()

# let object fall onto the tray
for i in range(20000):
    torque = kinova_env.inversedynamics(joint_pos[0], np.zeros(7), np.zeros(7))
    kinova_env.torque_control(torque)
    # kinova_env.position_control(joint_pos[0])
    pbt.stepSimulation()
    # if i%200 == 0:
    #     recorder.add_keyframe()
    # time.sleep(timestep)

breakpoint()

# letting object fall onto the tray
# for i in range(10000):
#     torque = kinova_env.inversedynamics(joint_pos[0], np.zeros(7), np.zeros(7))
#     kinova_env.torque_control(torque)
#     pbt.stepSimulation()
#     # if i%200 == 0:
#     #     recorder.add_keyframe()

# initializing trajectories
# breakpoint()
joint_pos = joint_pos[:] # 1: if NaN
joint_vel = joint_vel[:] # 1: if NaN
num_step = np.size(joint_pos,0) * 100
qpos = np.zeros((num_step,7))
qvel = np.zeros((num_step,7))
qpos_d = np.zeros((num_step,7))
qvel_d = np.zeros((num_step,7))
qpos_d[0] = joint_pos[0]
qvel_d[0] = joint_vel[0]
qacc_d = np.zeros(7)
t = np.zeros(num_step)

# control loop
print("stepping into control loop \n")
for i in range(num_step):

    t[i] = i * timestep

    # calculate desired trajectory
    if i%100 == 0:
        j = i // 100
        if j == np.size(joint_vel,0) - 1:
            break
        qacc_d = (joint_vel[j+1] - joint_vel[j])/0.01
    qpos_d[i+1], qvel_d[i+1] = kinova_env.get_joint_traj(qpos_d[i],qvel_d[i],qacc_d)

    # inverse dynamics controller
    qpos[i], qvel[i] = kinova_env.get_joint_states()
    torque = kinova_env.inversedynamics(qpos_d[i+1],qvel_d[i+1],qacc_d)
    kinova_env.torque_control(torque)
    pbt.stepSimulation()
    # print("stepped simulation \n")

    # record the video at 50 fps
    # if t%200 == 0: 
    #     recorder.add_keyframe()
    #     recorder_zono.add_keyframe()

    # dump recording and reset
    #recorder_zono.save("../data/pkl/zonopy_step"+str(step+1)+".pkl")
    #recorder_zono.reset()

    # remove zonotope visual
    #for Id in zonoIds:
    #    kinova_env.remove_body(Id)
    #    recorder_zono.unregister_object(Id)

print("printing \n")
# plot results
plt.subplot(211)
plt.plot(t,qpos)
plt.plot(t[::100],joint_pos,'k--')
plt.legend(['pos des'])
plt.subplot(212)
plt.plot(t,qvel)
plt.plot(t[::100],joint_vel,'k--')
plt.legend(['vel des'])
plt.show()

# recorder.save("../data/pkl/force_kinova.pkl")
kinova_env.Disconnect()