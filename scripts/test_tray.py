from bullet.bulletRtdEnv import bulletRtdEnv
from pybullet_blender_recorder.pyBulletSimRecorder import PyBulletRecorder
import numpy as np
import pybullet as pbt
import matplotlib.pyplot as plt
import math as m
import time

timestep = 0.001
fetch_env = bulletRtdEnv(urdf_path="../assets/fetch/fetch_waiter.urdf", timestep=timestep, useGravity=True, useRobot=True)
fetch_env.load("../assets/objects/cube_small.urdf", [1.2479, 0, 0.12]) # center of the tray
joint_pos = np.genfromtxt('../data/ARMTD_Force_Debug_v4.csv', delimiter=',')
joint_acc = np.genfromtxt('../data/ARMTD_Force_Debug_v4_Accelerations.csv', delimiter=',')
joint_pos = joint_pos.T
joint_acc = joint_acc.T
joint_acc = np.concatenate((np.zeros((1,7)), joint_acc), 0)
# initialize robot joint position
fetch_env.forwardkinematics(joint_pos[0])

recorder = PyBulletRecorder()
for i in range(len(fetch_env.EnvId)):
    recorder.register_object(fetch_env.EnvId[i], fetch_env.path[i])

# let object fall onto the tray
for i in range(1000):
    torque = fetch_env.inversedynamics(joint_pos[0], np.zeros(7), np.zeros(7))
    fetch_env.torque_control(torque)
    pbt.stepSimulation()
    if i%20 == 0:
        recorder.add_keyframe()
    # time.sleep(timestep)

steps = np.size(joint_acc, 0) * 10
qpos = np.zeros((steps, 7))
qvel = np.zeros((steps, 7))
qpos_d = np.zeros((steps, 7))
qvel_d = np.zeros((steps, 7))
t = np.zeros(steps)
for i in range(steps):
    t[i] = i * timestep

    # calculate desired trajectory
    if i > 0:
        qpos_d[i], qvel_d[i] = fetch_env.get_joint_traj(qpos_d[i-1], qvel_d[i-1], joint_acc[i//10])
    else:
        qpos_d[i], qvel_d[i] = fetch_env.get_joint_traj(joint_pos[0], np.zeros(7), joint_acc[i//10])
    
    # inverse dynamics controller
    qpos[i], qvel[i] = fetch_env.get_joint_states()
    torque = fetch_env.inversedynamics(qpos_d[i], qvel_d[i], joint_acc[i//10])
    # print(f"joint torques: {torque}")
    fetch_env.torque_control(torque)
    pbt.stepSimulation()

    # record the video at 50 fps
    if i%20 == 0:
        recorder.add_keyframe()

    # wait some time to make the sim more realistic
    time.sleep(timestep)

# plot results
plt.subplot(211)
plt.plot(t, qpos_d, 'k--')
plt.plot(t, qpos)
plt.legend(['pos des'])
plt.subplot(212)
plt.plot(t, qvel_d, 'k--')
plt.plot(t, qvel)
plt.legend(['vel des'])
plt.show()

breakpoint()
recorder.save("../data/tray.pkl")
fetch_env.Disconnect()