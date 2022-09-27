from bullet.bulletRtdEnv import bulletRtdEnv
from recorder.pyBulletSimRecorder import PyBulletRecorder
import numpy as np
import pybullet as pbt
import matplotlib.pyplot as plt
import math as m
import time

timestep = 0.0001
fetch_env = bulletRtdEnv(urdf_path="../assets/fetch/fetch_waiter.urdf", timestep=timestep, useGravity=True, useRobot=True, control_gain=100000)
fetch_env.load("../assets/objects/cube_small.urdf", pos=[0.91501, -0.88236, 1.0], ori=[0, 0, -m.pi/4], scale=0.1) # center of the tray
fetch_env.load("plane.urdf", pos=[0, 0, 0], saveid=False)
data = np.genfromtxt('../data/ARMTD_force/ARMTD_Force_Amazon_Demo_Success.csv', delimiter=',')
joint_pos = data[:, :7]
joint_vel = data[:, 7:]
# initialize robot joint position
fetch_env.forwardkinematics(joint_pos[0])

recorder = PyBulletRecorder()
for i in range(len(fetch_env.EnvId)):
    recorder.register_object(fetch_env.EnvId[i], fetch_env.path[i], fetch_env.scale[i])

# let object fall onto the tray
for i in range(10000):
    torque = fetch_env.inversedynamics(joint_pos[0], np.zeros(7), np.zeros(7))
    fetch_env.torque_control(torque)
    pbt.stepSimulation()
    if i%200 == 0:
        recorder.add_keyframe()
    # time.sleep(timestep)

steps = np.size(joint_vel, 0) * 100
qpos = np.zeros((steps, 7))
qvel = np.zeros((steps, 7))
qpos_d = np.zeros((steps, 7))
qvel_d = np.zeros((steps, 7))
qpos_d[0] = joint_pos[0]
qvel_d[0] = joint_vel[0]
qacc_d = np.zeros(7)
t = np.zeros(steps)
for i in range(steps):
    t[i] = i * timestep

    # calculate desired trajectory
    if i % 100 == 0:
        j = i // 100
        if j == np.size(joint_vel, 0) - 1:
            break
        qacc_d = (joint_vel[j+1] - joint_vel[j])/0.01
    qpos_d[i+1], qvel_d[i+1] = fetch_env.get_joint_traj(qpos_d[i], qvel_d[i], qacc_d)

    # inverse dynamics controller
    qpos[i], qvel[i] = fetch_env.get_joint_states()
    torque = fetch_env.inversedynamics(qpos_d[i+1], qvel_d[i+1], qacc_d)
    # print(f"joint torques: {torque}")
    fetch_env.torque_control(torque)
    pbt.stepSimulation()

    # record the video at 50 fps
    if i%200 == 0:
        recorder.add_keyframe()

    # wait some time to make the sim more realistic
    # time.sleep(timestep)


# plot results
plt.subplot(211)
# plt.plot(t, qpos_d, 'b--')
plt.plot(t, qpos)
plt.plot(t[::100], joint_pos, 'k--')
plt.legend(['pos des'])
plt.subplot(212)
# plt.plot(t, qvel_d, 'b--')
plt.plot(t, qvel)
plt.plot(t[::100], joint_vel, 'k--')
plt.legend(['vel des'])
plt.show()

recorder.save("../data/pkl/tray.pkl")
fetch_env.Disconnect()