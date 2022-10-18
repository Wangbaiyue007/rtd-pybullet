from bullet.bulletRtdEnv import bulletRtdEnv
from recorder.pyBulletSimRecorder import PyBulletRecorder
import numpy as np
import pybullet as pbt
import matplotlib.pyplot as plt
import math as m
import time

timestep = 0.0001
fetch_env = bulletRtdEnv(urdf_path=".\\assets\\kinova_w_tray\\CC_Kinova_combined.urdf", timestep=timestep, useGravity=True, useRobot=True, control_gain=100000)
joint_pos = np.array([[m.pi/4, -m.pi/8,0, -m.pi/2-m.pi/8, 0, m.pi/4,0],
[m.pi/8, -m.pi/8,0, -m.pi/2-m.pi/8, 0, m.pi/4, m.pi/8],
[0, -m.pi/8, 0, -m.pi/2-m.pi/8, 0, m.pi/4, 0],
[-m.pi/8, -m.pi/8, 0, -m.pi/2-m.pi/8, 0, m.pi/4, -m.pi/8],
[-m.pi/4, -m.pi/8, 0, -m.pi/2-m.pi/8, 0, m.pi/4, 0]])

recorder = PyBulletRecorder()
for i in range(len(fetch_env.EnvId)):
    recorder.register_object(fetch_env.EnvId[i], fetch_env.path[i], fetch_env.scale[i])

# loop through robot joint position
for i in range(np.size(joint_pos, axis=0)):
    fetch_env.forwardkinematics(joint_pos[i])
    recorder.add_keyframe()
    breakpoint()

recorder.save("../data/pkl/intermittent.pkl")
fetch_env.Disconnect()