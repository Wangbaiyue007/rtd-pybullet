from bullet.bulletRtdEnv import bulletRtdEnv
from pybullet_blender_recorder.pyBulletSimRecorder import PyBulletRecorder
import numpy as np
import pybullet as pbt
import math as m
import csv
import os

NumOfZono = 5
path = '../zonotope/'
# vertices = np.genfromtxt(path+'/data/vertices.csv', delimiter=',')
# vertices = np.split(vertices, NumOfZono)

timestep = 0.002
fetch_env = bulletRtdEnv(timestep=timestep, useRobot=False)

mesh_list = os.listdir(path+'meshes')
for mesh in mesh_list:
    if '.obj' in mesh and 'zonomesh' in mesh:
        fetch_env.create_visual('meshes/'+mesh)

# for i in range(NumOfZono):
#     zonotope = vertices[i].reshape(8, 3)
#     fetch_env.plot_zonotope(zonotope)

recorder = PyBulletRecorder()
for i in range(len(fetch_env.EnvId)):
    recorder.register_object(fetch_env.EnvId[i], fetch_env.path[i])

breakpoint()

t = 0.0
t_total = 100
action = np.random.uniform(-500, -300, len(fetch_env.rp))
print(f"action = {action}")
for i in range(int(t_total/timestep)):
    pbt.setJointMotorControlArray(fetch_env.robotId, fetch_env.actuation_index, pbt.TORQUE_CONTROL, forces=action)
    pbt.stepSimulation()
    # if i%10 == 0:
    #     recorder.add_keyframe()

breakpoint()
# recorder.save("data/zono.pkl")
fetch_env.Disconnect()