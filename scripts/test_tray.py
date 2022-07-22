from bullet.btKinematicsEnv import KinematicsEnv
from pybullet_blender_recorder.pyBulletSimRecorder import PyBulletRecorder
import numpy as np
import pybullet as pbt
import math as m

timestep = 0.002
fetch_env = KinematicsEnv(timestep=timestep, useGravity=True)
cubeId, cubePath = fetch_env.load('./assets/objects/cube_small.urdf', [1, 1, 2])
trayId, trayPath = fetch_env.load('./assets/objects/tray.urdf', [1, 1, 1])

recorder = PyBulletRecorder()
for i in range(len(fetch_env.EnvId)):
    recorder.register_object(fetch_env.EnvId[i], fetch_env.path[i])

t = 0.0
t_total = 100
action = np.random.uniform(-500, -300, len(fetch_env.rp))
print(f"action = {action}")
for i in range(int(t_total/timestep)):
    pbt.setJointMotorControlArray(fetch_env.robotId, fetch_env.actuation_index, pbt.TORQUE_CONTROL, forces=action)
    pbt.stepSimulation()
    # if i%10 == 0:
    #     recorder.add_keyframe()
    if i%100 == 0:
        breakpoint()

breakpoint()
recorder.save("data/tray.pkl")
fetch_env.Disconnect()