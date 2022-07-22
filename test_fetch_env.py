from btKinematicsEnv import KinematicsEnv
from pybullet_blender_recorder.pyBulletSimRecorder import PyBulletRecorder
import numpy as np
import pybullet as p
import math as m

timestep = 0.002
fetch_env = KinematicsEnv(timestep=timestep, useGravity=True)
legoId, legoPath = fetch_env.load('./assets/objects/lego.urdf', [1, 0, 2])
tableId, tablePath = fetch_env.load('./assets/table/table.urdf', [1, 0, 0], [0, 0, m.pi/2])
cubeId, cubePath = fetch_env.load('./assets/objects/cube_small.urdf', [1, 0.5, 2])

recorder = PyBulletRecorder()
for i in range(len(fetch_env.EnvId)):
    recorder.register_object(fetch_env.EnvId[i], fetch_env.path[i])

t = 0.0
t_total = 10
action = np.random.uniform(-500, -300, len(fetch_env.rp))
print(f"action = {action}")
for i in range(int(t_total/timestep)):
    p.setJointMotorControlArray(fetch_env.robotId, fetch_env.actuation_index, p.TORQUE_CONTROL, forces=action)
    p.stepSimulation()
    if i%10 == 0:
        recorder.add_keyframe()

breakpoint()
recorder.save("data/demo_with_obstacle.pkl")
fetch_env.Disconnect()