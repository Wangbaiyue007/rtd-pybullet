from bullet.bulletRtdEnv import bulletRtdEnv
from recorder.pyBulletSimRecorder import PyBulletRecorder
import numpy as np
import pybullet as pbt
import math as m

# set timestep for simulation, better be smaller than 0.001
timestep = 0.0001

# initialize environment
fetch_env = bulletRtdEnv(urdf_path="../assets/fetch/fetch_arm_new_dumbbell.urdf", timestep=timestep, useGravity=True)

# load a few objects to the environment
tableId, tablePath = fetch_env.load('../assets/table/table.urdf', [1, 0, 0], [0, 0, m.pi/2])
cubeId, cubePath = fetch_env.load('../assets/objects/cube_small.urdf', [1, 0.5, 2])

# simulation process
t = 0.0
t_total = 10

# loop through time
for i in range(int(t_total/timestep)):
    # inverse dynamics controller
    qacc = np.random.uniform(-0.5, 0.5, len(fetch_env.rp)) # use random acceleration
    qpos, qvel = fetch_env.get_joint_states() # get current position and velocity
    qpos_d, qvel_d = fetch_env.get_joint_traj(qpos, qvel, qacc) # get desired position and velocity
    torque = fetch_env.inversedynamics(qpos_d, qvel_d, qacc) # calculate inverse dynamics
    fetch_env.torque_control(torque) # command torque to robot

    # step simulation
    pbt.stepSimulation()

breakpoint()
fetch_env.Disconnect()