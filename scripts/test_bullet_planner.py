import numpy as np
from bullet.bulletRtdEnv import bulletRtdEnv

# initialize environment
urdf_path="../assets/kinova_gen3_7dof/kinova_with_robotiq_85.urdf"
useGUI=True
useGravity=True
useRobot=True
useTorqueControl=True
planner='armour'
q0 = [0]*7
# qgoal = np.random.uniform(-1, 1, 7).tolist()
qgoal = np.array([0.63676238, 0.92068022, -0.4803974, 2.03640556, 2.83735251, -0.12433296, -3.05607319])

# generate nice random obstacles
obs_pos = np.random.uniform(low=[0.2, 0.2, 0.1], high=[0.5, 0.5, 0.5], size=(5,3))
obs_pos = (-2*np.random.randint(low=0, high=2, size=(5,3)) + 1) * obs_pos + np.array([0, 0, 0.5])
obs_size = np.random.uniform(low=0.1, high=0.2, size=(5,3))

btZonopy = bulletRtdEnv(urdf_path=urdf_path, zonopyGUI=useGUI, useGravity=useGravity, \
    useRobot=useRobot, useTorqueControl=useTorqueControl, planner=planner, \
    q0=q0, qgoal=qgoal, obs_pos=obs_pos, obs_size=obs_size)

btZonopy.simulate(qgoal)
breakpoint()