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
qgoal = np.random.uniform(-1, 1, 7).tolist()
obs_pos = [[0.5, 0.5, 0], [-0.2, 0.3, 0.5]]
obs_size = [[0.1, 0.1, 0.1], [0.2, 0.2, 0.2]]

btZonopy = bulletRtdEnv(urdf_path=urdf_path, zonopyGUI=useGUI, useGravity=useGravity, \
    useRobot=useRobot, useTorqueControl=useTorqueControl, planner=planner, \
    q0=q0, qgoal=qgoal, obs_pos=obs_pos, obs_size=obs_size)

btZonopy.simulate(qgoal)