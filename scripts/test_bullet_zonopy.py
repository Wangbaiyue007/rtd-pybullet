import numpy as np
from bullet.bulletRtdEnv import bulletRtdEnv

# initialize environment
urdf_path="../assets/kinova_gen3_7dof/kinova_with_robotiq_85.urdf"
useGUI=True
useGravity=True
useRobot=True
useTorqueControl=True
useZonopy=True
q0 = [0]*7
qgoal = np.random.uniform(-1, 1, 7).tolist()
obs_pos = [[0.5, 0.5, 0], [-0.2, 0.3, 0.5]]

btZonopy = bulletRtdEnv(urdf_path=urdf_path, GUI=useGUI, useGravity=useGravity, \
    useRobot=useRobot, useTorqueControl=useTorqueControl, useZonopy=useZonopy, \
    q0=q0, qgoal=qgoal, obs_pos=obs_pos)

btZonopy.simulate(qgoal)