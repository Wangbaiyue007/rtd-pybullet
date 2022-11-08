import numpy as np
import matplotlib.pyplot as plt
from bullet.bulletRtdEnv import bulletRtdEnv
from bullet.bulletPlot import plotTraj

# initialize environment
urdf_path="../assets"
useGUI=True
useGravity=True
useRobot=True
useTorqueControl=True
planner='armour'
record=True
q0 = np.zeros(7)
# qgoal = np.random.uniform(-1, 1, 7).tolist()
qgoal = np.zeros(7)

# generate obstacles
obs_pos = np.array([[0.370, 0.265, 0.15], [0.553, -0.058, 0.15], [0.553, -0.058, 0.33]])
obs_size = np.array([[0.3175, 0.3175, 0.34], [0.3175, 0.3175, 0.34], [0.3175, 0.3175, 0.67]])

btEnv = bulletRtdEnv(urdf_path=urdf_path, zonopyGUI=useGUI, useGravity=useGravity, \
    useRobot=useRobot, useTorqueControl=useTorqueControl, planner=planner, record=record, \
    q0=q0, qgoal=qgoal, obs_pos=obs_pos, obs_size=obs_size)
breakpoint()

# simulate tracking from q0 to qgoal
btEnv.simulate(qgoal)

# plotting
plotTraj(btEnv)
breakpoint()