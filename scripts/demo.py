import numpy as np
from bullet.bulletRtdEnv import bulletRtdEnv
from bullet.bulletPlot import plotTraj
import pybullet as pbt

# initialize environment
urdf_path="../assets"
useGUI=True
useGravity=True
useRobot=True
useTorqueControl=True
planner='armour'
record=True

# key positions
qpos = np.array([[0.28159142, 0.6, 3.6350845 , 4.73481646, 2.55565072, 0.83405794, 2.05711487],
       [4.56258737, 0.4, 3.39192523, 4.20470761, 2.86698255, 0.41219441, 6.05075981],
       [4.12150776, 5, 4.92619181, 5.32175324, 4.43859918, 1.44261935, 1.08688634], 
       [4.72399542, 1.02936029, 3.94552621, 5.32058386, 2.7662945, 1.23406995, 0.05042256]])
qpos = (qpos + np.pi) % (2 * np.pi) - np.pi # wrap to pi

# generate obstacles
obs_pos = np.array([[0.370, 0.265, 0.15], [0.553, -0.058, 0.15], [0.553, -0.058, 0.33]])
obs_size = np.array([[0.3175, 0.3175, 0.34], [0.3175, 0.3175, 0.34], [0.3175, 0.3175, 0.67]])
rack_pos = np.array([[-0.3, 0., 0.5], [0.8, 0, 1], [1, -0.7, 0], [1, 0.81, 0]])
rack_size = np.array([[0.01, 2, 2], [2, 2, 0.01], [2, 0.01, 2], [2, 0.01, 2]])

for i in [2]:
    obs_pos_i = np.append([obs_pos[i]], rack_pos, axis=0)
    obs_size_i = np.append([obs_size[i]], rack_size, axis=0)
    btEnv = bulletRtdEnv(urdf_path=urdf_path, zonopyGUI=useGUI, useGravity=useGravity, \
        useRobot=useRobot, useTorqueControl=useTorqueControl, planner=planner, record=record, \
        q0=qpos[i], qgoal=qpos[i+1], obs_pos=obs_pos_i, obs_size=obs_size_i)

    # simulate tracking from q0 to qgoal
    btEnv.simulate(qpos[i+1])

    # plotting
    plotTraj(btEnv)

    # save data
    waypoints = np.array([btEnv.qpos_record[0]])
    for waypoint in btEnv.waypoints: 
        waypoints = np.append(waypoints, [waypoint.pos], axis=0)
    data = {"k" : btEnv.k, "qdd0": btEnv.qdd0, "qd0" : btEnv.qd0, "q0" : btEnv.q0, "waypoints" : waypoints}
    np.savez("../data/trajectories/traj_dumbbell"+str(i+1)+".npz", **data)

    breakpoint()
    btEnv.Disconnect()
