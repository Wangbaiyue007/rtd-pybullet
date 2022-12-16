# NOTE: this is a temprary file to rerun hardware experiments in sim

import numpy as np
import os
from bullet.bulletRtdEnv import bulletRtdEnv
from bullet.bulletPlot import plotTraj
import pybullet as pbt
from dataclasses import dataclass

def bernstein_q_des(q0, qd0, qdd0, k, t):
    def pow(x,y):
        return x ** y
    B0 = -pow(t - 1, 5)
    B1 = 5 * t * pow(t - 1, 4)
    B2 = -10 * pow(t, 2) * pow(t - 1, 3)
    B3 = 10 * pow(t, 3) * pow(t - 1, 2)
    B4 = -5 * pow(t, 4) * (t - 1)
    B5 = pow(t, 5)
    beta0 = q0
    beta1 = q0 + qd0 / 5
    beta2 = q0 + (2 * qd0) / 5 + qdd0 / 20
    beta3 = q0 + k
    beta4 = q0 + k
    beta5 = q0 + k
    return (B0 * beta0 + B1 * beta1 + B2 * beta2 + B3 * beta3 + B4 * beta4 +
            B5 * beta5)


def wrap_to_pi(q: np.ndarray):
    """
    helper function to wrap q to [-pi, pi)
    """
    q_wrapped = (q + np.pi) % (2 * np.pi) - np.pi # wrap to pi
    return q_wrapped

# remove previous data
dir_list = os.listdir("../data/ARMOUR/reachsets/")
for file in dir_list:
    os.remove("../data/ARMOUR/reachsets/"+file)

# load waypoints
@dataclass
class JointPose:
    pos: np.ndarray

traj = np.load("../data/trajectories/traj_dumbbell1.npz")
_waypoints = traj['waypoints']
waypoints = [JointPose(waypoint) for waypoint in _waypoints]

# initialize environment
urdf_path="../assets"
useGUI=True
useGravity=True
useRobot=True
useTorqueControl=True
planner='armour'
record=True
blender=True

# key positions
qpos = np.array([[0.28159142, 0.6, 3.6350845 , 4.73481646, 2.55565072, 0.83405794, 2.05711487],
                [4.56258737, 0.4, 3.39192523, 4.20470761, 2.86698255, 0.41219441, 6.05075981]])
qpos = (qpos + np.pi) % (2 * np.pi) - np.pi # wrap to pi

# generate obstacles
obs_pos = np.array([[0.43014, 0.16598, 0.172]])
obs_size = np.array([[0.33, 0.33, 0.33]])
obs_ori = [[0, 0, 0.362*2]]
rack_pos = np.array([[-0.3, 0., 0.5], [0.8, 0, 1], [1, -0.7, 0], [1, 0.81, 0]])
rack_size = np.array([[0.01, 2, 2], [2, 2, 0.01], [2, 0.01, 2], [2, 0.01, 2]])

obs_pos_i = np.append([obs_pos[0]], rack_pos, axis=0)
obs_size_i = np.append([obs_size[0]], rack_size, axis=0)
obs_ori_i = np.array(obs_ori + [[0, 0, 0] for _ in range(4)])
btEnv = bulletRtdEnv(urdf_path=urdf_path, zonopyGUI=useGUI, useGravity=useGravity, \
    useRobot=useRobot, useTorqueControl=useTorqueControl, planner=planner, record=record, blender_record=blender,\
    q0=qpos[0], qgoal=qpos[1], obs_pos=obs_pos_i, obs_size=obs_size_i, obs_ori=obs_ori_i)

# simulate tracking from q0 to qgoal
btEnv.plot_waypoints(waypoints)
btEnv.waypoints = waypoints
for i in range(100):
    # last waypoint, minimize error
    if  i + 1 >= len(waypoints):
        dist_error: float = np.linalg.norm(bernstein_q_des(btEnv.q0[-1], btEnv.qd0[-1], btEnv.qdd0[-1], k, 1) - wrap_to_pi(qpos[1]))
        if dist_error > 0.2:
            print(f"minimizing error: {dist_error}")
            k, done = btEnv.armtd_plan(waypoints[-1].pos, i)
        else:
            break
    else:
        k, done = btEnv.armtd_plan(waypoints[i].pos, i)
    btEnv.step(k)

# save data
btEnv.dump_video('../data/trajectories/ARMOUR_1214')
btEnv.dump_traj('../data/trajectories/ARMOUR_1214')

breakpoint()
btEnv.Disconnect()
