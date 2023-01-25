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

@dataclass
class JointPose:
    pos: np.ndarray

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
qpos = np.array([
            [0.28159142, 0.6, 3.6350845 , 4.73481646, 2.55565072, 0.83405794, 2.05711487],
            [4.56258737, 0.4, 3.39192523, 4.20470761, 2.86698255, 0.41219441, 6.05075981],
            [4.12150776, 5.0, 4.92619181, 5.32175324, 4.43859918, 1.44261935, 1.08688634],
            [4.72399542, 1.02936029, 3.94552621, 5.32058386, 2.7662945, 1.23406995, 0.05042256]])
qpos = (qpos + np.pi) % (2 * np.pi) - np.pi # wrap to pi

# generate obstacles
obs_pos_1 = np.array([[0.43014, 0.16598, 0.172]])
# obs_pos_1 = np.array([[0.6, 0.3, 0.172]])
obs_size_1 = np.array([[0.33, 0.33, 0.33]])
obs_ori_1 = [[0, 0, 0.362*2]]

obs_pos_2 = np.array([[0.540638, 0.386875, 0.172]])
# obs_pos_2 = np.array([[0.6, 0.3, 0.172]])
obs_size_2 = np.array([[.36, .36, .9]])
obs_ori_2 = [[0, 0, 0.320772*2]]

# obs_pos_3 = np.array([[0.899289, 0.202362, 0.172], [0.507197, 0.136198, 0.172]])
obs_pos_3 = np.array([[0.8, 0.3, 0.172], [0.6, 0.1, 0.172]])
obs_size_3 = np.array([[0.33, 0.33, 0.33], [0.33, 0.33, 0.33]])
obs_ori_3 = [[0, 0, -0.114447*2], [0, 0, -0.312354*2]]

obs_pos = [obs_pos_1, obs_pos_2, obs_pos_3]
obs_size = [obs_size_1, obs_size_2, obs_size_3]
obs_ori = [obs_ori_1, obs_ori_2, obs_ori_3]
rack_pos = np.array([[-0.3, 0., 0.5], [0.8, 0, 1], [1, -0.6, 0], [1, 0.81, 0]])
rack_size = np.array([[0.01, 2, 2], [2, 2, 0.01], [2, 0.01, 2], [2, 0.01, 2]])

q0 = qpos[0]
for number in [0]:
    # load waypoints
    # traj = np.load(f"../data/trajectories/traj_dumbbell{number+1}.npz")
    # _waypoints = traj['waypoints']
    # waypoints = [JointPose(waypoint) for waypoint in _waypoints]
    
    # load obstacles
    obs_pos_i = np.append(obs_pos[number], rack_pos, axis=0)
    obs_size_i = np.append(obs_size[number], rack_size, axis=0)
    obs_ori_i = np.array(obs_ori[number] + [[0, 0, 0] for _ in range(4)])

    # create env
    btEnv = bulletRtdEnv(urdf_path=urdf_path, zonopyGUI=useGUI, useGravity=useGravity, \
        useRobot=useRobot, useTorqueControl=useTorqueControl, planner=planner, record=record, blender_record=blender,\
        q0=qpos[number], qgoal=qpos[number+1], obs_pos=obs_pos_i, obs_size=obs_size_i, obs_ori=obs_ori_i)
    btEnv.rrt_buffer = 0.25

    # simulate tracking from q0 to qgoal
    # btEnv.plot_waypoints(waypoints)
    # btEnv.waypoints = waypoints
    # k = 0
    # for i in range(100):
    #     # last waypoint, minimize error
    #     if  i + 1 >= len(waypoints):
    #         dist_error: float = np.linalg.norm(wrap_to_pi(bernstein_q_des(btEnv.q0[-1], btEnv.qd0[-1], btEnv.qdd0[-1], k, 1)) - wrap_to_pi(qpos[number+1]))
    #         if dist_error > 0.2:
    #             print(f"minimizing error: {dist_error}")
    #             k, done = btEnv.armtd_plan(waypoints[-1].pos, i)
    #         else:
    #             q0_last = btEnv.q0[-1]
    #             qd0_last = btEnv.qd0[-1]
    #             qdd0_last = btEnv.qdd0[-1]
    #             btEnv.step(k, qpos=q0_last, qvel=qd0_last, qacc=qdd0_last, stop=True)
    #             break
    #     else:
    #         k, done = btEnv.armtd_plan(waypoints[i].pos, i)
    #     btEnv.step(k)
    btEnv.simulate(qpos[number+1])

    # use last position as the next start position
    q0 = btEnv.qpos_record[-1]

    # save data
    btEnv.dump_video(f'../data/trajectories/ARMOUR_0120_{number+1}')
    btEnv.dump_traj(f'../data/trajectories/ARMOUR_0120_{number+1}')

    breakpoint()
    btEnv.Disconnect()
