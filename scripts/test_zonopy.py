# run zonopy 3D simulation: obstacle avoidance

import torch
import numpy as np
from zonopy.environments.arm_3d import Arm_3D
from zonopy.optimize.armtd_3d import ARMTD_3D_planner
import os

def save_zono(env:Arm_3D):
    for i in range(env.n_links):
        FO_poly = np.empty((1,3))
        FO_poly_slc = np.empty((1,3))
        for j in range(100):
            FO_poly_j = env.FO_poly[i][j].numpy()
            FO_poly = np.concatenate([FO_poly, FO_poly_j.transpose(2,1,0).reshape(3,-1).T], axis=0)
            FO_poly_j_slc = env.FO_poly_slc[i][j].numpy()
            FO_poly_slc = np.concatenate([FO_poly_slc, FO_poly_j_slc.transpose(2,1,0).reshape(3,-1).T], axis=0)
        filename = "../data/ARMTD_zonopy/zonopy_sim/zonopy_vertices_step"+str(step+1)+"_link"+str(i+1)+".csv"
        np.savetxt(filename, FO_poly, delimiter=",")
        filename_slc = "../data/ARMTD_zonopy/zonopy_sim/zonopy_vertices_slc_step"+str(step+1)+"_link"+str(i+1)+".csv"
        np.savetxt(filename_slc, FO_poly_slc, delimiter=",")

def remove(path):
    dir_list = os.listdir(path)
    for file in dir_list:
        os.remove(path+file)

# Obstacles data
obstacle_info = np.genfromtxt('../data/ARMTD_matlab/matlab_obstacles.csv', delimiter=',')
obstacle_pos = obstacle_info[::4]
obstacle_scale = np.zeros((int(np.size(obstacle_pos, 0)), 3))
index = 0
for i in range(np.size(obstacle_info, 0)):
    if i % 4 == 1:
        obstacle_scale[index] = np.array([obstacle_info[i, 0], obstacle_info[i+1, 1], obstacle_info[i+2, 2]])
        index += 1
obstacle_pos = obstacle_pos[:-4] # remove self collision
obstacle_scale = obstacle_scale[:-4]
obs_pos = torch.tensor(obstacle_pos) - torch.tensor([0.2, 0, 0.5])
obs_size = torch.tensor(obstacle_scale)

# remove previous zonotope files
remove("../data/ARMTD_zonopy/zonopy_sim/")
# initialize zonopy environment
env = Arm_3D(robot="Kinova3", n_obs=np.size(obstacle_pos, 0), obs_size_max=[0.15,0.15,0.15], obs_size_min=[0.05,0.05,0.05], FO_render_freq=25)
observations = env.reset()
# obs_pos = observations['obstacle_pos'] # obs_pos = [torch.tensor([.5,.5,0], dtype=torch.float, device='cuda:0')]
# obs_size = observations['obstacle_size']
joint_pos = np.genfromtxt('../data/ARMTD_matlab/joint_pos.csv', delimiter=',').T
q0 = torch.tensor([-.5, 1.1, 0, 0.25, 0, 0.1, 0], dtype=torch.float, device='cuda:0')
qd0 = torch.zeros(env.n_links, dtype=torch.float, device='cuda:0')
qgoal = torch.tensor([.5, 1., 0, 0.25, 0, 0.1, 0], dtype=torch.float, device='cuda:0')
observation = env.set_initial(q0, qd0, qgoal, obs_pos, obs_size)
planner = ARMTD_3D_planner(env, device='cuda:0', dtype=torch.float)
ka_0 = torch.zeros(env.dof)
iter = 0

# save obstacle positions to file
qpos = np.zeros((30, 7))
qacc = np.zeros((30, 7))
for step in range(30):
    iter += 1
    print(f"Iteration {iter}")
    ka, flag, _ = planner.plan(env, ka_0)
    ka_break = (-env.qvel) / 0.5

    qpos[step] = env.qpos
    print(f"qvel_prestep: {env.qvel}")
    observations, reward, done, info = env.step(ka.cpu(), flag)
    print(f"qvel_poststep: {env.qvel}")

    if env.safe:
        print("--safe move--")
        qacc[step] = ka.cpu()
    else:
        print("--safe break--")
        qacc[step] = ka_break.cpu()
    print(f"qacc: {qacc[step]}")

    # save reach set to vertices files
    env.render(planner.FO_link)
    save_zono(env)
    if done: 
        print("Done!")
        break
breakpoint()
np.savetxt("../assets/zonotope/data/zonopy_obstacles_pos.csv", observations['obstacle_pos'].numpy(), delimiter=",")
np.savetxt("../assets/zonotope/data/zonopy_obstacles_size.csv", observations['obstacle_size'].numpy(), delimiter=",")
np.savetxt("../data/ARMTD_zonopy/zonopy_qpos.csv", qpos, delimiter=",")
np.savetxt("../data/ARMTD_zonopy/zonopy_qacc.csv", qacc, delimiter=",")
# remove previous mesh
dir_list = os.listdir("../assets/zonotope/meshes")
for file in dir_list:
    if '.urdf' not in file:
        os.remove("../assets/zonotope/meshes/"+file)