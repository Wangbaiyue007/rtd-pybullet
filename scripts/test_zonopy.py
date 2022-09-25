# run zonopy 3D simulation: obstacle avoidance

import torch
import numpy as np
from zonopy.environments.arm_3d import Arm_3D
from zonopy.optimize.armtd_3d import ARMTD_3D_planner
import os

def save_zono(env):
    for i in range(env.n_links):
        for j in range(100):
            filename = "../data/ARMTD_zonopy/zonopy_sim/zonopy_vertices_step"+str(step+1)+"_link"+str(i+1)+"_num"+str(j+1)+".csv"
            FO_poly = env.FO_poly[i][j].numpy()
            FO_poly = FO_poly.transpose(2,1,0).reshape(3,-1).T
            np.savetxt(filename, FO_poly, delimiter=",")

env = Arm_3D(robot="Kinova3", n_obs=1, FO_render_freq=25)
obs_pos = [torch.tensor([.5,.5,0], dtype=torch.float, device='cuda:0')]
q0 = torch.zeros(env.n_links, dtype=torch.float, device='cuda:0')
qd0 = torch.zeros(env.n_links, dtype=torch.float, device='cuda:0')
qgoal = torch.zeros(env.n_links, dtype=torch.float, device='cuda:0')
observation = env.set_initial(q0, qd0, qgoal, obs_pos)
planner = ARMTD_3D_planner(env, device='cuda:0', dtype=torch.float)
ka_0 = torch.zeros(env.dof)
iter = 0

# save obstacle positions to file
# np.savetxt("../zonotope/data/zonopy_obstacles.csv", observation['obstacle_pos'].numpy(), delimiter=",")
qpos = np.zeros((30, 7))
qacc = np.zeros((30, 7))
for step in range(30):
    iter += 1
    print(f"Iteration {iter}")
    ka, flag = planner.plan(env, ka_0)
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
    env.render()
    # save_zono(env)
    if done: 
        print("Done!")
        break
breakpoint()
np.savetxt("../data/ARMTD_zonopy/zonopy_qpos.csv", qpos, delimiter=",")
np.savetxt("../data/ARMTD_zonopy/zonopy_qacc.csv", qacc, delimiter=",")
# remove previous mesh
dir_list = os.listdir("../zonotope/meshes")
for file in dir_list:
    if '.urdf' not in file:
        os.remove("../zonotope/meshes/"+file)