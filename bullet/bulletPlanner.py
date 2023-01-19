import torch
import numpy as np
import shutil

def stack_obstacles(obs_pos, obs_size):
    """
    Stacking obstacles information to be armour input
    """
    obs_pos_np = np.array(obs_pos)
    obs_size_np = np.array(obs_size / 2)
    obs_size_stack = np.empty([1,9])
    for size in obs_size_np:
        obs_size_stack = np.concatenate([obs_size_stack, (size*np.eye(3)).reshape(1,9)], axis=0)
    obs_size_stack = obs_size_stack[1:]
    obs_stack = np.concatenate([obs_pos_np, obs_size_stack], axis=1)

    return obs_stack

def rotEuler(ori):
    alpha = ori[0]
    beta = ori[1]
    gamma = ori[2]
    Rz = np.array([[np.cos(gamma), -np.sin(gamma), 0], [np.sin(gamma), np.cos(gamma), 0], [0, 0, 1]])
    Ry = np.array([[np.cos(beta), 0, np.sin(beta)], [0, 1, 0], [-np.sin(beta), 0, np.cos(beta)]])
    Rx = np.array([[1, 0, 0], [0, np.cos(alpha), -np.sin(alpha)], [0, np.sin(alpha), np.cos(alpha)]])
    R = Rz.dot(Ry.dot(Rx))
    return R

class bulletPlanner:

    class ARMOUR:
        def __init__(self, obs_pos, obs_size, obs_ori):
            # import armtd_main_pybind
            import armour_main_pybind
            obs_stack = stack_obstacles(obs_pos=obs_pos, obs_size=obs_size)
            # rotate generators if orientation is given
            if len(obs_ori[0]) != 0:
                obs_stack = obs_stack.reshape(1, 12*len(obs_ori)).reshape(4*len(obs_ori), 3)
                for i in range(np.size(obs_ori, axis=0)):
                    R = rotEuler(obs_ori[i])
                    obs_stack[4*i+1:4*(i+1), :] = R.dot(obs_stack[4*i+1:4*(i+1), :]).T
                obs_stack = obs_stack.reshape(len(obs_ori), 12)
            # self.planner = armtd_main_pybind.pzsparse(obs_stack)
            self.planner = armour_main_pybind.pzsparse(obs_stack)
        
        def plan(self, q0: np.ndarray, qd0: np.ndarray, qdd0: np.ndarray, goal: np.ndarray, step: int) -> np.ndarray:
            """
            Return Berenstein coefficients
            """
            k_opt = self.planner.optimize(q0, qd0, qdd0, goal)
            # copy reachset data
            for link in range(8): 
                shutil.copy(f'../armour-dev/kinova_src/kinova_simulator_interfaces/kinova_planner_realtime/reachsets/armour_joint_position_center.out', f'../data/ARMOUR/reachsets/step_{step+1}_center.txt')
                shutil.copy(f'../armour-dev/kinova_src/kinova_simulator_interfaces/kinova_planner_realtime/reachsets/armour_joint_position_radius.out', f'../data/ARMOUR/reachsets/step_{step+1}_radius.txt')
            return k_opt

        def get_des_traj(self, q0: np.ndarray, qd0: np.ndarray, qdd0: np.ndarray, k:np.ndarray, t: float) -> np.ndarray:
            """
            Generate desired trajectory of the Bezier curve, returns a 7 by 3 matrix [q, qd, qdd]
            """
            qdes = self.planner.getDesTraj(q0, qd0, qdd0, k, t)
            return qdes.reshape(7,3)


    class Zonopy:
        def __init__(self, q0 = [0]*7, qgoal = [0]*7, obs_pos = [[]], obs_size = [], dtype = torch.float, device = 'cuda:0', debug=True):
            from zonopy.environments.arm_3d import Arm_3D
            from zonopy.optimize.armtd_3d import ARMTD_3D_planner

            self.arm3d = Arm_3D(robot="Kinova3", n_obs=len(obs_pos), FO_render_freq=25, goal_threshold=0.1)
            q = torch.tensor(q0, dtype=dtype, device=device)
            qd = torch.zeros(self.arm3d.n_links, dtype=dtype, device=device)
            qgoal = torch.tensor(qgoal, dtype=dtype, device=device)
            obs_pos = torch.tensor(obs_pos, dtype=dtype, device=device)
            obs_size = torch.tensor(obs_size,dtype=dtype,device=device) / 2
            self.arm3d.set_initial(q, qd, qgoal, obs_pos, obs_size)
            self.planner = ARMTD_3D_planner(self.arm3d, dtype=dtype, device=device)
            self.ka_0 = torch.zeros(self.arm3d.dof)
            self.ka = torch.zeros(self.arm3d.dof)
            self.debug = debug

        def step(self):
            
            if self.debug:
                print(f"Iteration {iter}")
            ka, flag, _ = self.planner.plan(self.arm3d, self.ka_0)
            ka_break = (-self.arm3d.qvel) / 0.5

            if self.debug:
                print(f"qvel_prestep: {self.arm3d.qvel}")
            observations, reward, done, info = self.arm3d.step(ka.cpu(), flag)
            if self.debug:
                print(f"qvel_poststep: {self.arm3d.qvel}")

            safe = self.arm3d.safe
            if safe:
                if self.debug:
                    print("--safe move--")
                qacc = ka.cpu()
            else:
                if self.debug:
                    print("--safe break--")
                qacc = ka_break.cpu()
            if self.debug:
                print(f"qacc: {qacc}")

            return qacc, done

        def step_hardware(self, qpos_hardware, qvel_hardware):
            # correct env state using hardware data
            self.arm3d.qpos = qpos_hardware
            self.arm3d.qvel = qvel_hardware

            # plan for the future 0.5 sec, which should be used for the next step
            ka, flag = self.planner.plan_hardware(self.arm3d, self.ka_0, self.ka)
            print(ka)
            ka_break = (-self.arm3d.qvel) / 0.5

            # step the previous plan in the zonopy environment
            observations, reward, done, info = self.arm3d.step(self.ka.cpu(), flag)

            # fail save manuvoir
            safe = self.arm3d.safe
            if safe:
                print("--safe move--")
                qacc = ka.cpu()
            else:
                print("--safe break--")
                qacc = ka_break.cpu()
            # print(f"qacc: {qacc}")

            # Update the plan
            self.ka_pre = self.ka
            self.ka = qacc

            return qacc, done
        
        def free(self):
            """
            Free GPU memory
            """
            import gc
            gc.collect()
            torch.cuda.empty_cache()