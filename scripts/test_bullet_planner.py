import numpy as np
import matplotlib.pyplot as plt
from bullet.bulletRtdEnv import bulletRtdEnv

# initialize environment
urdf_path="../assets"
useGUI=True
useGravity=True
useRobot=True
useTorqueControl=True
planner='armour'
record=True
q0 = [0]*7
# qgoal = np.random.uniform(-1, 1, 7).tolist()
qgoal = np.array([0.63676238, 0.92068022, -0.4803974, 2.03640556, 2.83735251, -0.12433296, -3.05607319])

# generate nice random obstacles
num_obs = 2
obs_pos = np.random.uniform(low=[0.2, 0.2, 0.1], high=[0.5, 0.5, 0.5], size=(num_obs,3))
obs_pos = (-2*np.random.randint(low=0, high=2, size=(num_obs,3)) + 1) * obs_pos + np.array([0, 0, 0.5])
obs_size = np.random.uniform(low=0.1, high=0.2, size=(num_obs,3))

btEnv = bulletRtdEnv(urdf_path=urdf_path, zonopyGUI=useGUI, useGravity=useGravity, \
    useRobot=useRobot, useTorqueControl=useTorqueControl, planner=planner, record=record, \
    q0=q0, qgoal=qgoal, obs_pos=obs_pos, obs_size=obs_size)

# simulate tracking from q0 to qgoal
btEnv.simulate(qgoal)

# plotting
t = np.arange(np.size(btEnv.qpos_record, axis=0), dtype=float) * btEnv.timestep
waypoints = np.array([np.array(q0)])
for waypoint in btEnv.waypoints: 
    waypoints = np.append(waypoints, [waypoint.pos], axis=0)
index = np.arange(np.size(waypoints, axis=0), dtype=float) * 1
fig, axs = plt.subplots(nrows=7, ncols=3, constrained_layout=True)
for i, ax in enumerate(axs.flat):
    joint = int(i/3)
    if i % 3 == 0:
        line2, = ax.plot(t, btEnv.qpos_record[:,joint], 'g-', label='sim', linewidth=3.0)
        line1, = ax.plot(t, btEnv.qpos_des_record[:,joint], 'r--', label='desired', linewidth=1.5)
        line3, = ax.plot(index, waypoints[:,joint], 'bs', label='waypoints', linewidth=1.5)
        ax.set_title('$q$'+str(joint+1))
        ax.grid(True)
    if i % 3 == 1:
        ax.plot(t, btEnv.qvel_record[:,joint], 'g-', linewidth=3.0)
        ax.plot(t, btEnv.qvel_des_record[:,joint], 'r--', linewidth=1.5)
        ax.set_title('$\dot{q}$'+str(joint+1))
        ax.grid(True)
    if i % 3 == 2:
        ax.plot(t, btEnv.qacc_record[:,joint], 'g-', linewidth=3.0)
        ax.plot(t, btEnv.qacc_des_record[:,joint], 'r--', linewidth=1.5)
        ax.set_title('$\ddot{q}$'+str(joint+1))
        ax.grid(True)
plt.legend(handles=[line1, line2, line3], loc='upper right')
plt.show()
breakpoint()