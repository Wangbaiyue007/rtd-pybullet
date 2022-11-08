import matplotlib.pyplot as plt
import numpy as np
from bullet.bulletRtdEnv import bulletRtdEnv

def plotTraj(btEnv: bulletRtdEnv):
    assert btEnv.record == True, f"The environment recorder is not turned on."

    t = np.arange(np.size(btEnv.qpos_record, axis=0), dtype=float) * btEnv.timestep
    waypoints = np.array([btEnv.qpos_record[0]])
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