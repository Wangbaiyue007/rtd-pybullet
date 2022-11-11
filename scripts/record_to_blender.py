import numpy as np
from bullet.bulletRtdEnv import bulletRtdEnv

data = np.load("../data/trajectories/traj1.npz")
k = data['k'] #* np.array([np.pi / 72, np.pi / 72, np.pi / 72, np.pi / 72, np.pi / 72, np.pi / 72, np.pi / 72])
q0 = data['q0'] #+ np.array([np.pi, 0, 0, 0, 0, 0, 0])
qd0 = data['qd0']
qdd0 = data['qdd0']
# waypoints = data['waypoints']

# initialize environment
urdf_path="../assets"
useGUI=True
useGravity=True
useRobot=True
useTorqueControl=True
planner='armour'
record=True
blender=True

obs_pos = np.array([[0.370, 0.265, 0.15], [0.553, -0.058, 0.15], [0.553, -0.058, 0.33]])
obs_size = np.array([[0.3175, 0.3175, 0.34], [0.3175, 0.3175, 0.34], [0.3175, 0.3175, 0.67]])
rack_pos = np.array([ [0.8, 0, 1]])
rack_size = np.array([ [2, 2, 0.01]])

btEnv = bulletRtdEnv(urdf_path=urdf_path, zonopyGUI=useGUI, useGravity=useGravity, \
        useRobot=useRobot, useTorqueControl=useTorqueControl, planner=planner, record=record, blender_record=blender,\
        q0=q0[0], qgoal=q0[-1], obs_pos=np.array(rack_pos), obs_size=np.array(rack_size))

for i in range(np.size(k, axis=0)):
    btEnv.step(k[i])

btEnv.dump_video('../data/trajectories/traj1.pkl')
breakpoint()