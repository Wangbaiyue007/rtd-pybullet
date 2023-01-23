import numpy as np
from bullet.bulletRtdEnv import bulletRtdEnv
from recorder.pyBulletSimRecorder import PyBulletRecorder
import os

data = np.load("../data/trajectories/ARMOUR_0120_1.npz")
k = data['k'] #* np.array([np.pi / 72, np.pi / 72, np.pi / 72, np.pi / 72, np.pi / 72, np.pi / 72, np.pi / 72])
q0 = data['q0'] #+ np.array([np.pi, 0, 0, 0, 0, 0, 0])
qd0 = data['qd0']
qdd0 = data['qdd0']

# remove previous data
dir_list = os.listdir("../data/pkl/ARMOUR/")
for file in dir_list:
    os.remove("../data/pkl/ARMOUR/"+file)

# initialize environment
urdf_path="../assets"
useGUI=True
useGravity=True
useRobot=True
useTorqueControl=True
planner='armour'
record=True
blender=True

# generate obstacles
obs_pos_1 = np.array([[0.43014, 0.16598, 0.172]])
obs_size_1 = np.array([[0.33, 0.33, 0.33]])
obs_ori_1 = np.array([[0, 0, 0.362*2]])

obs_pos_2 = np.array([[0.540638, 0.386875, 0.172]])
obs_size_2 = np.array([[.36, .36, .9]])
obs_ori_2 = np.array([[0, 0, 0.320772*2]])

obs_pos_3 = np.array([[0.899289, 0.202362, 0.172], [0.507197, 0.136198, 0.172]])
obs_size_3 = np.array([[0.33, 0.33, 0.33], [0.33, 0.33, 0.33]])
obs_ori_3 = np.array([[0, 0, -0.114447*2], [0, 0, -0.312354*2]])

obs_pos = [obs_pos_1, obs_pos_2, obs_pos_3]
obs_size = [obs_size_1, obs_size_2, obs_size_3]
obs_ori = [obs_ori_1, obs_ori_2, obs_ori_3]

btEnv = bulletRtdEnv(urdf_path=urdf_path, zonopyGUI=useGUI, useGravity=useGravity, \
        useRobot=useRobot, useTorqueControl=useTorqueControl, planner=planner, record=record, blender_record=blender,\
        q0=q0[0], qgoal=q0[-1], obs_pos=obs_pos[0], obs_size=obs_size[0], obs_ori=obs_ori[0])
breakpoint()

# find reachsets
path_to_zono = '../assets/zonotope/ARMOUR/1/'
files_zono = os.listdir(path_to_zono)

# initialize renderer
recorder_zono_slc = PyBulletRecorder()

for step in range(np.size(k, axis=0)):
    # create reachsets visuals and record
    zonoIds_slc = []
    for file in files_zono:
        if 'step_'+str(step+1)+'_' in file and '.obj' in file and '.urdf' not in file:
            zonoId_slc = btEnv.create_visual(path_to_zono+file)
            recorder_zono_slc.register_object(zonoId_slc, path_to_zono+file+'.urdf')
            zonoIds_slc.append(zonoId_slc)
    
    # step simulation
    btEnv.step(k[step])
    # breakpoint()

    # dump zonotope recording
    recorder_zono_slc.save("../data/pkl/ARMOUR/step_"+str(step+1)+"_slc.pkl")
    # remove zonotope visual
    for Id in zonoIds_slc:
        btEnv.remove_body(Id)
        recorder_zono_slc.unregister_object(Id)


# btEnv.dump_video('../data/pkl/ARMOUR/traj.pkl')
breakpoint()