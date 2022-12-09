import numpy as np
from bullet.bulletRtdEnv import bulletRtdEnv
from recorder.pyBulletSimRecorder import PyBulletRecorder
import os

data = np.load("../data/trajectories/ARMOUR_1208.npz.npz")
k = data['k'] #* np.array([np.pi / 72, np.pi / 72, np.pi / 72, np.pi / 72, np.pi / 72, np.pi / 72, np.pi / 72])
q0 = data['q0'] #+ np.array([np.pi, 0, 0, 0, 0, 0, 0])
qd0 = data['qd0']
qdd0 = data['qdd0']

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
obs_pos = np.array([[0.32, 0.1, 0.125]])
obs_size = np.array([[0.3, 0.3, 0.25]])

btEnv = bulletRtdEnv(urdf_path=urdf_path, zonopyGUI=useGUI, useGravity=useGravity, \
        useRobot=useRobot, useTorqueControl=useTorqueControl, planner=planner, record=record, blender_record=blender,\
        q0=q0[0], qgoal=q0[-1], obs_pos=np.array(obs_pos), obs_size=np.array(obs_size))

# find reachsets
path_to_zono = '../assets/zonotope/ARMOUR/'
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


btEnv.dump_video('../data/pkl/ARMOUR/traj.pkl')
breakpoint()