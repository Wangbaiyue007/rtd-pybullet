import numpy as np
from bullet.bulletRtdEnv import bulletRtdEnv
from recorder.pyBulletSimRecorder import PyBulletRecorder

obs_pos = np.loadtxt('../data/Agents/env_1117.txt', delimiter=',')
obs_size = np.array([[0.1, 0.1, 0.1] for _ in range(np.size(obs_pos, axis=0))])
k_RL = np.loadtxt('../data/Agents/actions_RL_1117.txt', delimiter=',')
k_DART = np.loadtxt('../data/Agents/actions_DART_1117.txt', delimiter=',')
q0 = np.array([ 0.61529517, 1.66432738, 0.0, -1.08245885, 0.08671975, 0.0, 2.50734258])
qgoal = np.array([-3.05307007, 0.37813687, 0.0, 2.22425866, 1.13432384, 0.0, -2.43718553])

urdf_path="../assets"
useGUI=True
useGravity=True
useRobot=True
useTorqueControl=True
controlGain=10000
planner='zonopy'
record=False
blender_record=True

# RL
btEnvRL = bulletRtdEnv(urdf_path=urdf_path, zonopyGUI=useGUI, useGravity=useGravity, \
    useRobot=useRobot, useTorqueControl=useTorqueControl, controlGain=controlGain, \
    planner=planner, record=record, blender_record=blender_record, q0=q0, qgoal=qgoal, \
    obs_pos=obs_pos, obs_size=obs_size)

# record goal position
goal_recorder = PyBulletRecorder()
goal_recorder.register_object(btEnvRL.EnvId[0], btEnvRL.path[0])
btEnvRL.forwardkinematics(qgoal)
goal_recorder.add_keyframe()
btEnvRL.forwardkinematics(q0)
goal_recorder.save('../data/Agents/goal.pkl')

# step actions
for i in range(np.size(k_RL, axis=0)):
    btEnvRL.step(k_RL[i])

# save trajectory
btEnvRL.dump_video('../data/Agents/RL.pkl')
btEnvRL.Disconnect()

# DART
btEnvDART = bulletRtdEnv(urdf_path=urdf_path, zonopyGUI=useGUI, useGravity=useGravity, \
    useRobot=useRobot, useTorqueControl=useTorqueControl, controlGain=controlGain, \
    planner=planner, record=record, blender_record=blender_record, q0=q0, qgoal=qgoal, \
    obs_pos=obs_pos, obs_size=obs_size)

# step actions
for i in range(np.size(k_DART, axis=0)):
    btEnvDART.step(k_DART[i])

# save trajectory
btEnvDART.dump_video('../data/Agents/DART.pkl')
btEnvDART.Disconnect()

breakpoint()