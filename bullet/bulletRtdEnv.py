import sys
import time
from typing import Tuple
import pybullet as p
import numpy as np
import pybullet_data
import torch
from bullet.bulletPlanner import bulletPlanner
from bullet.rrt import BuildRRT, Node
from recorder.pyBulletSimRecorder import PyBulletRecorder
import xml.etree.ElementTree as ET
from typing import List

clid = p.connect(p.SHARED_MEMORY)

def wrap_to_pi(q: np.ndarray):
    q_wrapped = (q + np.pi) % (2 * np.pi) - np.pi # wrap to pi
    return q_wrapped

class bulletRtdEnv:

    def __init__(
        self, 
        urdf_path="../assets", 
        bulletGUI=True, 
        zonopyGUI=True,
        timestep=0.001, 
        control_gain = 1000,
        useGravity=True, 
        useRobot=True, 
        useTorqueControl=True,
        planner=None,
        record=False,
        blender_record=False,
        q0 = [0]*7, 
        qgoal = [0]*7, 
        obs_pos = [[]],
        obs_size = [[]],
        obs_ori = [[]]
        ):

        ############################## bullet #############################
        # enable GUI or not
        self.bulletGUI = bulletGUI
        if bulletGUI:
            if (clid < 0): p.connect(p.GUI)
        else:
            if (clid < 0): p.connect(p.DIRECT)

        self.timestep = timestep
        p.setTimeStep(timestep)
        p.setRealTimeSimulation(0)

        if useGravity:
            p.setGravity(0, 0, -9.8)

        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        # p.loadURDF('plane.urdf')

        self.EnvId = []
        if useRobot:
            self.robotId = p.loadURDF(urdf_path+'/kinova_gen3_7dof/kinova_with_dumbbell.urdf', [0, 0, 0], useFixedBase=True)
            self.EnvId = [self.robotId]
            # choose the end effector tool frame as end effector index
            self.actuation_index = []
            self.numJoints = p.getNumJoints(self.robotId)
            self.EndEffectorIndex = 8
            # set robot joint limits and rest positions
            self.ll, self.ul, self.jr, self.rp = self.get_joint_limits(self.robotId)

        if useTorqueControl and useRobot:
            # Disable the motors for torque control
            p.setJointMotorControlArray(self.robotId, self.actuation_index, p.VELOCITY_CONTROL, forces=np.zeros(len(self.actuation_index))) 
            # Disable link damping
            for i in range(self.numJoints):
                p.changeDynamics(self.robotId, i-1, linearDamping=0, angularDamping=0)
        
        # inv dynamics controller gain
        self.Kp = np.eye(7)
        self.Kd = np.eye(7)
        for i in range(7):
            self.Kp[i, i] = control_gain*(1-0.15*i)
            self.Kd[i, i] = 1.3*(self.Kp[i, i]/2)**0.5
        self.path = [urdf_path+'/kinova_gen3_7dof/kinova_with_dumbbell.urdf']
        self.scale = [[1, 1, 1]]

        # load obstacles
        if len(obs_ori[0]) == 0:
            obs_ori = [[0, 0, 0]]*len(obs_pos)
        for i in range(len(obs_pos)):
            self.load(urdf_path+"/objects/cube_small_zero.urdf", pos=obs_pos[i], ori=obs_ori[i], scale=obs_size[i], urdf_index=i)

        ############################## planner #############################
        # initialize planner and its environment
        self.qgoal = qgoal
        self.planner_agent = None
        self.obs_pos = obs_pos
        self.obs_size = obs_size
        self.obs_ori = obs_ori
        self.k = np.array([np.zeros(7)])
        self.q0 = np.array([q0])
        self.qd0 = np.array([np.zeros(7)])
        self.qdd0 = np.array([np.zeros(7)])
        self.forwardkinematics(q0)
        self.qpos_sim = q0
        self.qvel_sim = np.zeros(7)
        self.qacc_sim = np.zeros(7)
        self.record = record
        self.qpos_record = np.array([q0])
        self.qvel_record = np.array([np.zeros(7)])
        self.qacc_record = np.array([np.zeros(7)])
        self.qpos_des_record = np.array([q0])
        self.qvel_des_record = np.array([np.zeros(7)])
        self.qacc_des_record = np.array([np.zeros(7)])
        self.planner_name = planner

        if planner == 'zonopy':
            self.planner_agentGUI = zonopyGUI
            self.planner_agent = bulletPlanner.Zonopy(q0=q0, qgoal=self.qgoal, obs_pos=obs_pos, obs_size=obs_size)
        elif planner == 'armour':
            self.zonopy = bulletPlanner.Zonopy(q0=q0, qgoal=self.qgoal, obs_pos=obs_pos, obs_size=obs_size)
            self.initialize_armour()

        ############################## blender #############################
        self.blender_record = blender_record
        self.blender = PyBulletRecorder()
        self.blender_waypoints = PyBulletRecorder()
        # register robot
        self.blender.register_object(self.EnvId[0], self.path[0])
        self.blender_waypoints.register_object(self.EnvId[0], self.path[0])
        # register obstacles with correct scale
        for i in range(1, len(self.EnvId)):
            self.blender.register_object(self.EnvId[i], self.path[i], obs_size[i-1])


    def initialize_armour(self):
        self.planner_agent = bulletPlanner.ARMOUR(obs_pos=self.obs_pos, obs_size=self.obs_size, obs_ori=self.obs_ori)

    def initialize_zonopy(self, qpos, qgoal, obs_pos, obs_size):
        self.planner_agent = self.planner_agent(q0=qpos, qgoal=qgoal, obs_pos=obs_pos, obs_size=obs_size)

    def step(self, ka: np.ndarray, qpos=[], qvel=[]):
        """
        Step for 1 planning iteration (0.5 s) in pybullet
        """
        steps = int(0.5/self.timestep)

        if len(qpos) == 0 and len(qvel) == 0:
            qpos, qvel = self.get_joint_states()
            qacc = (qvel - self.qvel_sim) / self.timestep
            self.qacc_sim = qacc

        for i in range(steps):
            if self.planner_name == 'zonopy':
                # calculate desired trajectory: linear function
                qpos, qvel = self.get_joint_traj(qpos, qvel, ka)
                torque = self.inversedynamics(qpos, qvel, ka)
            elif self.planner_name == 'armour':
                # calculate desired trajectory: Bezier curve
                t = self.timestep * i
                qdes = self.planner_agent.get_des_traj(q0=qpos, qd0=qvel, qdd0=qacc, k=ka, t=t)
                torque = self.inversedynamics(qdes[:,0], qdes[:,1], qdes[:,2])
            self.torque_control(torque)
            p.stepSimulation()
            # time.sleep(self.timestep)
            # update states
            qpos_sim, qvel_sim = self.get_joint_states()
            self.qacc_sim = (qvel_sim - self.qvel_sim) / self.timestep
            self.qpos_sim = qpos_sim
            self.qvel_sim = qvel_sim
            if self.record:
                self.qpos_des_record = np.append(self.qpos_des_record, np.array([qdes[:,0]]), axis=0)
                self.qvel_des_record = np.append(self.qvel_des_record, np.array([qdes[:,1]]), axis=0)
                self.qacc_des_record = np.append(self.qacc_des_record, np.array([qdes[:,2]]), axis=0)
                self.qpos_record = np.append(self.qpos_record, np.array([qpos_sim]), axis=0)
                self.qvel_record = np.append(self.qvel_record, np.array([qvel_sim]), axis=0)
                self.qacc_record = np.append(self.qacc_record, np.array([self.qacc_sim]), axis=0)
            if self.blender_record and i % 20 == 0:
                self.blender.add_keyframe()

    def rrt(self, goal_pos=[]) -> Tuple[List[Node], bool]:
        """
        RRT algorithm that builds waypoints
        """
        start_pos, _ = self.get_joint_states()
        self.goal_pos = np.array(goal_pos)

        # build rrt tree with specified goal bias and step size
        goal_bias = 0.2
        step_size = 0.1
        rrt_builder = BuildRRT(self, start_pos, self.goal_pos, goal_bias, step_size)
        success = rrt_builder.build_tree()
        if success:
            rrt_builder.backtrace()
            rrt_builder.shortcut_smoothing()
            waypoints = rrt_builder.solution
            # waypoints = waypoints[0::2]
            self.plot_waypoints(waypoints)
            return waypoints, success
        else:
            return 0, False

    def plot_waypoints(self, waypoints: List[Node]):
        """
        Plot waypoints in the environment
        """
        for node in waypoints:
            joint_node = node.pos
            euclidean_node = self.forwardkinematics(joint_node)
            p.addUserDebugPoints(pointPositions=[euclidean_node[0:3].tolist()], pointColorsRGB=[[0.2, 0.2, 1.0]], pointSize=10)  
            self.blender_waypoints.add_keyframe()

        # return to the start position
        self.forwardkinematics(self.qpos_sim)

    def armtd_plan(self, goal: np.ndarray):
        """
        Plan for 1 planning iteration of ARMTD
        """
        if self.planner_name == 'zonopy':
            self.planner_agent.arm3d.qgoal = torch.tensor(goal, dtype=self.planner_agent.arm3d.dtype, device=self.planner_agent.arm3d.device)
            k, done = self.planner_agent.step()
            self.planner_agent.arm3d.render()
        elif self.planner_name == 'armour':
            # calculate the goal state of last plan
            qdes_last = self.planner_agent.get_des_traj(q0=self.q0[-1], qd0=self.qd0[-1], qdd0=self.qdd0[-1], k=self.k[-1], t=0.5)
            qf_last = qdes_last[:,0]
            qdf_last = qdes_last[:,1]
            qddf_last = qdes_last[:,2]
            # use the goal state of last plan to generate new goal
            k = self.planner_agent.plan(q0=qf_last, qd0=qdf_last, qdd0=qddf_last, goal=goal)
            self.k = np.append(self.k, np.array([k]), axis=0)
            self.q0 = np.append(self.q0, np.array([qf_last]), axis=0)
            self.qd0 = np.append(self.qd0, np.array([qdf_last]), axis=0)
            self.qdd0 = np.append(self.qdd0, np.array([qddf_last]), axis=0)
            # TODO: done
            done = False

        return k, done

    def _armtd_track(self, waypoints: List[Node]):
        """
        Tracking waypoints using armtd
        """
        for point in range(len(waypoints)):
            print("point: ", point)
            waypoint = waypoints[point]
            for _ in range(1):
                k, done = self.armtd_plan(waypoint.pos)
                self.step(k)
            # TODO: minimize goal position error
            if point == len(waypoints)-1:
                while np.linalg.norm(wrap_to_pi(self.qpos_sim) - wrap_to_pi(waypoint.pos)) > 0.2:
                    print(f"goal error: {np.linalg.norm(wrap_to_pi(self.qpos_sim) - wrap_to_pi(waypoint.pos))}")
                    k, done = self.armtd_plan(waypoint.pos)
                    self.step(k)
                
        return done

    def armtd_track_hardware(self, waypoint: Node):
        """
        Tracking a waypoint using armtd
        """

        # modify goal position as next waypoint
        self.planner_agent.arm3d.qgoal = torch.tensor(waypoint.pos, dtype=self.planner_agent.arm3d.dtype, device=self.planner_agent.arm3d.device)
        # modify robot state using hardware data;
        # plan and step zonopy environment
        qacc, done = self.planner_agent.step_hardware(self.qpos_hardware, self.qvel_hardware)
        if self.planner_agentGUI[0]:
            self.planner_agent.arm3d.render()
        # step pybullet environment using the previous plan
        if self.bulletGUI[0]:
            self.step(ka=self.planner_agent.ka_pre, qpos=self.qpos_hardware, qvel=self.qvel_hardware)
        
        # return new plan
        return done, qacc
            
    def simulate(self, goal_pos):
        """
        Simulate process of rrt + armtd
        """
        self.waypoints, success = self.rrt(goal_pos=goal_pos)
        del self.zonopy
        if success:
            self._armtd_track(self.waypoints)
        else:
            sys.exit("RRT failed to build tree.")


    def get_joint_limits(self, bodyId: int):
        lowerLimits, upperLimits, jointRanges, restPoses = [], [], [], []
        for i in range(self.numJoints):
            jointInfo = p.getJointInfo(bodyId, i)
            print(f'\n joint{i}: {jointInfo}')

            # only actuate revolute joint
            if jointInfo[3] > -1:
                self.actuation_index.append(jointInfo[0])

                ll, ul = jointInfo[8:10]
                jr = ul - ll

                # For simplicity, assume resting state == initial state
                rp = p.getJointState(bodyId, i)[0]

                lowerLimits.append(ll)
                upperLimits.append(ul)
                jointRanges.append(jr)
                restPoses.append(rp)

        return lowerLimits, upperLimits, jointRanges, restPoses
    
    def get_joint_states(self):
        """
        Return the joint state. \n
        return: [joint position, joint velocity]
        """
        Qpos = []
        Qvel = []
        for i in range(self.numJoints):
            jointInfo = p.getJointInfo(self.robotId, i)
            if jointInfo[3] > -1 and bytes('gripper', 'utf-8') not in jointInfo[1]:
                qpos, qvel, _, _ = p.getJointState(self.robotId, i)
                Qpos.append(qpos)
                Qvel.append(qvel)
        return np.array(Qpos), np.array(Qvel)

    def set_joint_states_hardware(self, qpos, qvel):
        self.qpos_hardware = torch.tensor(qpos, dtype=self.planner_agent.arm3d.dtype)
        self.qvel_hardware = torch.tensor(qvel, dtype=self.planner_agent.arm3d.dtype)

    def get_joint_traj(self, qpos_pre: np.array, qvel_pre: np.array, qacc_d: np.array) -> Tuple[np.array, np.array]:
        """
        Return the desired trajectory.
        """
        qpos = qpos_pre
        qvel = qvel_pre
        qpos_d = qpos + qvel * self.timestep + 0.5 * qacc_d * self.timestep**2
        qvel_d = qvel + qacc_d * self.timestep
        return qpos_d, qvel_d
    
    def inversekinematics(self, goal: np.array) -> np.array:
        pos = goal[0:3]
        if len(goal) > 3:
            orn = goal[3:6]
            orn_q = self.Euler2Quat(orn)
            for i in range(100):
                jointPoses = p.calculateInverseKinematics(self.robotId, self.EndEffectorIndex, pos, orn_q, self.ll, self.ul, self.jr, self.rp)
                self.forwardkinematics(jointPoses)
        else:
            jointPoses = p.calculateInverseKinematics(self.robotId, self.EndEffectorIndex, pos, self.ll, self.ul, self.jr, self.rp)
        return jointPoses

    def forwardkinematics(self, qpos: np.ndarray) -> np.ndarray:
        """
        Force set the joint positions of actuated joints. \n
        return: position and orientation of the end effector.
        """
        index = 0
        for i in range(self.numJoints):
            jointInfo = p.getJointInfo(self.robotId, i)
            # assign values to only revolute joints
            if jointInfo[3] > -1:
                p.resetJointState(self.robotId, i, qpos[index])
                index += 1
        ls = p.getLinkState(self.robotId, self.EndEffectorIndex)
        # end effector position in world frame
        pos = np.array(ls[0])
        # end effector orientation in its inertial frame
        ori = np.array(p.getEulerFromQuaternion(ls[1]))

        return np.append(pos, ori)
    
    def inversedynamics(self, qpos_des, qvel_des, qacc_des: np.array) -> np.array:
        """
        Calculate inverse dynamics. \n
        return: applied torque.
        """
        qpos, qvel = self.get_joint_states()
        qacc = qacc_des + self.Kd.dot(qvel_des - np.array(qvel)) + self.Kp.dot(qpos_des - np.array(qpos))
        torque = p.calculateInverseDynamics(self.robotId, qpos.tolist(), qvel.tolist(), qacc.tolist())
        return torque

    def torque_control(self, torque: np.array):
        """
        Control torque of the robot joints.
        """
        p.setJointMotorControlArray(self.robotId, self.actuation_index, p.TORQUE_CONTROL, forces=torque)

    def position_control(self, position: np.array):
        """
        Control position of the robot joints.
        """
        p.setJointMotorControlArray(self.robotId, self.actuation_index, p.POSITION_CONTROL, forces=position)
    
    def Euler2Quat(self, euler: np.array) -> np.array:
        quat = p.getQuaternionFromEuler(euler)
        return quat

    def load(self, filename: str, pos: list=[0, 0, 0], ori: list=[0, 0, 0], scale: list=[1.0], urdf_index: int=0, saveid: bool=True) -> Tuple[int, str]:
        """
        Loading an object from URDF. \n
        return: object id.
        """
        if len(scale) == 1:
            objId = p.loadURDF(filename, globalScaling=scale)
        else:
            filename_new = self.scale_urdf(filename, scale, urdf_index)
            objId = p.loadURDF(filename_new)
        
        if saveid:
            self.EnvId.append(objId)
            self.path.append(filename)
            self.scale.append([scale])
            
        ori = self.Euler2Quat(ori)
        p.resetBasePositionAndOrientation(objId, pos, ori)
        return objId, filename
    
    def scale_urdf(self, filename, scale, index) -> str:
        """
        Scale the urdf file by direct access. This will modify the file permanently.
        """

        assert len(scale) == 3, "Input scale should have length 3."
        scale_str = ' '.join(str(e) for e in scale)
        tree = ET.parse(filename)
        root = tree.getroot()

        root[0].find('visual').find('geometry')[0].set('scale', scale_str)
        root[0].find('collision').find('geometry')[0].set('size', scale_str)

        filename_new = filename+str(index)+'temp.urdf'
        tree.write(filename_new)

        return filename_new

    def set_obj_config(self, objId: int, objPos: np.array, objOri: np.array):
        """
        Force set the object configuration.
        """
        objOriQuat = self.Euler2Quat(objOri)
        p.resetBasePositionAndOrientation(objId, objPos, objOriQuat)
    
    def force_static_step(self, jointPos: np.array, objIds: list, objPos: list, objOri: list):
        """
        Changing the object position without stepping the environment.
        """
        self.forwardkinematics(jointPos)
        for i in range(len(objIds)):
            self.set_obj_config(objIds[i], objPos[i], objOri[i])

    def plot_zonotope(self, zonotope: np.array) -> list:
        """
        Plot the zonotope in the environment.
        """
        idx = []
        lineIds = []
        color = np.random.uniform(0, 1, 3)
        for i in range(np.size(zonotope, 0)):
            idx.append(i)
            point1 = zonotope[i]
            for point2 in np.delete(zonotope, idx, 0):
                lineId = p.addUserDebugLine(point1, point2, lineColorRGB=color, lineWidth=2)
                lineIds.append(lineId)
        return lineIds

    def create_visual(self, filename, scale: list=[1, 1, 1], pos: list=[0, 0, 0]):
        """
        Create a visual shape using a mesh file.
        """
        visualShapeId = p.createVisualShape(shapeType=p.GEOM_MESH,
                                    fileName=filename,
                                    flags=p.GEOM_FORCE_CONCAVE_TRIMESH,
                                    rgbaColor=[0.2, 0.2, 0.6, 0.2],
                                    visualFramePosition=[0, 0, 0],
                                    meshScale=scale)
        Id = p.createMultiBody(baseMass=0,
                    baseInertialFramePosition=[0, 0, 0],
                    baseVisualShapeIndex=visualShapeId,
                    # baseCollisionShapeIndex=collisionShapeId,
                    basePosition=pos,
                    useMaximalCoordinates=False)
        self.EnvId.append(Id)
        self.path.append(filename)
        return Id

    def dump_video(self, filename):
        self.blender.save(filename+'.pkl')
        self.blender_waypoints.save(filename+'_waypoints.pkl')
    
    def remove_body(self, bodyId):
        p.removeBody(bodyId)
        self.EnvId.remove(bodyId)

    def Disconnect(self):
        p.disconnect()
