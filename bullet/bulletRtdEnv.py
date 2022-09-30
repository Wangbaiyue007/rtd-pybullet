import sys
import time
from typing import Tuple
import pybullet as p
import numpy as np
import pybullet_data
# zonopy
import torch
from zonopy.environments.arm_3d import Arm_3D
from zonopy.optimize.armtd_3d import ARMTD_3D_planner

clid = p.connect(p.SHARED_MEMORY)

class bulletRtdEnv:

    def __init__(
        self, 
        urdf_path="../assets/fetch/fetch.urdf", 
        GUI=True, 
        timestep=0.001, 
        control_gain = 1000,
        useGravity=True, 
        useRobot=True, 
        useTorqueControl=True,
        useZonopy=False,
        q0 = [0]*7, 
        qgoal = [0]*7, 
        obs_pos = [[]],
        ):

        ### PyBullet ###
        # enable GUI or not
        if GUI:
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
            self.robotId = p.loadURDF(urdf_path, [0, 0, 0], useFixedBase=True)
            self.EnvId = [self.robotId]
        
            # create offset so that the base position is the origin
            # p.resetBasePositionAndOrientation(self.robotId, [0, 0, 0], [0, 0, 0, 1])

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

        self.path = [urdf_path]
        self.scale = [[1, 1, 1]]

        # initialize zonopy and its environment
        self.qgoal = qgoal
        if useZonopy:
            self.zonopy = self.Zonopy(q0=q0, qgoal=self.qgoal, obs_pos=obs_pos)
            for obs in obs_pos:
                self.load("../assets/objects/cube_small_zero.urdf", pos= obs, scale=0.1)

    def step(self, ka):
        """
        Step for 1 planning iteration (0.5 s) in pybullet
        """
        ka = ka.numpy()
        steps = int(0.5/self.timestep)
        qpos_d, qvel_d = self.get_joint_states()
        for i in range(steps):
            # calculate desired trajectory
            qpos_d, qvel_d = self.get_joint_traj(qpos_d, qvel_d, ka)
            # inverse dynamics controller
            torque = self.inversedynamics(qpos_d, qvel_d, ka)
            self.torque_control(torque)
            p.stepSimulation()
            time.sleep(self.timestep)

    def _rrt(self, goal_pos=[]):
        """
        RRT algorithm that builds waypoints.
        """
        from bullet.rrt import BuildRRT
        start_pos, _ = self.get_joint_states()
        self.goal_pos = np.array(goal_pos)

        # build rrt tree with specified goal bias and step size
        goal_bias = 0.2
        step_size = 0.05
        rrt_builder = BuildRRT(self, start_pos, self.goal_pos, goal_bias, step_size)
        success = rrt_builder.build_tree()
        breakpoint()
        if success:
            rrt_builder.backtrace()
            rrt_builder.shortcut_smoothing()
            waypoints = rrt_builder.solution
            return waypoints, success
        else:
            return 0, False

    def _armtd_track(self, waypoints):
        """
        Tracking waypoints using armtd
        """
        for point in range(len(waypoints)):
            print("point: ", point)
            waypoint = waypoints[point]
            self.zonopy.arm3d.qgoal = torch.tensor(waypoint.pos, dtype=self.zonopy.arm3d.dtype, device=self.zonopy.arm3d.device)
            qacc, done = self.zonopy.step()
            self.zonopy.arm3d.render()
            self.step(qacc)
            # minimize goal position error
            '''
            if point == len(waypoints) -1:
                print("\nApproaching goal...\n")
                index = 0
                self.qpos_e = np.zeros(7)
                while True:
                    info = self.local_attractor(self.goal_pos, index)
                    index += 1
                    collision = info['collision']
                    qpos, qvel = self._get_state()
                    if precise_goal and (self.goal_distance(qpos, self.goal_pos) < 0.02 and np.linalg.norm(qvel) < 0.005): 
                        success = True
                        break
                    if (not precise_goal) and (self.goal_distance(qpos, self.goal_pos) < 0.1 and np.linalg.norm(qvel) < 0.1):
                        success = True
                        break
            '''
                
        return done

    def _armtd_track_hardware(self, waypoints):
        """
        Tracking waypoints using armtd
        """
        for point in range(len(waypoints)):
            print("point: ", point)
            waypoint = waypoints[point]
            self.zonopy.arm3d.qgoal = torch.tensor(waypoint.pos, dtype=self.zonopy.arm3d.dtype, device=self.zonopy.arm3d.device)
            qacc, done = self.zonopy.step_hardware()
            self.zonopy.arm3d.render()
            self.step(qacc)
                
        return done
            
    def simulate(self, goal_pos):
        """
        Simulate process of rrt + armtd
        """
        waypoints, success = self._rrt(goal_pos=goal_pos)
        if success:
            self._armtd_track(waypoints)
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

    def forwardkinematics(self, qpos: np.array) -> np.array:
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

    def load(self, filename: str, pos: list=[0, 0, 0], ori: list=[0, 0, 0], scale: float=1, saveid: bool=True) -> Tuple[int, str]:
        """
        Loading an object from URDF. \n
        return: object id.
        """
        objId = p.loadURDF(filename, globalScaling=scale)
        if saveid:
            self.EnvId.append(objId)
            self.path.append(filename)
            self.scale.append([scale]*3)
            
        ori = self.Euler2Quat(ori)
        p.resetBasePositionAndOrientation(objId, pos, ori)
        return objId, filename

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
    
    def remove_body(self, bodyId):
        p.removeBody(bodyId)
        self.EnvId.remove(bodyId)

    def Disconnect(self):
        p.disconnect()

    class Zonopy:

        def __init__(self, q0 = [0]*7, qgoal = [0]*7, obs_pos = [[]], dtype = torch.float, device = 'cuda:0'):
            self.arm3d = Arm_3D(robot="Kinova3", n_obs=len(obs_pos), FO_render_freq=25)
            q = torch.tensor(q0, dtype=dtype, device=device)
            qd = torch.zeros(self.arm3d.n_links, dtype=dtype, device=device)
            qgoal = torch.tensor(qgoal, dtype=dtype, device=device)
            obs_pos = torch.tensor(obs_pos, dtype=dtype, device=device)
            self.arm3d.set_initial(q, qd, qgoal, obs_pos)
            self.planner = ARMTD_3D_planner(self.arm3d, dtype=dtype, device=device)
            self.ka_0 = torch.zeros(self.arm3d.dof)
            self.ka = torch.zeros(self.arm3d.dof)

        def step(self):
            print(f"Iteration {iter}")
            ka, flag = self.planner.plan(self.arm3d, self.ka_0)
            ka_break = (-self.arm3d.qvel) / 0.5

            print(f"qvel_prestep: {self.arm3d.qvel}")
            observations, reward, done, info = self.arm3d.step(ka.cpu(), flag)
            print(f"qvel_poststep: {self.arm3d.qvel}")

            safe = self.arm3d.safe
            if safe:
                print("--safe move--")
                qacc = ka.cpu()
            else:
                print("--safe break--")
                qacc = ka_break.cpu()
            print(f"qacc: {qacc}")

            return qacc, done

        def step_hardware(self):
            print(f"Iteration {iter}")
            ka, flag = self.planner.plan_hardware(self.arm3d, self.ka_0, self.ka)
            ka_break = (-self.arm3d.qvel) / 0.5

            print(f"qvel_prestep: {self.arm3d.qvel}")
            observations, reward, done, info = self.arm3d.step(self.ka.cpu(), flag)
            print(f"qvel_poststep: {self.arm3d.qvel}")

            safe = self.arm3d.safe
            if safe:
                print("--safe move--")
                qacc = ka.cpu()
            else:
                print("--safe break--")
                qacc = ka_break.cpu()
            print(f"qacc: {qacc}")
            self.ka = qacc

            return qacc, done