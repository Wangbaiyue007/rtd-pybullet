from email.charset import QP
from typing import Tuple
import pybullet as p
import numpy as np
import pybullet_data

clid = p.connect(p.SHARED_MEMORY)

class bulletRtdEnv:

    def __init__(
        self, 
        urdf_path="../assets/fetch/fetch.urdf", 
        GUI=True, 
        timestep=0.001, 
        useGravity=True, 
        useRobot=True, 
        useTorqueControl=True,
        gain = 1000
        ):

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
            self.Kp[i, i] = gain*(1-0.15*i)
            self.Kd[i, i] = 1.3*(self.Kp[i, i]/2)**0.5

        self.path = [urdf_path]

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

    def load(self, filename: str, pos: list=[0, 0, 0], ori: list=[0, 0, 0], scale: float=1) -> Tuple[int, str]:
        """
        Loading an object from URDF. \n
        return: object id.
        """
        objId = p.loadURDF(filename, globalScaling=scale)
        self.EnvId.append(objId)
        self.path.append(filename)
        
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