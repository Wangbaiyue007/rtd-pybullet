from typing import Tuple
import pybullet as p
import numpy as np
import pybullet_data

clid = p.connect(p.SHARED_MEMORY)


class KinematicsEnv:

    def __init__(self, urdf_path="./assets/fetch/fetch.urdf", GUI=True, timestep=0.001, useGravity=True, useRobot=True):
        # enable GUI or not
        if GUI:
            if (clid < 0): p.connect(p.GUI)
        else:
            if (clid < 0): p.connect(p.DIRECT)

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
            self.EndEffectorIndex = 17
            self.numJoints = p.getNumJoints(self.robotId)

            # set robot joint limits and rest positions
            self.ll, self.ul, self.jr, self.rp = self.get_joint_limits(self.robotId)
            self.forwardkinematics(self.rp)

        self.path = [urdf_path]

    def get_joint_limits(self, bodyId: int):
        lowerLimits, upperLimits, jointRanges, restPoses = [], [], [], []
        for i in range(self.numJoints):
            jointInfo = p.getJointInfo(bodyId, i)
            print(f'\n joint{i}: {jointInfo}')

            # avoid fixed joints
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
    
    def inversekinematics(self, goal: np.array) -> np.array:
        pos = goal[0:3]
        if len(goal) > 3:
            orn = goal[3:6]
            orn_q = self.Euler2Quat(orn)
            for i in range(100):
                # jointPoses = p.calculateInverseKinematics(self.robotId, self.EndEffectorIndex, pos, orn_q, solver=0,
                #                                       maxNumIterations=100, residualThreshold=.01)
                jointPoses = p.calculateInverseKinematics(self.robotId, self.EndEffectorIndex, pos, orn_q, self.ll, self.ul, self.jr, self.rp)
                # self.jointPoses = p.calculateInverseKinematics(self.robotId, self.EndEffectorIndex, pos, orn_q)
                self.forwardkinematics(jointPoses)
        else:
            jointPoses = p.calculateInverseKinematics(self.robotId, self.EndEffectorIndex, pos, self.ll, self.ul, self.jr, self.rp)
            # self.jointPoses = p.calculateInverseKinematics(self.robotId, self.EndEffectorIndex, pos)
        return jointPoses

    def forwardkinematics(self, qpos: np.array) -> np.array:
        """
        Force set the joint positions of actuated joints. \n
        return: position and orientation of the end effector.
        """
        index = 0
        for i in range(self.numJoints):
            jointInfo = p.getJointInfo(self.robotId, i)
            qIndex = jointInfo[3]
            if qIndex > -1:
                p.resetJointState(self.robotId, i, qpos[index])
                index += 1
        ls = p.getLinkState(self.robotId, self.EndEffectorIndex)
        # end effector position in world frame
        pos = np.array(ls[0])
        # end effector orientation in its inertial frame
        ori = np.array(p.getEulerFromQuaternion(ls[1]))
        return np.append(pos, ori)
    
    def Euler2Quat(self, euler: np.array) -> np.array:
        quat = p.getQuaternionFromEuler(euler)
        return quat

    def load(self, filename: str, pos:list=[0, 0, 0], ori:list=[0, 0, 0]) -> Tuple[int, str]:
        """
        Loading an object from URDF. \n
        return: object id.
        """
        objId = p.loadURDF(filename)
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

    def Disconnect(self):
        p.disconnect()