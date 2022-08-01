# A simple experiment to test the penetration behavior in pybullet
from bullet.bulletRtdEnv import bulletRtdEnv
import numpy as np
import pybullet as pbt
import matplotlib.pyplot as plt
import math as m
import time

timestep = 0.001
fetch_env = bulletRtdEnv(timestep=timestep, useGravity=True, useRobot=False)
fetch_env.load("../assets/objects/cube_small.urdf", [0, -0.5, 1.0 + 0.05*m.sqrt(2)], [0.463, 0, 0])
fetch_env.load("../assets/objects/cube_small.urdf", [0, 0.5, 0.99 + 0.05*m.sqrt(2)], [0.463, 0, 0])
fetch_env.load("../assets/objects/tray.urdf", [0, -0.5, 1], [0.463, 0, 0])
fetch_env.load("../assets/objects/tray.urdf", [0, 0.5, 1], [0.463, 0, 0])

t = 50
# breakpoint()
for i in range(int(t/timestep)):
    pbt.stepSimulation()
    time.sleep(timestep)

breakpoint()
fetch_env.Disconnect()