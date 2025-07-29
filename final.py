import pybullet as p
import time
import math
import numpy as np
import random

# Establish connection with PyBullet
p.connect(p.GUI)
p.resetSimulation()
p.setGravity(0, 0, -10)
useRealTimeSim = 0
p.setRealTimeSimulation(useRealTimeSim)

# Load the plane and car models
track = p.loadURDF("data/plane/plane.urdf")
car_id = p.loadURDF("f10_racecar/racecar_differential.urdf", [0,0,0])

# Function to create random obstacles
def random_obstacles():
    np.random.seed()
    xy_position_float = np.random.rand(2)
    x_position_range = np.random.randint(1, 10)
    y_position_range = np.random.randint(1, 10)
    xy_position = [xy_position_float[0] + x_position_range, xy_position_float[1] + y_position_range]
    z_position = 0.1
    return np.append(xy_position, z_position)

# Load random obstacles
cube_list = ['cube_black', 'cube_green', 'cube']
for _ in range(10):
    cube_choice = random.choice(cube_list)
    cube_position = random_obstacles()
    p.loadURDF(f'data/{cube_choice}/marble_cube.urdf', cube_position)

# Constants for car control
TARGET_POSITION = np.array([11, 0])
MAX_FORCE = 200
MAX_VELOCITY = 50
STEERING_RANGE = [-0.6, 0.6]
wheels = [8, 15]
steering = [0, 2]

# Function to calculate the steering angle
def calculate_steering_angle(carPos, targetPos):
    angle_to_target = np.arctan2(targetPos[1] - carPos[1], targetPos[0] - carPos[0])
    carPos, carOrn = p.getBasePositionAndOrientation(car_id)
    carEuler = p.getEulerFromQuaternion(carOrn)
    angle_of_car = carEuler[2]
    steering_angle = angle_to_target - angle_of_car
    return np.clip(steering_angle, STEERING_RANGE[0], STEERING_RANGE[1])

# LIDAR setup
numRays = 100
rayFrom = []
rayTo = []
rayIds = []
rayLen = 8
rayStartLen = 0.25
lidarHeight = 0.2
rayHitColor = [1, 0, 0]
rayMissColor = [0, 1, 0]
hokuyo_joint = 4
for i in range(numRays):
    angle = 2. * math.pi * float(i) / numRays
    rayFrom.append([rayStartLen * math.sin(angle), rayStartLen * math.cos(angle), lidarHeight])
    rayTo.append([rayLen * math.sin(angle), rayLen * math.cos(angle), lidarHeight])
    rayIds.append(p.addUserDebugLine(rayFrom[i], rayTo[i], rayMissColor, parentObjectUniqueId=car_id,
                                     parentLinkIndex=hokuyo_joint))

# Main loop
while True:
    # LIDAR data processing
    results = p.rayTestBatch(rayFrom, rayTo, parentObjectUniqueId=car_id, parentLinkIndex=hokuyo_joint)
    for i in range(numRays):
        hitObjectUid = results[i][0]
        hitFraction = results[i][2]
        if hitFraction == 1.:
            p.addUserDebugLine(rayFrom[i], rayTo[i], rayMissColor, replaceItemUniqueId=rayIds[i], parentObjectUniqueId=car_id, parentLinkIndex=hokuyo_joint)
        else:
            hitPosition = results[i][3]
            p.addUserDebugLine(rayFrom[i], hitPosition, rayHitColor, replaceItemUniqueId=rayIds[i], parentObjectUniqueId=car_id, parentLinkIndex=hokuyo_joint)

    # Navigate and control the car
    carPos, _ = p.getBasePositionAndOrientation(car_id)
    carPos = np.array(carPos[:2])
    if np.linalg.norm(carPos - TARGET_POSITION) < 0.5:
        targetVelocity = 0
    else:
        targetVelocity = MAX_VELOCITY
    steeringAngle = calculate_steering_angle(carPos, TARGET_POSITION)

    for wheel in wheels:
        p.setJointMotorControl2(car_id, wheel, p.VELOCITY_CONTROL, targetVelocity=targetVelocity, force=MAX_FORCE)
    for steer in steering:
        p.setJointMotorControl2(car_id, steer, p.POSITION_CONTROL, targetPosition=-steeringAngle)

    if useRealTimeSim == 0:
        p.stepSimulation()

p.disconnect()
