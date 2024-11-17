import pybullet as p
import time
import pybullet_data
physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
log_id = p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, "log/pathSim.mp4")
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,0)
planeId = p.loadURDF("plane.urdf")
startPos = [0,0,1]
startOrientation = p.getQuaternionFromEuler([0,0,0])
#boxId = p.loadURDF("r2d2.urdf",startPos, startOrientation)
boxId = p.loadURDF("kuka_iiwa/model.urdf",startPos, startOrientation)
cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
print(cubePos,cubeOrn)
for i in range (480):
    p.stepSimulation()
    time.sleep(1./240.)
p.stopStateLogging(log_id)
cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
print(cubePos,cubeOrn)
p.disconnect()