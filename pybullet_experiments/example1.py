import pybullet as p
from time import sleep
import pybullet_data


p.connect(p.GUI)  # or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
p.setGravity(0, 0, -10)
plane_id = p.loadURDF("plane.urdf")
start_pos = [0, 0, 1]
start_orientation = p.getQuaternionFromEuler([0, 0, 0])

box_id = p.loadURDF("r2d2.urdf", start_pos, start_orientation)

# set the center of mass frame (loadURDF sets base link frame)
# startPos/Ornp.resetBasePositionAndOrientation(boxId, startPos, startOrientation)
for i in range(10000):
    p.stepSimulation()
    if i % 100 == 0:
        pos, orn = p.getBasePositionAndOrientation(box_id)
        print(i, (pos, orn))
    sleep(1./240.)

p.disconnect()
