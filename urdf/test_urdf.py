import pybullet as p
import pybullet_data
import time

physics_client = p.connect(p.GUI)
p.setGravity(0, 0, -9.81)

p.setAdditionalSearchPath(pybullet_data.getDataPath())

plane_id = p.loadURDF("plane.urdf")
start_pos = [0, 0, 0.1]
start_orientation = p.getQuaternionFromEuler([0, 0, 0])
rover_id = p.loadURDF("urdf/rover.urdf", start_pos, start_orientation)

num_joints = p.getNumJoints(rover_id)
print(f"Number of joints in the rover: {num_joints}")
for i in range(num_joints):
    print(p.getJointInfo(rover_id, i))

print("Simulation running. Press Ctrl+C to stop.")
try:
    while True:
        p.stepSimulation()
        time.sleep(1./240.)
except KeyboardInterrupt:
    print("Simulation stopped by user.")    
    p.disconnect()