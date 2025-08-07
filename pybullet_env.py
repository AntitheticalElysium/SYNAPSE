import pybullet as p
import pybullet_data
import numpy as np
import time

class PyBulletEnv:
    def __init__(self):
        # Connect to server
        self.client = p.connect(p.GUI)
        p.setGravity(0, 0, -9.81)
        p.setRealTimeSimulation(0) # Manual stepping

        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.plane_id = p.loadURDF("plane.urdf")
        self.start_pos = [0, 0, 0.1]
        self.rover_id = p.loadURDF("urdf/rover.urdf", self.start_pos)

        self.motorized_joints = []
        self.joint_names = []
        for i in range(p.getNumJoints(self.rover_id)):
            info = p.getJointInfo(self.rover_id, i)
            joint_name = info[1].decode('utf-8')
            self.joint_names.append(joint_name)
            if joint_name in ["chassis_to_left_rear_wheel", "chassis_to_right_rear_wheel"]:
                self.motorized_joints.append(i)

        print(f"Motorized joints: {self.motorized_joints}")

        # Environment setup
        self.max_velocity = 5.0 # rad/s
        self.ray_length = 2.0 # meters

    def add_obstacle(self, urdf_path, position):
        p.loadURDF(urdf_path, position)

    def get_proximity_data(self):
        rover_pos, rover_orn = p.getBasePositionAndOrientation(self.rover_id)
        
        rot_matrix = p.getMatrixFromQuaternion(rover_orn)
        # Forward vector is the first column of the rotation matrix
        forward_vec = [rot_matrix[0], rot_matrix[3], rot_matrix[6]]
        # Right vector is the second column
        right_vec = [rot_matrix[1], rot_matrix[4], rot_matrix[7]]

        # Define ray start and end points
        ray_from = [rover_pos] * 3
        
        center_end = [rover_pos[i] + forward_vec[i] * self.ray_length for i in range(3)]
        left_offset = [right_vec[i] * -0.15 for i in range(3)] # Offset to the left
        right_offset = [right_vec[i] * 0.15 for i in range(3)] # Offset to the right
        left_end = [center_end[i] + left_offset[i] for i in range(3)]
        right_end = [center_end[i] + right_offset[i] for i in range(3)]

        ray_to = [left_end, center_end, right_end]
        
        # Perform ray tests in a batch
        results = p.rayTestBatch(ray_from, ray_to)

        distances = np.array([res[2] for res in results])

        p.addUserDebugLine(ray_from[0], ray_to[0], lineColorRGB=[1,0,0], lifeTime=0.1)
        p.addUserDebugLine(ray_from[1], ray_to[1], lineColorRGB=[0,1,0], lifeTime=0.1)
        p.addUserDebugLine(ray_from[2], ray_to[2], lineColorRGB=[0,0,1], lifeTime=0.1)

        return distances

    def step(self, motor_commands):
        motor_commands = np.array(motor_commands)
        
        # Set velocity for each motorized joint
        p.setJointMotorControlArray(
            bodyUniqueId=self.rover_id,
            jointIndices=self.motorized_joints,
            controlMode=p.VELOCITY_CONTROL,
            targetVelocities=motor_commands * self.max_velocity
        )
        
        p.stepSimulation()

    def close(self):
        p.disconnect(self.client)