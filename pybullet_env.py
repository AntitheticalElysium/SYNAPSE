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
        self.max_velocity = 15.0 # rad/s
        self.ray_length = 2.0 # meters

    def add_obstacle(self, urdf_path, position):
        p.loadURDF(urdf_path, position)

    def get_proximity_data(self):
        rover_pos, rover_orn = p.getBasePositionAndOrientation(self.rover_id)
        
        ray_start_z_offset = 0.08 
        ray_from_pos = [rover_pos[0], rover_pos[1], rover_pos[2] + ray_start_z_offset]
        
        # Base forward vec
        base_forward_vec = [self.ray_length, 0, 0] 
        # Angles for the 5 sensors in degrees
        angles_deg = [-45, -25, 0, 25, 45]
        
        # Get the rover's rotation matrix to orient the rays globally
        rover_rot_matrix = p.getMatrixFromQuaternion(rover_orn)
        
        ray_from = [ray_from_pos] * 5
        ray_to = []
        
        for angle in angles_deg:
            angle_rad = np.deg2rad(angle)
            sensor_rot_quat = p.getQuaternionFromEuler([0, 0, angle_rad])
            
            rotated_forward_vec = p.rotateVector(sensor_rot_quat, base_forward_vec)
            
            # Transform the rotated vector into world coordinates using the rover's orientation
            world_vec = np.dot(np.array(rover_rot_matrix).reshape(3,3), np.array(rotated_forward_vec))
            
            # Calculate the final ray endpoint
            ray_to_pos = [ray_from_pos[i] + world_vec[i] for i in range(3)]
            ray_to.append(ray_to_pos)
            
        results = p.rayTestBatch(ray_from, ray_to)

        # Extract hit fractions
        distances = np.array([res[2] for res in results])

        # Debugging: visualize the rays
        colors = [[1,0,0], [1,0.5,0], [0,1,0], [1,0.5,0], [1,0,0]] # Red, Orange, Green
        for i in range(5):
            p.addUserDebugLine(ray_from[i], ray_to[i], lineColorRGB=colors[i], lifeTime=0.1)

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