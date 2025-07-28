import pybullet as p
import pybullet_data
import numpy as np
import time

class PyBulletEnv:
    def __init__(self):
        p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.loadURDF("plane.urdf")
        
        # Load rover URDF
        self.rover_id = p.loadURDF("urdf/rover.urdf", [0, 0, 0.2])

        # Identify motorized wheel joints
        self.wheel_indices = []
        num_joints = p.getNumJoints(self.rover_id)
        for i in range(num_joints):
            joint_name = p.getJointInfo(self.rover_id, i)[1].decode('utf-8')
            if joint_name in ["back_left_wheel_joint", "back_right_wheel_joint"]:
                self.wheel_indices.append(i)

        # Set wheel friction
        for i in range(num_joints):
            p.changeDynamics(self.rover_id, i, lateralFriction=2.0, rollingFriction=0.01, spinningFriction=0.01)

    def set_motor_speeds(self, speeds):
        for i, speed in zip(self.wheel_indices, speeds):
            p.setJointMotorControl2(
                bodyUniqueId=self.rover_id,
                jointIndex=i,
                controlMode=p.VELOCITY_CONTROL,
                targetVelocity=speed,
                force=50.0
            )

    def get_proximity_data(self):
        origin = [0.5, 0, 0.2]  # Front-center of the chassis
        ray_length = 5.0
        angles = [np.pi / 6, 0, -np.pi / 6]  # +30, 0, -30 degrees
        rays = []

        for a in angles:
            dx = ray_length * np.cos(a)
            dy = ray_length * np.sin(a)
            start = origin
            end = [origin[0] + dx, origin[1] + dy, origin[2]]
            rays.append((start, end))

        results = p.rayTestBatch([r[0] for r in rays], [r[1] for r in rays])
        hit_fractions = np.array([r[2] for r in results])  # 1.0 = no hit
        return hit_fractions

    def get_camera_image(self):
        cam_target = [1, 0, 0.2]
        cam_pos = [0.4, 0, 0.35]
        view_matrix = p.computeViewMatrix(
            cameraEyePosition=cam_pos,
            cameraTargetPosition=cam_target,
            cameraUpVector=[0, 0, 1]
        )
        proj_matrix = p.computeProjectionMatrixFOV(
            fov=60,
            aspect=1.0,
            nearVal=0.1,
            farVal=10.0
        )
        width, height, rgbImg, _, _ = p.getCameraImage(
            width=128, height=128, viewMatrix=view_matrix, projectionMatrix=proj_matrix
        )
        img = np.reshape(rgbImg, (128, 128, 4))[:, :, :3]
        return img

# --- TEST BLOCK ---
if __name__ == "__main__":
    import pygame
    from pygame.locals import *

    pygame.init()
    screen = pygame.display.set_mode((300, 100))
    pygame.display.set_caption("Rover Keyboard Controller")

    env = PyBulletEnv()

    running = True
    while running:
        keys = pygame.key.get_pressed()

        left_speed, right_speed = 0, 0
        if keys[K_w]:
            left_speed, right_speed = 5.0, 5.0
        elif keys[K_s]:
            left_speed, right_speed = -5.0, -5.0
        elif keys[K_a]:
            left_speed, right_speed = -2.0, 2.0
        elif keys[K_d]:
            left_speed, right_speed = 2.0, -2.0

        env.set_motor_speeds([left_speed, right_speed])
        p.stepSimulation()

        prox = env.get_proximity_data()
        print("Proximity:", prox)

        for event in pygame.event.get():
            if event.type == QUIT:
                running = False

        time.sleep(1./240.)

    pygame.quit()
    p.disconnect()

