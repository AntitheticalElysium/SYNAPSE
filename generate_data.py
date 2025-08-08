# generate_data.py
import os
import cv2
import keyboard
from pybullet_env import PyBulletEnv

os.makedirs("data/novel", exist_ok=True)
os.makedirs("data/boring", exist_ok=True)

print("--- Starting Data Generation ---")
print("Drive the rover with arrow keys.")
print("Press 's' to save a 'novel' image.")
print("Press 'b' to save a 'boring' image.")
print("Press 'q' to quit.")

env = PyBulletEnv()

# Add some "novel" objects
env.add_obstacle("urdf/sphere_10cm.urdf", [3, 2, 0.05])
env.add_obstacle("urdf/sphere_10cm.urdf", [4, -3, 0.05])

novel_count = len(os.listdir("data/novel"))
boring_count = len(os.listdir("data/boring"))

try:
    while True:
        keys = keyboard.read_event(suppress=True)
        if keys.event_type == keyboard.KEY_DOWN:
            left_vel, right_vel = 0.0, 0.0
            if keys.name == 'up':
                left_vel, right_vel = 1.0, 1.0
            elif keys.name == 'down':
                left_vel, right_vel = -1.0, -1.0
            elif keys.name == 'left':
                left_vel, right_vel = -0.5, 0.5
            elif keys.name == 'right':
                left_vel, right_vel = 0.5, -0.5
            elif keys.name == 's':
                img = env.get_camera_image()
                filename = f"data/novel/img_{novel_count:04d}.png"
                cv2.imwrite(filename, cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
                print(f"Saved {filename}")
                novel_count += 1
            elif keys.name == 'b':
                img = env.get_camera_image()
                filename = f"data/boring/img_{boring_count:04d}.png"
                cv2.imwrite(filename, cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
                print(f"Saved {filename}")
                boring_count += 1
            elif keys.name == 'q':
                print("Quitting.")
                break
            
            env.step([left_vel, right_vel])

finally:
    env.close()