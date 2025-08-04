import time
from pybullet_env import PyBulletEnv

print("--- Starting Rover Control Test ---")
env = PyBulletEnv()

try:
    # Test 1
    print("Testing: Drive Forward")
    start_time = time.time()
    while time.time() - start_time < 10.0:
        # Command: Full speed forward for both wheels
        forward_command = [1.0, 1.0]
        env.step(forward_command)
        
        prox_data = env.get_proximity_data()
        print(f"Proximity: Left={prox_data[0]:.2f}, Center={prox_data[1]:.2f}, Right={prox_data[2]:.2f}")
        
        time.sleep(1./240.)

    # Test 2
    print("Testing: Turn Left")
    start_time = time.time()
    while time.time() - start_time < 10.0:
        # Command: Slow down left wheel, speed up right wheel
        turn_command = [0.2, 0.8]
        env.step(turn_command)
        time.sleep(1./240.)
        
    # Test 3
    print("Testing: Stop")
    start_time = time.time()
    while time.time() - start_time < 2.0:
        # Command: Zero velocity
        stop_command = [0.0, 0.0]
        env.step(stop_command)
        time.sleep(1./240.)

except Exception as e:
    print(f"An error occurred: {e}")

finally:
    print("--- Test Finished ---")
    env.close()