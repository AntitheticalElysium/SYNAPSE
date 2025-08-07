import nengo
import nengo_dl
import numpy as np
import time

from pybullet_env import PyBulletEnv
from snn_controller import create_snn_controller

env = PyBulletEnv()
snn_net, snn_inputs, snn_outputs = create_snn_controller()

with snn_net:
    def environment_process(t, motor_commands):
        # Apply motor commands from SNN to sim 
        env.step(motor_commands)
        proximity_data = env.get_proximity_data()
        return proximity_data
    
    # Takes 2 inputs (motor_commands) and has 3 outputs (proximity_data).
    sim_node = nengo.Node(environment_process, size_in=2, size_out=3)
    nengo.Connection(snn_outputs['motors'], sim_node, synapse=None)
    nengo.Connection(sim_node, snn_inputs['prox'], synapse=None)

    # Goal input for the SNN: go forwards but avoid obstacles
    goal_node = nengo.Node([3.0, 0.0])
    nengo.Connection(goal_node, snn_inputs['goal'])

    # Probe the network
    p_goal = nengo.Probe(snn_inputs['goal'])
    p_proximity = nengo.Probe(snn_inputs['prox'])
    p_motor_output = nengo.Probe(snn_outputs['motors'])

with nengo_dl.Simulator(snn_net, dt=0.001, seed=1) as sim:
    print("Starting SNN reactive control simulation...")

    # Add some obstacles for the rover to avoid
    env.add_obstacle("urdf/sphere_10cm.urdf", [2, 1, 0.05])
    env.add_obstacle("urdf/sphere_10cm.urdf", [3, -1, 0.05])
    env.add_obstacle("urdf/sphere_10cm.urdf", [4, 0, 0.05])
    
    try:
        sim.run(60.0)
    except KeyboardInterrupt:
        print("\nSimulation interrupted by user.")
    finally:
        print("--- Simulation Finished ---")
        env.close()