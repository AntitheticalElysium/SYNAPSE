# snn_controller.py
import nengo
import numpy as np

def create_snn_controller():
    net = nengo.Network(label="SNN Reactive Controller V5")
    
    with net:
        goal_input = nengo.Node(size_in=2, label="goal_vector")
        proximity_input = nengo.Node(size_in=5, label="proximity_sensors")
        
        # Represents the forward driving command from the goal
        goal_representation = nengo.Ensemble(n_neurons=200, dimensions=2)
        nengo.Connection(goal_input, goal_representation, transform=[[1, -1], [1, 1]])

        # Represents the danger level (1 - proximity) 
        danger_representation = nengo.Ensemble(n_neurons=500, dimensions=5, radius=1) # 5D since 5 sensors
        bias_node = nengo.Node(1)
        nengo.Connection(bias_node, danger_representation, transform=np.ones((5, 1)))
        nengo.Connection(proximity_input, danger_representation, transform=-1, synapse=0.01)

        # Represents the avoidance command (GATED SUBNETWORK: OFF BY DEFAULT)
        avoidance_maneuver = nengo.Ensemble(n_neurons=400, dimensions=2, radius=4,
                                           # High intercepts mean it needs strong input to activate
                                           intercepts=nengo.dists.Uniform(0.4, 0.9))
        
        # Connect the danger signals to the avoidance subnetwork.
        # Far left danger: gentle turn right
        nengo.Connection(danger_representation[0], avoidance_maneuver, transform=[[-2.0], [2.0]])
        # Mid left danger: turn right hard
        nengo.Connection(danger_representation[0], avoidance_maneuver, transform=[[-4.0], [4.0]])

        # Center danger: reverse/stop
        nengo.Connection(danger_representation[1], avoidance_maneuver, transform=[[-5.0], [-5.0]])

        # Mid right danger: turn left hard
        nengo.Connection(danger_representation[2], avoidance_maneuver, transform=[[4.0], [-4.0]])
        # Far right danger: gentle turn left
        nengo.Connection(danger_representation[3], avoidance_maneuver, transform=[[2.0], [-2.0]])

        # Motor command: sum of the forward goal and the avoidance maneuver.
        motor_output = nengo.Node(size_in=2, label="motor_output")
        # Add the forward drive signal
        nengo.Connection(goal_representation, motor_output, synapse=0.05)
        # Add the corrective avoidance signal
        nengo.Connection(avoidance_maneuver, motor_output, synapse=0.05)
        
    return net, {'goal': goal_input, 'prox': proximity_input}, {'motors': motor_output}