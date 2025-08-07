import nengo 
import numpy as np

def create_snn_controller(model, env):
    net = nengo.Network(label='SNN Reactive Controller')
    with net:
        # 2D vect [drive_speed, steer_direction]
        goal_input = nengo.Node(size_in=2, label='goal_vector')
        # 3D vect [left_distance, center_distance, right_distance]
        proximity_input = nengo.Node(size_in=3, label='proximity_vector')

        # Invert proximity signals to represent danger
        danger_sensors = nengo.Ensemble(n_neurons=150, dimensions=3)
        nengo.Connection(proximity_input, danger_sensors, transform=-1, synapse=0.01)
        nengo.Connection(danger_sensors, danger_sensors, transform=1) # Bias 

        motor_neurons = nengo.Ensemble(n_neurons=400, dimensions=2, neuron_type=nengo.SpikingRectifiedLinear, label='motor_neurons')
        motor_output = nengo.Node(size_in=2, label='motor_output')
        nengo.Connection(motor_neurons, motor_output, synapse=0.05)

        # The transform maps [drive, steer] to [left_wheel, right_wheel].
        # If steer is positive (right), left wheel is faster.
        # If steer is negative (left), right wheel is faster.
        nengo.Connection(goal_input, motor_neurons, transform=[[1, -1], [1, 1]], synapse=0.05)

        # If danger on the LEFT (danger_sensors[0]), turn RIGHT
        nengo.Connection(danger_sensors[0], motor_neurons, transform=[[-2.5], [2.5]], synapse=0.05)
        # If danger on the RIGHT (danger_sensors[2]), turn LEFT
        nengo.Connection(danger_sensors[2], motor_neurons, transform=[[2.5], [-2.5]], synapse=0.05)
        # If danger in the CENTER (danger_sensors[1]), REVERSE
        nengo.Connection(danger_sensors[1], motor_neurons, transform=[[-3.0], [-3.0]], synapse=0.05)


    return net, {'goal': goal_input, 'prox': proximity_input}, {'motors': motor_output}