#!/usr/bin/env python3
"""
Error estimator top level module
"""
# Standard libraries
# Third-party libraries
import numpy as np
import rospy
import geometry_msgs.msg
import std_msgs.msg
# Local libraries


def np2multiarray(data):
    """
    Convert a 2D square numpy.array into a Float64MultiArray msg
    """
    n_drones = data.shape[0]
    # Define the 2 dimensions of the array.
    dim_1 = std_msgs.msg.MultiArrayDimension(label="drone_n", size=n_drones,
                                             stride=n_drones**2)
    dim_2 = std_msgs.msg.MultiArrayDimension(label="drone_n", size=n_drones,
                                             stride=n_drones)
    # Create the layout of the message, necessary for deserializing it.
    layout = std_msgs.msg.MultiArrayLayout()
    layout.dim.append(dim_1)
    layout.dim.append(dim_2)
    # Create the output message with the data and the created layout.
    message = std_msgs.msg.Float64MultiArray()
    message.layout = layout
    message.data = data.reshape(data.size).tolist()
    return message

def multiarray2np(data):
    """
    Convert a Float64MultiArray msg into a 2D square numpy.array
    """
    np_data = np.array(data.data)
    # Assuming that the output is a square matrix simplifies the problem.
    dim_size = int(np.sqrt(np_data.size))
    data_array = np_data.reshape([dim_size, dim_size])
    return data_array
