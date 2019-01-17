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


def np2multiarray(array, label1="drone_n", label2="coords", offset=0.0):
    """
    Convert a 2D numpy.array into a Float64MultiArray msg
    """
    # Copy the input data to avoid corruption of the input data.
    data = array.copy()
    size_1 = data.shape[0]
    size_2 = data.shape[1]
    # Define the 2 dimensions of the array.
    dim_1 = std_msgs.msg.MultiArrayDimension(label=label1, size=size_1,
                                             stride=size_1*size_2)
    dim_2 = std_msgs.msg.MultiArrayDimension(label=label2, size=size_2,
                                             stride=size_2)
    # Create the layout of the message, necessary for deserializing it.
    layout = std_msgs.msg.MultiArrayLayout()
    layout.dim.append(dim_1)
    layout.dim.append(dim_2)
    # Add the offset value. Normally for indicating the system error.
    layout.data_offset = offset
    # Create the output message with the data and the created layout.
    message = std_msgs.msg.Float64MultiArray()
    message.layout = layout
    message.data = data.reshape(data.size).tolist()
    return message

def np2multiarray_3D(array, n_drones=3, coords=2):
    # Copy the input data to avoid corruption of the input data.
    data = array.copy()
    # Define the 3 dimensions of the array.
    dim_1 = std_msgs.msg.MultiArrayDimension(label="drone_n", size=n_drones,
                                             stride=n_drones*n_drones*coords)
    dim_2 = std_msgs.msg.MultiArrayDimension(label="drone_n", size=n_drones,
                                             stride=n_drones*coords)
    dim_3 = std_msgs.msg.MultiArrayDimension(label="coords", size=coords,
                                             stride=coords)
    # Create the output message with the data and the created layout.
    message = std_msgs.msg.Float64MultiArray()
    message.layout.dim.append(dim_1)
    message.layout.dim.append(dim_2)
    message.layout.dim.append(dim_3)
    message.data = data.reshape(data.size).tolist()
    return message

def multiarray2np_sqr(data):
    """
    Convert a squared Float64MultiArray msg into a 2D square numpy.array
    """
    np_data = np.array(data.data)
    # Assuming that the output is a square matrix simplifies the problem.
    dim_size = int(np.sqrt(np_data.size))
    data_array = np_data.reshape([dim_size, dim_size])
    return data_array

def multiarray2np(data):
    np_data = np.array(data.data)
    sizes = []
    for _, dim in enumerate(data.layout.dim):
        sizes.append(dim.size)
    # If input matrix is 2D
    if len(sizes) == 2:
        data_array = np_data.reshape([sizes[0], sizes[1]])
    # If input matrix is 3D
    if len(sizes) == 3:
        data_array = np_data.reshape([sizes[0], sizes[1], sizes[2]])
    return data_array
