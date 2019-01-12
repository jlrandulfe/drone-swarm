#!/usr/bin/env python
import rospy
import numpy as np
import message_filters
import std_msgs.msg
import sys

#global variable
drone_number = 9999

def dist(vector):
    length = np.sqrt(pow(vector[0],2)+pow(vector[1],2))
    return length

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



def callback(Z1, E1):
    Pub = rospy.Publisher("controller/control_value", std_msgs.msg.Float64MultiArray)# publish data


    Z = multiarray2np(Z1)
    E = multiarray2np(E1)

    #kalman P gain
    Kp = 1.2
    #U0 = -(Z[1,0]/dist(Z[1,0]))*E[1,0] + (Z[1,2]/dist(Z[1,2]))*E[1,2]

    units = np.zeros([1,2])
    #temp = np.zeros([1,2])

    drone = drone_number
    for j in range(0, len(E)):
        if(np.any(E[drone-1,j] != 0)):
        #this is the row of the drone i am

            #add op the unit vector*e of all in this row
            if drone-1 > j:
                units = units - (Z[drone-1,j]/dist(Z[drone-1,j]))*E[drone-1,j]
            elif drone-1 < j:
                units = units + (Z[drone-1,j]/dist(Z[drone-1,j]))*E[drone-1,j]

    control_variable = np2multiarray(Kp*units)

    Pub.publish(control_variable)



def listener(drone_id):
    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    drone_number = drone_id
    rospy.init_node('controller'+str(drone_id), anonymous=False)

    #rospy.Subscriber("error_node", String, callback) #somehow subscribe to this with correct data type(figure this out later

    Z_sub = message_filters.Subscriber("pattern_generator/start",std_msgs.msg.Float64MultiArray) #somehow subscribe to this with correct data type(figure this out later
    #Z_star_sub = message_filters.Subscriber("kalman/state", std_msgs.msg.Float64MultiArray)#somehow subscribe to this with correct data type(figure this out later
    E_sub = message_filters.Subscriber("error_estimator/errors", std_msgs.msg.Float64MultiArray)#somehow subscribe to this with correct data type(figure this out later

    ts = message_filters.TimeSynchronizer([Z_sub, E_sub], 10)
    ts.registerCallback(callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    if(len(sys.argv) < 2):
        print("Usage: controller.py drone_id")
    else:
        listener(sys.argv[1])
