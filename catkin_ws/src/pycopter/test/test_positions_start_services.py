#!/usr/bin/env python3
"""
Module for testing the service client functionality of pycopter
"""
# Standard libraries
# Third-party libraries
import rospy
import std_msgs.msg
# Local libraries
from pycopter.srv import DroneSwarmMultiArray
from pycopter.srv import DroneSwarmMultiArrayResponse
from pycopter.srv import PycopterStartStop

start = False
ndrones = 3

def handle_start_positions(req):
    resp = DroneSwarmMultiArrayResponse()
    if ndrones == 2:
        resp.n_rows = 2
        resp.data = [1, 2, 0, 1]
    elif ndrones == 3:
        resp.n_rows = 3
        resp.data = [1, 2, 1, 3, 2, 3] 
    resp.param1 = 40 # seconds
    resp.param2 = 50 # milliseconds
    rospy.loginfo("Response sent back")
    global start
    start = True
    return resp

def main():
    # Instantiate the error_estimator node class and run it
    rospy.Service('supervisor/pycopter', DroneSwarmMultiArray,
                  handle_start_positions)
    rospy.loginfo("The service is ready")
    while not start:
        pass
    rospy.wait_for_service("pycopter/start_stop")
    try:
        start_pycopter = rospy.ServiceProxy("pycopter/start_stop",
                                            PycopterStartStop)
        resp = start_pycopter(start=True, stop=False)
        ack = resp.ack
    except rospy.ServiceException as e:
        print("Service call failed: {}".format(e))
        return -1
    if ack:
        rospy.loginfo("Start command acknowledged")
    else:
        rospy.logwarn("Start command rejected")
    return

if __name__ == "__main__":
    rospy.init_node("test_positions_services")
    main()
