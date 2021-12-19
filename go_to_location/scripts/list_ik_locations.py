#!/usr/bin/env python3
import rospy
from manipulation_msgs.srv import *
import manipulation_msgs.msg
import actionlib
import random
import time
if __name__ == "__main__":
    rospy.init_node('list_ik_location')
    rospy.wait_for_service('/go_to_location_server/list_locations')

    try:
        list_locations_srv = rospy.ServiceProxy('/go_to_location_server/list_locations', ListOfLocations)
        resp = list_locations_srv()
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

    print(resp.resume)
