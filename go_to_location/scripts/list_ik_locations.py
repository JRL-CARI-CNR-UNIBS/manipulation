#!/usr/bin/env python3
import rospy
import rosservice
from manipulation_msgs.srv import *
import manipulation_msgs.msg
import actionlib
import random
import time
import sys


if __name__ == "__main__":
    rospy.init_node('list_ik_location')

    service_list = rosservice.get_service_list()
    for service in service_list:
        if "list_locations" in service:
            print("***********************************")
            print(service)
            print("***********************************")
            rospy.wait_for_service(service)

            try:
                list_locations_srv = rospy.ServiceProxy(service, ListOfLocations)
                resp = list_locations_srv()
            except rospy.ServiceException as e:
                print("Service call failed: %s"%e)

            print(resp.resume)
