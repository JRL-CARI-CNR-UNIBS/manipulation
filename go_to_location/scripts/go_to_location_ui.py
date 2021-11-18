#!/usr/bin/env python3
import rospy
from manipulation_msgs.srv import *
import manipulation_msgs.msg
import actionlib

if __name__ == "__main__":
    rospy.init_node('goto_ui')
    rospy.wait_for_service('/go_to_location_server/list_locations')
    while not rospy.is_shutdown():
        try:
            list_locations_srv = rospy.ServiceProxy('/go_to_location_server/list_locations', ListOfLocations)
            resp = list_locations_srv()
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
        print("select location: ")
        for idx in range(0,len(resp.locations)):
            print(idx,": ",resp.locations[idx])
        value = input("Please enter a string:\n")
        try:
            idx=int(value)
        except:
            print("you have to insert a number")
            continue
        if (idx<0 or idx>=len(resp.locations)):
            print("location index is out of bound")
            continue
        loc_name=resp.locations[idx]
        print('selected: ',idx, ' ',loc_name)

        client = actionlib.SimpleActionClient('/go_to_location_server/manipulator/go_to', manipulation_msgs.msg.GoToAction)
        client.wait_for_server()

        goal = manipulation_msgs.msg.GoToActionGoal()
        goal.goal.location_names.append(loc_name)
        client.send_goal(goal.goal)
        client.wait_for_result()
