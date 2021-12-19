#!/usr/bin/env python3
import rospy
from manipulation_msgs.srv import *
import manipulation_msgs.msg
import actionlib
import random
import time
if __name__ == "__main__":
    rospy.init_node('goto_ui')
    rospy.wait_for_service('/go_to_location_server/list_locations')

    length =0;
    expected_execution_duration = 0
    actual_duration = 0
    planning_duration = 0

    n_planning=0
    n_fails=0
    while not rospy.is_shutdown():
        try:
            list_locations_srv = rospy.ServiceProxy('/go_to_location_server/list_locations', ListOfLocations)
            resp = list_locations_srv()
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

        idx=random.randint(0, len(resp.locations)-1)

        loc_name=resp.locations[idx]
        print('selected: ',idx, ' ',loc_name)

        client = actionlib.SimpleActionClient('/go_to_location_server/manipulator/go_to', manipulation_msgs.msg.GoToAction)
        client.wait_for_server()

        goal = manipulation_msgs.msg.GoToActionGoal()
        goal.goal.location_names.append(loc_name)
        client.send_goal(goal.goal)
        client.wait_for_result()
        res = client.get_result()

        if (res.result<0):
            rospy.logerr(res)
            n_fails+=1
            break
        else:
            length=(res.path_length+length*n_planning)/(n_planning+1.0)
            expected_execution_duration = (res.expected_execution_duration.to_sec()+expected_execution_duration*n_planning)/(n_planning+1.0)
            actual_duration = (res.actual_duration.to_sec()+actual_duration*n_planning)/(n_planning+1.0)
            planning_duration = (res.planning_duration.to_sec()+planning_duration*n_planning)/(n_planning+1.0)
            n_planning+=1
            rospy.loginfo("mean length                      = "+str(length))
            rospy.loginfo("mean expected_execution_duration = "+str(expected_execution_duration))
            rospy.loginfo("mean actual_duration             = "+str(actual_duration))
            rospy.loginfo("mean planning_duration           = "+str(planning_duration))
            rospy.loginfo("successful planning              = "+str(n_planning))
            rospy.loginfo("failed planning                  = "+str(n_fails))
        time.sleep(1)
