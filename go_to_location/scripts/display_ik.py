#!/usr/bin/env python3
import rospy
import rosservice
import manipulation_msgs.srv
import moveit_msgs.msg
import manipulation_msgs.msg
import actionlib
import random
import time
import sys


if __name__ == "__main__":
    rospy.init_node('display_ik')


    if len(sys.argv)<4:
        rospy.logerr("usage: rosrun go_to_location name_of_service group_name name_of_location")
        exit()

    service_name=sys.argv[1]
    group=sys.argv[2]
    location_name=sys.argv[3]


    state_pub = rospy.Publisher('/ik_solution',moveit_msgs.msg.DisplayRobotState,queue_size=10)
    service = '/'+service_name+'/get_location_ik'
    r = rospy.Rate(1) # 10hz
    try:
        ik_locations_srv = rospy.ServiceProxy(service, manipulation_msgs.srv.GetLocationIkSolution)
        req = manipulation_msgs.srv.GetLocationIkSolutionRequest()
        req.location_name = location_name
        req.group_name = group
        resp = ik_locations_srv(req)
        print(resp)

        ik_sol=moveit_msgs.msg.DisplayRobotState()
        ik_sol.state.joint_state.name=['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        ik_sol.state.joint_state.velocity= [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        ik_sol.state.joint_state.effort= [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        idx=0
        if len(resp.ik_solutions)==0:
            rospy.logerr("no ik solution")
            exit()
        while not rospy.is_shutdown():
            ik_sol.state.joint_state.position=resp.ik_solutions[idx].configuration
            state_pub.publish(ik_sol)
            print(ik_sol)
            r.sleep()
            idx+=1
            if (idx==len(resp.ik_solutions)):
                idx=0

    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
