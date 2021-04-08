/*
Copyright (c) 2019, Manuel Beschi CNR-STIIMA manuel.beschi@stiima.cnr.it
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the <organization> nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <eigen_conversions/eigen_msg.h>
#include <actionlib/client/simple_action_client.h>
#include <manipulation_msgs/PlaceObjectsAction.h>
#include <manipulation_msgs/RemoveObjectFromSlot.h>
#include <manipulation_utils/manipulation_load_params_utils.h>


int main(int argc, char **argv)
{
  std::string object_name;
  if( argc >= 2 && argc <= 4) 
  {
    object_name = argv[1];    
  }
  else
  {
    ROS_ERROR("Wrong number of input, the input needed are: object_name.");
    return -1;
  }
  
  ros::init(argc, argv, "outbound_place_client");
  ros::NodeHandle nh;

  actionlib::SimpleActionClient<manipulation_msgs::PlaceObjectsAction> place_ac("/outbound_place_server/manipulator/place");
  ROS_INFO("Waiting for place server");
  place_ac.waitForServer();
  ROS_INFO("Connection ok");

  manipulation_msgs::PlaceObjectsGoal place_goal;
  place_goal.object_name = object_name;
  //place_goal.slot_names.push_back("A1");
  place_goal.slot_names.push_back("A3");
  //place_goal.slot_names.push_back("B1");

  place_goal.tool_id = "fake_gripper";
  place_goal.property_execution_id = "open";

  place_ac.sendGoalAndWait(place_goal);

  if (place_ac.getResult()->result >=0)
  {
    ros::ServiceClient remove_object_from_slot_clnt = nh.serviceClient<manipulation_msgs::RemoveObjectFromSlot>("/outbound_place_server/remove_obj_from_slot");

    manipulation_msgs::RemoveObjectFromSlot remove_object_from_slot;
    remove_object_from_slot.request.object_to_remove_name = object_name;
    remove_object_from_slot.request.slot_name = "A3";

    if (!remove_object_from_slot_clnt.call(remove_object_from_slot))
    {
      ROS_ERROR("Unaspected error calling %s service",remove_object_from_slot_clnt.getService().c_str());
      return false;
    }
  }
  else
  { 
    ROS_ERROR("Place action not successfull.");
    return -1;
  }

  ROS_INFO("Place client stopped");
  return 0;
}
