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
#include <manipulation_msgs/GoToAction.h>
#include <manipulation_utils/manipulation_load_params_utils.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "go_to_location_client");
  ros::NodeHandle nh;

  std::vector<std::string> group_names;
  if (!nh.getParam("/go_to_location_server/move_group_names",group_names))
  {
    ROS_ERROR("Unable to find /go_to_location_server/move_group_names");
    return false;
  }

  // N.B. As example it is supposed to have a single move_group, in case of multiple move groups
  // it is necessary to create multiple GoToAction client, one for each move group
  actionlib::SimpleActionClient<manipulation_msgs::GoToAction> goto_ac("/go_to_location_server/"+group_names.at(0)+"/go_to");
  ROS_INFO("Waiting for GoToLocation server");
  goto_ac.waitForServer();
  ROS_INFO("Connection ok");

  manipulation_msgs::GoToGoal goto_goal;
  goto_goal.location_names.push_back("home");
  goto_goal.to_loc_ctrl_id = "trajectory_tracking";
  goto_goal.job_exec_name = "go_to";
  goto_goal.tool_id = "fake_gripper";
  goto_goal.property_exec_id = "open";

  goto_ac.sendGoalAndWait(goto_goal);
  
  ROS_INFO("GoTo client stopped");
  return 0;
}
