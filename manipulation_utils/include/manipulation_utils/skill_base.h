#pragma once

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

#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>

#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryGoal.h>
#include <control_msgs/FollowJointTrajectoryResult.h>

#include <manipulation_msgs/JobExecution.h>
#include <manipulation_msgs/ListOfJobExecuters.h>
#include <manipulation_msgs/RegisterJobExecuter.h>

#include <manipulation_utils/location.h>

#include <moveit/move_group_interface/move_group_interface.h>

#include <tf/transform_broadcaster.h>
namespace manipulation
{  
  class SkillBase: public LocationManager
  {
    protected:

      ros::NodeHandle m_pnh;

      std::string m_skill_name;
      std::map<std::string,double> m_fjt_result;
      
      ros::Publisher m_target_pub;
      ros::ServiceClient m_set_ctrl_srv;
      
      ros::ServiceServer m_list_job_executers;
      ros::ServiceServer m_register_job_executer;

      std::vector<std::string> m_job_executers;

      std::map<std::string,std::shared_ptr<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>>> m_fjt_clients;

      bool execute( const std::string& group_name,
                    const moveit::planning_interface::MoveGroupInterface::Plan& plan);

      bool wait(const std::string& group_name);

      void doneCb(const actionlib::SimpleClientGoalState& state,
                  const control_msgs::FollowJointTrajectoryResultConstPtr& result,
                  const std::string& group_name);

      void fjtClientWaitForResult(const std::string& group_name);

      bool jobExecute(const std::string& job_executor_name,
                      const std::string& tool_id,
                      const std::string& property_id );

      bool setController( const std::string& controller_name );

      bool registerExecuter( manipulation_msgs::RegisterJobExecuter::Request&  req,
                              manipulation_msgs::RegisterJobExecuter::Response& res);

      bool listExecuters( manipulation_msgs::ListOfJobExecuters::Request&  req,
                          manipulation_msgs::ListOfJobExecuters::Response& res);

    public:
      SkillBase(const ros::NodeHandle& nh,
                const ros::NodeHandle& pnh,
                const std::string& skill_name,
                const std::string reference_frame = "world");

      bool init();

  };

} // end namespace manipulation
