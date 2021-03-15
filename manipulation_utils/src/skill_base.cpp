/*
Copyright (c) 2020, Manuel Beschi 
CARI Joint Research Lab
UNIBS-DIMI manuel.beschi@unibs.it
CNR-STIIMA manuel.beschi@stiima.cnr.it
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

#include <std_srvs/SetBool.h>

#include <manipulation_msgs/SkillFeedback.h>

#include <manipulation_utils/skill_base.h>

namespace manipulation
{
  SkillBase::SkillBase( const ros::NodeHandle& nh,
                        const ros::NodeHandle& pnh,
                        const std::string& skill_name):
                        m_nh(nh),
                        m_pnh(pnh),
                        m_skill_name(skill_name),
                        m_init(false),
                        LocationManager(pnh)
  {
    // nothing to do ...
  }

  bool SkillBase::init()
  {
    if (!LocationManager::init())
      m_init = false;

    m_skill_srv = m_pnh.serviceClient<manipulation_msgs::SkillFeedback>("skill/"+m_skill_name);    
    m_target_pub = m_pnh.advertise<geometry_msgs::PoseStamped>("target",1);

    m_init = true;
    return m_init;
  }

  bool SkillBase::execute(const std::string& group_name,
                          const moveit::planning_interface::MoveGroupInterface::Plan& plan)
  {
    moveit::planning_interface::MoveGroupInterfacePtr group = m_groups.at(group_name);
    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory = plan.trajectory_.joint_trajectory;

    if (m_fjt_result.find(group_name) == m_fjt_result.end())
    {
      ROS_ERROR("Can't find FollowJointTrajectory result for group %s", group_name.c_str());
      return false;
    }
      
    m_fjt_result.at(group_name) = std::nan("1");

    auto cb = boost::bind(&manipulation::SkillBase::doneCb,this,_1,_2,group_name);
    
    if (m_fjt_clients.find(group_name) == m_fjt_clients.end())
    {
      ROS_ERROR("Can't find FollowJointTrajectory client for group %s", group_name.c_str());
      return false;
    }

    m_fjt_clients.at(group_name)->sendGoal(goal, cb);

    ROS_INFO("Execution done!");
    return true;
  }

  bool SkillBase::wait(const std::string& group_name)
  {
    ros::Time t0 = ros::Time::now();
    while (std::isnan(m_fjt_result.at(group_name)))
    {
      ros::Duration(0.01).sleep();

      if ((ros::Time::now()-t0).toSec()>600)
      {
        ROS_ERROR("%s is waiting more than 10 minutes, stop it",group_name.c_str());
        return false;
      }
      if ((ros::Time::now()-t0).toSec()>10)
        ROS_WARN_THROTTLE(10,"%s is waiting for %f seconds",group_name.c_str(),(ros::Time::now()-t0).toSec());
    }
    return !std::isnan(m_fjt_result.at(group_name));
  }

  void SkillBase::doneCb( const actionlib::SimpleClientGoalState& state,
                          const control_msgs::FollowJointTrajectoryResultConstPtr& result,
                          const std::string& group_name)
  {
    if (m_fjt_result.find(group_name) == m_fjt_result.end())
    {
      ROS_ERROR("Can't find FollowJointTrajectory result for group %s", group_name.c_str());
      return;
    }

    m_fjt_result.at(group_name) = result->error_code;
    
    if (result->error_code < 0)
    {
      ROS_ERROR("Error executing %s/follow_joint_trajectory: %s",
                group_name.c_str(),result->error_string.c_str() );
    }
    return;
  }

} // end namespace manipulation