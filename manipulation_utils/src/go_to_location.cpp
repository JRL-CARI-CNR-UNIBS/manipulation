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

#include <manipulation_utils/go_to_location.h>
#include <moveit_planning_helper/manage_trajectories.h>

namespace manipulation
{

GoToLocation::GoToLocation( const ros::NodeHandle& nh, 
                            const ros::NodeHandle& pnh):
                            m_nh(nh),
                            m_pnh(pnh),
                            SkillBase(nh,pnh,"go_to")
{
  // nothing to do here    
}

bool GoToLocation::init()
{
  if (!SkillBase::init())  
    return false;

  if(m_group_names.size() > 0)
  {
    for (const std::string& group_name: m_group_names)
    {
      std::shared_ptr<actionlib::SimpleActionServer<manipulation_msgs::GoToAction>> as;
      as.reset(new actionlib::SimpleActionServer<manipulation_msgs::GoToAction>(m_pnh,
                                                                                group_name+"/go_to",
                                                                                boost::bind(&GoToLocation::gotoGoalCb,this,_1,group_name),
                                                                                false));
      m_goto_location_server.insert(std::pair<std::string,std::shared_ptr<actionlib::SimpleActionServer<manipulation_msgs::GoToAction>>>(group_name,as));
      m_goto_location_server.at(group_name)->start();
    }
  }
  else
  {
    ROS_ERROR("The group_names vector is empty, no ActionServer can be created for GoToLocation Skill."); 
    return false;
  }
  
  return true;
}

void GoToLocation::gotoGoalCb(const manipulation_msgs::GoToGoalConstPtr& goal,
                              const std::string& group_name)
{
  if (m_goto_location_server.find(group_name) == m_goto_location_server.end())
  {
    ROS_ERROR("GoToAction server for group %s is not available.", group_name.c_str());
    return;
  }

  manipulation_msgs::GoToResult action_res;
  std::shared_ptr<actionlib::SimpleActionServer<manipulation_msgs::GoToAction>> as = m_goto_location_server.at(group_name);
  
  try
  {
    ros::Time t_start = ros::Time::now();

    if (m_locations.find(goal->location_name) == m_locations.end())
    {
      action_res.result = manipulation_msgs::GoToResult::TrajectoryError;
      ROS_ERROR("Can't find the location '%s' in the location manager.", goal->location_name.c_str());
      as->setAborted(action_res,"the location is not present in the location manager");
      return;
    }

    std::vector<std::string> location_names(1, goal->location_name);
    
    // Set the controller for the movement 
    if(!goal->to_loc_ctrl_id.empty())
    {
      if (!setController( goal->to_loc_ctrl_id ))
      {
        action_res.result = manipulation_msgs::GoToResult::ControllerError;
        ROS_ERROR("Error on service %s result on starting controller %s", m_set_ctrl_srv.getService().c_str(), goal->to_loc_ctrl_id.c_str() );
        as->setAborted(action_res,"error on setController result");
        return;
      }
    }

    moveit::core::JointModelGroup* jmg = m_joint_models.at(group_name);
    robot_state::RobotState state = *m_groups.at(group_name)->getCurrentState();
    
    Eigen::VectorXd actual_jconf;
    if (jmg)
      state.copyJointGroupPositions(jmg, actual_jconf);

    Eigen::VectorXd location_jconf;
    std::string plan_to_location_name;

    moveit::planning_interface::MoveItErrorCode result;

    ros::Time t_planning_init = ros::Time::now();
    ROS_INFO("Planning to desired position. Group %s",group_name.c_str());
    moveit::planning_interface::MoveGroupInterface::Plan plan = planTo( group_name,
                                                                        location_names,
                                                                        Location::Destination::To,
                                                                        actual_jconf,
                                                                        actual_jconf,
                                                                        result,
                                                                        location_jconf,
                                                                        plan_to_location_name);

    if (!result)
    {
      action_res.result = manipulation_msgs::GoToResult::NoAvailableTrajectories;
      ROS_ERROR("Error in planning to the location, code = %d.",result.val);
      as->setAborted(action_res,"error in planning to the location");
      return;
    }

    ros::Time t_planning = ros::Time::now();
    action_res.planning_duration += (t_planning-t_planning_init);
    action_res.expected_execution_duration += plan.trajectory_.joint_trajectory.points.back().time_from_start;
    action_res.path_length += trajectory_processing::computeTrajectoryLength(plan.trajectory_.joint_trajectory);

    moveit_msgs::DisplayTrajectory disp_trj;
    disp_trj.trajectory.push_back(plan.trajectory_);
    disp_trj.model_id = m_kinematic_model->getName();
    m_display_publisher.publish(disp_trj);
    disp_trj.trajectory_start = plan.start_state_;

    ROS_INFO("Execute move to location. Group %s, location name %s",group_name.c_str(), plan_to_location_name.c_str());
    execute(group_name,plan);
    if (!wait(group_name))
    {
      action_res.result = manipulation_msgs::GoToResult::TrajectoryError;
      ROS_ERROR("error executing %s/follow_joint_trajectory",group_name.c_str());
      as->setAborted(action_res,"error in trajectory execution");
      return;
    }

    // Set the desired tool behaviour 
    if(!goal->property_exec_id.empty())
    {
      if (!jobExecute(goal->tool_id,goal->property_exec_id) )
      {
        action_res.result = manipulation_msgs::GoToResult::ToolError;
        ROS_ERROR("Error on service %s result during job execution", m_job_srv.getService().c_str() );
        as->setAborted(action_res,"error on service JobExecution result");
        return;
      }
    }

    action_res.result = manipulation_msgs::GoToResult::Success;
    as->setSucceeded(action_res,"ok");
    action_res.actual_duration += (ros::Time::now() - t_start);

  }
  catch(const std::exception& ex)
  {
    ROS_ERROR("GoToLocation::gotoGoalCb Exception: %s", ex.what());
    return;
  }

  return;
}

}
