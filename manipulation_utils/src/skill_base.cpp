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

#include <configuration_msgs/StartConfiguration.h>

#include <manipulation_utils/skill_base.h>
#include <manipulation_msgs/JobExecution.h>
namespace manipulation
{
SkillBase::SkillBase(const ros::NodeHandle& nh,
                     const ros::NodeHandle& pnh,
                     const std::string& skill_name,
                     const std::string reference_frame):
  LocationManager(pnh,reference_frame),
  m_pnh(pnh),
  m_skill_name(skill_name)
{
  // nothing to do ...
}

bool SkillBase::init()
{
  if (!LocationManager::init())
  {
    ROS_ERROR("Unable to init SkillBase");
    return false;
  }

  for (const std::string& group_name: m_group_names)
  {
    m_fjt_result.insert(std::pair<std::string,double>(group_name,0));

    std::shared_ptr<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>> fjt_ac;
    fjt_ac.reset(new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>("/"+group_name+"/follow_joint_trajectory",true));
    m_fjt_clients.insert(std::pair<std::string,std::shared_ptr<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>>>(group_name,fjt_ac));
  }

  m_target_pub = m_nh.advertise<geometry_msgs::PoseStamped>("target_visualization",1);

  m_set_ctrl_srv = m_pnh.serviceClient<configuration_msgs::StartConfiguration>("/configuration_manager/start_configuration");
  m_set_ctrl_srv.waitForExistence();

  m_list_job_executers    = m_pnh.advertiseService("list_executers",&SkillBase::listExecuters,this);
  m_register_job_executer = m_pnh.advertiseService("register_executer",&SkillBase::registerExecuter,this);
  return true;
}

bool SkillBase::execute(const std::string& group_name,
                        const moveit::planning_interface::MoveGroupInterface::Plan& plan)
{
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

  if (!m_fjt_clients.at(group_name)->waitForServer(ros::Duration(10)))
  {
    ROS_ERROR("Timeout FollowJointTrajectory client for group %s", group_name.c_str());
    return false;
  }
  m_fjt_clients.at(group_name)->sendGoal(goal, cb);

  ROS_INFO("Execution started!");
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

void SkillBase::fjtClientWaitForResult(const std::string& group_name)
{
  m_fjt_clients.at(group_name)->waitForResult();
}

bool SkillBase::jobExecute( const std::string& job_executor_name,
                            const std::string& tool_id,
                            const std::string& property_id,
                            const std::string& object_id  )
{
  ros::ServiceClient job_srv = m_pnh.serviceClient<manipulation_msgs::JobExecution>(job_executor_name);
  if (not job_srv.waitForExistence(ros::Duration(10)))
  {
    ROS_ERROR("Unable to connect to %s service server during job execution of skill %s",job_srv.getService().c_str(), job_executor_name.c_str());
    return false;
  }

  manipulation_msgs::JobExecution job_req;
  job_req.request.skill_name = m_skill_name;
  job_req.request.tool_id = tool_id;
  job_req.request.property_id = property_id;
  job_req.request.object_id=object_id;

  if (!job_srv.call(job_req))
  {
    ROS_ERROR("Unable to call %s service during job execution of skill %s",job_srv.getService().c_str(), job_executor_name.c_str());
    return false;
  }

  if (job_req.response.results == manipulation_msgs::JobExecution::Response::Success)
  {
    ROS_INFO("Job Execution done!");
    return true;
  }
  else
  {
    ROS_ERROR("Job execution error: %d", job_req.response.results);
    return false;
  }
}

bool SkillBase::setController( const std::string& controller_name )
{
  configuration_msgs::StartConfiguration start_ctrl_req;
  start_ctrl_req.request.start_configuration = controller_name;
  start_ctrl_req.request.strictness = 1;

  if (!m_set_ctrl_srv.call(start_ctrl_req))
  {
    ROS_ERROR("Unable to call %s service to set controller: %s",m_set_ctrl_srv.getService().c_str(), controller_name.c_str());
    return false;
  }

  if (!start_ctrl_req.response.ok)
  {
    ROS_ERROR("Error on service %s response", m_set_ctrl_srv.getService().c_str());
    return false;
  }

  ROS_INFO("Controller %s started.", controller_name.c_str());
  return true;
}

bool SkillBase::registerExecuter(manipulation_msgs::RegisterJobExecuter::Request &req, manipulation_msgs::RegisterJobExecuter::Response &res)
{
  if (req.executer_name.empty())
  {
    ROS_ERROR("you are trying to register a job executer with no name");
    return false;
  }
  m_job_executers.push_back(req.executer_name);
  return true;
}

bool SkillBase::listExecuters(manipulation_msgs::ListOfJobExecuters::Request &req, manipulation_msgs::ListOfJobExecuters::Response &res)
{
  for (std::string s: m_job_executers)
    res.executers.push_back(s);
  return true;
}

} // end namespace manipulation
