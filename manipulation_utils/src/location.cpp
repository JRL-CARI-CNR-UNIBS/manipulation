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

#include <manipulation_utils/location.h>
namespace manipulation 
{

// Location Class
Location::Location( const std::string& name,
                    const Eigen::Affine3d& T_w_location,
                    const Eigen::Affine3d& T_w_approach,
                    const Eigen::Affine3d& T_w_leave):
  m_name(name),
  m_T_w_location(T_w_location),
  m_T_w_approach(T_w_approach),
  m_T_w_leave(T_w_leave)
{

}

Location::Location(const manipulation_msgs::Location &msg)
{
  tf::poseMsgToEigen(msg.pose,m_T_w_location);
  
  Eigen::Affine3d T_location_approach;
  tf::poseMsgToEigen(msg.approach_relative_pose,T_location_approach);
  m_T_w_approach = m_T_w_location * T_location_approach;

  Eigen::Affine3d T_location_leave;
  tf::poseMsgToEigen(msg.leave_relative_pose,T_location_leave);
  m_T_w_leave = m_T_w_location * T_location_leave;

  m_name = msg.name;
  m_frame = msg.frame;
}

bool Location::canBePickedBy(const std::string &group_name)
{
  return ( m_location_configurations.find(group_name) != m_location_configurations.end() );
}

void Location::addLocationIk( const std::string &group_name, 
                              const std::vector<Eigen::VectorXd> &solutions )
{
  if ( m_location_configurations.find(group_name) == m_location_configurations.end() )
    m_location_configurations.insert(std::pair<std::string,std::vector<Eigen::VectorXd>>(group_name, solutions));
  else
    m_location_configurations.at(group_name) = solutions;
}

void Location::addApproachIk( const std::string &group_name, 
                              const std::vector<Eigen::VectorXd> &solutions )
{
  if ( m_approach_location_configurations.find(group_name) == m_approach_location_configurations.end() )
    m_approach_location_configurations.insert(std::pair<std::string,std::vector<Eigen::VectorXd>>(group_name, solutions));
  else
    m_approach_location_configurations.at(group_name) = solutions;
}

void Location::addLeaveIk(const std::string &group_name, const std::vector<Eigen::VectorXd> &solutions)
{
  if ( m_leave_location_configurations.find(group_name) == m_leave_location_configurations.end() )
    m_leave_location_configurations.insert(std::pair<std::string,std::vector<Eigen::VectorXd>>(group_name, solutions));
  else
    m_leave_location_configurations.at(group_name) = solutions;
}



// LocationManager class
LocationManager::LocationManager( const ros::NodeHandle& nh):
  m_nh(nh),
  world_frame("world")
{
  // nothing to do
}

bool LocationManager::init()
{
  ROS_INFO("Init Location Manager. Using namespace %s", m_nh.getNamespace().c_str());

  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  m_kinematic_model = robot_model_loader.getModel();

  m_display_publisher = m_nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true); // verificare se il nome del messaggio può essere sposta in file di configurazione

  if (!m_nh.getParam("request_adapters", m_request_adapters))
  {
    ROS_ERROR("Could not find request_adapters in namespace %s ", m_nh.getNamespace().c_str());
    return false;
  }

  if (!m_nh.getParam("ik_sol_number",m_ik_sol_number))
  {
    ROS_WARN("parameter %s/ik_sol_number is not defined, used default value = 200", m_nh.getNamespace().c_str());
    m_ik_sol_number = 200;
  }

  if (!m_nh.getParam("max_stall_iter", m_max_stall_iter))
  {
    ROS_WARN("parameter %s/max_stall_iter is not defined, used default value = 100", m_nh.getNamespace().c_str());
    m_max_stall_iter = 100;
  }

  if (!m_nh.getParam("groups",m_tool_names))
  {
    ROS_ERROR("parameter %s/groups is not defined",m_nh.getNamespace().c_str());
    if (m_nh.hasParam("groups"))
    {
      ROS_ERROR("parameter %s/groups is not wrong",m_nh.getNamespace().c_str());
    }
    return false;
  }
  for (const std::pair<std::string,std::string>& p: m_tool_names)
  {
    m_group_names.push_back(p.first);
  }

  // create groups
  for (const std::string& group_name: m_group_names)
  {
    std::string planner_plugin_name;
    if (!m_nh.getParam(group_name+"/planning_plugin", planner_plugin_name))
    {
      ROS_ERROR("Could not find planner plugin name");
      return false;
    }

    bool use_single_goal;
    if (!m_nh.getParam(group_name+"/use_single_goal",use_single_goal))
    {
      ROS_WARN("parameter %s/use_single_goal is not defined, use false (multi goal enable)",m_nh.getNamespace().c_str());
      use_single_goal = false;
    }
    m_use_single_goal.insert(std::pair<std::string,bool>(group_name,use_single_goal));

    ROS_INFO("Setting PlanningPipeline.");
    planning_pipeline::PlanningPipelinePtr planning_pipeline = std::make_shared<planning_pipeline::PlanningPipeline>(m_kinematic_model, m_nh, planner_plugin_name, m_request_adapters);
    m_planning_pipeline.insert(std::pair<std::string,planning_pipeline::PlanningPipelinePtr>(group_name,planning_pipeline));

    ROS_INFO("Setting MoveGroupInterface.");
    moveit::planning_interface::MoveGroupInterfacePtr group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(group_name);
    if (!group->startStateMonitor(3))
    {
      ROS_ERROR("unable to get robot state for group %s",group_name.c_str());
      return false;
    }

    group->setStartState(*group->getCurrentState());
    m_groups.insert(std::pair<std::string,moveit::planning_interface::MoveGroupInterfacePtr>(group_name,group));


    ROS_INFO("Setting JointModelGroup.");
    moveit::core::JointModelGroup* jmg = m_kinematic_model->getJointModelGroup(group_name);
    m_joint_models.insert(std::pair<std::string,moveit::core::JointModelGroup*>(group_name,jmg));

    
    ROS_INFO("Setting PlanningScene.");
    m_planning_scene.insert(std::pair<std::string,std::shared_ptr<planning_scene::PlanningScene>>(group_name,std::make_shared<planning_scene::PlanningScene>(m_kinematic_model)));
    collision_detection::AllowedCollisionMatrix acm = m_planning_scene.at(group_name)->getAllowedCollisionMatrixNonConst();
    
    std::vector<std::string> allowed_collisions;
    bool use_disable_collisions;
    if (m_nh.getParam(group_name+"/use_disable_collisions",use_disable_collisions))
    {
      if (!m_nh.getParam(group_name+"/disable_collisions",allowed_collisions))
      {
        ROS_INFO("parameter %s/%s/disable_collisions is not defined, use default",m_nh.getNamespace().c_str(),group_name.c_str());
      }
      else
      {
        for (const std::string& link: allowed_collisions)
        {
          ROS_INFO("Disable collision detection for group %s and link %s",group_name.c_str(),link.c_str());
          acm.setEntry(link,true);
        }
      }
    }
    else
    {
      if (m_nh.getParam(group_name+"/disable_collisions",allowed_collisions))
      {
        ROS_WARN("in group %s/%s you set disable_collisions but not use_disable_collisions, it is ignored",m_nh.getNamespace().c_str(),group_name.c_str());
      }
    }

    ///////// TESTING
    // Set the collision detector
    m_collision_loader.setupScene(m_nh,m_planning_scene.at(group_name));
    //m_planning_scene.at(group_name)->setActiveCollisionDetector(collision_detection::CollisionDetectorAllocatorBullet::create());
    ///////// TESTING

    Eigen::Vector3d gravity;
    gravity << 0,0,-9.806; // Muovere in file di configurazione
    rosdyn::ChainPtr chain = rosdyn::createChain(*robot_model_loader.getURDF(),world_frame,m_tool_names.at(group_name),gravity);
    chain->setInputJointsName(jmg->getActiveJointModelNames());
    m_chains.insert(std::pair<std::string,rosdyn::ChainPtr>(group_name,chain));

    size_t n_joints = jmg->getActiveJointModelNames().size();
    std::vector<double> tmp;
    if (m_nh.getParam(group_name+"/preferred_configuration",tmp))
    {
      assert(tmp.size() == n_joints);
      Eigen::VectorXd preferred_position(n_joints);
      for (size_t idof=0; idof<n_joints; idof++)
        preferred_position(idof) = tmp.at(idof);

      ROS_INFO_STREAM("preferred configuration of " << group_name << " is " << preferred_position.transpose());
      m_preferred_configuration.insert(std::pair<std::string,Eigen::VectorXd>(group_name,preferred_position));
      if (m_nh.getParam(group_name+"/preferred_configuration_weight",tmp))
      {
        assert(tmp.size() == n_joints);
        Eigen::VectorXd preferred_position_weight(n_joints);
        for (size_t idof=0; idof<n_joints; idof++)
          preferred_position_weight(idof) = tmp.at(idof);
        ROS_INFO_STREAM("preferred configuration weight of "<<group_name<<" is " << preferred_position_weight.transpose());

        m_preferred_configuration_weight.insert(std::pair<std::string,Eigen::VectorXd>(group_name,preferred_position_weight));
      }
      else
      {
        Eigen::VectorXd preferred_position_weight(n_joints,1);
        ROS_INFO_STREAM("preferred configuration weight of " << group_name << " is " << preferred_position_weight.transpose());
        m_preferred_configuration_weight.insert(std::pair<std::string,Eigen::VectorXd>(group_name,preferred_position_weight));
      }
    }
    else
    {
      ROS_WARN("no preferred configuration for group %s",group_name.c_str());
      ROS_WARN("to do it set parameters %s/%s/preferred_configuration and %s/%s/preferred_configuration_weight",m_nh.getNamespace().c_str(),group_name.c_str(),m_nh.getNamespace().c_str(),group_name.c_str());
    }

    int max_ik_goal_number;
    if (!m_nh.getParam(group_name+"/max_ik_goal_number",max_ik_goal_number))
    {
      max_ik_goal_number = N_ITER;
    }
    m_max_ik_goal_number.insert(std::pair<std::string,int>(group_name,max_ik_goal_number));

  }

  m_add_locations_srv    = m_nh.advertiseService("add_locations",    &LocationManager::addLocationsCb,    this);
  m_remove_locations_srv = m_nh.advertiseService("remove_locations", &LocationManager::removeLocationsCb, this);
  m_list_locations_srv   = m_nh.advertiseService("list_locations",   &LocationManager::listLocationsCb,   this);
  m_get_location_ik_srv  = m_nh.advertiseService("get_location_ik",  &LocationManager::getLocationIkCb,   this);
  ROS_INFO("Location Manager initialized.");

  return true;
}

bool LocationManager::addLocationsCb( manipulation_msgs::AddLocations::Request& req,
                                      manipulation_msgs::AddLocations::Response& res)
{
  if(!addLocationsFromMsg(req.locations))
  {
    res.results = manipulation_msgs::AddLocations::Response::Error;
    return false;
  }

  res.results = manipulation_msgs::AddLocations::Response::Success;
  return true;
}

bool LocationManager::removeLocationsCb(manipulation_msgs::RemoveLocations::Request& req,
                                        manipulation_msgs::RemoveLocations::Response& res)
{
  if(!removeLocations(req.location_names))
  {
    res.results = manipulation_msgs::RemoveLocations::Response::Error;
    return false;
  }

  res.results = manipulation_msgs::RemoveLocations::Response::Success;
  return true;
}

bool LocationManager::listLocationsCb(manipulation_msgs::ListOfLocations::Request &req,
                                      manipulation_msgs::ListOfLocations::Response &res)
{
  for (const std::pair<std::string,LocationPtr>& p: m_locations)
    res.locations.push_back(p.first);
  return true;
}

bool LocationManager::getLocationIkCb(manipulation_msgs::GetLocationIkSolution::Request &req,
                                      manipulation_msgs::GetLocationIkSolution::Response &res)
{
  if ( m_locations.find(req.location_name) == m_locations.end() )
  {
    ROS_WARN("Location %s is not present",req.location_name.c_str());
    return false;
  }
  LocationPtr loc=m_locations.at(req.location_name);
  std::vector<Eigen::VectorXd> ik_sols=loc->getApproachIk(req.group_name);
  for (const Eigen::VectorXd& q: ik_sols)
  {
    manipulation_msgs::Configuration configuration;
    for (int iq=0;iq<q.size();iq++)
      configuration.configuration.push_back(q(iq));
    res.approach_ik_solutions.push_back(configuration);
  }

  ik_sols=loc->getLocationIk(req.group_name);
  for (const Eigen::VectorXd& q: ik_sols)
  {
    manipulation_msgs::Configuration configuration;
    for (int iq=0;iq<q.size();iq++)
      configuration.configuration.push_back(q(iq));
    res.ik_solutions.push_back(configuration);
  }

  ik_sols=loc->getLeaveIk(req.group_name);
  for (const Eigen::VectorXd& q: ik_sols)
  {
    manipulation_msgs::Configuration configuration;
    for (int iq=0;iq<q.size();iq++)
      configuration.configuration.push_back(q(iq));
    res.leave_ik_solutions.push_back(configuration);
  }

  return true;
}

bool LocationManager::addLocationFromMsg(const manipulation_msgs::Location& location)
{
  LocationPtr location_ptr(new Location(location));

  if ( m_locations.find(location_ptr->m_name) != m_locations.end() )
  {
    ROS_WARN("Location %s is already present",location_ptr->m_name.c_str());
    return false;
  }

  bool get_ik_group = false;

  for (const std::pair<std::string,moveit::planning_interface::MoveGroupInterfacePtr>& group: m_groups)
  {
    std::vector<Eigen::VectorXd> sols;
    std::vector<Eigen::VectorXd> seed;
    std::vector<Eigen::VectorXd> location_sols;
    std::vector<Eigen::VectorXd> leave_sols;
    std::vector<Eigen::VectorXd> approach_sols;


    if (!m_nh.hasParam(location_ptr->m_name+"/"+group.first))
    {
      if (!ik(group.first,location_ptr->m_T_w_location,seed,location_sols,m_ik_sol_number))
      {
        ROS_WARN("Location %s can't be reached by group %s",location_ptr->m_name.c_str(),group.first.c_str());
        continue;
      }

      for (const Eigen::VectorXd& q: location_sols)
      {
        std::vector<Eigen::VectorXd> tmp_sols;
        std::vector<Eigen::VectorXd> tmp_approach_sols;
        std::vector<Eigen::VectorXd> tmp_leave_sols;
        tmp_sols.push_back(q);
        if (!ik(group.first,location_ptr->m_T_w_approach,tmp_sols,tmp_approach_sols,1))
          continue;
        if (!ik(group.first,location_ptr->m_T_w_leave,tmp_sols,tmp_leave_sols,1))
          continue;
        sols.push_back(tmp_sols.at(0));
        approach_sols.push_back(tmp_approach_sols.at(0));
        leave_sols.push_back(tmp_leave_sols.at(0));
      }

      rosparam_utilities::setParam(m_nh,std::string(location_ptr->m_name+"/"+group.first),sols);
      rosparam_utilities::setParam(m_nh,std::string(location_ptr->m_name+"/approach/"+group.first),approach_sols);
      rosparam_utilities::setParam(m_nh,std::string(location_ptr->m_name+"/leave/"+group.first),leave_sols);
    }
    else
    {
      std::string what;
      if (!rosparam_utilities::getParam(m_nh,location_ptr->m_name+"/"+group.first,sols,what))
      {
        ROS_ERROR("Parameter %s/%s/%s is not correct.",m_nh.getNamespace().c_str(),location_ptr->m_name.c_str(),group.first.c_str());
        return false;
      }
      if (!rosparam_utilities::getParam(m_nh,location_ptr->m_name+"/approach/"+group.first,approach_sols,what))
      {
        ROS_ERROR("Parameter %s/%s/approach/%s is not correct.",m_nh.getNamespace().c_str(),location_ptr->m_name.c_str(),group.first.c_str());
        return false;
      }
      if (!rosparam_utilities::getParam(m_nh,location_ptr->m_name+"/leave/"+group.first,leave_sols,what))
      {
        ROS_ERROR("Parameter %s/%s/leave/%s is not correct.",m_nh.getNamespace().c_str(),location_ptr->m_name.c_str(),group.first.c_str());
        return false;
      }
    }

    location_ptr->addLocationIk(group.first,sols);
    location_ptr->addApproachIk(group.first,approach_sols);
    location_ptr->addLeaveIk(group.first,leave_sols);
    
    get_ik_group = true;
  }

  if(get_ik_group)
    m_locations.insert(std::pair<std::string,LocationPtr>(location_ptr->m_name,location_ptr));
  else
  {
    ROS_WARN("The location %s was not added to the location manager", location_ptr->m_name.c_str());
    return false;
  }
  
  return true;
}

bool LocationManager::addLocationsFromMsg(const std::vector<manipulation_msgs::Location>& locations)
{
  bool loc_added_ok = true;
  for (const manipulation_msgs::Location& location: locations)
  {
    if(!addLocationFromMsg(location))
    {
      ROS_WARN("Can't add the location %s",location.name.c_str());
      loc_added_ok = false;
    }
  }
  return loc_added_ok;
}

bool LocationManager::removeLocation(const std::string& location_name)
{
  if (location_name.compare("delete_all")==0)
  {
    ROS_INFO("delete_all shortcut: delete all the locations in %s",m_nh.getNamespace().c_str());
    m_locations.clear();
    return true;
  }

  if ( m_locations.find(location_name) == m_locations.end() )
  {
    ROS_WARN("Location %s is not present",location_name.c_str());
    return false;
  }
  m_locations.erase(m_locations.find(location_name));
  return true;
}

bool LocationManager::removeLocations(const std::vector<std::string>& location_names)
{
  for (const std::string& location_name: location_names)
  {
    if(!removeLocation(location_name))
    {
      ROS_WARN("Can't remove the location %s",location_name.c_str());
      return false;
    }
  }
  
  return true;
}

moveit::planning_interface::MoveGroupInterface::Plan LocationManager::planTo( const std::string& group_name,
                                                                              const std::vector<std::string>& location_names,
                                                                              const Location::Destination& destination,
                                                                              const Eigen::VectorXd& starting_jconf,
                                                                              const Eigen::VectorXd& preferred_jconf,
                                                                              moveit::planning_interface::MoveItErrorCode& result,
                                                                              Eigen::VectorXd& final_configuration,
                                                                              std::string& plan_to_location_name )
{
  moveit::planning_interface::MoveGroupInterface::Plan plan;

  try
  {
    moveit::planning_interface::MoveGroupInterfacePtr group = m_groups.at(group_name);
    moveit::core::JointModelGroup* jmg = m_joint_models.at(group_name);
    
    int max_ik_goal_number = m_max_ik_goal_number.at(group_name);

    if (!group->startStateMonitor(2))
    {
      ROS_ERROR("%s: unable to get actual state",m_nh.getNamespace().c_str());
      result = moveit::planning_interface::MoveItErrorCode::UNABLE_TO_AQUIRE_SENSOR_DATA;
      return plan;
    }

    robot_state::RobotState state = *group->getCurrentState();
    state.setJointGroupPositions(jmg,starting_jconf);
    moveit::core::robotStateToRobotStateMsg(state,plan.start_state_);

    planning_interface::MotionPlanRequest req;
    planning_interface::MotionPlanResponse res;
    req.group_name = group_name;
    req.start_state = plan.start_state_;
    req.allowed_planning_time = 5;

    robot_state::RobotState goal_state(m_kinematic_model);

    m_scene_mtx.lock();
    planning_scene::PlanningScenePtr planning_scene = planning_scene::PlanningScene::clone(m_planning_scene.at(group_name));
    m_scene_mtx.unlock();


    std::vector<Eigen::VectorXd> sols;
    for(const std::string& location_name: location_names)
    {
      ROS_INFO("Planning to location: %s", location_name.c_str());
      std::vector<Eigen::VectorXd> sols_single_location;
      if(!getIkSolForLocation(location_name,destination,group_name,sols_single_location))
      {
        result = res.error_code_;
        return plan;
      }

      sols.insert(sols.end(),sols_single_location.begin(),sols_single_location.end());
    }

    if (sols.size() == 0)
    {
      ROS_WARN("Found %zu solution can't plan trajectory.", sols.size());
      return plan;
    }

    std::map<double,Eigen::VectorXd> solutions;

    for (const Eigen::VectorXd& goal: sols)
    {
      goal_state.setJointGroupPositions(jmg, goal);
      goal_state.updateCollisionBodyTransforms();
      if (!planning_scene->isStateValid(goal_state,group_name))
        continue;

      double normsol = (goal - preferred_jconf).norm();
      solutions.insert(std::pair<double,Eigen::VectorXd>(normsol,goal));

      if (m_use_single_goal.at(group_name))
        break;
    }

    for (const std::pair<double,Eigen::VectorXd>& p: solutions)
    {
      if (req.goal_constraints.size() >= max_ik_goal_number)
        break;
      goal_state.setJointGroupPositions(jmg, p.second);
      moveit_msgs::Constraints joint_goal = kinematic_constraints::constructGoalConstraints(goal_state, jmg);
      req.goal_constraints.push_back(joint_goal);
    }

    ROS_INFO("Found %zu solution",solutions.size());

    if (req.goal_constraints.size()==0)
    {
      ROS_ERROR("Inbound server: no valid goals");
      result = res.error_code_;
      return plan;
    }
    ROS_INFO("Adding %zu goals to the Planning Pipeline.",req.goal_constraints.size());


    if (!m_planning_pipeline.at(group_name)->generatePlan(planning_scene, req, res))
    {
      ROS_ERROR("Could not compute plan successfully");
      result = res.error_code_;
      return plan;
    }

    plan.planning_time_ = res.planning_time_;
    ROS_INFO("Planning time: %f.",plan.planning_time_);

    res.trajectory_->getRobotTrajectoryMsg(plan.trajectory_);
    if (res.trajectory_->getWayPointCount()==0)
      ROS_WARN("Trajectory has 0 waypoint");
    else
      res.trajectory_->getLastWayPoint().copyJointGroupPositions(jmg,final_configuration);

    result = res.error_code_;

    std::vector<double> min_dist;
    for(const std::string& location_name: location_names)
      min_dist.push_back(computeDistanceBetweenLocations(location_name, group_name, destination, final_configuration));
    
    plan_to_location_name = location_names.at(std::min_element(min_dist.begin(),min_dist.end()) - min_dist.begin());
    ROS_INFO("Planning completed. Selected location: %s", plan_to_location_name.c_str());

    Eigen::Affine3d loc_ = m_locations.at(plan_to_location_name)->getLocation();
    
    return plan;
  }
  catch(const std::exception& ex)
  {
    ROS_ERROR("LocationManager::planTo Exception: %s",ex.what());
    return plan;
  }

}

bool LocationManager::getIkSolForLocation(const std::string& location_name,
                                          const Location::Destination& destination,
                                          const std::string& group_name,
                                          std::vector<Eigen::VectorXd>& jconf_single_location)
{
  jconf_single_location.clear();
  if (m_locations.find(location_name) != m_locations.end())
  {
    switch (destination)
    {
    case Location::Approach:
      jconf_single_location = m_locations.at(location_name)->getApproachIk(group_name);
      break;
    case Location::To:
      jconf_single_location = m_locations.at(location_name)->getLocationIk(group_name);
      break;
    case Location::Leave:
      jconf_single_location = m_locations.at(location_name)->getLeaveIk(group_name);
      break;
    }
  }
  else
  {
    ROS_ERROR("Can't find the location: %s in the location manager.", location_name.c_str());
    return false;
  }

  return true;
}  

double LocationManager::computeDistanceBetweenLocations(const std::string& location_name,
                                                        const std::string& group_name,
                                                        const Location::Destination& destination,
                                                        const Eigen::VectorXd& jconf)
{
  std::vector<Eigen::VectorXd> jconfs_location_name;
  if (getIkSolForLocation(location_name, destination, group_name, jconfs_location_name))
  {
    std::vector<double> jconf_dist;
    for (const Eigen::VectorXd& jconf_single: jconfs_location_name )
    {
      if (jconf_single.size() == jconf.size())
        jconf_dist.push_back((jconf_single - jconf).lpNorm<1>());
      else
      {
        ROS_ERROR("Can't compute the distance between joint configuration.");
        return std::numeric_limits<double>::quiet_NaN();
      }
    }
    return *std::min_element(jconf_dist.begin(),jconf_dist.end());
  }
  else
  {
    ROS_ERROR("Can't compute the distance between joint configuration.");
    return std::numeric_limits<double>::quiet_NaN();
  }
}

bool LocationManager::ik( const std::string& group_name,
                          const Eigen::Affine3d& T_w_a,
                          const std::vector<Eigen::VectorXd>& seed,
                          std::vector<Eigen::VectorXd >& sols,
                          unsigned int ntrial)
{
  std::map<double,Eigen::VectorXd> solutions;
  moveit::planning_interface::MoveGroupInterfacePtr group = m_groups.at(group_name);
  moveit::core::JointModelGroup* jmg = m_joint_models.at(group_name);
  robot_state::RobotState state = *group->getCurrentState();
  
  m_scene_mtx.lock();
  planning_scene::PlanningScenePtr planning_scene = planning_scene::PlanningScene::clone(m_planning_scene.at(group_name));
  m_scene_mtx.unlock();

  Eigen::VectorXd act_joints_conf;
  state.copyJointGroupPositions(group_name,act_joints_conf);
  
  unsigned int n_seed = seed.size();
  bool found = false;

  int stall=0;
  for (unsigned int iter=0;iter<N_MAX_ITER;iter++)
  {
    if (solutions.size()>=ntrial)
      break;
    if (solutions.size()==0 && iter>ntrial*5)
      break;

    if (stall++>m_max_stall_iter)
      break;

    if (iter<n_seed)
    {
      state.setJointGroupPositions(jmg,seed.at(iter));
    }
    else if (iter==n_seed)
    {
      state = *group->getCurrentState();
    }
    else
    {
      state.setToRandomPositions();
    }

    Eigen::VectorXd js;
    Eigen::VectorXd start;
    state.copyJointGroupPositions(group_name,start);
    if (m_chains.at(group_name)->computeLocalIk(js,T_w_a,start,1e-4,ros::Duration(0.002)))
    {
      state.setJointGroupPositions(group_name,js);

      if (!state.satisfiesBounds())
        continue;
      
      state.updateCollisionBodyTransforms();
      if (!planning_scene->isStateValid(state,group_name))
        continue;

      double dist = (js-act_joints_conf).norm();
      
      if (solutions.size() == 0)
      {
        stall=0;
        std::vector<Eigen::VectorXd> multiturn = m_chains.at(group_name)->getMultiplicity(js);
        for (const Eigen::VectorXd& tmp: multiturn)
        {
          state.setJointGroupPositions(group_name,tmp);
          if (!state.satisfiesBounds())
            continue;

          double dist = (m_preferred_configuration_weight.at(group_name).cwiseProduct(tmp-m_preferred_configuration.at(group_name))).norm();
          solutions.insert(std::pair<double,Eigen::VectorXd>(dist,tmp));
        }

        found = true;
      }
      else
      {
        bool is_diff = true;
        for (const std::pair<double,Eigen::VectorXd>& sol: solutions)
        {
          if ((sol.second-js).norm()<TOLERANCE)
          {
            is_diff = false;
            break;
          }
        }
        if (is_diff)
        {
          stall=0;
          std::vector<Eigen::VectorXd> multiturn = m_chains.at(group_name)->getMultiplicity(js);
          for (const Eigen::VectorXd& tmp: multiturn)
          {
            state.setJointGroupPositions(group_name,tmp);
            if (!state.satisfiesBounds())
              continue;

            double dist = (m_preferred_configuration_weight.at(group_name).cwiseProduct( tmp - m_preferred_configuration.at(group_name) )).norm();
            solutions.insert(std::pair<double,Eigen::VectorXd>(dist,tmp));
          }
          found = true;
        }
      }
    }
    
  }

  sols.clear();
  for (const std::pair<double,Eigen::VectorXd>& sol: solutions)
    sols.push_back(sol.second);

  ROS_DEBUG("Found %lu solutions for the IK.", sols.size());

  return found;
}

void LocationManager::updatePlanningScene(const moveit_msgs::PlanningScene& scene)
{
  m_scene_mtx.lock();
  for (const std::string& group: m_group_names)
  {
    if (!m_planning_scene.at(group)->setPlanningSceneMsg(scene))
      ROS_ERROR("unable to update planning scene");
  }
  m_scene_mtx.unlock();
}                    

}  // end namespace manipulation

