
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

#include <manipulation_msgs/Slot.h>

#include <manipulation_utils/place_objects.h>

#include <object_loader_msgs/DetachObject.h>
#include <object_loader_msgs/RemoveObjects.h>

#include <moveit_planning_helper/manage_trajectories.h>

namespace manipulation
{
  PlaceObjects::PlaceObjects( const ros::NodeHandle& nh, 
                              const ros::NodeHandle& pnh):
                              m_nh(nh),
                              m_pnh(pnh),
                              SkillBase(nh,pnh,"place")
  {

  }

  bool PlaceObjects::init()
  {
    if (!SkillBase::init())
      return false;

    m_add_slots_group_srv = m_pnh.advertiseService("add_slots_group",&PlaceObjects::addSlotsGroupCb,this);
    m_remove_slots_group_srv = m_pnh.advertiseService("remove_slots_group",&PlaceObjects::removeSlotsGroupCb,this);
    m_add_slots_srv = m_pnh.advertiseService("add_slots",&PlaceObjects::addSlotsCb,this);
    m_remove_slots_srv = m_pnh.advertiseService("remove_slots",&PlaceObjects::removeSlotsCb,this);
    m_remove_obj_from_slot_srv = m_pnh.advertiseService("remove_obj_from_slot",&PlaceObjects::removeObjectFromSlotCb,this);
    m_reset_slots_srv = m_pnh.advertiseService("outbound/reset_slot",&PlaceObjects::resetSlotsCb,this);

    m_detach_object_srv = m_nh.serviceClient<object_loader_msgs::DetachObject>("detach_object_to_link");
    m_detach_object_srv.waitForExistence();

    m_remove_object_from_scene_srv = m_nh.serviceClient<object_loader_msgs::RemoveObjects>("remove_object_from_scene");
    m_remove_object_from_scene_srv.waitForExistence();

    if (m_group_names.size() > 0)
    {
      for (const std::string& group_name: m_group_names)
      {
        std::shared_ptr<actionlib::SimpleActionServer<manipulation_msgs::PlaceObjectsAction>> as;
        as.reset(new actionlib::SimpleActionServer<manipulation_msgs::PlaceObjectsAction>(m_pnh,
                                                                                          group_name+"/place",
                                                                                          boost::bind(&PlaceObjects::placeObjectGoalCb,this,_1,group_name),
                                                                                          false));
        m_place_servers.insert(std::pair<std::string,std::shared_ptr<actionlib::SimpleActionServer<manipulation_msgs::PlaceObjectsAction>>>(group_name,as));
        m_place_servers.at(group_name)->start();
      }
    }
    else
    {
      ROS_ERROR("The group_names vector is empty, no ActionServer can be created for PlaceObjects Skill."); 
      return false;
    }
      
    return true;
  }

  bool PlaceObjects::addSlotsGroupCb( manipulation_msgs::AddSlotsGroup::Request& req, 
                                      manipulation_msgs::AddSlotsGroup::Response& res)
  { 
    bool slot_added, slots_group_added = false;
    for (const manipulation_msgs::SlotsGroup& slots_group: req.add_slots_groups )
    {
      if(m_slots_group.find(slots_group.name) != m_slots_group.end())
      {
        for(const manipulation_msgs::Slot& slot: slots_group.slots)
        {
          if (!m_slots_group.at(slots_group.name)->addSlot(slot))
            m_slots_group.erase(m_slots_group.find(slots_group.name));
          else
          {
            ROS_INFO("The slot %s has been added to the group %s.", slot.name.c_str(), slots_group.name.c_str() );
            slot_added = true;
          }
        }
      }
      else
      {
        m_slots_group.insert(std::pair<std::string,SlotsGroupPtr>(slots_group.name,std::make_shared<manipulation::SlotsGroup>(m_pnh,slots_group)));
        if (!m_slots_group.at(slots_group.name)->getIntState())
        {
          m_slots_group.erase(m_slots_group.find(slots_group.name));
          slots_group_added = false;
        }
        else
          slots_group_added = true;
      }
    }

    if (slot_added || slots_group_added)
      res.results = manipulation_msgs::AddSlotsGroup::Response::Success;
    else
      res.results = manipulation_msgs::AddSlotsGroup::Response::Error;

    return (slot_added || slots_group_added);
  }

  bool PlaceObjects::removeSlotsGroupCb(manipulation_msgs::RemoveSlotsGroup::Request& req, 
                                        manipulation_msgs::RemoveSlotsGroup::Response& res)
  {
    for (const std::string& slot_name: req.slots_group_names)
    {
      if(m_slots_group.find(slot_name) != m_slots_group.end())
      {
        m_slots_group.erase(m_slots_group.find(slot_name));
        ROS_INFO("The slot %s has been removed.", slot_name.c_str() );
      }
      else
        ROS_ERROR("Can't remove slot %s, the group doesn't exists.", slot_name.c_str() );
    }
    return true; 
  }

  bool PlaceObjects::addSlotsCb(manipulation_msgs::AddSlots::Request& req, 
                                manipulation_msgs::AddSlots::Response& res)
  {
    if (m_slots_group.find(req.slots_group_name) == m_slots_group.end())
    {
      ROS_ERROR("Can't add slots the group %s is not available.", req.slots_group_name.c_str());
      res.results = manipulation_msgs::AddSlots::Response::SlotsGroupNotFound;
      return false;
    }

    bool slot_added = false;

    for ( const manipulation_msgs::Slot& slot: req.add_slots )
    {
      if (!m_slots_group.at(req.slots_group_name)->addSlot(slot))
        m_slots_group.erase(m_slots_group.find(req.slots_group_name));
      else
      {
        ROS_INFO("The slot %s has been added to the group %s.", slot.name.c_str(), req.slots_group_name.c_str() );
        slot_added = true;
      }
    }

    if (slot_added)
      res.results = manipulation_msgs::AddSlots::Response::Success;
    else
      res.results = manipulation_msgs::AddSlots::Response::Error;
    
    return slot_added;
  }

  bool PlaceObjects::removeSlotsCb( manipulation_msgs::RemoveSlots::Request& req, 
                                    manipulation_msgs::RemoveSlots::Response& res)
  {
    for (std::map<std::string,SlotsGroupPtr>::iterator it = m_slots_group.begin(); it != m_slots_group.end(); ++it)
    {
      for (const std::string& slot_name: req.slots_names)
      {
        if(it->second->findSlot(slot_name))
        {
          if (!it->second->removeSlot(slot_name))
          {
            ROS_WARN("Can't remove the slot %s from the group %s", slot_name.c_str(), it->first.c_str());
            continue;
          }

          if (m_tf.find("place/approach/"+slot_name) != m_tf.end())
            m_tf.erase(m_tf.find("place/approach/"+slot_name));

          if (m_tf.find("place/to/"+slot_name) != m_tf.end())
            m_tf.erase(m_tf.find("place/to/"+slot_name));

          if (m_tf.find("place/leave/"+slot_name) != m_tf.end())
            m_tf.erase(m_tf.find("place/leave/"+slot_name));

          ROS_INFO("The slot %s has been removed from group %s", slot_name.c_str(), it->first.c_str() );

        }          
      }    
    }

    return true;
  }

  bool PlaceObjects::removeObjectFromSlotCb(manipulation_msgs::RemoveObjectFromSlot::Request& req, 
                                            manipulation_msgs::RemoveObjectFromSlot::Response& res)
  {
    bool slot_removed = false;
    for (std::map<std::string,SlotsGroupPtr>::iterator  it = m_slots_group.begin(); 
                                                        it != m_slots_group.end(); ++it)
    {
      if(it->second->findSlot(req.slot_name))
      {
        object_loader_msgs::RemoveObjects remove_srv;
        remove_srv.request.obj_ids.push_back(req.object_to_remove_name);
        ROS_INFO("Remove %s",req.object_to_remove_name.c_str());
        if (!m_remove_object_from_scene_srv.call(remove_srv))
        {
          ROS_ERROR("Unaspected error calling %s service",m_remove_object_from_scene_srv.getService().c_str());
          return false;
        }
        if (!remove_srv.response.success)
        {
          ROS_ERROR("Unable to remove object id %s",req.object_to_remove_name.c_str());
          return false;
        }
        ROS_INFO("Remove collision object %s ",req.object_to_remove_name.c_str());

        it->second->removeObjectFromSlot(req.slot_name);
        ROS_INFO("Removed %s from the slot %s.", req.object_to_remove_name.c_str(), req.slot_name.c_str() );   
        slot_removed = true;
      }
    }

    if (!slot_removed)
      ROS_ERROR("The slot %s is not available.", req.slot_name.c_str() );
   
    return slot_removed; 
  }

  bool PlaceObjects::resetSlotsCb(manipulation_msgs::ResetSlots::Request& req, 
                                  manipulation_msgs::ResetSlots::Response& res)
  {
    for (const std::string& slot_group_name: req.slots_group_name)
    {
      if (m_slots_group.find(slot_group_name) != m_slots_group.end())
      {
        m_slots_group.find(slot_group_name)->second->resetSlot();
        ROS_INFO("Reset the objects in the slots of group %s", slot_group_name.c_str());
      }
      else
        ROS_ERROR("Can't reset slots in the group %s the specified group doesn't exist.", slot_group_name.c_str());
    }
    return true;
  }
  
  void PlaceObjects::placeObjectGoalCb( const manipulation_msgs::PlaceObjectsGoalConstPtr& goal,
                                        const std::string& group_name)
  {

    if (m_place_servers.find(group_name) == m_place_servers.end())
    {
      ROS_ERROR("PlaceObjectsAction server for group %s is not available.", group_name.c_str());
      return;
    }
    manipulation_msgs::PlaceObjectsResult action_res;
    std::shared_ptr<actionlib::SimpleActionServer<manipulation_msgs::PlaceObjectsAction>> as = m_place_servers.at(group_name);

    try
    {
      ros::Time t_start = ros::Time::now();
    
      /* Check if there is an available slot */
      if (m_slots_group.size() == 0)
      {
        ROS_ERROR("No available slot to place objects");
        action_res.result = manipulation_msgs::PlaceObjectsResult::NotInitialized;
        as->setAborted(action_res,"no available slot");
        return;
      }

      std::vector<std::string> available_slot_names;
      for (const std::string& slots_group_names: goal->slots_group_names)
      {
        if (m_slots_group.find(slots_group_names) == m_slots_group.end())
        {
          ROS_WARN("Slot group %s is not available to place the object", slots_group_names.c_str());
          continue;
        }

        std::vector<SlotPtr> slot_vct_ptr = m_slots_group.at(slots_group_names)->getAllSlots();     

        for (const manipulation::SlotPtr& slot_ptr: slot_vct_ptr)
        {
          if(slot_ptr->getSlotAvailability())  
            available_slot_names.push_back(slot_ptr->getLocationName());
        }
      }

      if(available_slot_names.size() == 0)
      {
        ROS_ERROR("All the slots are full.");
        action_res.result = manipulation_msgs::PlaceObjectsResult::NotInitialized;
        as->setAborted(action_res,"all the slots are full");
        return;
      }
      
      if (!m_groups.at(group_name)->startStateMonitor(2))
      {
        ROS_ERROR("%s: enable to get actual state", m_pnh.getNamespace().c_str());
        action_res.result = manipulation_msgs::PlaceObjectsResult::SceneError;
        as->setAborted(action_res,"unable to get actual state");        
        return;
      }

      std::string object_name = goal->object_name;

      // Set the controller for the movement 
      if(!goal->approach_loc_ctrl_id.empty())
      {
        if (!setController( goal->approach_loc_ctrl_id ))
        {
          action_res.result = manipulation_msgs::PlaceObjectsResult::ControllerError;
          ROS_ERROR("Error on service %s result on starting controller %s", m_set_ctrl_srv.getService().c_str(), goal->approach_loc_ctrl_id.c_str() );
          as->setAborted(action_res,"error on setController result");
          return;
        }
      }

      moveit::core::JointModelGroup* jmg = m_joint_models.at(group_name);
      robot_state::RobotState state = *m_groups.at(group_name)->getCurrentState();
      
      Eigen::VectorXd actual_jconf;
      if (jmg)
        state.copyJointGroupPositions(jmg, actual_jconf);

      std::string best_slot_name;
      Eigen::VectorXd slot_approach_jconf;
            
      moveit::planning_interface::MoveItErrorCode result;


      /* Planning to approach position */

      ros::Time t_planning_init = ros::Time::now();
      ROS_INFO("Planning to slot approach position for object placing. Group %s", group_name.c_str());
      moveit::planning_interface::MoveGroupInterface::Plan plan = planTo( group_name,
                                                                          available_slot_names,
                                                                          Location::Destination::Approach,
                                                                          actual_jconf,
                                                                          actual_jconf,
                                                                          result,
                                                                          slot_approach_jconf,
                                                                          best_slot_name);

      if (!result)
      {
        action_res.result = manipulation_msgs::PlaceObjectsResult::NoAvailableTrajectories;
        ROS_ERROR("Error in plan for placing slot, code = %d",result.val);
        as->setAborted(action_res,"error in planning for placing");
        return;
      }

      manipulation::SlotPtr selected_slot;
      std::string selected_group_name;  
      for (std::map<std::string,SlotsGroupPtr>::iterator  it = m_slots_group.begin(); 
                                                          it != m_slots_group.end(); ++it)   
      {
        if(it->second->findSlot(best_slot_name))
        {
          selected_slot = it->second->getSlot(best_slot_name);      
          selected_group_name = it->first;
          break;
        }
      }

      if (!selected_slot)
      {
        action_res.result = manipulation_msgs::PlaceObjectsResult::ReturnError;
        ROS_ERROR("Nullptr for the selected slot %s",best_slot_name.c_str());
        as->setAborted(action_res,"can't find the a valid slot");
        return;
      }


      ROS_INFO("Group %s: plan to approach in %f second",group_name.c_str(),plan.planning_time_);
      ros::Time t_planning = ros::Time::now();
      action_res.planning_duration += t_planning - t_planning_init;
      action_res.expected_execution_duration += plan.trajectory_.joint_trajectory.points.back().time_from_start;
      action_res.path_length += trajectory_processing::computeTrajectoryLength(plan.trajectory_.joint_trajectory);

      moveit_msgs::DisplayTrajectory disp_trj;
      disp_trj.trajectory.push_back(plan.trajectory_);
      disp_trj.model_id = m_kinematic_model->getName();
      disp_trj.trajectory_start = plan.start_state_;
      m_display_publisher.publish(disp_trj);
      
      geometry_msgs::PoseStamped target;
      target.header.frame_id = world_frame;
      target.header.stamp = ros::Time::now();
      tf::poseEigenToMsg(m_locations.at(selected_slot->getLocationName())->getApproach(),target.pose);
      m_target_pub.publish(target);

      ROS_INFO("Execute move to approach position for object placing. Group %s, slot name: %s",group_name.c_str(),best_slot_name.c_str());
      if(!execute(group_name, plan))
      {
        action_res.result = manipulation_msgs::PlaceObjectsResult::TrajectoryError;
        ROS_ERROR("Error while executing trajectory");
        as->setAborted(action_res,"error while executing trajectory.");
        return;
      }

      fjtClientWaitForResult(group_name);
      
      if (!wait(group_name))
      {
        action_res.result = manipulation_msgs::PlaceObjectsResult::TrajectoryError;
        ROS_ERROR("Error executing %s/follow_joint_trajectory",group_name.c_str());
        as->setAborted(action_res,"error in trajectory execution");
        return;
      }

      // Set the desired tool behaviour 
      if(!goal->property_pre_exec_id.empty())
      {
        if (!jobExecute(goal->job_exec_name,goal->tool_id,goal->property_pre_exec_id) )
        {
          action_res.result = manipulation_msgs::PlaceObjectsResult::ReleaseError;
          ROS_ERROR("Error on service during job pre execution for object name %s ", goal->object_name.c_str());
          as->setAborted(action_res,"error on service JobExecution result");
          return;
        }
      }


      /* Planning to slot */

      // Set the controller for the movement 
      if(!goal->to_loc_ctrl_id.empty())
      {
        if (!setController( goal->to_loc_ctrl_id ))
        {
          action_res.result = manipulation_msgs::PlaceObjectsResult::ControllerError;
          ROS_ERROR("Error on service %s result on starting controller %s", m_set_ctrl_srv.getService().c_str(), goal->to_loc_ctrl_id.c_str() );
          as->setAborted(action_res,"error on setController result");
          return;
        }
      }

      Eigen::VectorXd object_release_jconf;
      std::vector<std::string> slot_names(1, selected_slot->getLocationName());

      t_planning_init = ros::Time::now();

      ROS_INFO("Planning to object release position. Group %s, Slot name %s",group_name.c_str(), best_slot_name.c_str());
      plan = planTo(group_name,
                    slot_names,
                    Location::Destination::To,
                    slot_approach_jconf,
                    slot_approach_jconf,
                    result,
                    object_release_jconf,
                    best_slot_name);

      if (!result)
      {
        action_res.result = manipulation_msgs::PlaceObjectsResult::NoAvailableTrajectories;
        ROS_ERROR("Group %s: error in plan to release the object %s in the slot %s, code = %d", group_name.c_str(), object_name.c_str(), best_slot_name.c_str(), result.val);
        as->setAborted(action_res,"error in planning for placing");
        return;
      }

      tf::Transform transform;
      tf::transformEigenToTF(m_locations.at(selected_slot->getLocationName())->getApproach(), transform);
      m_tf.insert(std::pair<std::string,tf::Transform>("place/approach/"+selected_slot->getName(),transform));

      tf::transformEigenToTF(m_locations.at(selected_slot->getLocationName())->getLocation(), transform);
      m_tf.insert(std::pair<std::string,tf::Transform>("place/to/"+selected_slot->getName(),transform));

      tf::transformEigenToTF(m_locations.at(selected_slot->getLocationName())->getLeave(), transform);
      m_tf.insert(std::pair<std::string,tf::Transform>("place/leave/"+selected_slot->getName(),transform));


      t_planning = ros::Time::now();
      action_res.planning_duration += (t_planning-t_planning_init);
      action_res.expected_execution_duration += plan.trajectory_.joint_trajectory.points.back().time_from_start;
      action_res.path_length += trajectory_processing::computeTrajectoryLength(plan.trajectory_.joint_trajectory);

      //// 
      // fjtClientWaitForResult(group_name);
      
      // if (!wait(group_name))
      // {
      //   action_res.result = manipulation_msgs::PlaceObjectsResult::TrajectoryError;
      //   ROS_ERROR("Error executing %s/follow_joint_trajectory",group_name.c_str());
      //   as->setAborted(action_res,"error in trajectory execution");
      //   return;
      // }
      /////

      disp_trj.trajectory.at(0) = (plan.trajectory_);
      disp_trj.trajectory_start = plan.start_state_;
      m_display_publisher.publish(disp_trj);
      tf::poseEigenToMsg(m_locations.at(selected_slot->getLocationName())->getLocation(),target.pose);
      m_target_pub.publish(target);

      ROS_INFO("Execute move to leave position. Group %s, object name %s",group_name.c_str(), object_name.c_str());
      if(!execute(group_name, plan))
      {
        action_res.result = manipulation_msgs::PlaceObjectsResult::TrajectoryError;
        ROS_ERROR("Error while executing trajectory");
        as->setAborted(action_res,"error while executing trajectory.");
        return;
      }

      fjtClientWaitForResult(group_name);

      if (!wait(group_name))
      {
        action_res.result = manipulation_msgs::PlaceObjectsResult::TrajectoryError;
        ROS_ERROR("Error executing %s/follow_joint_trajectory",group_name.c_str());
        as->setAborted(action_res,"error in trajectory execution");
        return;
      }


      /* Release object */

      ros::Time t_release_init = ros::Time::now();
      ros::Duration(0.5).sleep();

      object_loader_msgs::DetachObject detach_srv;
      detach_srv.request.obj_id = object_name;
      if (!m_detach_object_srv.call(detach_srv))
      {
        action_res.result = manipulation_msgs::PlaceObjectsResult::ReleaseError;
        ROS_ERROR("Unaspected error calling %s service",m_detach_object_srv.getService().c_str());
        as->setAborted(action_res,"unaspected error calling detach server");
        return;
      }
      if (!detach_srv.response.success)
      {
        action_res.result = manipulation_msgs::PlaceObjectsResult::ReleaseError;
        ROS_ERROR("Unable to detach object name %s",goal->object_name.c_str());
        as->setAborted(action_res,"unable to detach object");
        return;
      }
      ROS_INFO("Group %s: detached object %s ", group_name.c_str(), detach_srv.request.obj_id.c_str());

      // Set the desired tool behaviour 
      if(!goal->property_exec_id.empty())
      {
        if (!jobExecute(goal->job_exec_name,goal->tool_id,goal->property_exec_id) )
        {
          action_res.result = manipulation_msgs::PlaceObjectsResult::ReleaseError;
          ROS_ERROR("Error on service during job execution for object name %s ", goal->object_name.c_str());
          as->setAborted(action_res,"error on service JobExecution result");
          return;
        }
      }

      action_res.release_object_duration = ros::Time::now() - t_release_init;

      if (!m_groups.at(group_name)->startStateMonitor(2))
      {
        ROS_ERROR("%s: unable to get actual state", m_pnh.getNamespace().c_str());
        action_res.result = manipulation_msgs::PlaceObjectsResult::SceneError;
        as->setAborted(action_res,"unable to get actual state");
        return;
      }

      m_slots_group.at(selected_group_name)->addObjectToSlot(selected_slot->getName());

      /* Planning to leave position after object placing */

      // Set the controller for the movement 
      if(!goal->leave_loc_ctrl_id.empty())
      {
        if (!setController( goal->leave_loc_ctrl_id ))
        {
          action_res.result = manipulation_msgs::PlaceObjectsResult::ControllerError;
          ROS_ERROR("Error on service %s result on starting controller %s", m_set_ctrl_srv.getService().c_str(), goal->leave_loc_ctrl_id.c_str() );
          as->setAborted(action_res,"error on setController result");
          return;
        }
      }

      Eigen::VectorXd slot_leave_jconf;
      t_planning_init = ros::Time::now();

      ROS_INFO("Planning to leave position after object release. Group %s",group_name.c_str());

      plan = planTo(group_name,
                    slot_names,
                    Location::Destination::Leave,
                    object_release_jconf,
                    object_release_jconf,
                    result,
                    slot_leave_jconf,
                    best_slot_name);

      if (!result)
      {
        action_res.result = manipulation_msgs::PlaceObjectsResult::ReturnError;
        ROS_ERROR("Error in plan back from slot after object release, code = %d",result.val);
        as->setAborted(action_res,"error in planning back from slot after object release");
        return;
      }

      t_planning = ros::Time::now();
      action_res.planning_duration += (t_planning-t_planning_init);
      action_res.expected_execution_duration += plan.trajectory_.joint_trajectory.points.back().time_from_start;
      action_res.path_length += trajectory_processing::computeTrajectoryLength(plan.trajectory_.joint_trajectory);
      disp_trj.trajectory.at(0) = plan.trajectory_;
      disp_trj.trajectory_start = plan.start_state_;
      m_display_publisher.publish(disp_trj);

      tf::poseEigenToMsg(m_locations.at(selected_slot->getLocationName())->getLeave(),target.pose);
      m_target_pub.publish(target);

      ROS_INFO("Execute move to leave after object release. Group %s, Object name %s", group_name.c_str(), object_name.c_str());
      if(!execute(group_name, plan))
      {
        action_res.result = manipulation_msgs::PlaceObjectsResult::TrajectoryError;
        ROS_ERROR("Error while executing trajectory");
        as->setAborted(action_res,"error while executing trajectory.");
        return;
      }

      fjtClientWaitForResult(group_name);

      if (!wait(group_name))
      {
        action_res.result = manipulation_msgs::PlaceObjectsResult::ReturnError;
        ROS_ERROR("Error executing %s/follow_joint_trajectory",group_name.c_str());
        as->setAborted(action_res,"error in trajectory execution");
        return;
      }

      // Set the desired tool behaviour 
      if(!goal->property_post_exec_id.empty())
      {
        if (!jobExecute(goal->job_exec_name,goal->tool_id,goal->property_post_exec_id) )
        {
          action_res.result = manipulation_msgs::PlaceObjectsResult::ReleaseError;
          ROS_ERROR("Error on service during job post execution for object name %s ", goal->object_name.c_str());
          as->setAborted(action_res,"error on service JobExecution result");
          return;
        }
      }

      action_res.result = manipulation_msgs::PlaceObjectsResult::Success;
      action_res.slot_name = best_slot_name;
      action_res.actual_duration = ros::Time::now() - t_start;
      action_res.cost = action_res.path_length;
      as->setSucceeded(action_res,"ok");

      return;

    }
    catch(const std::exception& ex )
    {
      ROS_ERROR("PlaceObject::PlaceObjectGoalCb Exception: %s", ex.what());
      action_res.result = manipulation_msgs::PlaceObjectsResult::UnexpectedError;
      as->setAborted(action_res,"exception");
      return;
    }
  
  }
  
  void PlaceObjects::publishTF()
  {
    try
    {
      for (const std::pair<std::string,tf::Transform>& t: m_tf)
        m_broadcaster.sendTransform(tf::StampedTransform(t.second, ros::Time::now(), world_frame, t.first)); 
    }
    catch(const std::exception& ex)
    {
      ROS_ERROR("PlaceObject::publishTF Exception thrown while publishing TF: %s",ex.what());
      return;
    }
  }

}