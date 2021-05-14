
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

#include <std_srvs/SetBool.h> 

#include <tf_conversions/tf_eigen.h>
#include <tf/transform_datatypes.h>


#include <manipulation_msgs/AddBoxes.h>
#include <manipulation_msgs/AddObjects.h>
#include <manipulation_msgs/ListOfObjects.h>
#include <manipulation_msgs/RemoveBoxes.h>
#include <manipulation_msgs/RemoveObjects.h>
#include <manipulation_msgs/ResetBoxes.h>
#include <manipulation_msgs/PickObjectsAction.h>

#include <manipulation_utils/skill_base.h>
#include <manipulation_utils/manipulation_utils.h>

namespace manipulation
{
  //typedef std::multimap<std::string, Eigen::Affine3d, std::less<std::string>, Eigen::aligned_allocator<std::pair<const std::string, Eigen::Affine3d>>>  PosesMap;
  //typedef std::pair<std::string,Eigen::Affine3d>  PosesPair;

  class PickObjects: public SkillBase
  {
    protected:
      
      ros::NodeHandle m_nh;
      ros::NodeHandle m_pnh; 

      std::map<std::string,BoxPtr> m_boxes;

      std::map<std::string,tf::Transform> m_tf;

      ros::ServiceServer m_add_boxes_srv;
      ros::ServiceServer m_remove_boxes_srv;
      ros::ServiceServer m_add_objects_srv;
      ros::ServiceServer m_remove_objects_srv;
      ros::ServiceServer m_list_objects_srv;
      ros::ServiceServer m_reset_srv;
      ros::ServiceClient m_attach_object_srv;
      
      std::map<std::string,std::shared_ptr<actionlib::SimpleActionServer<manipulation_msgs::PickObjectsAction>>> m_pick_servers;
  
    public:
      PickObjects(const ros::NodeHandle& nh,
                  const ros::NodeHandle& pnh,
                  const std::string& skill_name );

      bool init();

      bool addBoxesCb(manipulation_msgs::AddBoxes::Request& req,
                      manipulation_msgs::AddBoxes::Response& res);

      bool removeBoxesCb( manipulation_msgs::RemoveBoxes::Request& req,
                          manipulation_msgs::RemoveBoxes::Response& res);

      bool addObjectsCb(manipulation_msgs::AddObjects::Request& req,
                        manipulation_msgs::AddObjects::Response& res);

      bool removeObjectsCb( manipulation_msgs::RemoveObjects::Request& req,
                            manipulation_msgs::RemoveObjects::Response& res);

      bool listObjectsCb( manipulation_msgs::ListOfObjects::Request& req,
                          manipulation_msgs::ListOfObjects::Response& res);

      bool resetBoxesCb(manipulation_msgs::ResetBoxes::Request& req, 
                        manipulation_msgs::ResetBoxes::Response& res); 
                        
      void pickObjectGoalCb(const manipulation_msgs::PickObjectsGoalConstPtr& goal,
                            const std::string& group_name);

      void publishTF();

  };

}