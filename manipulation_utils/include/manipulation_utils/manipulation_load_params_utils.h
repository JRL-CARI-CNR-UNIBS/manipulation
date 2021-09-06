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
#include <Eigen/Geometry>
#include <tf/transform_listener.h>

#include <manipulation_msgs/AddObjects.h>

namespace manipulation 
{

  Eigen::Affine3d poseFromParam(const XmlRpc::XmlRpcValue& config);
  Eigen::Affine3d poseFromTF(tf::TransformListener& listener, const std::string& frame);

  manipulation_msgs::Grasp graspFromParam(const XmlRpc::XmlRpcValue& config);
  manipulation_msgs::Object objectFromParam(const XmlRpc::XmlRpcValue& config);

  bool loadObjectGrapFromParam(const ros::NodeHandle& nh,
                                   const XmlRpc::XmlRpcValue& config,
                                   const Eigen::Affine3d &T_w_frame,
                                   const Eigen::Affine3d &T_w_object,
                                   manipulation_msgs::Object& object);

  class InboundPickFromParam
  {
  protected:
    ros::NodeHandle nh_;

    ros::ServiceClient add_box_client_;
    ros::ServiceClient add_objs_client_;
    ros::ServiceClient add_objs_to_scene_client_;
    ros::ServiceClient change_color_client_;
    tf::TransformListener listener_;

  public:
    InboundPickFromParam(const ros::NodeHandle& nh);
    bool readBoxesFromParam();
    bool readObjectFromParam();
  };

  class OutboundPlaceFromParam
  {
  protected:
    ros::NodeHandle nh_;
    tf::TransformListener listener_;

    ros::ServiceClient add_slots_group_client_;
    ros::ServiceClient add_slots_client_;

  public:
    OutboundPlaceFromParam(const ros::NodeHandle& nh);
    bool readSlotsGroupFromParam();
    bool readSlotsFromParam();
  };


  class GoToLocationFromParam
  {
  protected:
    ros::NodeHandle nh_;

  public:
    GoToLocationFromParam(const ros::NodeHandle& nh);
    bool readLocationsFromParam();
  };

}

