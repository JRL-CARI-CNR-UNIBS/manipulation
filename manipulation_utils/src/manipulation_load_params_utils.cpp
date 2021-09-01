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

#include <Eigen/Geometry>
#include <geometry_msgs/PoseStamped.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <rosparam_utilities/rosparam_utilities.h>
#include <object_loader_msgs/AddObjects.h>

#include <manipulation_msgs/Box.h>
#include <manipulation_msgs/Grasp.h>
#include <manipulation_msgs/Location.h>
#include <manipulation_msgs/AddBoxes.h>
#include <manipulation_msgs/AddObjects.h>
#include <manipulation_msgs/AddSlots.h>
#include <manipulation_msgs/AddSlotsGroup.h>
#include <manipulation_msgs/PickObjectsAction.h>
#include <manipulation_utils/manipulation_utils.h> 
#include <manipulation_utils/manipulation_load_params_utils.h> 

namespace manipulation 
{

InboundPickFromParam::InboundPickFromParam( const ros::NodeHandle &nh):
                                            nh_(nh)
{
  add_box_client_ = nh_.serviceClient<manipulation_msgs::AddBoxes>("add_boxes");
  add_objs_client_ = nh_.serviceClient<manipulation_msgs::AddObjects>("add_objects");
  add_objs_to_scene_client_ = nh_.serviceClient<object_loader_msgs::AddObjects>("/add_object_to_scene");
  
  ROS_INFO("Waiting for: %s server", add_objs_client_.getService().c_str());
  add_objs_client_.waitForExistence();
  ROS_INFO("Client %s connected to server", add_objs_client_.getService().c_str());

  ROS_INFO("Waiting for: %s server", add_box_client_.getService().c_str());
  add_box_client_.waitForExistence();
  ROS_INFO("Client %s connected to server", add_box_client_.getService().c_str());

  ROS_INFO("Scene spawner is waiting %s", add_objs_to_scene_client_.getService().c_str());
  add_objs_to_scene_client_.waitForExistence();
  ROS_INFO("Client %s connected to server", add_objs_to_scene_client_.getService().c_str());  

}

bool InboundPickFromParam::readBoxesFromParam()
{
  XmlRpc::XmlRpcValue config;
  if (!nh_.getParam("/inbound/boxes",config))
  {
    ROS_ERROR("Unable to find /inboud/boxes");
    return false;
  }

  if (config.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_ERROR("The param is not a list of boxes" );
    return false;
  }

  std::vector<manipulation_msgs::Box> boxes;

  ROS_INFO("There are %u boxes",config.size());
  for(size_t i=0; i < config.size(); i++)
  {
    XmlRpc::XmlRpcValue box = config[i];
    if( box.getType() != XmlRpc::XmlRpcValue::TypeStruct)
    {
      ROS_WARN("The element #%zu is not a struct", i);
      continue;
    }

    if( !box.hasMember("name") )
    {
      ROS_WARN("The element #%zu has not the field 'name'", i);
      continue;
    }
    std::string box_name = rosparam_utilities::toString(box["name"]);
    ROS_INFO("Found box named: %s",box_name.c_str());

    if( !box.hasMember("frame") )
    {
      ROS_WARN("The element #%zu has not the field 'frame'", i);
      continue;
    }
    std::string frame_name = rosparam_utilities::toString(box["frame"]);
    ROS_INFO("Found box frame name: %s",frame_name.c_str());

    std::string what;
    std::vector<double> position;
    if( !rosparam_utilities::getParam(box,"position",position,what) )
    {
      ROS_WARN("The element #%zu has not the field 'position'", i);
      continue;
    }
    assert(position.size()==3);

    std::vector<double> quaternion;
    if( !rosparam_utilities::getParam(box,"quaternion",quaternion,what) )
    {
      ROS_WARN("The element #%zu has not the field 'quaternion'", i);
      continue;
    }
    assert(quaternion.size()==4);

    Eigen::Quaterniond q(quaternion.at(3),
                         quaternion.at(0),
                         quaternion.at(1),
                         quaternion.at(2));

    Eigen::Affine3d T_frame_box;
    T_frame_box = q;
    T_frame_box.translation()(0) = position.at(0);
    T_frame_box.translation()(1) = position.at(1);
    T_frame_box.translation()(2) = position.at(2);

    std::vector<double> approach_distance_d;
    if( !rosparam_utilities::getParam(box,"approach_distance",approach_distance_d,what) )
    {
      ROS_WARN("The box %s has not the field 'approach_distance'",box_name.c_str());
      return false;
    }
    assert(approach_distance_d.size() == 3);
    Eigen::Vector3d approach_distance_in_frame;
    approach_distance_in_frame(0) = approach_distance_d.at(0);
    approach_distance_in_frame(1) = approach_distance_d.at(1);
    approach_distance_in_frame(2) = approach_distance_d.at(2);


    tf::TransformListener listener;
    tf::StampedTransform transform;
    ros::Time t0 = ros::Time::now();
    if (!listener.waitForTransform("world",frame_name,t0,ros::Duration(10)))
    {
      ROS_WARN("Unable to find a transform from world to %s", frame_name.c_str());
      continue;
    }

    try
    {
      listener.lookupTransform("world", frame_name, t0, transform);
    }
    catch (tf::TransformException& ex)
    {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }

    Eigen::Affine3d T_w_frame;
    tf::poseTFToEigen(transform,T_w_frame);

    Eigen::Affine3d T_w_box = T_w_frame * T_frame_box;

    Eigen::Vector3d approach_distance_in_world = T_w_frame.linear() * approach_distance_in_frame;

    Eigen::Affine3d T_w_approach = T_w_box;
    T_w_approach.translation() += approach_distance_in_world;

    Eigen::Affine3d T_box_approach = T_w_box.inverse() * T_w_approach;  

    manipulation_msgs::Box box_;
    box_.name = box_name;
    box_.location.name = box_.name;
    box_.location.frame = "world";
    tf::poseEigenToMsg(T_w_box,box_.location.pose);
    tf::poseEigenToMsg(T_box_approach,box_.location.approach_relative_pose);
    tf::poseEigenToMsg(T_box_approach,box_.location.leave_relative_pose);
    boxes.push_back(box_);
  }

  if (boxes.size()!=0)
  {
    manipulation_msgs::AddBoxes add_boxes_srv;
    add_boxes_srv.request.add_boxes = boxes;

    if (!add_box_client_.call(add_boxes_srv))
      return false;

    ROS_INFO("Added %d boxes and %d objects.",  add_boxes_srv.response.added_boxes,
                                                add_boxes_srv.response.added_objects);  
  }
  else
    ROS_WARN("Can't add any box to the location manager.");
  
  return true;
}


bool InboundPickFromParam::readObjectFromParam()
{

  XmlRpc::XmlRpcValue config;
  if (!nh_.getParam("/inbound/objects",config))
  {
    ROS_ERROR("Unable to find /inbound/objects");
    return false;
  }

  if (config.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_ERROR("The param is not a list of objects" );
    return false;
  }
  ROS_INFO("There are %d objects",config.size());

  object_loader_msgs::AddObjects srv;
  std::map<std::string,std::shared_ptr<manipulation_msgs::AddObjects>> add_objs_srv;

  int obj_type_idx = 0;
  std::string prev_obj_type;

  for(size_t i=0; i < config.size(); i++)
  {
    XmlRpc::XmlRpcValue object = config[i];
    if( object.getType() != XmlRpc::XmlRpcValue::TypeStruct)
    {
      ROS_WARN("The element #%zu is not a struct", i);
      continue;
    }
    if( !object.hasMember("type") )
    {
      ROS_WARN("The element #%zu has not the field 'type'", i);
      continue;
    }
    std::string type = rosparam_utilities::toString(object["type"]);
    
    XmlRpc::XmlRpcValue type_config;
    if (!nh_.getParam("/manipulation_objects/" + type,type_config))
    {
      ROS_WARN("Type %s does not exist",type.c_str());
      continue;
    }

    if( !object.hasMember("inbound") )
    {
      ROS_WARN("The element #%zu has not the field 'inbound'", i);
      continue;
    }
    std::string box_name = rosparam_utilities::toString(object["inbound"]);

    if (add_objs_srv.count(box_name)==0)
      add_objs_srv.insert(std::pair<std::string,std::shared_ptr<manipulation_msgs::AddObjects>>(box_name,std::make_shared<manipulation_msgs::AddObjects>()));
    
    if (type.compare(prev_obj_type) != 0) 
    {
      obj_type_idx = 0;
      prev_obj_type = type;
    }

    add_objs_srv.at(box_name)->request.box_name = box_name;
    manipulation_msgs::Object obj;
    obj.type = type;
    obj.name = type + "_" + std::to_string(obj_type_idx++);

    if( !object.hasMember("frame") )
    {
      ROS_WARN("The element #%zu has not the field 'frame'", i);
      continue;
    }
    std::string frame_name = rosparam_utilities::toString(object["frame"]);

    ROS_INFO("Object type: %s, box name: %s, frame: %s", type.c_str(),box_name.c_str(),frame_name.c_str());

    std::string what;
    std::vector<double> position;
    if( !rosparam_utilities::getParam(object,"position",position, what) )
    {
      ROS_WARN("Pose has not the field 'position'");
      continue;
    }

    assert(position.size()==3);

    std::vector<double> quaternion;
    if( !rosparam_utilities::getParam(object,"quaternion",quaternion,what) )
    {
      ROS_WARN("pose has not the field 'quaternion'");
      continue;
    }
    assert(quaternion.size()==4);

    std::vector<double> approach_distance_d;
    if( !rosparam_utilities::getParam(object,"approach_distance",approach_distance_d,what) )
    {
      ROS_WARN("Object %s has not the field 'approach_distance'",obj.name.c_str());
      return false;
    }
    assert(approach_distance_d.size()==3);
    Eigen::Vector3d approach_distance_in_frame;
    approach_distance_in_frame(0) = approach_distance_d.at(0);
    approach_distance_in_frame(1) = approach_distance_d.at(1);
    approach_distance_in_frame(2) = approach_distance_d.at(2);


    tf::TransformListener listener;
    tf::StampedTransform transform;
    ros::Time t0 = ros::Time::now();
    if (!listener.waitForTransform("world",frame_name,t0,ros::Duration(10)))
    {
      ROS_WARN("Unable to find a transform from world to %s", frame_name.c_str());
      continue;
    }

    try
    {
      listener.lookupTransform("world",frame_name,t0,transform);
    }
    catch (tf::TransformException& ex)
    {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }

    Eigen::Affine3d T_w_frame;
    tf::poseTFToEigen(transform,T_w_frame);

    Eigen::Quaterniond q(quaternion.at(3),
                         quaternion.at(0),
                         quaternion.at(1),
                         quaternion.at(2));
    Eigen::Affine3d T_frame_object;
    T_frame_object = q;
    T_frame_object.translation()(0) = position.at(0);
    T_frame_object.translation()(1) = position.at(1);
    T_frame_object.translation()(2) = position.at(2);

    Eigen::Affine3d T_w_object = T_w_frame * T_frame_object;

    manipulation_msgs::Grasp grasp_obj;

    if( !type_config.hasMember("grasp_poses") )
    {
      ROS_WARN("The element #%zu has not the field 'box'", i);
      continue;
    }
    if (type_config["grasp_poses"].getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
      ROS_ERROR("The param is not a list of grasping poses" );
      return false;
    }
    for (unsigned int ig=0;ig<type_config["grasp_poses"].size();ig++)
    {
      XmlRpc::XmlRpcValue pose = type_config["grasp_poses"][ig];

      if( !pose.hasMember("tool") )
      {
        ROS_WARN("The element #%u has not the field 'tool'", ig);
        continue;
      }
      std::string tool_name = rosparam_utilities::toString(pose["tool"]);


      std::vector<double> position;
      if( !rosparam_utilities::getParam(pose,"position",position,what) )
      {
        ROS_WARN("The element #%u has not the field 'name'", ig);
        continue;
      }
      assert(position.size()==3);

      std::vector<double> quaternion;
      if( !rosparam_utilities::getParam(pose,"quaternion",quaternion,what) )
      {
        ROS_WARN("The element #%u has not the field 'name'", ig);
        continue;
      }
      assert(quaternion.size()==4);

      Eigen::Quaterniond q(quaternion.at(3),
                           quaternion.at(0),
                           quaternion.at(1),
                           quaternion.at(2));

      Eigen::Affine3d T_obj_grasp;
      T_obj_grasp = q;
      T_obj_grasp.translation()(0) = position.at(0);
      T_obj_grasp.translation()(1) = position.at(1);
      T_obj_grasp.translation()(2) = position.at(2);

      Eigen::Affine3d T_w_grasp = T_w_object * T_obj_grasp;
      
      Eigen::Vector3d approach_distance_in_world = T_w_frame.linear() * approach_distance_in_frame;

      Eigen::Affine3d T_w_approach = T_w_grasp;
      T_w_approach.translation() += approach_distance_in_world;

      Eigen::Affine3d T_grasp_approach = T_w_grasp.inverse() * T_w_approach;  

      grasp_obj.location.name = obj.name + "_gl"+std::to_string(ig);
      grasp_obj.location.frame = "world";

      tf::poseEigenToMsg(T_w_grasp,grasp_obj.location.pose);
      tf::poseEigenToMsg(T_grasp_approach,grasp_obj.location.approach_relative_pose);
      tf::poseEigenToMsg(T_grasp_approach,grasp_obj.location.leave_relative_pose);

      grasp_obj.tool_name = tool_name;
      obj.grasping_locations.push_back(grasp_obj);
    }

    add_objs_srv.at(box_name)->request.add_objects.push_back(obj);
    if (!add_objs_client_.call(*add_objs_srv.at(box_name)))
    {
      ROS_ERROR("Something went wrong when calling the service ~/add_objects");
      continue;
    }

    if (add_objs_srv.at(box_name)->response.results == manipulation_msgs::AddObjects::Response::Success)
    {
      object_loader_msgs::Object col_obj;
      tf::poseEigenToMsg(T_w_object,col_obj.pose.pose);
      col_obj.pose.header.frame_id = "world";
      col_obj.object_type = type;
      srv.request.objects.push_back(col_obj);

      if (!add_objs_to_scene_client_.call(srv))
      {
        ROS_ERROR("Something went wrong when calling the service ~/add_object_to_scene");
        continue;
      }

      if (!srv.response.success)
      {
        ROS_ERROR("Something wrong when adding collision object");
        continue;
      }

      srv.request.objects.clear();
    }
  }

  //for (const std::pair<std::string,std::shared_ptr<manipulation_msgs::AddObjects>>& p: add_objs_srv)
  //  add_objs_client_.call(*p.second);

  return true;
}

bool InboundPickFromParam::readObjectFromParamWithoutSceneSpawn()
{
  XmlRpc::XmlRpcValue config;
  if (!nh_.getParam("/inbound/objects",config))
  {
    ROS_ERROR("Unable to find /inbound/objects");
    return false;
  }

  if (config.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_ERROR("The param is not a list of objects" );
    return false;
  }
  ROS_INFO("There are %d objects",config.size());

  object_loader_msgs::AddObjects srv;
  std::map<std::string,std::shared_ptr<manipulation_msgs::AddObjects>> add_objs_srv;

  int obj_type_idx = 0;
  std::string prev_obj_type;

  for(size_t i=0; i < config.size(); i++)
  {
    XmlRpc::XmlRpcValue object = config[i];
    if( object.getType() != XmlRpc::XmlRpcValue::TypeStruct)
    {
      ROS_WARN("The element #%zu is not a struct", i);
      continue;
    }
    if( !object.hasMember("type") )
    {
      ROS_WARN("The element #%zu has not the field 'type'", i);
      continue;
    }
    std::string type = rosparam_utilities::toString(object["type"]);

    XmlRpc::XmlRpcValue type_config;
    if (!nh_.getParam("/manipulation_objects/" + type,type_config))
    {
      ROS_WARN("Type %s does not exist",type.c_str());
      continue;
    }

    if( !object.hasMember("inbound") )
    {
      ROS_WARN("The element #%zu has not the field 'inbound'", i);
      continue;
    }
    std::string box_name = rosparam_utilities::toString(object["inbound"]);

    if (add_objs_srv.count(box_name)==0)
      add_objs_srv.insert(std::pair<std::string,std::shared_ptr<manipulation_msgs::AddObjects>>(box_name,std::make_shared<manipulation_msgs::AddObjects>()));

    if (type.compare(prev_obj_type) != 0)
    {
      obj_type_idx = 0;
      prev_obj_type = type;
    }

    add_objs_srv.at(box_name)->request.box_name = box_name;
    manipulation_msgs::Object obj;
    obj.type = type;
    obj.name = type + "_" + std::to_string(obj_type_idx++);

    if( !object.hasMember("frame") )
    {
      ROS_WARN("The element #%zu has not the field 'frame'", i);
      continue;
    }
    std::string frame_name = rosparam_utilities::toString(object["frame"]);

    ROS_INFO("Object type: %s, box name: %s, frame: %s", type.c_str(),box_name.c_str(),frame_name.c_str());

    std::string what;
    std::vector<double> position;
    if( !rosparam_utilities::getParam(object,"position",position, what) )
    {
      ROS_WARN("Pose has not the field 'position'");
      continue;
    }

    assert(position.size()==3);

    std::vector<double> quaternion;
    if( !rosparam_utilities::getParam(object,"quaternion",quaternion,what) )
    {
      ROS_WARN("pose has not the field 'quaternion'");
      continue;
    }
    assert(quaternion.size()==4);

    std::vector<double> approach_distance_d;
    if( !rosparam_utilities::getParam(object,"approach_distance",approach_distance_d,what) )
    {
      ROS_WARN("Object %s has not the field 'approach_distance'",obj.name.c_str());
      return false;
    }
    assert(approach_distance_d.size()==3);
    Eigen::Vector3d approach_distance_in_frame;
    approach_distance_in_frame(0) = approach_distance_d.at(0);
    approach_distance_in_frame(1) = approach_distance_d.at(1);
    approach_distance_in_frame(2) = approach_distance_d.at(2);


    tf::TransformListener listener;
    tf::StampedTransform transform;
    ros::Time t0 = ros::Time::now();
    if (!listener.waitForTransform("world",frame_name,t0,ros::Duration(10)))
    {
      ROS_WARN("Unable to find a transform from world to %s", frame_name.c_str());
      continue;
    }

    try
    {
      listener.lookupTransform("world",frame_name,t0,transform);
    }
    catch (tf::TransformException& ex)
    {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }

    Eigen::Affine3d T_w_frame;
    tf::poseTFToEigen(transform,T_w_frame);

    Eigen::Quaterniond q(quaternion.at(3),
                         quaternion.at(0),
                         quaternion.at(1),
                         quaternion.at(2));
    Eigen::Affine3d T_frame_object;
    T_frame_object = q;
    T_frame_object.translation()(0) = position.at(0);
    T_frame_object.translation()(1) = position.at(1);
    T_frame_object.translation()(2) = position.at(2);

    Eigen::Affine3d T_w_object = T_w_frame * T_frame_object;

    manipulation_msgs::Grasp grasp_obj;

    if( !type_config.hasMember("grasp_poses") )
    {
      ROS_WARN("The element #%zu has not the field 'box'", i);
      continue;
    }
    if (type_config["grasp_poses"].getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
      ROS_ERROR("The param is not a list of grasping poses" );
      return false;
    }
    for (unsigned int ig=0;ig<type_config["grasp_poses"].size();ig++)
    {
      XmlRpc::XmlRpcValue pose = type_config["grasp_poses"][ig];

      if( !pose.hasMember("tool") )
      {
        ROS_WARN("The element #%u has not the field 'tool'", ig);
        continue;
      }
      std::string tool_name = rosparam_utilities::toString(pose["tool"]);


      std::vector<double> position;
      if( !rosparam_utilities::getParam(pose,"position",position,what) )
      {
        ROS_WARN("The element #%u has not the field 'name'", ig);
        continue;
      }
      assert(position.size()==3);

      std::vector<double> quaternion;
      if( !rosparam_utilities::getParam(pose,"quaternion",quaternion,what) )
      {
        ROS_WARN("The element #%u has not the field 'name'", ig);
        continue;
      }
      assert(quaternion.size()==4);

      Eigen::Quaterniond q(quaternion.at(3),
                           quaternion.at(0),
                           quaternion.at(1),
                           quaternion.at(2));

      Eigen::Affine3d T_obj_grasp;
      T_obj_grasp = q;
      T_obj_grasp.translation()(0) = position.at(0);
      T_obj_grasp.translation()(1) = position.at(1);
      T_obj_grasp.translation()(2) = position.at(2);

      Eigen::Affine3d T_w_grasp = T_w_object * T_obj_grasp;

      Eigen::Vector3d approach_distance_in_world = T_w_frame.linear() * approach_distance_in_frame;

      Eigen::Affine3d T_w_approach = T_w_grasp;
      T_w_approach.translation() += approach_distance_in_world;

      Eigen::Affine3d T_grasp_approach = T_w_grasp.inverse() * T_w_approach;

      grasp_obj.location.name = obj.name + "_gl"+std::to_string(ig);
      grasp_obj.location.frame = "world";

      tf::poseEigenToMsg(T_w_grasp,grasp_obj.location.pose);
      tf::poseEigenToMsg(T_grasp_approach,grasp_obj.location.approach_relative_pose);
      tf::poseEigenToMsg(T_grasp_approach,grasp_obj.location.leave_relative_pose);

      grasp_obj.tool_name = tool_name;
      obj.grasping_locations.push_back(grasp_obj);
    }

    add_objs_srv.at(box_name)->request.add_objects.push_back(obj);
    if (!add_objs_client_.call(*add_objs_srv.at(box_name)))
    {
      ROS_ERROR("Something went wrong when calling the service ~/add_objects");
      continue;
    }
  }
  return true;
}

OutboundPlaceFromParam::OutboundPlaceFromParam( const ros::NodeHandle &nh):
                                                nh_(nh)
{
  add_slots_group_client_ = nh_.serviceClient<manipulation_msgs::AddSlotsGroup>("add_slots_group");
  add_slots_client_ = nh_.serviceClient<manipulation_msgs::AddSlots>("add_slots");

  ROS_INFO("Waiting for: %s server", add_slots_group_client_.getService().c_str());
  add_slots_group_client_.waitForExistence();
  ROS_INFO("Client %s connected to server", add_slots_group_client_.getService().c_str());
  
  ROS_INFO("Waiting for: %s server", add_slots_client_.getService().c_str());
  add_slots_client_.waitForExistence();
  ROS_INFO("Client %s connected to server", add_slots_client_.getService().c_str());

}

bool OutboundPlaceFromParam::readSlotsGroupFromParam()
{
  XmlRpc::XmlRpcValue config;
  if (!nh_.getParam("/outbound/slots_group",config))
  {
    ROS_ERROR("Unable to find /outbound/slots_group");
    return false;
  }

  if (config.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_ERROR("The param is not a list of boxed" );
    return false;
  }
  ROS_INFO("There are %d slots group",config.size());

  std::vector<manipulation_msgs::SlotsGroup> slots_group;

  for (int i=0; i < config.size(); i++)
  {
    XmlRpc::XmlRpcValue slot = config[i];
    if( slot.getType() != XmlRpc::XmlRpcValue::TypeStruct)
    {
      ROS_WARN("The element #%d is not a struct", i);
      continue;
    }

    if( !slot.hasMember("name") )
    {
      ROS_WARN("The element #%d has not the field 'name'", i);
      return false;
    }
    std::string name = rosparam_utilities::toString(slot["name"]);

    manipulation_msgs::SlotsGroup slots_group_;
    slots_group_.name = name;

    slots_group.push_back(slots_group_);
  }

  if (slots_group.size()!=0)
  {
    manipulation_msgs::AddSlotsGroup add_slots_group_srv;
    add_slots_group_srv.request.add_slots_groups = slots_group;

    if (!add_slots_group_client_.call(add_slots_group_srv))
      return false;

    ROS_INFO("Added %lu slots groups.", slots_group.size());  
  }
  else
  {
    ROS_WARN("Can't add any slots group.");
    return false;
  }
    
  return true;
}

bool OutboundPlaceFromParam::readSlotsFromParam()
{
  XmlRpc::XmlRpcValue config;
  if (!nh_.getParam("/outbound/slots",config))
  {
    ROS_ERROR("Unable to find /outbound/slots");
    return false;
  }

  if (config.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_ERROR("The param is not a list of boxed" );
    return false;
  }
  ROS_INFO("There are %d slots",config.size());

  for (int i=0; i < config.size(); i++)
  {
    XmlRpc::XmlRpcValue slot = config[i];
    if( slot.getType() != XmlRpc::XmlRpcValue::TypeStruct)
    {
      ROS_WARN("The element #%d is not a struct", i);
      continue;
    }

    if( !slot.hasMember("name") )
    {
      ROS_WARN("The element #%d has not the field 'name'", i);
      return false;
    }
    std::string name = rosparam_utilities::toString(slot["name"]);

    if( !slot.hasMember("slots_group") )
    {
      ROS_WARN("The element #%d has not the field 'slots_group'", i);
      return false;
    }
    std::string slots_group = rosparam_utilities::toString(slot["slots_group"]);

    if( !slot.hasMember("frame") )
    {
      ROS_WARN("The element #%d has not the field 'frame'", i);
      return false;
    }
    std::string frame = rosparam_utilities::toString(slot["frame"]);

    if( !slot.hasMember("max_objects") )
    {
      ROS_WARN("The element #%d has not the field 'max_objects'", i);
      return false;
    }
    int max_objects = rosparam_utilities::toInt(slot["max_objects"]);

    std::string what;
    std::vector<double> approach_distance_d;
    if( !rosparam_utilities::getParam(slot,"approach_distance",approach_distance_d,what) )
    {
      ROS_WARN("Slot %s has not the field 'approach_distance'",name.c_str());
      return false;
    }
    assert(approach_distance_d.size()==3);
    Eigen::Vector3d approach_distance_in_frame;
    approach_distance_in_frame(0) = approach_distance_d.at(0);
    approach_distance_in_frame(1) = approach_distance_d.at(1);
    approach_distance_in_frame(2) = approach_distance_d.at(2);

    std::vector<double> position;
    if( !rosparam_utilities::getParam(slot,"position",position,what) )
    {
      ROS_WARN("Slot %s has not the field 'position'",name.c_str());
      return false;
    }
    assert(position.size()==3);

    std::vector<double> quaternion;
    if( !rosparam_utilities::getParam(slot,"quaternion",quaternion,what) )
    {
      ROS_WARN("Slot %s has not the field 'quaternion'",name.c_str());
      return false;
    }
    assert(quaternion.size()==4);

    Eigen::Quaterniond q(quaternion.at(3),
                          quaternion.at(0),
                          quaternion.at(1),
                          quaternion.at(2));

    Eigen::Affine3d T_frame_slot;
    T_frame_slot = q;
    T_frame_slot.translation()(0) = position.at(0);
    T_frame_slot.translation()(1) = position.at(1);
    T_frame_slot.translation()(2) = position.at(2);

    tf::TransformListener listener;
    tf::StampedTransform transform;
    ros::Time t0 = ros::Time::now();
    if (!listener.waitForTransform("world",frame,t0,ros::Duration(10)))
    {
      ROS_WARN("Unable to find a transform from world to %s", frame.c_str());
      return false;
    }

    try
    {
      listener.lookupTransform("world", frame, t0, transform);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("Exception %s",ex.what());
      ros::Duration(1.0).sleep();
      return false;
    }

    Eigen::Affine3d T_w_frame;
    tf::poseTFToEigen(transform,T_w_frame);

    Eigen::Affine3d T_w_slot = T_w_frame * T_frame_slot;

    Eigen::Vector3d approach_distance_in_world = T_w_frame.linear()*approach_distance_in_frame;

    Eigen::Affine3d T_w_approach = T_w_slot;
    T_w_approach.translation() += approach_distance_in_world;

    Eigen::Affine3d T_slot_approach = T_w_slot.inverse() * T_w_approach;

    manipulation_msgs::Slot slot_;
    slot_.name = name;
    slot_.slot_size = max_objects;
    slot_.location.name = slot_.name;
    slot_.location.frame = "world";
    tf::poseEigenToMsg(T_w_slot,slot_.location.pose);
    tf::poseEigenToMsg(T_slot_approach,slot_.location.approach_relative_pose);
    tf::poseEigenToMsg(T_slot_approach,slot_.location.leave_relative_pose);
    
    std::vector<manipulation_msgs::Slot> slot_vct;
    slot_vct.push_back(slot_);

    manipulation_msgs::AddSlots add_slots_srv;
    add_slots_srv.request.slots_group_name = slots_group;
    add_slots_srv.request.add_slots = slot_vct;

    if (!add_slots_client_.call(add_slots_srv))
    {
      ROS_WARN("Can't add slot to the location manager.");
      return false;
    }
  }

  return true;
}

GoToLocationFromParam::GoToLocationFromParam( const ros::NodeHandle &nh):
                                              nh_(nh)
{
  // nothing to do here
}

bool GoToLocationFromParam::readLocationsFromParam()
{
  XmlRpc::XmlRpcValue go_to_locations;
  if (!nh_.getParam("/go_to_location",go_to_locations))
  {
    ROS_ERROR("Unable to find /go_to_location");
    return false;
  }

  if (go_to_locations.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_ERROR("The param is not a struct of locations" );
    return false;
  }
  ROS_INFO("There are %d objects",go_to_locations.size());

  for (int i=0; i < go_to_locations.size(); i++)
  {   
    XmlRpc::XmlRpcValue single_location = go_to_locations[i];
    if( single_location.getType() != XmlRpc::XmlRpcValue::TypeStruct)
    {
      ROS_WARN("The element #%d is not a struct", i);
      continue;
    }

    if( !single_location.hasMember("name") )
    {
      ROS_WARN("The element #%d has not the field 'name'", i);
      return false;
    }
    std::string name = rosparam_utilities::toString(single_location["name"]);

    if( !single_location.hasMember("frame") )
    {
      ROS_WARN("The element #%d has not the field 'frame'", i);
      return false;
    }
    std::string frame = rosparam_utilities::toString(single_location["frame"]);

    std::string what;
    std::vector<double> position;
    if( !rosparam_utilities::getParam(single_location,"position",position,what) )
    {
      ROS_WARN("Slot %s has not the field 'position'",name.c_str());
      return false;
    }
    assert(position.size()==3);

    std::vector<double> quaternion;
    if( !rosparam_utilities::getParam(single_location,"quaternion",quaternion,what) )
    {
      ROS_WARN("Slot %s has not the field 'quaternion'",name.c_str());
      return false;
    }
    assert(quaternion.size()==4);

    Eigen::Quaterniond q( quaternion.at(3),
                          quaternion.at(0),
                          quaternion.at(1),
                          quaternion.at(2));

    Eigen::Affine3d T_frame_tool;
    T_frame_tool = q;
    T_frame_tool.translation()(0) = position.at(0);
    T_frame_tool.translation()(1) = position.at(1);
    T_frame_tool.translation()(2) = position.at(2);

    tf::TransformListener listener;
    tf::StampedTransform transform;
    ros::Time t0 = ros::Time::now();
    if (!listener.waitForTransform("world",frame,t0,ros::Duration(10)))
    {
      ROS_WARN("Unable to find a transform from world to %s", frame.c_str());
      return false;
    }

    try
    {
      listener.lookupTransform("world", frame, t0, transform);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("Exception %s",ex.what());
      ros::Duration(1.0).sleep();
      return false;
    }

    Eigen::Affine3d T_w_frame;
    tf::poseTFToEigen(transform,T_w_frame);

    Eigen::Affine3d T_w_tool = T_w_frame * T_frame_tool;

    manipulation_msgs::Location goto_location;
    
    goto_location.name = name;
    goto_location.frame = "world";
    tf::poseEigenToMsg(T_w_tool,goto_location.pose);

    manipulation::addLocation(nh_,goto_location);
  }
  return true;
}

}


