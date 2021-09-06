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

#include <geometry_msgs/PoseStamped.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf_conversions/tf_eigen.h>
#include <rosparam_utilities/rosparam_utilities.h>
#include <object_loader_msgs/AddObjects.h>
#include <object_loader_msgs/ChangeColor.h>

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

Eigen::Affine3d poseFromTF(tf::TransformListener& listener, const std::string& frame)
{
  tf::StampedTransform transform;
  ros::Time t0 = ros::Time::now();
  if (!listener.waitForTransform("world",frame,t0,ros::Duration(10)))
  {
    ROS_WARN("Unable to find a transform from world to %s", frame.c_str());
    throw std::invalid_argument("unable to compute transform");
  }
  try
  {
    listener.lookupTransform("world", frame, t0, transform);
  }
  catch (tf::TransformException& ex)
  {
    ROS_ERROR("%s",ex.what());
    throw std::invalid_argument("unable to compute transform");
  }

  Eigen::Affine3d T_w_frame;
  tf::poseTFToEigen(transform,T_w_frame);
  return T_w_frame;
}

bool poseFromParam(const XmlRpc::XmlRpcValue& config, Eigen::Affine3d& T)
{
  std::string what;
  std::vector<double> position;
  if( !rosparam_utilities::getParam(config,"position",position,what) )
  {
    ROS_WARN("The element  has not the field 'position'");
    return false;
  }
  assert(position.size()==3);

  std::vector<double> quaternion;
  if( !rosparam_utilities::getParam(config,"quaternion",quaternion,what) )
  {
    ROS_WARN("The element has not the field 'quaternion'");
    return false;
  }
  assert(quaternion.size()==4);

  Eigen::Quaterniond q(quaternion.at(3),
                       quaternion.at(0),
                       quaternion.at(1),
                       quaternion.at(2));

  T = q;
  T.translation()(0) = position.at(0);
  T.translation()(1) = position.at(1);
  T.translation()(2) = position.at(2);
  return true;
}



manipulation_msgs::Grasp graspFromParam(const XmlRpc::XmlRpcValue& config)
{
  manipulation_msgs::Grasp grasp;
  if (not (config.hasMember("position") &&
           config.hasMember("quaternion") &&
           config.hasMember("approach_distance") &&
           config.hasMember("tool")))
  {
    ROS_WARN_STREAM("Wrong grasp format:\n" << config);
    throw std::invalid_argument("wrong grasp format");
  }
  assert(config["position"].size()==3);
  assert(config["quaternion"].size()==4);
  assert(config["approach_distance"].size()==4);

  std::vector<double> position;
  rosparam_utilities::getParam(config,"position",position);
  grasp.location.pose.position.x=position.at(0);
  grasp.location.pose.position.y=position.at(1);
  grasp.location.pose.position.z=position.at(2);

  std::vector<double> quaternion;
  rosparam_utilities::getParam(config,"quaternion",quaternion);
  grasp.location.pose.orientation.x=quaternion.at(0);
  grasp.location.pose.orientation.y=quaternion.at(1);
  grasp.location.pose.orientation.z=quaternion.at(2);
  grasp.location.pose.orientation.w=quaternion.at(3);

  std::vector<double> approach_distance;
  rosparam_utilities::getParam(config,"approach_distance",approach_distance);
  grasp.location.approach_relative_pose.position.x=approach_distance.at(0);
  grasp.location.approach_relative_pose.position.y=approach_distance.at(1);
  grasp.location.approach_relative_pose.position.z=approach_distance.at(2);
  grasp.location.approach_relative_pose.orientation.x=0.0;
  grasp.location.approach_relative_pose.orientation.y=0.0;
  grasp.location.approach_relative_pose.orientation.z=0.0;
  grasp.location.approach_relative_pose.orientation.w=1.0;

  grasp.location.leave_relative_pose=grasp.location.approach_relative_pose;
  grasp.tool_name=rosparam_utilities::toString( config["tool"] );
  return grasp;
}

manipulation_msgs::Object objectFromParam(const XmlRpc::XmlRpcValue& config)
{
  manipulation_msgs::Object object;
  if (not (config.hasMember("type") &&
           config.hasMember("grasp_poses")))
  {
    ROS_WARN_STREAM("Wrong object format:\n" << config);
    throw std::invalid_argument("wrong object format");
  }
  object.type=rosparam_utilities::toString( config["type"] );
  if (config.hasMember("name"))
    object.name=rosparam_utilities::toString( config["name"] );

  XmlRpc::XmlRpcValue graspes=config["grasp_poses"];
  if (graspes.getType()!=XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_WARN_STREAM("Wrong object format:\n" << config);
    throw std::invalid_argument("wrong object format");
  }
  for (int idx=0;idx<graspes.size();idx++)
  {
    XmlRpc::XmlRpcValue& grasp=graspes[idx];
    object.grasping_locations.push_back(graspFromParam(grasp));
  }
  return object;
}

InboundPickFromParam::InboundPickFromParam( const ros::NodeHandle &nh):
                                            nh_(nh)
{
  add_box_client_ = nh_.serviceClient<manipulation_msgs::AddBoxes>("add_boxes");
  add_objs_client_ = nh_.serviceClient<manipulation_msgs::AddObjects>("add_objects");
  add_objs_to_scene_client_ = nh_.serviceClient<object_loader_msgs::AddObjects>("/add_object_to_scene");
  change_color_client_ = nh_.serviceClient<object_loader_msgs::ChangeColor>("/change_color");

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
  for(int i=0; i < config.size(); i++)
  {
    XmlRpc::XmlRpcValue box = config[i];
    if( box.getType() != XmlRpc::XmlRpcValue::TypeStruct)
    {
      ROS_WARN("The element #%d is not a struct", i);
      continue;
    }

    if( !box.hasMember("name") )
    {
      ROS_WARN("The element #%d has not the field 'name'", i);
      continue;
    }
    std::string box_name = rosparam_utilities::toString(box["name"]);
    ROS_INFO("Found box named: %s",box_name.c_str());

    if( !box.hasMember("frame") )
    {
      ROS_WARN("The element #%d has not the field 'frame'", i);
      continue;
    }
    std::string frame_name = rosparam_utilities::toString(box["frame"]);
    ROS_INFO("Found box frame name: %s",frame_name.c_str());

    Eigen::Affine3d T_frame_box;
    if (not poseFromParam(box,T_frame_box))
    {
      ROS_WARN("The element #%d has not the field 'position' and/or 'quaternion'", i);
      continue;
    }

    std::vector<double> approach_distance_d;
    if( !rosparam_utilities::getParam(box,"approach_distance",approach_distance_d) )
    {
      ROS_WARN("The box %s has not the field 'approach_distance'",box_name.c_str());
      return false;
    }
    assert(approach_distance_d.size() == 3);
    Eigen::Vector3d approach_distance_in_frame;
    approach_distance_in_frame(0) = approach_distance_d.at(0);
    approach_distance_in_frame(1) = approach_distance_d.at(1);
    approach_distance_in_frame(2) = approach_distance_d.at(2);

    Eigen::Affine3d T_w_frame=poseFromTF(listener_,frame_name);

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

  XmlRpc::XmlRpcValue type_config;
  if (!nh_.getParam("manipulation_object_types",type_config))
  {
    ROS_WARN("Type manipulation_object_types does not exist");
    return false;
  }
  if (type_config.getType()!=XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_ERROR("manipulation_object_types is not a vector of types");
    return false;
  }
  std::map<std::string,manipulation_msgs::Object> types;
  for (int itype=0;itype<type_config.size();itype++)
  {
    manipulation_msgs::Object obj=objectFromParam(type_config[itype]);
    types.insert(std::pair<std::string,manipulation_msgs::Object>(obj.type,obj));
  }


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
  ROS_DEBUG("There are %d objects",config.size());

  std::map<std::string,std::shared_ptr<manipulation_msgs::AddObjects>> add_objs_srv;

  for(int i=0; i < config.size(); i++)
  {
    XmlRpc::XmlRpcValue object_config = config[i];
    if( object_config.getType() != XmlRpc::XmlRpcValue::TypeStruct)
    {
      ROS_WARN("The element #%d is not a struct", i);
      continue;
    }
    if( !object_config.hasMember("type") )
    {
      ROS_WARN("The element #%d has not the field 'type'", i);
      continue;
    }
    std::string type = rosparam_utilities::toString(object_config["type"]);
    if (types.find(type)==types.end())
    {
      ROS_WARN("The element #%d has a unrecognized type '%s'", i,type.c_str());
      continue;
    }


    if( !object_config.hasMember("inbound") )
    {
      ROS_WARN("The element #%d has not the field 'inbound'", i);
      continue;
    }
    std::string box_name = rosparam_utilities::toString(object_config["inbound"]);

    if (add_objs_srv.count(box_name)==0)
      add_objs_srv.insert(std::pair<std::string,std::shared_ptr<manipulation_msgs::AddObjects>>(box_name,std::make_shared<manipulation_msgs::AddObjects>()));


    manipulation_msgs::Object obj=types.at(type);

    if( !object_config.hasMember("frame") )
    {
      ROS_WARN("The element #%d has not the field 'frame'", i);
      continue;
    }
    std::string frame_name = rosparam_utilities::toString(object_config["frame"]);

    ROS_DEBUG("Object type: %s, box name: %s, frame: %s", type.c_str(),box_name.c_str(),frame_name.c_str());


    if( object_config.hasMember("approach_distance") )
    {
      std::vector<double> approach_distance;
      if( !rosparam_utilities::getParam(object_config,"approach_distance",approach_distance) )
      {
        ROS_WARN("Object %s has not the field 'approach_distance'",obj.type.c_str());
        return false;
      }
      assert(approach_distance_d.size()==3);
      for (manipulation_msgs::Grasp& g: obj.grasping_locations)
      {
        g.location.approach_relative_pose.position.x=approach_distance.at(0);
        g.location.approach_relative_pose.position.y=approach_distance.at(1);
        g.location.approach_relative_pose.position.z=approach_distance.at(2);
      }
    }

    if( object_config.hasMember("leave_distance") )
    {
      std::vector<double> leave_distance;
      if( !rosparam_utilities::getParam(object_config,"leave_distance",leave_distance) )
      {
        ROS_WARN("Object %s has not the field 'leave_distance'",obj.type.c_str());
        return false;
      }
      assert(approach_distance_d.size()==3);
      for (manipulation_msgs::Grasp& g: obj.grasping_locations)
      {
        g.location.leave_relative_pose.position.x=leave_distance.at(0);
        g.location.leave_relative_pose.position.y=leave_distance.at(1);
        g.location.leave_relative_pose.position.z=leave_distance.at(2);
      }
    }

    Eigen::Affine3d T_frame_object;
    if (not poseFromParam(object_config,T_frame_object))
    {
      ROS_WARN("The element #%d has not the field 'position' and/or 'quaternion'", i);
      continue;
    }

    object_loader_msgs::Object col_obj;
    tf::poseEigenToMsg(T_frame_object,col_obj.pose.pose);
    col_obj.pose.header.frame_id = frame_name;
    col_obj.object_type = type;
    object_loader_msgs::AddObjects srv;
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
    obj.name=srv.response.ids.at(0);

    for (manipulation_msgs::Grasp& g: obj.grasping_locations)
    {
      g.location.frame=obj.name;
    } // locations are in frame name!!!

    // if manipulator is unable to manage the object, color it red
    object_loader_msgs::ChangeColor color_srv;
    std_msgs::ColorRGBA color_msg;
    color_msg.a=0.5;
    color_msg.r=1.0;
    color_msg.g=0.0;
    color_msg.b=0.0;
    color_srv.request.ids.push_back(obj.name);
    color_srv.request.colors.push_back(color_msg);
    add_objs_srv.at(box_name)->request.box_name = box_name;
    add_objs_srv.at(box_name)->request.add_objects.clear();

    add_objs_srv.at(box_name)->request.add_objects.push_back(obj);
    if (!add_objs_client_.call(*add_objs_srv.at(box_name)))
    {
      ROS_ERROR("Something went wrong when calling the service ~/add_objects");
      change_color_client_.call(color_srv);
      continue;
    }

    if (add_objs_srv.at(box_name)->response.results != manipulation_msgs::AddObjects::Response::Success)
    {
      change_color_client_.call(color_srv);
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


    Eigen::Affine3d T_frame_slot;
    if (not poseFromParam(slot,T_frame_slot))
    {
      ROS_WARN("The slot #%d has not the field 'position' and/or 'quaternion'", i);
      continue;
    }

    Eigen::Affine3d T_w_frame=poseFromTF(listener_,frame);

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
