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
#include <std_srvs/SetBool.h>
#include <manipulation_utils/manipulation_load_params_utils.h>

std::shared_ptr<manipulation::InboundPickFromParam> inb;

bool addObjectsCb(std_srvs::SetBoolRequest& req,
                  std_srvs::SetBoolResponse& res)
{
  if (!inb->readObjectFromParam())
  {
    ROS_ERROR("Unable to load objects in the boxes");
    return false;
  }
  ROS_INFO("load objects complete");
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "inbound_pick_loader");
  ros::NodeHandle server_nh("inbound_pick_server");
  ros::NodeHandle pnh("inbound_pick_loader");

  inb = std::make_shared<manipulation::InboundPickFromParam>(server_nh);

  // Boxes need to be loaded before the Objects because it supposed that objects are
  // always contained by a box
  if (!inb->readBoxesFromParam())
  {
    ROS_ERROR("Unable to load boxes");
    return 0;
  }

  if (!inb->readObjectFromParam())
  {
    ROS_ERROR("Unable to load objects in the boxes");
    return 0;
  }

  ROS_INFO("Inbound boxed loaded");

  ros::ServiceServer s=pnh.advertiseService("add_objects",&addObjectsCb);
  ros::spin();
  return 0;
}
