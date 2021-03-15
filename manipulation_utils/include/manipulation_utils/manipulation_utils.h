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

#include <manipulation_msgs/Box.h>
#include <manipulation_msgs/Grasp.h>
#include <manipulation_msgs/Object.h>
#include <manipulation_msgs/Slot.h>


namespace manipulation 
{

  /* add a location from the location manager
  */
  bool addLocation( const ros::NodeHandle& nh,
                    const manipulation_msgs::Location& location);

  /* remove a location from the location manager
  */
  bool removeLocation(const ros::NodeHandle& nh,
                      const std::string& location_name);

  class Grasp
  {
  protected:
    bool m_int_state;
    std::string m_tool_name;
    std::string m_location_name;  // to keep trace about the location inserted in the LocationManager

    ros::NodeHandle m_nh;

  public:
    /* Grasp constructor
    */
    Grasp(const ros::NodeHandle& nh,
          const manipulation_msgs::Grasp& grasp);
      
    /* destructor
    */
    ~Grasp();

    /* get the tool name to grasp the object
    */
    std::string getToolName(){return m_tool_name;}

    /* get the location name
    */
    std::string getLocationName(){return m_location_name;}

    /* get object internal state
    */
    bool getIntState(){return m_int_state;}

  };
  typedef std::shared_ptr<Grasp> GraspPtr;

  class Object 
  {
  protected:
    bool m_int_state;
    std::string m_name;
    std::string m_type;

    ros::NodeHandle m_nh;

    std::vector<GraspPtr> m_grasp;
  
  public:
    /* Object constructor
    */
    Object( const ros::NodeHandle& nh,
            const manipulation_msgs::Object& object);

    /* destructor
    */
    ~Object();

    /* get the name of the object
    */
    std::string getName(){return m_name;}

    /* get the name of the grasping location
    */
    std::vector<std::string> getGraspLocationNames();

    /* get the grasping location 
    */
    manipulation::GraspPtr getGrasp(const std::string& grasp_location_name);

    /* get the type of the object
    */
    std::string getType(){return m_type;}

    /* get object internal state
    */
    bool getIntState(){return m_int_state;}

  };
  typedef std::shared_ptr<Object> ObjectPtr;

  class Box
  {
  protected:
    bool m_int_state;
    std::string m_name;
    double m_height;
    std::string m_location_name;  // to keep trace about the location inserted in the LocationManager
    
    ros::NodeHandle m_nh;

    std::map<std::string,ObjectPtr> m_objects;

  public:
    /* Box constructor
    */
    Box(const ros::NodeHandle& nh,
        const manipulation_msgs::Box& box);

    /* destructor
    */
    ~Box();

    /* get box's name
    */
    std::string getName(){return m_name;}

    /* add an object
    */
    bool addObject(const manipulation_msgs::Object& object);

    /* add an object
    */
    bool addObject(const manipulation::ObjectPtr& object);

    /* remove all objects
    */
    void removeAllObjects();

    /* remove object with a specific name, return false if the object is not in the box
    */
    bool removeObject(const std::string& object_name);
 
    /* find an object with a specific name, return false if the object is not in the box
    */
    bool findObject(const std::string& object_name);

    /* find an object by a grasping location name, return the object name correpondig to a grasping location name
    */
    std::string findObjectByGraspingLocation(const std::string& grasp_location_name);

    /* get a specific object 
    */
    ObjectPtr getObject(const std::string& object_name){return m_objects.at(object_name);}

    /* get all objects
    */
    std::vector<ObjectPtr> getAllObjects();

    /* get all the objects of this type
    */
    std::vector<ObjectPtr> getObjectsByType(const std::string& object_type);

    /* get box height
    */
    const double& getHeight(){return m_height;}

    /* get the location name
    */
    std::string getLocationName(){return m_location_name;}

    /* get object internal state
    */
    bool getIntState(){return m_int_state;}

  };
  typedef std::shared_ptr<Box> BoxPtr;


  class Slot 
  {
  protected:
    bool m_int_state;
    std::string m_name;
    int m_slot_size; // m_slot_size < 0 means infinite space

    std::string m_location_name;  // to keep trace about the location inserted in the LocationManager
    
    int m_slot_availability;

    ros::NodeHandle m_nh;

  public:
    /* Slot constructor
    */
    Slot( const ros::NodeHandle& nh,
          const manipulation_msgs::Slot& slot);

    /* destructor
    */
    ~Slot();

    /* get the name of the slot
    */
    std::string getName(){return m_name;}

    /* get the availability of the slot
    */
    bool getSlotAvailability();

    /* get the slot size
    */
    bool getSlotSize(){return m_slot_size;};
    
    /* get the location name
    */
    std::string getLocationName(){return m_location_name;}

    /* get object internal state
    */
    bool getIntState(){return m_int_state;}

    /* add object to slot
    */
    void addObjectToSlot();

    /* remove object from slot
    */
    void removeObjectFromSlot();

    /* reset object of the slot
    */
    void resetSlot();

  };
  typedef std::shared_ptr<Slot> SlotPtr;


}