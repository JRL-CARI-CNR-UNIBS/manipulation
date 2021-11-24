#include <algorithm>

#include <ros/ros.h>

#include <manipulation_msgs/AddLocations.h>
#include <manipulation_msgs/RemoveLocations.h>

#include <manipulation_utils/manipulation_utils.h>

namespace manipulation
{

bool addLocation( const ros::NodeHandle& nh,
                  const manipulation_msgs::Location& location)
{  
  ros::NodeHandle nh_= nh; 
  ros::ServiceClient add_locations_client = nh_.serviceClient<manipulation_msgs::AddLocations>("add_locations");

  add_locations_client.waitForExistence();

  if(add_locations_client.exists())
  {
    manipulation_msgs::AddLocations add_locations;
    add_locations.request.locations.push_back(location);

    if(!add_locations_client.call(add_locations))
      return false;
    
    if (add_locations.response.results != manipulation_msgs::AddLocations::Response::Error)
      ROS_DEBUG("Added the location %s to the location manager.",location.name.c_str());
    else
      return false;
  }
  else
  {
    ROS_ERROR("The service %s is not available.",add_locations_client.getService().c_str());
    return false;
  }

  return true;
}

bool removeLocation(const ros::NodeHandle& nh,
                    const std::string& location_name)
{
  ros::NodeHandle nh_= nh; 
  ros::ServiceClient remove_locations_client = nh_.serviceClient<manipulation_msgs::RemoveLocations>("remove_locations");
  
  if(remove_locations_client.exists())
  {
    manipulation_msgs::RemoveLocations remove_location;
    remove_location.request.location_names.push_back(location_name);

    if(!remove_locations_client.call(remove_location))
      return false;
    
    if (remove_location.response.results != manipulation_msgs::RemoveLocations::Response::Error)
      ROS_INFO("Removed the location %s from the location manager.",location_name.c_str());
  }
  else
  {
    ROS_ERROR("The service %s is not available.", remove_locations_client.getService().c_str());
    return false;
  }
  
  return true; 
}


// Grasp
Grasp::Grasp( const ros::NodeHandle& nh,
              const manipulation_msgs::Grasp& grasp):
              m_nh(nh),
              m_int_state(true)
{
  if(!addLocation(m_nh,grasp.location))
  {
    m_int_state = false;
    return;
  }
  
  m_tool_name = grasp.tool_name;
  m_location_name = grasp.location.name;
  ROS_DEBUG("Added the new location %s to the location manager.",grasp.location.name.c_str());

  return;
}

Grasp::~Grasp()
{
  if (m_int_state)
  {
    if(!removeLocation(m_nh, m_location_name))
      ROS_ERROR("Can't remove the location %s from location manager.", m_location_name.c_str());
  }
}


// Object
Object::Object( const ros::NodeHandle& nh,
                const manipulation_msgs::Object& object ):
                m_nh(nh),
                m_int_state(true)
{
  m_name = object.name;
  m_type = object.type;


  int igrasp=0;
  std::vector<manipulation_msgs::Grasp> grasps=object.grasping_locations;
  for (manipulation_msgs::Grasp& grasp: grasps )
  {
    if (grasp.location.name.empty())
      grasp.location.name=m_name+"/grasp_"+std::to_string(igrasp++)+"_"+grasp.tool_name;
    if (grasp.location.frame.empty())
    {
      ROS_ERROR("grasp %s has no frame, discard",grasp.location.name.c_str());
      continue;
    }

    m_grasp.push_back(std::make_shared<manipulation::Grasp>(m_nh,grasp));
    if(!m_grasp.back()->getIntState())
    {
      m_int_state = m_grasp.back()->getIntState();
      m_grasp.pop_back();
    }
    else
      ROS_DEBUG("Added the object: %s of the type: %s.", m_name.c_str(), m_type.c_str());
  }
}

Object::~Object()
{
  // nothing to do 
}

std::vector<std::string> Object::getGraspLocationNames(const std::string& tool_name)
{
  std::vector<std::string> grasp_location_names;

  for (std::vector<GraspPtr>::iterator it = m_grasp.begin(); it != m_grasp.end(); ++it)
  {

    if (tool_name.empty() or (*it)->getToolName().compare(tool_name)==0)
      grasp_location_names.push_back((*it)->getLocationName());
  }
  
  return grasp_location_names;
}

manipulation::GraspPtr Object::getGrasp(const std::string& grasp_location_name)
{
  for(const manipulation::GraspPtr& grasp: m_grasp)
  {
    if (grasp->getLocationName() == grasp_location_name)
      return grasp;
  }
  
  ROS_ERROR("Can't find grasping location %s in the location manager.", grasp_location_name.c_str());
  return nullptr;
}


// Box
Box::Box( const ros::NodeHandle& nh,
          const manipulation_msgs::Box& box):
          m_nh(nh),
          m_int_state(true)
{
  if(!addLocation(m_nh,box.location))
  {
    m_int_state = false;
    return;
  }
 
  m_name = box.name;
  m_location_name = box.location.name;
    
  for (const manipulation_msgs::Object& object: box.objects)
  {
    m_objects.insert(std::pair<std::string,ObjectPtr>(object.name, std::make_shared<manipulation::Object>(m_nh,object)));
    if (!m_objects.at(object.name)->getIntState())
      m_objects.erase(m_objects.find(object.name));
  }
  
  ROS_INFO("Added the box %s.", m_name.c_str()); 
}

Box::~Box()
{
  if (m_int_state)
  {
    if(!removeLocation(m_nh,m_location_name))
      ROS_ERROR("Can't remove the location %s from location manager.", m_location_name.c_str());
  }
}

bool Box::addObject(const manipulation_msgs::Object& object)
{
  if (m_objects.find(object.name) != m_objects.end())
  {
    ROS_ERROR("The object: %s of the type: %s already exists in the box %s.", object.name.c_str(),object.type.c_str(),m_name.c_str());
    ROS_ERROR("List of objects in the box");
    for (const std::pair<std::string,ObjectPtr>& p: m_objects)
      ROS_ERROR("- %s",p.first.c_str());
    return false;
  }

  m_objects.insert(std::pair<std::string,ObjectPtr>(object.name,std::make_shared<Object>(m_nh,object)));

  if (!m_objects.at(object.name)->getIntState())
  {
    m_objects.erase(m_objects.find(object.name));
    return false;
  }
    

  return true;
}

bool Box::addObject(const manipulation::ObjectPtr& object)
{  
  if (m_objects.find(object->getName()) != m_objects.end())
  {
    ROS_ERROR("The object: %s of the type: %s already exists in the box %s.", object->getName().c_str(),object->getType().c_str(),m_name.c_str());
    ROS_ERROR("List of objects in the box");
    for (const std::pair<std::string,ObjectPtr>& p: m_objects)
      ROS_ERROR("- %s",p.first.c_str());
    return false;
  }

  m_objects.insert(std::pair<std::string,ObjectPtr>(object->getName(),object));

  if (!m_objects.at(object->getName())->getIntState())
  {
    m_objects.erase(m_objects.find(object->getName()));
    return false;
  }

  return true;
}

void Box::removeAllObjects()
{
  m_objects.clear();
  ROS_INFO("All the objects in the box %s are removed", m_name.c_str());
}

bool Box::removeObject(const std::string& object_name)
{
  if (m_objects.find(object_name) == m_objects.end())
  {
    ROS_ERROR("The object %s is not in the box %s", object_name.c_str(), m_name.c_str());
    return false;
  }

  m_objects.erase(m_objects.find(object_name));
  ROS_INFO("The object %s has been removed from the box %s", object_name.c_str(), m_name.c_str());

  return true;
}

bool Box::findObject(const std::string& object_name)
{
  if (m_objects.find(object_name) == m_objects.end())
  {
    ROS_INFO("Can't find the object %s in the box %s", object_name.c_str(), m_name.c_str());
    return false;
  }

  ROS_INFO("Found the object %s in the box %s", object_name.c_str(), m_name.c_str());

  return true;
}

std::string Box::findObjectByGraspingLocation(const std::string& grasp_location_name)
{
  for(std::map<std::string,ObjectPtr>::iterator it = m_objects.begin(); it != m_objects.end(); ++it)
  {
    std::vector<std::string> location_names = it->second->getGraspLocationNames();
    if (find(location_names.begin(),location_names.end(),grasp_location_name) != location_names.end() )
      return it->first;
  }
  ROS_ERROR("Can't find the corresponding object for grasping location name %s ", grasp_location_name.c_str());
  return std::string();
}

ObjectPtr Box::getObject(const std::string& object_name)
{
  if (m_objects.find(object_name) != m_objects.end())
    return m_objects.at(object_name);
  else
    return nullptr;
}

std::vector<ObjectPtr> Box::getAllObjects()
{
  std::vector<ObjectPtr> objects;
  for (std::map<std::string,ObjectPtr>::iterator it = m_objects.begin(); it != m_objects.end(); ++it)
    objects.push_back(it->second);

  return objects;
}

std::vector<ObjectPtr> Box::getObjectsByType(const std::string& object_type)
{
  std::vector<ObjectPtr> objects;
  for (std::map<std::string,ObjectPtr>::iterator it = m_objects.begin(); it != m_objects.end(); ++it)
  {
    if ( it->second->getType() == object_type )
      objects.push_back(it->second);
  }  

  return objects;
}


// Slot
Slot::Slot( const ros::NodeHandle& nh,
            const manipulation_msgs::Slot& slot):
            m_nh(nh),
            m_int_state(true)
{
  m_name = slot.name;
  m_slot_size = slot.slot_size;

  if (m_slot_size > 0)
    m_slot_availability = m_slot_size;
  else
    m_slot_availability = 0;

  if(!addLocation(m_nh,slot.location))
  {
    m_int_state = false;
    return;
  }
  
  m_location_name = slot.location.name;
  ROS_DEBUG("Added the new location %s to the location manager.",slot.location.name.c_str());

  return;
}

Slot::~Slot()
{
  if (m_int_state)
  {
    if(!removeLocation(m_nh,m_location_name))
      ROS_ERROR("Can't remove the location %s from location manager.", m_location_name.c_str());
  }
}

bool Slot::getSlotAvailability()
{
  // The slot size can be:
  // < 0 it simulate a slot with infinite space such as a conveyor track 
  // = 1 it simulate a slot with that can contain a single element
  // > 1 it simulate a slot a slot that can contain multiple objects, 
  // such as a box where objects are placed in random order

  if (m_slot_size < 0)
    return true;
  
  if (m_slot_availability > 0)
    return true;
  else
    return false;
}

void Slot::addObjectToSlot()
{
  if (m_slot_size > 0 && m_slot_availability > 0)
    m_slot_availability--;
}

void Slot::removeObjectFromSlot()
{
  if (m_slot_size > 0 && m_slot_availability <= m_slot_size)
    m_slot_availability++;
}

void Slot::resetSlot()
{
  if (m_slot_size > 0)
    m_slot_availability = m_slot_size;
}


// SlotsGroup
SlotsGroup::SlotsGroup( const ros::NodeHandle& nh,
                        const manipulation_msgs::SlotsGroup& slots_group):
                        m_nh(nh),
                        m_group_size(0),
                        m_int_state(true)
{
  m_group_name = slots_group.name;

  for (const manipulation_msgs::Slot& slot: slots_group.manipulation_slots)
  {
    m_slots.insert(std::pair<std::string,SlotPtr>(slot.name, std::make_shared<manipulation::Slot>(m_nh,slot)));
    if (!m_slots.at(slot.name)->getIntState())
    {
      m_slots.erase(m_slots.find(slot.name));
      continue;
    }
    computeGroupSize();
  }
  
  ROS_DEBUG("Added the slots group %s.", m_group_name.c_str());

}

SlotsGroup::~SlotsGroup()
{
  // nothing to do
}

bool SlotsGroup::addSlot(const manipulation_msgs::Slot& slot)
{
  if (m_slots.find(slot.name) != m_slots.end())
  {
    ROS_ERROR("The slot: %s already exists in the group %s.", slot.name.c_str(),m_group_name.c_str());
    return false;
  }

  m_slots.insert(std::pair<std::string,SlotPtr>(slot.name,std::make_shared<manipulation::Slot>(m_nh,slot)));

  if (!m_slots.at(slot.name)->getIntState())
  {
    m_slots.erase(m_slots.find(slot.name));
    return false;
  }

  computeGroupSize();

  return true;
}

bool SlotsGroup::addSlot(const manipulation::SlotPtr& slot)
{
  if (m_slots.find(slot->getName()) != m_slots.end())
  {
    ROS_ERROR("The slot: %s already exists in the group %s.", slot->getName().c_str(),m_group_name.c_str());
    return false;
  }

  m_slots.insert(std::pair<std::string,SlotPtr>(slot->getName(),slot));
  if (!m_slots.at(slot->getName())->getIntState())
  {
    m_slots.erase(m_slots.find(slot->getName()));
    return false;
  }

  computeGroupSize();

  return true;
}

bool SlotsGroup::removeSlot(const std::string& slot_name)
{
  if (m_slots.find(slot_name) == m_slots.end())
  {
    ROS_ERROR("The slot %s is not in the group %s", slot_name.c_str(), m_group_name.c_str());
    return false;
  }

  m_slots.erase(m_slots.find(slot_name));
  ROS_INFO("The slot %s has been removed from the group %s", slot_name.c_str(), m_group_name.c_str());

  computeGroupSize();

  return true;

}

bool SlotsGroup::findSlot(const std::string& slot_name)
{
  if (m_slots.find(slot_name) == m_slots.end())
  {
    ROS_DEBUG("Can't find the slot %s in the group %s", slot_name.c_str(), m_group_name.c_str());
    return false;
  }

  ROS_DEBUG("Found the slot %s in the group %s", slot_name.c_str(), m_group_name.c_str());

  return true;
}

SlotPtr SlotsGroup::getSlot(const std::string& slot_name)
{
  if (m_slots.find(slot_name) != m_slots.end())
    return m_slots.at(slot_name);
  else
    return nullptr;
}

std::vector<SlotPtr> SlotsGroup::getAllSlots()
{
  std::vector<SlotPtr> slots;

  for (std::map<std::string,SlotPtr>::iterator it = m_slots.begin(); it != m_slots.end(); ++it)
    slots.push_back(it->second);

  return slots;
}

void SlotsGroup::addObjectToSlot(const std::string& slot_name)
{
  if(m_slots.find(slot_name) != m_slots.end())
    m_slots.at(slot_name)->addObjectToSlot();

  return;
}

void SlotsGroup::removeObjectFromSlot(const std::string& slot_name)
{
  if(m_slots.find(slot_name) != m_slots.end())
    m_slots.at(slot_name)->removeObjectFromSlot();

  return;  
}

void SlotsGroup::resetSlot()
{
  for (std::map<std::string,SlotPtr>::iterator it = m_slots.begin(); it != m_slots.end(); ++it)
    it->second->resetSlot();

  return; 
}

void SlotsGroup::computeGroupSize()
{
  m_group_size = 0;
  for (std::map<std::string,SlotPtr>::iterator it = m_slots.begin(); it != m_slots.end(); ++it)
  {
    if(m_slots.at(it->first)->getSlotSize() < 0 || m_group_size < 0)  
      m_group_size = -1;
    else
      m_group_size += m_slots.at(it->first)->getSlotSize();
  }
    
}

} // end of manipulation namespace
