# inbound pick #

The PickObjects class provide the following actions/services:
 
| Actions | Type | Description | 
|:--- | :----  | :------------------ | 
| `GROUP_NAME/pick` | manipulation_msgs::PickObjects | Standard action, used to pick an object from a box |


| Services | Type | Description | 
|:--- | :----  | :------------------ | 
| `add_boxes` | manipulation_msgs::AddBoxes | To add a list of boxes |
| `remove_boxes` | manipulation_msgs::RemoveBoxes | To remove a list of boxes |
| `add_objects` | manipulation_msgs::AddObjects | To add a list of objects to a specific box N.B. an object need to be always contained in a box |
| `remove_objects` | manipulation_msgs::RemoveObjects | To remove a list of objects from all the available boxes  |
| `add_locations` | manipulation_msgs::AddLocations | To add a list of locations N.B. the use of this service is not recommended, instead is preferred to add locations through the services add_boxes and add_objects |
| `remove_locations` | manipulation_msgs::RemoveLocations | To remove a list of locations N.B. the use of this service is not recommended, instead is preferred to remove locations through the services remove_boxes and remove_objects  |
| `list_objects` | manipulation_msgs::ListOfObjects | To get a list of all the objects available for picking providing the names and the types | 
| `inbound/reset_box` | manipulation_msgs::ResetBoxes | To reset all the objects in a list of boxes |

