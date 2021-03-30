# inbound pick #

The PickObjects class provide the following actions/services:
 
| Actions | Type | Description | 
|:--- | :----  | :------------------ | 
| `~/GROUP_NAME/pick` | manipulation_msgs::PickObjects | Standard action, used to pick an object from a box |


| Services | Type | Description | 
|:--- | :----  | :------------------ | 
| `~/add_objects` | manipulation_msgs::AddObjects | To add a list of objects to a specific box |
| `~/add_boxes` | manipulation_msgs::AddBoxes | To add a list of boxes |
| `~/remove_objects` | manipulation_msgs::RemoveObjects | To remove a list of objects from all the available boxes  |
| `~/list_objects` | manipulation_msgs::ListOfObjects | To get a list of all the objects available for picking providing the names and the types | 
| `~/inbound/reset_box` | manipulation_msgs::ResetBoxes | To reset all the objects in a list of boxes |

