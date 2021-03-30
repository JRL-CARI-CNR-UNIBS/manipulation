# outbound place #

The PlaceObjects class provide the following actions/services:
 
| Actions | Type | Description | 
|:--- | :----  | :------------------ | 
| `~/GROUP_NAME/place` | manipulation_msgs::PlaceObjects | Standard action. Used to place an object in a slot |


| Services | Type | Description | 
|:--- | :----  | :------------------ | 
| `~/add_slots` | manipulation_msgs::AddSlots | To add a list of slots |
| `~/remove_slots` | manipulation_msgs::RemoveSlots | To remove a list of slots |
| `~/remove_obj_from_slot` | manipulation_msgs::RemoveObjectFromSlot | To remove a single object from a specific slot  |
| `~/outbound/reset_slot` | manipulation_msgs::ResetSlots | To reset a list of slots from the objects contained |

