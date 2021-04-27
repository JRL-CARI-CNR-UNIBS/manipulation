# go to location #

The GoToLocation class provide the following actions/services:
 
| Actions | Type | Description | 
|:--- | :----  | :------------------ | 
| `GROUP_NAME/go_to` | manipulation_msgs::GoTo | Standard action. Used to move a group in a position |

| Services | Type | Description | 
| `add_locations` | manipulation_msgs::AddLocations | To add a list of locations N.B. the use of this service is not recommended, instead is preferred to add locations through the services add_boxes and add_objects |
| `remove_locations` | manipulation_msgs::RemoveLocations | To remove a list of locations N.B. the use of this service is not recommended, instead is preferred to remove locations through the services remove_boxes and remove_objects  |