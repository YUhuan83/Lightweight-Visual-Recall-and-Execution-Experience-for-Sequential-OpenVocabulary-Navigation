# Current tracked obstacles for one sensor/tracker update. Publish this on a
# separate topic from NavigationGraph so its update rate can match perception.

std_msgs/Header header
DynamicObstacle[] obstacles
