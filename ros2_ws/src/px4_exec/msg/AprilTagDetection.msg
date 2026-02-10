std_msgs/Header header

# Define the tag id:
int32 tag_id 
# Define if hte tag is determined
bool apriltag_detected

# Post the relative position of the camera respect to the camera frame:
geometry_msgs/Point position
# Post the relative orientation of the Apriltag repect to teh cmaera frame:
geometry_msgs/Quaternion orientation