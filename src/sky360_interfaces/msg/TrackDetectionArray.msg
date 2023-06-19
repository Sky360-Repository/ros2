# A list of detections, for a multi-object 2D detector.

std_msgs/Header header

# A list of the detections with each trajectory being 
#  associated with a 2D detected blob.
string id
int32 state
vision_msgs/BoundingBox2D[] detections