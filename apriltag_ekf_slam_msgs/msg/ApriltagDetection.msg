# Inside header:
#   seq: No use
#   stamp: Time at which the tags were detected
#   frame_id: Frame associated with the pose of the tags
std_msgs/Header header

# Family in which the tags belongs ex: 36h11
string tag_family

# Array of id associated with the tags detected
# For example in case of tag family 36h11 tag_id should be in between [0, 586]
std_msgs/UInt32[] tag_id

# Array of ose of the tags with covariance
geometry_msgs/PoseWithCovariance[] pose
