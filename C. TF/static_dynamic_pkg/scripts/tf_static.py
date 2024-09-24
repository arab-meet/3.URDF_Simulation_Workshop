#!/usr/bin/env python3

import rospy

from tf2_ros import StaticTransformBroadcaster
# massage that used in tf static 
from geometry_msgs.msg import TransformStamped

class tf_static_ex(object):
    
    def __init__(self):
        
        # instance from static transform broadcaster class 
        # need this object to publish static transform in TF static topic
        self.static_broadcaster = StaticTransformBroadcaster()
        
        # create a static transform message
        self.static_transform_stamped = TransformStamped()
        
        # add information about time when this static transform has been generated
        self.static_transform_stamped.header.stamp = rospy.Time.now()
        
        # as we learn that tf connect two frames together
        # let's define the parent frame and the child frame 
        
        # parent frame
        self.static_transform_stamped.header.frame_id = "base_link"
        # child frame any frame like laser of camera or any other frame
        # important to note that the child frame should be static frame
        self.static_transform_stamped.child_frame_id = "laser" 
        
        # we know that child frame is connected to parent frame by translation and rotation vectors
        # let's define translation vectors
        self.static_transform_stamped.transform.translation.x = 0.0
        self.static_transform_stamped.transform.translation.y = 0.0
        # in z axes 10 cm above the base_link frame
        self.static_transform_stamped.transform.translation.z = 0.1
        
        # let's define rotation vectors 
        self.static_transform_stamped.transform.rotation.x = 0.0
        self.static_transform_stamped.transform.rotation.y = 0.0
        self.static_transform_stamped.transform.rotation.z = 0.0
        self.static_transform_stamped.transform.rotation.w = 1.0
        
        self.static_broadcaster.sendTransform(self.static_transform_stamped)
        rospy.loginfo("TF static has been published between %s and %s frames" % (self.static_transform_stamped.header.frame_id, self.static_transform_stamped.child_frame_id))
        
if __name__ == "__main__":
    rospy.init_node("tf_static_node")
    tf_static_ex()
    rospy.spin()