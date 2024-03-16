#!/usr/bin/env python3

import rospy

from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
# massage that used in tf static 
from geometry_msgs.msg import TransformStamped

class tf_static_ex(object):
    
    def __init__(self):
        
        # instance from static transform broadcaster class 
        # need this object to publish static transform in TF static topic
        self.static_broadcaster = StaticTransformBroadcaster()
        
        # now we need to publish dynamic transform
        # same as static transform we need to define transform broadcaster object
        self.dynamic_broadcaster = TransformBroadcaster()
        
        
        # create a static transform message
        self.static_transform_stamped = TransformStamped()
        
        # now we need to create a dynamic transform message
        # same as static transform message but dynamic transform message
        self.dynamic_transform_stamped = TransformStamped()
        
        # i well add timer to publish dynamic transform every 0.1 second 
        # to let you see the difference between static and dynamic transform 
        
        # rospy.Timer is a class that used to call a function every specific time
        
        # take two parameters fristthe duration here 0.1
        # second: the function that executed every this duration 0.1 second
        
        self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_callback)
        
        # i will make the dynamic transform to move in x axes with 0.05 every 0.1 second
        self.x_increment= 0.05
        self.last_x = 0.0 #this for save last x value to add the increment to it
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
    
    # when timer expired this function will be executed
    # publish new transform every 0.1 second
    def timer_callback(self, event):
        # add information about time 
        self.dynamic_transform_stamped.header.stamp = rospy.Time.now()
        self.dynamic_transform_stamped.header.frame_id = "odom"
        self.dynamic_transform_stamped.child_frame_id = "base_link"
        # add translation and rotation vectors same static transform
        # but here we will change the translation vector every 0.1 second
        self.dynamic_transform_stamped.transform.translation.x = self.x_increment + self.last_x
        self.dynamic_transform_stamped.transform.translation.y = 0.0
        self.dynamic_transform_stamped.transform.translation.z = 0.0
        
        self.dynamic_transform_stamped.transform.rotation.x = 0.0
        self.dynamic_transform_stamped.transform.rotation.y = 0.0
        self.dynamic_transform_stamped.transform.rotation.z = 0.0
        self.dynamic_transform_stamped.transform.rotation.w = 1.0
        # now publish transform
        self.dynamic_broadcaster.sendTransform(self.dynamic_transform_stamped)
        # update last x value
        self.last_x = self.dynamic_transform_stamped.transform.translation.x
        
if __name__ == "__main__":
    rospy.init_node("tf_dynamic_node")
    tf_static_ex()
    rospy.spin()