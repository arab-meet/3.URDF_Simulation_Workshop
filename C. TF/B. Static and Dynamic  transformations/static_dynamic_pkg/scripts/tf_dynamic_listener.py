#!/usr/bin/env python3
import rospy
import tf


if __name__ == '__main__':
    rospy.init_node('dynamic_tf_listener')

    listener = tf.TransformListener()

    rate = rospy.Rate(1.0)
    while not rospy.is_shutdown():
        try:
            (trans_dy,rot_dy) = listener.lookupTransform('/odom', '/base_link', rospy.Time(0))
            (trans_st,rot_st) = listener.lookupTransform('/base_link', '/laser', rospy.Time(0))

            print(f"dynamic translation is {trans_dy} and rotation is {rot_dy}\n")
            print(f"static translation is {trans_st} and rotation is {rot_st}")


        except:
            continue


        rate.sleep()