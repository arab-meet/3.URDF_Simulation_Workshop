#!/usr/bin/env python3
import rospy
import tf


if __name__ == '__main__':
    rospy.init_node('turtle_tf_listener')

    listener = tf.TransformListener()

    rate = rospy.Rate(1.0)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/base_link', '/laser', rospy.Time(0))

            print(f"translation is {trans} and rotation is {rot}")
        except:
            continue


        rate.sleep()