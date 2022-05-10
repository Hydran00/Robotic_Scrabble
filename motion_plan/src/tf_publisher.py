#!/usr/bin/env python  
import roslib
import rospy
import tf

if __name__ == '__main__':
    rospy.init_node('broadcaster_fixed')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        rospy.sleep(10)
        
        br.sendTransform((0, 0, 1.78),(0.707, -0.7071063, 0, 0),rospy.Time.now(),"robot1_tf/world","world")
        rate.sleep()
