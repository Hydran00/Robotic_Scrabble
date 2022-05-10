#! /usr/bin/env python3
from scrabble.msg import Rack
import rospy

rack = Rack()

if __name__ == "__main__":
    rospy.init_node('rack_publisher', anonymous=True)
    pub = rospy.Publisher("scrabble/robot_rack",Rack,queue_size=1)
    while(not rospy.is_shutdown()):
    #COMPUTE VISION OF ROBOT RACK  
        rack = ['A','B','C','D','E','F','G']
        print(rack)
        pub.publish(rack)
        rospy.sleep(1)