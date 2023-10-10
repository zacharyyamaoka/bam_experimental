#!/usr/bin/env python

import rospy

if __name__ == '__main__':
    rospy.init_node('hello_world_node')
    rospy.loginfo('Hello, World!')
    rospy.spin()