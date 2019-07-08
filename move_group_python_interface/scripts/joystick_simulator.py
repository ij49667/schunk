#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point, Pose, PointStamped
from math import pi, cos, sin

def talker():


    pub = rospy.Publisher('/joystick_dref', Point, queue_size=1)
    rospy.init_node('joystick_simulator', anonymous=True)
    rate = rospy.Rate(1) # 1hz

    start_time = rospy.get_rostime()
    dx = 0.0
    dy = 0.0
    dz = 0.0

    period = 10.

    while not rospy.is_shutdown():
        dref_msg = Point()

        dt = rospy.get_rostime().secs - start_time.secs

        dx = 0.1 * (cos(2. * pi * dt/period) - 1.)
        dy = 0.1 * sin(2. * pi * dt/period)
        dz = 0.1 * (cos(2. * pi * dt/period) - 1.)

        dref_msg.x = dx
        dref_msg.y = dy
        dref_msg.z = dz
	
	print dref_msg.x, dref_msg.y, dref_msg.z

        pub.publish(dref_msg)
        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
