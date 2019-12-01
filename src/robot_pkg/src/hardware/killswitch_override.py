#!/usr/bin/env python

from __future__ import division, print_function
import rospy

from std_msgs.msg import Bool

KILLSWITCH_TOPIC = "killswitch"
killswitch_pub = None

def main():
    global KILLSWITCH_TOPIC
    global killswitch_pub

    rospy.init_node('killswitch_override')

    killswitch_timer = rospy.Timer(rospy.Duration(1.0/30.0), publish_killswitch)
    killswitch_pub = rospy.Publisher(KILLSWITCH_TOPIC, Bool)

    rospy.spin()

def publish_killswitch(time):
    global killswitch_pub

    msg = Bool()
    msg.data = False

    killswitch_pub.publish(msg)

if __name__ == '__main__':
    print('Hit enter to override killswitch (DO NOT DO ON ROBOT)')
    try:
        _ = input()
    except:
        pass
    
    main()
