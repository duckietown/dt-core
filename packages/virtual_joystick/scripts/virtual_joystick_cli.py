#!/usr/bin/env python
# license removed for brevity
import rospy
import sys

from sensor_msgs.msg import Joy

from __builtin__ import True


def keyCatcher(host):
    pub = rospy.Publisher('/'+host+'/joy', Joy, queue_size=1)
    rospy.init_node('joy-cli', anonymous=True)

    while not rospy.is_shutdown():
        direction = raw_input('Enter direction(a,w,s,d)--> ')
        if direction == 'w':
            axes = [0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] 
        elif direction == 's':
            axes = [0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] 
        elif direction == 'd':
            axes = [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0] 
        elif direction == 'a':
            axes = [0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0] 
        else:
            axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        msg = Joy(header=None, axes=axes, buttons=None)
        pub.publish(msg)
        rospy.sleep(0.5)

        axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        msg = Joy(header=None, axes=axes, buttons=None)
        pub.publish(msg)

if __name__ == '__main__':
    if len(sys.argv) < 2:
        raise Exception("No hostname specified!")
    else:
        hostname = sys.argv[1]

    try:
        keyCatcher(host = hostname)
    except rospy.ROSInterruptException:
        pass

