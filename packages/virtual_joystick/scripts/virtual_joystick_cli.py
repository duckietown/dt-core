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
        axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        direction = raw_input('Enter direction(a,w,s,d), or l to start lane following or q to stop lane following, and then press enter--> ')
        if direction == 'w':
            axes[1] = 1.0
        elif direction == 's':
            axes[1] = -1.0
        elif direction == 'd':
            axes[3] = -1.0
        elif direction == 'a':
            axes[3] = 1.0
        elif direction == 'l':
            buttons[7] = 1
        elif direction == 'q':
            buttons[6] = 1

            

        msg = Joy(header=None, axes=axes, buttons=buttons)
        pub.publish(msg)
        rospy.sleep(0.5)


if __name__ == '__main__':
    if len(sys.argv) < 2:
        raise Exception("No hostname specified!")
    else:
        hostname = sys.argv[1]

    try:
        keyCatcher(host = hostname)
    except rospy.ROSInterruptException:
        pass

