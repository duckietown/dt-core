#!/usr/bin/env python
import rospy
from rgb_led import *
import sys
import time
from std_msgs.msg import Float32, Int8, String
from rgb_led import RGB_LED
from duckietown_msgs.msg import BoolStamped, CoordinationSignalETHZ17, CoordinationSignal


class LEDEmitter(object):
    def __init__(self):
        self.led = RGB_LED()
        self.node_name = rospy.get_name()
        self.active = True
        self.pattern = [[0,0,0]] * 5
        self.current_pattern_name = 'OFF'
        self.changePattern_(self.current_pattern_name)

        # Parameter to scale the intensity
        self.intensity = 0.8

        # If True, the LED turn on and off. Else, they are always on
        self.onOff = True

        if self.onOff:
            self.dt          = 0.2
            self.is_on       = False
            self.cycle_timer = rospy.Timer(rospy.Duration.from_sec(self.dt/(2.0)),self.cycleTimer)

        # Publish
        #self.pub_state = rospy.Publisher("~current_led_state",Float32,queue_size=1)
        self.pub_state = rospy.Publisher("~current_led_state",String,queue_size=1)

        # Subscribe
        self.sub_pattern = rospy.Subscriber("~change_color_pattern",String,self.changePattern)

        # self.sub_switch = rospy.Subscriber("~switch",BoolStamped,self.cbSwitch)
        # self.cycle = None

        # self.protocol = rospy.get_param("~LED_protocol") #should be a list of tuples

        # self.pattern_off = [[0,0,0]]
        # self.pattern_on  = [[1,1,1]]

        #for i in range(5):
        #    self.pattern[i] = self.pattern_off

        # scale = 0.5
        # for _, c in self.protocol['colors'].items():
        #    for i in range(3):
        #        c[i] = c[i]  * scale

    def cbSwitch(self, switch_msg): # active/inactive switch from FSM
        self.active = switch_msg.data

    def cycleTimer(self,event):
        if not self.active:
            return
        elif not self.onOff:
            # No oscillation
            for i in range(5):
                self.led.setRGB(i,[self.pattern[i][0],self.pattern[i][1],self.pattern[i][2]])
        else:
            # Oscillate
            if self.is_on:
                for i in range(5):
                    self.led.setRGB(i,[0,0,0])
                self.is_on = False
            else:
                for i in range(5):
                    self.led.setRGB(i,[self.pattern[i][0],self.pattern[i][1],self.pattern[i][2]])
                self.is_on = True

    def changePattern(self, msg):
        self.changePattern_(msg.data)

    def changePattern_(self, pattern_name):
        if pattern_name:
            if self.current_pattern_name == pattern_name:
                return
            else:
                self.current_pattern_name = pattern_name

            # rospy.loginfo('changePattern(%r)' % pattern_name)
            # color = self.protocol['signals'][pattern_name]['color']
            # self.cycle = self.protocol['signals'][pattern_name]['frequency']
            # print("color: %s, freq (Hz): %s "%(color, self.cycle))

            # With joystick
            if self.current_pattern_name == 'ON_WHITE':
                rospy.loginfo('changePattern(%r)' % pattern_name)
                self.pattern = [[self.intensity,self.intensity,self.intensity]]*5
            elif self.current_pattern_name == 'ON_RED':
                rospy.loginfo('changePattern(%r)' % pattern_name)
                self.pattern = [[1,0,0]] * 5
            elif self.current_pattern_name == 'ON_BLUE':
                rospy.loginfo('changePattern(%r)' % pattern_name)
                self.pattern = [[0,0,1]] * 5
            elif self.current_pattern_name == 'ON_GREEN':
                rospy.loginfo('changePattern(%r)' % pattern_name)
                self.pattern = [[0,1,0]] * 5
            else:
                rospy.loginfo('changePattern(%r)' % pattern_name)
                self.pattern = [[0,0,0]]*5

            # With coordination
            if self.current_pattern_name == CoordinationSignalETHZ17.ON:
                rospy.loginfo('changePattern(%r)' % pattern_name)
                self.pattern = [[self.intensity,self.intensity,self.intensity]]*5
            elif self.current_pattern_name == CoordinationSignalETHZ17.OFF:
                rospy.loginfo('changePattern(%r)' % pattern_name)
                self.pattern = [[0,0,0]]*5

            # With coordination (new)
            if self.current_pattern_name == CoordinationSignal.SIGNAL_A:
                rospy.loginfo('changePattern(%r)' % pattern_name)
                self.pattern    = [[0,0,0]]*5
                self.pattern[2] = [self.intensity,self.intensity,self.intensity]
            elif self.current_pattern_name == CoordinationSignal.SIGNAL_GREEN:
                rospy.loginfo('changePattern(%r)' % pattern_name)
                self.pattern = [[0,self.intensity,0]]*5
            elif self.current_pattern_name == CoordinationSignal.OFF:
                rospy.loginfo('changePattern(%r)' % pattern_name)
                self.pattern = [[0,0,0]]*5

            # Set intensity
            # self.pattern = self.intensity*self.pattern

            # Change LEDs
            if not self.onOff:
                self.cycleTimer([])

            # Publish current pattern
            self.pub_state.publish(self.current_pattern_name)

            #self.pattern = [[0,0,0]] * 5
            #self.pattern[2] = self.protocol['colors'][color]

            # if pattern_name in ['traffic_light_go', 'traffic_light_stop']:
                # self.pattern = [self.protocol['colors'][color]] * 5

            # self.changeFrequency()

    #def changeFrequency(self):
    #    try:
    #        #self.cycle = msg.data
    #        self.cycle_timer.shutdown()
    #        #below, convert to hz
    #        d = 1.0/(2.0*self.cycle)
    #        self.cycle_timer = rospy.Timer(rospy.Duration.from_sec(d), self.cycleTimer)
    #    except ValueError as e:
    #        self.cycle = None
    #        self.current_pattern_name = None
    #    self.pub_state.publish(float(self.cycle))

if __name__ == '__main__':
    rospy.init_node('led_emitter',anonymous=False)
    node = LEDEmitter()
    rospy.spin()

