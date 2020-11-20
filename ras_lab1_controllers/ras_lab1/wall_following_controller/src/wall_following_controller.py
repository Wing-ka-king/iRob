#!/usr/bin/env python
# license removed for brevity
import rospy
import numpy as np
from geometry_msgs.msg import Twist
from ras_lab1_msgs.msg import PWM
from ras_lab1_msgs.msg import Encoders
from ras_lab1_msgs.msg import ADConverter

class wall_following_controller():

    def __init__(self):
        self.target_twist = Twist()
        self.linear_vel = 1
        self.gamma = 0.01

    def callback_dist_reading(self, msg):
        adc_reading = msg
        linear_vel = self.linear_vel
        angular_vel = self.gamma * ( adc_reading.ch1 - adc_reading.ch2)    
        self.target_twist.linear.x = linear_vel
        self.target_twist.angular.z = angular_vel
        pub_target_twist = rospy.Publisher('/motor_controller/twist', Twist, queue_size=10)
        pub_target_twist.publish(self.target_twist)

# MAIN
def main():
    rospy.init_node('wall_following_controller_node', anonymous=True)
    #call class
    wall_controller = wall_following_controller()

    '''
    Subscribe to ADC topic to get distance readings of both sensors: 
    ''' 
    rospy.Subscriber('/kobuki/adc', ADConverter, wall_controller.callback_dist_reading)
    rospy.spin()

if __name__ == '__main__':
    main()