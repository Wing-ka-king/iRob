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
        '''
        [[[[[STEP -1]]]]] 
        set design variables of robot
        '''
        self.ticks_per_rev = 360
        self.b = 0.23           # in meters
        self.r = 0.0352         # in meters
        self.control_freq = 10  # in Hz
        self.dt = 1/self.control_freq
        
        # set PI control variables
        # NOTE: index 1 is Left and index 2 is Right
        self.error = 0
        self.int_error = 0
        self.alpha1 = 15    #15
        self.alpha2 = 13.2  #13.2
        self.beta1 = 0.4
        self.beta2 = 0.5
        self.desired_w1 = 0 
        self.estimated_w1 = 0
        self.desired_w2 = 0 
        self.estimated_w2 = 0

        # formulate PWM message 
        self.pwm_msg = PWM()
        self.target_twist = Twist()
        self.linear_vel = 1
        self.gamma = 0.01

    def callback_dist_reading(self, msg):
        adc_reading = msg
        linear_vel = self.linear_vel
        angular_vel = self.gamma * ( adc_reading.ch1 - adc_reading.ch2)    
        self.target_twist.linear.x = linear_vel
        self.target_twist.angular.z = angular_vel
        # pub_target_twist = rospy.Publisher('/kobuki/pwm', PWM, queue_size=10)
        # pub_target_twist.publish(self.target_twist)
        self.desired_w1, self.desired_w2 = self.twist_2_wheel_w(self.target_twist)        

    def callback_obs_twist(self, msg):
        self.encoder_data = msg
        self.estimated_w1, self.estimated_w2 = self.encoder_2_wheel_w(self.encoder_data)

        # # since this callback is called after des_twist callback, we can call the PI control
        # # function here
        # self.pwm1 = self.PI_control(self.desired_w1, self.estimated_w1)
        # self.pwm2 = self.PI_control(self.desired_w2, self.estimated_w2)

        # # formulate PWM message and publish it
        # pwm_msg = PWM()
        # pwm_msg.PWM1 = self.pwm1
        # pwm_msg.PWM2 = self.pwm2

        #self.publish_pwm(pwm_msg)
    
    def control_action(self):
        term1 = self.PI_control(1) 
        term2 = self.PI_control(2) 
        
        self.pwm_msg.PWM1 = term1 
        self.pwm_msg.PWM2 = term2
        # # Execute PI control function here
        # pwm1 = self.PI_control(self.desired_w1, self.estimated_w1, 1)
        # pwm2 = self.PI_control(self.desired_w2, self.estimated_w2, 2)

        self.publish_pwm(self.pwm_msg)

    def twist_2_wheel_w(self, twist):
        #v = (twist.linear.x**2 + twist.linear.y**2)**0.5
        v = twist.linear.x
        #w = v/self.r
        w = twist.angular.z

        v_w2 = v - w * self.b    #v_w2 = Right wheel velocity
        v_w1 = v + w * self.b   #v_w1 = Left wheel velocity

        w1 = v_w1/self.r
        w2 = v_w2/self.r
        return w1, w2

    def encoder_2_wheel_w(self, encoder_data):
        delta1 = encoder_data.delta_encoder1
        delta2 = encoder_data.delta_encoder2

        w1 = 2 * np.pi * self.control_freq * delta1 / self.ticks_per_rev
        w2 = 2 * np.pi * self.control_freq * delta2 / self.ticks_per_rev

        return w1, w2

    #def PI_control(self, desired_w, estimated_w, ind):
    def PI_control(self, ind):    
        if ind == 1:    # Left motor tuning
            alpha = self.alpha1
            beta = self.beta1
            desired_w = self.desired_w1
            estimated_w = self.estimated_w1
        elif ind == 2:
            alpha = self.alpha2
            beta = self.beta2
            desired_w = self.desired_w2
            estimated_w = self.estimated_w2
        
        self.error = desired_w - estimated_w
        self.int_error = self.int_error + self.error * self.dt
        term = alpha*self.error + beta*self.int_error
        #term = alpha*self.error
        # term = beta*self.int_error
        return term

    def publish_pwm(self, pwm_msg):
        pub_pwm = rospy.Publisher('/kobuki/pwm', PWM, queue_size=10)
        pub_pwm.publish(pwm_msg)
        



# MAIN
def main():
    rospy.init_node('wall_following_controller_node', anonymous=True)
    #call class
    wall_controller = wall_following_controller()

    '''
    Subscribe to ADC topic to get distance readings of both sensors: 
    ''' 
    rospy.Subscriber('/kobuki/adc', ADConverter, wall_controller.callback_dist_reading)

    '''
    Get OBSERVED Twist. this can be done by subscribing to
    /kobuki/encoders topic
    '''
    rospy.Subscriber('/kobuki/encoders', Encoders, wall_controller.callback_obs_twist)

    rate = rospy.Rate(wall_controller.control_freq) # 10hz
    while not rospy.is_shutdown():
        wall_controller.control_action()
        rate.sleep()

    

    rospy.spin()

if __name__ == '__main__':
    main()