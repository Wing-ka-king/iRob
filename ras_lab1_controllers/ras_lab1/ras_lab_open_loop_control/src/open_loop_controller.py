#!/usr/bin/env python
# license removed for brevity
import rospy
from ras_lab1_msgs.msg import PWM

def main():
    # try calling publisher
    try:
        publisher_pwm()
    except rospy.ROSInterruptException:
        pass
    
    # rospy.init_node('open_loop_node', anonymous=True)
    # rospy.Subscriber('/kobuki/pwm', PWM, callback_pwm)
    # rospy.spin()

def callback_pwm(msg):
    pwm_msg = msg
    print('message received')

def publisher_pwm():
    pub_pwm = rospy.Publisher('/kobuki/pwm', PWM, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    # publishing rate is 10 Hz, or every 100 milli sec
    rate = rospy.Rate(10) 

    pwm_msg = PWM()
    pwm_msg.PWM1 = 110
    pwm_msg.PWM2 = 100

    while not rospy.is_shutdown():
        rospy.loginfo(pwm_msg)
        pub_pwm.publish(pwm_msg)
        rate.sleep()

if __name__ == '__main__':
    main()
