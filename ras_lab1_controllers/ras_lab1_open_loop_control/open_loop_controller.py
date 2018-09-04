#!/usr/bin/env python
# license removed for brevity
import rospy
from ras_lab1_msgs.msg import PWM

## def pwm_des(data):
##	vel = PWM()
##	data = input()
##	vel.PWM1 = data
##	vel.PWM2 = data
##      PWM_des = "PWM is "+  str(data)


def talker():
    pub = rospy.Publisher('/kobuki/pwm', PWM, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 1hz
    while not rospy.is_shutdown():
	pwm_des = PWM()
	x = input()
	pwm_des.PWM1 = x
	pwm_des.PWM2 = x
        rospy.loginfo(pwm_des)
        pub.publish(pwm_des)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
