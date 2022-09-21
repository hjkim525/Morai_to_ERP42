#!/usr/bin/env python3
# license removed for brevity
import rospy
import time
from std_msgs.msg import Int16

def talker():
    
    pub = rospy.Publisher('/velo', Int16, queue_size=1)
    rospy.init_node('velocity', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    count = 0
    
    while not rospy.is_shutdown():
        velocity=1
        pub.publish(velocity)
        # steer = 0
        # while True :
        #     if count%2 == 0:
        #         steer +=25
        #         time.sleep(0.05)
        #         if steer == 2000:
        #             count +=1
        #     elif count%2 == 1:
        #         steer -= 25
        #         time.sleep(0.05)
        #         if steer == -2000:
        #             count +=1
        
        #     pub.publish(steer)
        #     rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass