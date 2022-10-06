#!/usr/bin/env python

import rospy

from std_msgs.msg import Float64, Float32
from math import abs, pi

class controller:
    def __init__(self):
        
        self.des_yaw        = 0
        self.yaw            = 0
        self.error_input    = 0

        

        rospy.Subscriber('/imugen',Float32,self.imuCB)
        rospy.Subscriber('/cmd_vel_test',Float64,self.steerCB)

        self.pub_steer=rospy.Publisher('/ctrl_cmd',Float64,queue_size=1)

        self.rate = rospy.Rate(10)

        self.main()

    def imuCB(self, data:Float32):
        self.yaw=data.data

    def steerCB(self, data:Float64):
        self.des_yaw=data.data

    def main(self):
        self.error_input=self.des_yaw-self.yaw

        if abs(self.error_input) > pi:
            if self.error_input >0:
                self.error_input -= 2*pi
            else:
                self.error_input += 2*pi
        self.pub_steer.publish(self.error_input)

if __name__ == '__main__':

    rospy.init_node('controller', anonymous=True)
    
    _controller=controller()

    rospy.spin()                    
