#!/usr/bin/env python
# this version # forward = 0 #### 0920.1900
from cmath import pi
import rospy
import math
import tf

from std_msgs.msg import String,Float64
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3Stamped
from geometry_msgs.msg import PointStamped

from visualization_msgs.msg import Marker

class Waypoint:

    def __init__(self):

        self.rpy = Vector3Stamped()
        self.pose = PointStamped()
        self.L1 = PointStamped()
        self.L2 = PointStamped()
        self.R1 = PointStamped()
        self.R2 = PointStamped()
        self.C = PointStamped()

        self.ang = 0
        self.des_steer = 0
        self.deg2rad = 3.14/180
        self.rad2deg = 180/3.14
        
        self.target_point_x = []
        self.target_point_y = []
        self.target_avg_x = 0
        self.target_avg_y = 0

        self.scan_data_x = []
        self.scan_data_y = []
        
        self.avg_x = 0
        self.avg_y = 0
        
        self.bScan_data_in = False

        self.obs_sub = rospy.Subscriber('/zed/scan', LaserScan, self.obs_callback)

        self.cmd_pub = rospy.Publisher('/cmd_vel_test', Float64, queue_size=1)

        self.pub_marker = rospy.Publisher('/center_point', Marker, queue_size=1)

        self.marker = 0

        self.rate = rospy.Rate(20)
        self.calc_des_point()
        
    def obs_callback(self, _data):
        ##TODO: _data >> laser scan data >> list = self.scan_data
        _scan_data = list(_data.ranges)
        count = 0
        x = 0
        y = 0
        pi = 3.141592

        for i in range(len(_scan_data)):
            if _scan_data[i] != float("inf"):
                count += 1
                if i != 0:
                    rad = (i / 57.3) - (pi / 2)
                else:
                    rad = 0

                x = math.cos(rad) * _scan_data[i]
                y = math.sin(rad) * _scan_data[i]

                if x < 3 and abs(y) < 1.5:
                    self.scan_data_x.append(x)
                    self.scan_data_y.append(y)                    

                # elif abs(y) > 1.5:
                #     if y > 0:
                #         y = y-1.5
                #     else:
                #         y = y+1.5

        ##
        self.bScan_data_in = True
    
    def calc_des_point(self):
        
        while not rospy.is_shutdown():
            
            if self.bScan_data_in:
                _des_steer = 0
                _v_cmd = 0
                avg_x = 0
                avg_y = 0
                _temp_avg_x = 0
                _temp_avg_y = 0
            
                _scan_data_length = self.scan_data_x.__len__()

                if _scan_data_length != 0:
                
                    for i in self.scan_data_x:
                        _temp_avg_x += i
                    for j in self.scan_data_y:
                        _temp_avg_y += j
                        
                    _temp_avg_x = _temp_avg_x / _scan_data_length
                    _temp_avg_y = _temp_avg_y / _scan_data_length
                    
                    avg_x = _temp_avg_x
                    avg_y = _temp_avg_y

                    self.target_point_x.append(avg_x)
                    self.target_point_y.append(avg_y)

                    _temp_length = self.target_point_x.__len__()

                    if _temp_length > 5:
                        for k in self.target_point_x:
                            self.target_avg_x += k
                        
                        for l in self.target_point_y:
                            self.target_avg_y += l
                    
                        self.target_avg_x = self.target_avg_x / _temp_length
                        self.target_avg_y = self.target_avg_y / _temp_length

                        self.marker = self.fnc_make_marker(self.target_avg_x, self.target_avg_y)
                        self.pub_marker.publish(self.marker)

                        _ang = math.atan2(self.target_avg_x, self.target_avg_y)
                        self.cmd_pub.publish(_ang)
                        _des_steer = (_ang * self.rad2deg) - 90
                    
                        self.target_avg_x = 0
                        self.target_avg_y = 0
                        self.target_point_x = []
                        self.target_point_y = []

                        _v_cmd = 1

                        # self.move2goal(_v_cmd, _des_steer)
                    
                    self.scan_data_x = []
                    self.scan_data_y = []

            else:
                _des_steer = 0
                _v_cmd = 0
                
            self.rate.sleep()
            
    def move2goal(self, v_cmd, des_steer):    

        p_gain = 3

        cmd_vel = Twist()
        
        cmd_vel.linear.y = 0
        cmd_vel.linear.z = 0
        cmd_vel.angular.x = 0
        cmd_vel.angular.y = 0

        cmd_vel.linear.x = v_cmd

        cmd_vel.angular.z = des_steer * self.deg2rad# * p_gain

        if cmd_vel.angular.z >= 30:
            cmd_vel.angular.z = 30
        elif cmd_vel.angular.z <= -30:
            cmd_vel.angular.z = -30
        else:
            cmd_vel.angular.z = -cmd_vel.angular.z
        
        print("v_cmd : {0}, angular_z : {1}".format(cmd_vel.linear.x, cmd_vel.angular.z))
        # self.cmd_pub.publish(cmd_vel)

    def fnc_make_marker(self, x, y):
        _marker = Marker()
        _marker.header.frame_id = "/base_link"
        _marker.header.stamp = rospy.Time.now()
        _marker.type = 1
        _marker.pose.position.x = x
        _marker.pose.position.y = y
        _marker.pose.position.z = 0
        _marker.pose.orientation.x = 0.0
        _marker.pose.orientation.y = 0.0
        _marker.pose.orientation.z = 0.0
        _marker.pose.orientation.w = 1.0
        _marker.scale.x = 0.05
        _marker.scale.y = 0.05
        _marker.scale.z = 0.05

        _marker.color.r = 0
        _marker.color.g = 1
        _marker.color.b = 0
        _marker.color.a = 1

        return _marker

if __name__ == '__main__':
    
    rospy.init_node('primary_1', anonymous=False)

    rospy.loginfo("Track Drive Manager Start!")

    waypoint = Waypoint()

    rospy.spin()
        