#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import math

from geometry_msgs.msg import Twist
#from std_msgs.msg import String
#from sensor_msgs.msg import Image
#from cv_bridge import CvBridge, CvBridgeError
#import cv2


class HakataNoShioBot():
    def __init__(self, bot_name):
        # bot name 
        self.name = bot_name

        # velocity publisher
        self.vel_pub = rospy.Publisher('cmd_vel', Twist,queue_size=1)

    def action(self, linear_x, angular_z, sleep_time ):

        twist = Twist()
        twist.linear.x = linear_x
        twist.linear.y = 0
        twist.linear.z = 0

        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = math.radians(angular_z)

        # /cmd_velへpublish
        print(twist)
        self.vel_pub.publish(twist)

        # 指定時間sleep
#       print('sleep =', sleep_time)
        rospy.sleep(sleep_time)

    def strategy(self):
        while not rospy.is_shutdown():

	        # 0th
            self.action(0.0,                # linear_x
                          0,                # angular_z
                        0.3)                # sleep

            # 1st １番目の柱へ
            self.action(0.5,                # linear_x
                        10.7,                # angular_z
                        4.6)                # sleep

            # 2nd 2番めの柱へ
            self.action(0.5,                # linear_x
                        -30,                # angular_z
                        1.9)                # sleep

            # 3rd 2番めの柱を曲がる ★ぶつかりやすいので要調整
            self.action(0.2,                # linear_x
                        -32,                # angular_z
                        1.3)                # sleep

            # 4th sio待ち
            self.action(0.1,                # linear_x
                          0,                # angular_z
                        5.5)                # sleep

            # 5th sioのけつをとる
            self.action(0.3,                # linear_x
                        -55,                # angular_z
                        2.3)                # sleep

            # last(sioの後ろで待機)
            self.action(0.0,                # linear_x
                          0,                # angular_z
                        120)                # sleep

if __name__ == '__main__':
    rospy.init_node('HakataNoShio')
    bot = HakataNoShioBot('HakataNoShio')
    bot.strategy()

