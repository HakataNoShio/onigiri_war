#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import random
import time
import tf
import threading
from datetime import datetime
import math

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry


t_stop_flg = False

MOVE_MODE0   = 0
MOVE_MODE1   = 1
MOVE_MODE2_1 = 2
MOVE_MODE2_2 = 3
MOVE_MODE3_1 = 4
MOVE_MODE3_2 = 5
MOVE_MODE3_3 = 6
MOVE_MODE4_1 = 7
MOVE_MODE4_2 = 8
MOVE_MODE4_3 = 9


class RandomBot():
    PI = 3.1415926535

    move_mode = 0
    Kp = 0
    Ki = 0
    Kd = 0
    AngularZ_i = 0
    prev_p     = 0

    us_left   = 0
    us_right  = 0
    opt_left  = 0
    opt_right = 0

    enemy_direction = 65535
    enemy_distance  = 65535

    sleep_count = 0

    old_linear_x  = 0
    old_angular_z = 0

    base_time = 0
    seq_msg = [
        # 以下のlinear_x,angular_zを上から順に実行するが、
        #        指定時間経過後に次のメッセージにいくか
        #        敵検知位置情報を登録していた場合は検出値に入ったら、指定のシーケンス番号を加えた場所に行く
        # linear_x angular_z  指定時間(s)      detect_min_deg   detect_max_deg   detect_min_dis   detect_max_dis   add_seq_no
        #
        [         0,       0,          0.3,               -1,               -1,             -1,               -1,           1],   #
        [       0.5,    10.5,          3.4,               -1,               -1,             -1,               -1,           1],   # 第1障害物通過
        [       0.5,   -15.0,          1.4,               -1,               -1,             -1,               -1,           1],   # 柱のマークへ
        [       0.0,      0,         360.0,               -1,               -1,             -1,               -1,           1],   # 柱のマークへ
    ]

    def __init__(self, bot_name):
        # bot name
        self.name = bot_name
        # velocity publisher
        self.vel_pub = rospy.Publisher('cmd_vel', Twist,queue_size=1)
        self.base_time = time.time()
        # 超音波センサ
        self.sub_us_left  = rospy.Subscriber("us_left",    LaserScan, self.us_left_callback,  queue_size=1)
        self.sub_us_right = rospy.Subscriber("us_right",   LaserScan, self.us_right_callback, queue_size=1)
        # 赤外線センサ
        self.sub_opt_left  = rospy.Subscriber("opt_left",  LaserScan, self.opt_left_callback,  queue_size=1)
        self.sub_opt_right = rospy.Subscriber("opt_right", LaserScan, self.opt_right_callback, queue_size=1)

        self.sub_imu       = rospy.Subscriber("imu", Imu, self.imu_callback, queue_size=1)
        self.sub_odom      = rospy.Subscriber("odom", Odometry, self.odom_callback, queue_size=1)
        self.sub_scan      = rospy.Subscriber("scan", LaserScan, self.scan_callback, queue_size=1)

    def us_left_callback(self, sens):
        self.us_left = sens.ranges[0]

    def us_right_callback(self, sens):
        self.us_right = sens.ranges[0]

    def opt_left_callback(self, sens):
        if sens.ranges[0] > 0.2:
            self.opt_left = 1
        else:
            self.opt_left = sens.ranges[0]

    def opt_right_callback(self, sens):
        if sens.ranges[0] > 0.2:
            self.opt_right = 1
        else:
            self.opt_right = sens.ranges[0]

    def rad_to_deg(self, rad):
        return (rad * 180 / self.PI)

    def imu_callback(self, sens):
        e = tf.transformations.euler_from_quaternion((sens.orientation.x, sens.orientation.y, sens.orientation.z, sens.orientation.w))
        #print("x:{0} y:{1} z:{2}".format(self.rad_to_deg(e[0]), self.rad_to_deg(e[1]), self.rad_to_deg(e[2])))

    def odom_callback(self, sens):
        e = tf.transformations.euler_from_quaternion((sens.pose.pose.orientation.x, sens.pose.pose.orientation.y, sens.pose.pose.orientation.z, sens.pose.pose.orientation.w))
        # print(sens)
        #print("xyz {0} {1} {2} x:{3} y:{4} z:{5}  w:{6}".format(sens.pose.pose.position.x, sens.pose.pose.position.y, sens.pose.pose.position.z, self.rad_to_deg(e[0]), self.rad_to_deg(e[1]), self.rad_to_deg(e[2]), sens.pose.pose.orientation.w))

    # LIDAR情報から敵位置割出
    # 　自分からみた敵の背後の壁が遠くないと検出不能
    def scan_callback(self, sens):
        self.enemy_direction = 65535
        self.enemy_distance  = 65535
        self.scan_list = []               # sensを上から見た時の時計回り360度にしたもの
        diff_list  = []              #
        lidar_distance_min = 0.2-0.025
        #print(sens.ranges[0])
        i = 0
        small_diff_count = 0
        small_diff_clear = 0         # 連続クリア回数
        for val in sens.ranges:
            self.scan_list.append(sens.ranges[360-1-i])
            i = i + 1

        i   = 0
        deg = 0
        for val in sens.ranges:
            if i+1 >= 360:
                next_i = 0
            else:
                next_i = i + 1
            if i-1 < 0:
                before_i = 359
            else:
                before_i = i - 1

            diff_val = abs(self.scan_list[i] - self.scan_list[before_i])
            if diff_val > lidar_distance_min:
                small_diff_count = 0
                deg              = -1
                small_diff_clear = small_diff_clear + 1
            else:
                small_diff_count = small_diff_count + 1
                small_diff_clear = 0

            #if small_diff_clear >= 2:
            #    self.enemy_direction = i
            #    print self.enemy_direction, scan_list[i]
            #    return

            if (self.scan_list[i]<self.scan_list[before_i] and self.scan_list[i]<self.scan_list[next_i]) or (deg != -1):
                rad = math.atan(0.05/self.scan_list[i])
                deg = math.degrees(rad)
                diff_val = abs(self.scan_list[i] - self.scan_list[next_i])
                #print i, deg, math.floor(deg-2), math.floor(deg), small_diff_count
                if deg != -1 and diff_val > lidar_distance_min:
                    if small_diff_count>=math.floor(deg-2) and small_diff_count<=math.floor(deg):
                        self.enemy_direction = i
                        self.enemy_distance  = self.scan_list[i]
                    #    print self.enemy_direction, self.scan_list[i]
                        return
            i = i + 1

       # print "none"

    def getPIDAngularZ(self, target_value, current_value, diff_time):
        now = time.time()
        output = 0;
        p = 0.0
        d = 0.0

        p = target_value - current_value;
        self.AngularZ_i += p * diff_time;
        d = (p - prev_p) / diff_time;

        output = (int32_t)(Kp * p + Ki * self.AngularZ_i + Kd * d);

        prev_p = p;

        return output;

    def cmdVel(self, linear_x, angular_z):

        twist = Twist()
        twist.linear.x = linear_x; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = math.radians(angular_z)
        self.vel_pub.publish(twist)

        if self.old_linear_x != linear_x or self.old_angular_z != angular_z:
            print(twist)

        self.old_linear_x  = linear_x
        self.old_angular_z = angular_z

        timestamp_str = datetime.now().strftime("%Y/%m/%d %H:%M:%S.%f")
        #print timestamp_str, " ", linear_x, " ", self.opt_left, " ", self.opt_right


    def mode1(self):
        print "+++ mode1 +++"
        detect_cnt = 0

        '''
        # Dエリアのみ
        # 初回sleep
        rospy.sleep(0.3)

        # ⑧のマーカーへ
        self.cmdVel(0.5, 0)
        rospy.sleep(2.8)

        # マーカー検知待ち
        self.cmdVel(0.0, 0)
        rospy.sleep(0.5)

        # ⑪のマーカーへ
        self.cmdVel(-0.5, -78)
        rospy.sleep(1.8)

        self.cmdVel(-0.3, 0)
        rospy.sleep(0.2)

        self.cmdVel(0.0, 135)
        rospy.sleep(2.4)


        # ⑫のマーカーへ
        self.cmdVel(0.5, 14)
        rospy.sleep(2.15)

        # マーカー検知待ち
        self.cmdVel(0.0, 0)
        rospy.sleep(0.5)

        # 家に帰る
        self.cmdVel(-0.5, 60)
        rospy.sleep(1.7)

        self.cmdVel(-0.3, 0)
        rospy.sleep(1.2)
        '''

        # 助走
        self.cmdVel(0.3, 0)
        rospy.sleep(0.3)

        # 左上の柱へ
#        self.cmdVel(0.5, 10.6)
#        rospy.sleep(3.4)
        self.cmdVel(0.5, 11.0)
        rospy.sleep(3.0)

        # ③のマーカーへ
        self.cmdVel(0.5, -6.0)
        rospy.sleep(1.9)
#        self.cmdVel(0.5, -16.0)
#        rospy.sleep(1.45)

        # マーカー検知待ち
        self.cmdVel(0.0, 0)
        rospy.sleep(0.5)

        # バック
        self.cmdVel(-0.3, 0)
        rospy.sleep(1.8)
#        self.cmdVel(-0.5, 3.6)
#        rospy.sleep(1.2)

        min_dis = 5
        min_deg = 0
        for i in range(60, 120):
            if min_dis > self.scan_list[i]:
                min_dis = self.scan_list[i]
                min_deg = i
        print min_deg, min_dis

        min_dis = 5
        min_deg = 0
        for i in range(180, 240):
            if min_dis > self.scan_list[i]:
                min_dis = self.scan_list[i]
                min_deg = i
        print min_deg, min_dis

        # 安定待ち
        self.cmdVel(0.0, 0)
        rospy.sleep(0.5)

        # 回転しながら⑥と⑨をとる
        self.cmdVel(0.0, -45)
#        rospy.sleep(5.4)    # -135度における2周
#        rospy.sleep(7.7)    # -90度における2周
        rospy.sleep(7.2)    # -45度における1周
#        rospy.sleep(3.5)


        ## Dエリアへ
        self.cmdVel(-0.5, 3)
        rospy.sleep(1.9)

        # 安定待ち
        self.cmdVel(0.0, 0)
        rospy.sleep(0.5)

        self.cmdVel(0, -45)
        rospy.sleep(0.8)
#        self.cmdVel(-0.5, -12)
#        rospy.sleep(3.15)

        # 安定待ち
        self.cmdVel(0.0, 0)
        rospy.sleep(0.5)

        # ⑧のマーカーへ
        self.cmdVel(0.5, 0)
        rospy.sleep(0.87)

        # マーカー検知待ち
        self.cmdVel(0.0, 0)
        rospy.sleep(0.5)

        # ⑪のマーカーへ
        self.cmdVel(-0.5, -80)
        rospy.sleep(1.8)

        self.cmdVel(-0.2, 0)
        rospy.sleep(0.3)

        self.cmdVel(0.0, 90)
        rospy.sleep(3.8)


        # ⑫のマーカーへ
        self.cmdVel(0.5, 0)
        rospy.sleep(2.3)

        # マーカー検知待ち
        self.cmdVel(0.0, 0)
        rospy.sleep(0.5)

        # 家に帰る
        self.cmdVel(-0.5, 75)
        rospy.sleep(1.7)

        # 回る
        # self.cmdVel(0.0, 180)
        self.cmdVel(-0.3, 0)
        rospy.sleep(120)


        '''
        # 捨て
      #  self.cmdVel(-0.5, -20)
       # rospy.sleep(1.3)

#        self.cmdVel(-0.5, 0)
 #       rospy.sleep(1.3)

#        self.cmdVel(-0.5, -10)
#        rospy.sleep(2.0)


        self.cmdVel(0.0, 0)
        rospy.sleep(90)

        # self.cmdVel(-0.05, 90)
        # rospy.sleep(2.0)

        # ⑥に向けて回転  & 敵確認
        self.cmdVel(0.0, -90)
        for i in range(0, 1300/10):
#       self.cmdVel(-0.05, -48)
#       for i in range(0, 4000/10):
            if 0 <= self.enemy_direction and 90 >= self.enemy_direction and \
               0 <= self.enemy_distance  and 1.0 >= self.enemy_distance:
                detect_cnt = detect_cnt + 1

                print "detect_cnt = ", detect_cnt
                if detect_cnt > 30: # 誤検知防止のため30回条件一致したら確定
                    self.move_mode = MOVE_MODE2_2
            rospy.sleep(0.01)

#        self.cmdVel(0.0, -45)
#        rospy.sleep(0.3)

        # 安定のため一時停止
        # self.cmdVel(0.00, 0.0)
        # rospy.sleep(2.0)
        '''

        if self.move_mode == MOVE_MODE2_2:
            # 敵はこっちに来てると判断 → mode2_2へ
            self.mode2_2()
        else:
            # 敵はこっちに来てないと判断 → mode2_1へ
            self.move_mode = MOVE_MODE2_1
            self.mode2_1()

    # mode2_1
    def mode2_1(self):
        print "+++ mode2_1 +++"

        # ⑥のマーカーへ
        self.cmdVel(0.5, 0)
        rospy.sleep(0.4)

        # マーカー検知待ち
        self.cmdVel(0.0, 0.0)
        rospy.sleep(0.5)

        # ⑨に向けて回転
        self.cmdVel(0.05, -90)
        rospy.sleep(0.55)

        # ⑨のマーカーへ
        self.cmdVel(0.5, 3)
        rospy.sleep(0.9)

        # マーカー検知待ち
        self.cmdVel(0.0, 0.0)
        rospy.sleep(0.5)

        #mode3_1へ
        self.move_mode = MOVE_MODE3_1
        self.mode3_1()

    # mode2_2
    def mode2_2(self):
        print "+++ mode2_2 +++"

        # ⑥のマーカーへ
        self.cmdVel(0.5, 3)
        rospy.sleep(0.5)


        # ⑨に向けて回転
        self.cmdVel(-0.1, -90)
        rospy.sleep(1.2)

        # ⑨のマーカーへ
        self.cmdVel(0.5, 3)
        rospy.sleep(0.5)

    # mode3_1
    def mode3_1(self):
        print "+++ mode3_1 +++"

        # ⑪のマーカーへ向けて回転
        self.cmdVel(0.1, -90)
        rospy.sleep(0.5)

        # ⑪へ
        self.cmdVel(0.5, 85)
        rospy.sleep(3.0)

        self.cmdVel(0.0, 45)
        rospy.sleep(0.5)


        # ⑨のマーカーへ
        self.cmdVel(0.5, 0)
        rospy.sleep(3)

        # マーカー検知待ち
        self.cmdVel(0.00, 0.0)
        rospy.sleep(0.8)


    def strategy(self):
        self.move_mode = MOVE_MODE0

        while not rospy.is_shutdown():
            # mode0実行
            if self.move_mode == MOVE_MODE0:
                self.move_mode = MOVE_MODE1
                self.mode1()
            else:
                self.cmdVel(0, 0)
                rospy.sleep(120*1000)


###################################################################
# キー入力受付
###################################################################
def print_debug(bot):
    global t_stop_flg

    cycle = 0.01
    timestamp_str = datetime.now().strftime("%Y/%m/%d %H:%M:%S.%f")
    #print timestamp_str, " ", bot.opt_left, " ", bot.opt_right

    if t_stop_flg == False:
        t = threading.Timer(cycle, print_debug, args=(bot,))
        t.start()

if __name__ == '__main__':
    rospy.init_node('random_rulo')
    bot = RandomBot('Random')

    try:
        t = threading.Thread(target=print_debug, args=(bot,))
        t.start()

        bot.strategy()
    except KeyboardInterrupt:
        t_stop_flg = True


