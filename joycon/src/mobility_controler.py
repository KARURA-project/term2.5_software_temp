#! /usr/bin/env python3

import rospy
import math
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray

slide = 0
class Joy_controler:
    def __init__(self):
        rospy.init_node('joy_controler')
        self.sub = rospy.Subscriber('/joy', Joy, self.callback)
        self.wheel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.arm_pub = rospy.Publisher('/arm_joy', Float32MultiArray, queue_size=10)
        self.twist = Twist()
        
        self.motor = Float32MultiArray()
        self.v=[0, 0, 0, 0, 0, 0, 0]
        self.motor.data = self.v
        self.motor_flag = 1
        
       
        
        # rospy.Timer(rospy.Duration(0.2), self.timer_callback)

    def _init_motor_data(self):
        self.v= [0, 0, 0, 0, 0, 0, 0]
    

    

    #joy command callback
    def callback(self, msg):
        mode=msg.buttons[5]         #R1
        back_to_start=msg.buttons[4]         #L1
        L_horizontal = msg.axes[0]  #左ジョイスティック（左右）
        L_vertical = msg.axes[1]    #左ジョイスティック（上下）
        R_horizontal = msg.axes[3]  #左ジョイスティック（左右）
        R_vertical = msg.axes[4]    #左ジョイスティック（上下）
        up = msg.axes[7]            #-
        down = msg.axes[7]          #+
        right = msg.buttons[6]      #+
        left = msg.buttons[6]       #-
        circle = msg.buttons[1]     #O
        cross = msg.buttons[0]      #X
        open = msg.buttons[7]       #R2
        close = msg.buttons[6]      #L2
        stop = msg.buttons[10]
        if (open == 1):
            slide = -1
        elif (close == 1):
            slide = 1
        else:
            slide = 0
        
        if mode == 1:
            self.motor_flag*=-1
            if self.motor_flag==1:
                rospy.loginfo("mode changed! now: %s","Arm Control")
            else:
                rospy.loginfo("mode changed! now: %s","Wheel Control")
            rospy.sleep(0.2)


        #アーム制御　
        if self.motor_flag == 1:
            self.motor_flag = 1
            if back_to_start==1:
                self._init_motor_data()
            update=[-L_horizontal*(1+circle-cross), L_vertical*(1+circle-cross),-R_horizontal*(1+circle-cross),R_vertical*(1+circle-cross),(up - down)*(1+circle-cross),(right - left)*(1+circle-cross),slide]
            for i in range(len(self.v)):
                self.v[i] += update[i]
            
        #ホイール制御
        else:
            self.motor_flag = -1
            self.twist.linear.x = L_vertical*(1+circle-cross)
            self.twist.linear.y = L_horizontal*(1+circle-cross)
            self.twist.angular.z = math.atan2(L_horizontal,L_vertical)
            self.wheel_pub.publish(self.twist)
            
            if circle:
                self.twist.linear.x=0
                self.twist.linear.y=0
                self.twist.angular.z = math.atan2(L_horizontal,L_vertical)
                self.wheel_pub.publish(self.twist)
        #publish arm data
    
        

if __name__ == '__main__':
    Joy_controler()
    rospy.spin()
    
