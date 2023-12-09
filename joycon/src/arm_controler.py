#!/usr/bin/env python3
## coding: UTF-8

import rospy
from sensor_msgs.msg import Joy
#from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray

def control(msg, float_pub):#joy to twist
    L_horizontal = msg.axes[0]  #左ジョイスティック（左右）
    L_vertical = msg.axes[1]    #左ジョイスティック（上下）
    R_horizontal = msg.axes[0]  #左ジョイスティック（左右）
    R_vertical = msg.axes[1]    #左ジョイスティック（上下）
    up = msg.buttons[4] 
    down = msg.buttons[6] 
    right = msg.buttons[5] 
    left = msg.buttons[7] 
    circle = msg.buttons[13] #12?
    cross = msg.buttons[14]
    open = msg.buttons[11]
    close = msg.buttons[10]
    v = [L_horizontal*(1+circle-cross), L_vertical*(1+circle-cross),R_horizontal*(1+circle-cross),R_vertical*(1+circle-cross),(up - down)*(1+circle-cross),(right - left)*(1+circle-cross),(open - close)*(1+circle-cross)]
    float_pub.publish(v)    #twistを配信




if __name__ == '__main__':
    #ノードの初期化
    rospy.init_node('joy_to_float')
    #配信準備
    pub = rospy.Publisher('target_position_rpy', Float32MultiArray, queue_size=1)
    #購読
    rospy.Subscriber('/arm_joy', Joy, control, pub)

    rospy.spin()