#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist, TwistStamped
import numpy as np
from std_msgs.msg import Float64


def mock_sensors_prompt(v_param, lv_param, h_param):
    pub_v = rospy.Publisher('vel', Twist, queue_size=10)
    pub_lv = rospy.Publisher('leader_vel', Twist, queue_size=10)
    pub_h = rospy.Publisher('headway_est', Float64, queue_size=10)
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        v = Twist()
        v.linear.x = v_param
        lv = Twist()
        lv.linear.x = lv_param
        h = Float64()
        h.data = h_param
        pub_v.publish(v)
        pub_lv.publish(lv)
        pub_h.publish(h)
        rate.sleep()


def mock_sensors_sync(v_param, lv_param, h_param):
    pub_v = rospy.Publisher('vel', TwistStamped, queue_size=10)
    pub_lv = rospy.Publisher('leader_vel', TwistStamped, queue_size=10)
    pub_h = rospy.Publisher('headway_est', TwistStamped, queue_size=10)
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        v = TwistStamped()
        v.twist.linear.x = v_param
        lv = TwistStamped()
        lv.twist.linear.x = lv_param
        h = TwistStamped()
        h.twist.linear.x = h_param
        pub_v.publish(v)
        pub_lv.publish(lv)
        pub_h.publish(h)
        rate.sleep()


def main():
    rospy.init_node('mock_sensors', anonymous=True)
    mode_param = rospy.get_param('mode', 'prompt')
    v_param = rospy.get_param('v')
    print(v_param, type(v_param))
    lv_param = np.float32(rospy.get_param('lv'))
    h_param = np.float32(rospy.get_param('h'))
    try:
        if mode_param == 'sync':
            rospy.loginfo('Publish in [sync] mode.')
            mock_sensors_sync(v_param, lv_param, h_param)
        elif mode_param == 'prompt':
            rospy.loginfo('Publish in [prompt] mode.')
            mock_sensors_prompt(v_param, lv_param, h_param)
        else:
            raise ValueError('Invalid publish mode.')
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
