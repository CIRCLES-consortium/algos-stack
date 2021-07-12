#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, TwistStamped
from std_msgs.msg import Float64


def mock_sensors_prompt():
    pub_v = rospy.Publisher('vel', Twist, queue_size=10)
    pub_lv = rospy.Publisher('leader_vel', Twist, queue_size=10)
    pub_h = rospy.Publisher('headway_est', Float64, queue_size=10)
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        v = Twist()
        v.linear.x = 0.52693541
        lv = Twist()
        lv.linear.x = 0.5538559
        h = Float64()
        h.data = 0.25393719
        pub_v.publish(v)
        pub_lv.publish(lv)
        pub_h.publish(h)
        rate.sleep()


def mock_sensors_sync():
    pub_v = rospy.Publisher('vel', TwistStamped, queue_size=10)
    pub_lv = rospy.Publisher('leader_vel', TwistStamped, queue_size=10)
    pub_h = rospy.Publisher('headway_est', TwistStamped, queue_size=10)
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        v = TwistStamped()
        v.twist.linear.x = 0.52693541
        lv = TwistStamped()
        lv.twist.linear.x = 0.5538559
        h = TwistStamped()
        h.twist.linear.x = 0.25393719
        pub_v.publish(v)
        pub_lv.publish(lv)
        pub_h.publish(h)
        rate.sleep()


def main():
    rospy.init_node('mock_sensors', anonymous=True)
    mode = rospy.get_param('/mode', 'sync')
    try:
        if mode == 'sync':
            rospy.loginfo('Publish in [sync] mode.')
            mock_sensors_sync()
        elif mode == 'prompt':
            rospy.loginfo('Publish in [prompt] mode.')
            mock_sensors_prompt()
        else:
            raise ValueError('Invalid publish mode.')
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
