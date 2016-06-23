#!/usr/bin/python

import rosbag
import sys
import rospy
import os
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage

def talker():

    argc = len(sys.argv)

    if argc == 2:
        bag_folder_path = sys.argv[1]

        odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
        tf_pub = rospy.Publisher('/tf', TFMessage, queue_size=10)
        rospy.init_node("failure_pts", anonymous=True)
        rate = rospy.Rate(10) # 10hz

        for bagfile in os.listdir(os.curdir + os.path.sep + bag_folder_path):
            fullpath = os.curdir + os.path.sep + bag_folder_path + os.path.sep + bagfile
            bag = rosbag.Bag(fullpath)

            topic, odom_msg, t = bag.read_messages(topics=['odom']).next()
            topic, tf_msg, t = bag.read_messages(topics=['/tf']).next()

            #update time because it doesn't fucking work
            odom_msg.header.stamp
            for tf in tf_msg.transforms:
                tf.header.stamp = rospy.Time.now()

            if not rospy.is_shutdown():
                tf_pub.publish(tf_msg)
                odom_pub.publish(odom_msg)
                rate.sleep()
    else:
        print "usage: ./first_msg.py bag_folder_path"


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
