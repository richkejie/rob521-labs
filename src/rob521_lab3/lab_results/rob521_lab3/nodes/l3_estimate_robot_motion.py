#!/usr/bin/env python3
from __future__ import division, print_function
import time

import numpy as np
import rospy
import tf_conversions
import tf2_ros
import rosbag
import rospkg

# msgs
from turtlebot3_msgs.msg import SensorState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist, TransformStamped, Transform, Quaternion
from std_msgs.msg import Empty

from utils import convert_pose_to_tf, euler_from_ros_quat, ros_quat_from_euler


ENC_TICKS = 4096
RAD_PER_TICK = 0.001533981 # 2pi / 4096
# WHEEL_RADIUS = .066 / 2
WHEEL_RADIUS = .0323
# BASELINE = .287 / 2
BASELINE = .289 / 2


class WheelOdom:
    def __init__(self):
        # publishers, subscribers, tf broadcaster
        self.sensor_state_sub = rospy.Subscriber('/sensor_state', SensorState, self.sensor_state_cb, queue_size=1)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_cb, queue_size=1)
        self.wheel_odom_pub = rospy.Publisher('/wheel_odom', Odometry, queue_size=1)
        self.tf_br = tf2_ros.TransformBroadcaster()

        # attributes
        self.odom = Odometry()
        self.odom.pose.pose.position.x = 1e10
        self.wheel_odom = Odometry()
        self.wheel_odom.header.frame_id = 'odom'
        self.wheel_odom.child_frame_id = 'wo_base_link'
        self.wheel_odom_tf = TransformStamped()
        self.wheel_odom_tf.header.frame_id = 'odom'
        self.wheel_odom_tf.child_frame_id = 'wo_base_link'
        self.pose = Pose()
        self.pose.orientation.w = 1.0
        # self.pose.orientation.z = 1.0 # set for rosbag playback
        self.twist = Twist()
        self.last_enc_l = None
        self.last_enc_r = None
        self.last_time = None

        # rosbag
        rospack = rospkg.RosPack()
        path = rospack.get_path("rob521_lab3")
        # self.bag = rosbag.Bag(path+"/motion_estimate.bag", 'w')
        self.bag = rosbag.Bag(path+"/motion_estimate_2.bag", 'w')

        # reset current odometry to allow comparison with this node
        reset_pub = rospy.Publisher('/reset', Empty, queue_size=1, latch=True)
        reset_pub.publish(Empty())
        # while not rospy.is_shutdown() and (self.odom.pose.pose.position.x >= 1e-3 or self.odom.pose.pose.position.y >= 1e-3 or
        #        self.odom.pose.pose.orientation.z >= 1e-2):
        #     time.sleep(0.2)  # allow reset_pub to be ready to publish
        # FIXME: provided reset code is incorrect???
        while not rospy.is_shutdown() and (
            abs(self.odom.pose.pose.position.x) >= 1e-3 or
            abs(self.odom.pose.pose.position.y) >= 1e-3 or
            abs(euler_from_ros_quat(self.odom.pose.pose.orientation)[2]) >= 1e-2
        ):
            time.sleep(0.2)
        print('******************************\n******************************\nRobot odometry reset.\n******************************\n******************************\n')

        # FIXME: Initialize wheel odom from onboard odom?? maybe only for rosbag playback
        # self.pose.position.x = self.odom.pose.pose.position.x
        # self.pose.position.y = self.odom.pose.pose.position.y
        # self.pose.orientation = self.odom.pose.pose.orientation

        rospy.spin()
        self.bag.close()
        print("saving bag")

    def safeDelPhi(self, a, b):
        # Handle int32 encoder overflow/underflow
        INT32_MAX = 2**31
        diff = np.int64(b) - np.int64(a)
        if diff < -np.int64(INT32_MAX):
            delPhi = (INT32_MAX - 1 - a) + (INT32_MAX + b) + 1
        elif diff > np.int64(INT32_MAX) - 1:
            delPhi = (INT32_MAX + a) + (INT32_MAX - 1 - b) + 1
        else:
            delPhi = b - a
        return delPhi

    def sensor_state_cb(self, sensor_state_msg):
        # Callback for whenever a new encoder message is published
        # set initial encoder pose
        if self.last_enc_l is None:
            self.last_enc_l = sensor_state_msg.left_encoder
            self.last_enc_r = sensor_state_msg.right_encoder
            self.last_time = sensor_state_msg.header.stamp
        else:
            # update calculated pose and twist with new data
            le = sensor_state_msg.left_encoder
            re = sensor_state_msg.right_encoder

            # # YOUR CODE HERE!!!
            # # ===============================================================
            # Update your odom estimates with the latest encoder measurements and populate the relevant area
            # of self.pose and self.twist with estimated position, heading and velocity

            # self.pose.position.x = xx
            # self.pose.position.y = xx
            # self.pose.orientation = xx

            # self.twist.linear.x = mu_dot[0].item()
            # self.twist.linear.y = mu_dot[1].item()
            # self.twist.angular.z = mu_dot[2].item()

            current_time = sensor_state_msg.header.stamp
 
            # 1. Compute raw encoder deltas (handles overflow)
            del_enc_l = self.safeDelPhi(self.last_enc_l, le)
            del_enc_r = self.safeDelPhi(self.last_enc_r, re)
 
            # 2. Convert encoder ticks to wheel rotation angles (radians)
            del_phi_l = del_enc_l * RAD_PER_TICK
            del_phi_r = del_enc_r * RAD_PER_TICK
 
            # 3. Compute linear displacement and heading change
            #    ds    = average arc length of both wheels
            #    dtheta = difference in arc length divided by full baseline (2*BASELINE)
            ds     = WHEEL_RADIUS * (del_phi_r + del_phi_l) / 2.0
            dtheta = WHEEL_RADIUS * (del_phi_r - del_phi_l) / (2.0 * BASELINE)
 
            # 4. Get current heading from pose quaternion
            theta = euler_from_ros_quat(self.pose.orientation)[2]
 
            # 5. Integrate pose using midpoint method
            #    Evaluate direction at the midpoint heading (theta + dtheta/2)
            #    for better accuracy than simple Euler integration
            theta_mid = theta + dtheta / 2.0
            self.pose.position.x += ds * np.cos(theta_mid)
            self.pose.position.y += ds * np.sin(theta_mid)
 
            # 6. Update heading quaternion
            new_theta = theta + dtheta
            self.pose.orientation = ros_quat_from_euler([0.0, 0.0, new_theta])
 
            # 7. Compute twist (velocity) from pose delta over elapsed time in robot frame
            dt = (current_time - self.last_time).to_sec()
            if dt > 0:
                self.twist.linear.x  = ds / dt
                self.twist.linear.y  = 0.0  # Assuming no lateral movement
                self.twist.angular.z = dtheta / dt
 
            # 8. Store encoder and time for next iteration
            self.last_enc_l = le
            self.last_enc_r = re
            self.last_time  = current_time

            # # ===============================================================
            # publish the updates as a topic and in the tf tree
            current_time = rospy.Time.now()
            self.wheel_odom_tf.header.stamp = current_time
            self.wheel_odom_tf.transform = convert_pose_to_tf(self.pose)
            self.tf_br.sendTransform(self.wheel_odom_tf)

            self.wheel_odom.header.stamp = current_time
            self.wheel_odom.pose.pose = self.pose
            self.wheel_odom.twist.twist = self.twist
            self.wheel_odom_pub.publish(self.wheel_odom)

            # FIXME: use the timestamp from the sensor message for better synchronization in the bag file
            # pub_time = sensor_state_msg.header.stamp
            # self.wheel_odom_tf.header.stamp = pub_time
            # self.wheel_odom_tf.transform = convert_pose_to_tf(self.pose)
            # self.tf_br.sendTransform(self.wheel_odom_tf)

            # self.wheel_odom.header.stamp = pub_time
            # self.wheel_odom.pose.pose = self.pose
            # self.wheel_odom.twist.twist = self.twist
            # self.wheel_odom_pub.publish(self.wheel_odom)

            self.bag.write('odom_est', self.wheel_odom)

            # for testing against actual odom
            print("Wheel Odom: x: %2.3f, y: %2.3f, t: %2.3f" % (
                self.pose.position.x, self.pose.position.y, new_theta
            ))
            print("Tbot3 Odom: x: %2.3f, y: %2.3f, t: %2.3f" % (
                self.odom.pose.pose.position.x, self.odom.pose.pose.position.y,
                euler_from_ros_quat(self.odom.pose.pose.orientation)[2]
            ))

    def odom_cb(self, odom_msg):
        # get odom from turtlebot3 packages
        self.odom = odom_msg
        self.bag.write('odom_onboard', self.odom)

    def plot(self, bag):
        data = {"odom_est":{"time":[], "data":[]}, 
                "odom_onboard":{'time':[], "data":[]}}
        for topic, msg, t in bag.read_messages(topics=['odom_est', 'odom_onboard']):
            print(msg)


if __name__ == '__main__':
    try:
        rospy.init_node('wheel_odometry')
        wheel_odom = WheelOdom()
    except rospy.ROSInterruptException:
        pass