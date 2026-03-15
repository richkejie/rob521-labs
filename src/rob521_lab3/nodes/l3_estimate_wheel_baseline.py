#!/usr/bin/env python3

import rospy
import numpy as np
import threading
from turtlebot3_msgs.msg import SensorState
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist

INT32_MAX = 2**31
NUM_ROTATIONS = 3 
TICKS_PER_ROTATION = 4096
WHEEL_RADIUS = 0.066 / 2 #In meters


class wheelBaselineEstimator():
    def __init__(self):
        rospy.init_node('encoder_data', anonymous=True) # Initialize node

        #Subscriber bank
        rospy.Subscriber("cmd_vel", Twist, self.startStopCallback)
        rospy.Subscriber("sensor_state", SensorState, self.sensorCallback) #Subscribe to the sensor state msg

        #Publisher bank
        self.reset_pub = rospy.Publisher('reset', Empty, queue_size=1)

        #Initialize variables
        self.left_encoder_prev = None
        self.right_encoder_prev = None
        self.del_left_encoder = 0
        self.del_right_encoder = 0
        self.isMoving = False #Moving or not moving
        self.lock = threading.Lock()

        #Reset the robot 
        reset_msg = Empty()
        self.reset_pub.publish(reset_msg)
        print('Ready to start wheel baseline calibration!')
        return

    def safeDelPhi(self, a, b):
        #Need to check if the encoder storage variable has overflowed
        diff = np.int64(b) - np.int64(a)
        if diff < -np.int64(INT32_MAX): #Overflowed
            delPhi = (INT32_MAX - 1 - a) + (INT32_MAX + b) + 1
        elif diff > np.int64(INT32_MAX) - 1: #Underflowed
            delPhi = (INT32_MAX + a) + (INT32_MAX - 1 - b) + 1
        else:
            delPhi = b - a  
        return delPhi

    def sensorCallback(self, msg):
        #Retrieve the encoder data form the sensor state msg
        self.lock.acquire()
        if self.left_encoder_prev is None or self.right_encoder_prev is None:
            self.left_encoder_prev = msg.left_encoder #int32
            self.right_encoder_prev = msg.right_encoder #int32
        else:
            #Calculate and integrate the change in encoder value
            self.del_left_encoder += self.safeDelPhi(self.left_encoder_prev, msg.left_encoder)
            self.del_right_encoder += self.safeDelPhi(self.right_encoder_prev, msg.right_encoder)

            #Store the new encoder values
            self.left_encoder_prev = msg.left_encoder #int32
            self.right_encoder_prev = msg.right_encoder #int32
        self.lock.release()
        return

    def startStopCallback(self, msg):
        if self.isMoving is False and np.absolute(msg.angular.z) > 0:
            self.isMoving = True #Set state to moving
            print('Starting Calibration Procedure')

        elif self.isMoving is True and np.isclose(msg.angular.z, 0):
            self.isMoving = False #Set the state to stopped

            print(f"Cycle complete — left: {self.del_left_encoder}, right: {self.del_right_encoder}")

            # # YOUR CODE HERE!!!
            # Calculate the radius of the wheel based on encoder measurements

            # The robot spun in place for NUM_ROTATIONS full rotations, so the total
            # heading change is:
            #   theta_total = 2 * pi * NUM_ROTATIONS
            #
            # For a differential-drive robot spinning in place, each wheel travels an
            # arc length equal to (baseline / 2) * theta_total:
            #   arc_length = WHEEL_RADIUS * del_phi  =>  (b/2) * theta = r * del_phi
            #   b = 2 * r * del_phi / theta_total
            #
            # The left wheel moves backward (negative ticks) and the right wheel moves
            # forward (positive ticks), so we use their absolute values and average for
            # the best estimate.
 
            self.lock.acquire()
            del_left  = self.del_left_encoder
            del_right = self.del_right_encoder
            self.lock.release()
 
            # del_phi_left  = np.abs(del_left)  * (2.0 * np.pi / TICKS_PER_ROTATION)  # radians
            # del_phi_right = np.abs(del_right) * (2.0 * np.pi / TICKS_PER_ROTATION)  # radians
 
            # theta_total = 2.0 * np.pi * NUM_ROTATIONS  # total robot rotation in radians

            # separation_from_left  = 2.0 * WHEEL_RADIUS * del_phi_left  / theta_total
            # separation_from_right = 2.0 * WHEEL_RADIUS * del_phi_right / theta_total
            # separation = (separation_from_left + separation_from_right) / 2.0

            ticks = (np.abs(del_left) + np.abs(del_right)) / 2
            del_phi = ticks * (2 * np.pi / TICKS_PER_ROTATION)
            theta_total = 2 * np.pi * NUM_ROTATIONS
            separation = 2 * WHEEL_RADIUS * del_phi / theta_total
 
            print('Calibrated Separation: {} m'.format(separation))

            #Reset the robot and calibration routine
            self.lock.acquire()
            self.left_encoder_prev = None
            self.right_encoder_prev = None
            self.del_left_encoder = 0
            self.del_right_encoder = 0
            self.lock.release()
            reset_msg = Empty()
            self.reset_pub.publish(reset_msg)
            print('Resetted the robot to calibrate again!')

        return


if __name__ == '__main__':
    Estimator = wheelBaselineEstimator() #create instance
    rospy.spin()
