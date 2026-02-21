#!/usr/bin/env python3
from __future__ import division, print_function
import os

import numpy as np
from scipy.linalg import block_diag
from scipy.spatial.distance import cityblock
import rospy
import tf2_ros

# msgs
from geometry_msgs.msg import TransformStamped, Twist, PoseStamped
from nav_msgs.msg import Path, Odometry, OccupancyGrid
from visualization_msgs.msg import Marker

# ros and se2 conversion utils
import utils


TRANS_GOAL_TOL = .4  # m, tolerance to consider a goal complete
ROT_GOAL_TOL = 1.5  # rad, tolerance to consider a goal complete
TRANS_VEL_OPTS = [0, 0.08, 0.15]  # m/s, max of real robot is .26
ROT_VEL_OPTS = np.linspace(-1.6, 1.6, 17)  # rad/s, max of real robot is 1.82
CONTROL_RATE = 10  # Hz, how frequently control signals are sent
CONTROL_HORIZON = 2  # seconds. if this is set too high and INTEGRATION_DT is too low, code will take a long time to run!
INTEGRATION_DT = .025  # s, delta t to propagate trajectories forward by
COLLISION_RADIUS = 0.31  # m, radius from base_link to use for collisions, min of 0.2077 based on dimensions of .281 x .306
ROT_DIST_MULT = .3  # multiplier to change effect of rotational distance in choosing correct control
CUMUL_ROT_DIST_MULT = .33
FOLLOW_DIST_MULT = .2
OBS_DIST_MULT = .12  # multiplier to change the effect of low distance to obstacles on a path
MIN_TRANS_DIST_TO_USE_ROT = TRANS_GOAL_TOL  # m, robot has to be within this distance to use rot distance in cost
# PATH_NAME = 'path.npy'  # saved path from l2_planning.py, should be in the same directory as this file
# PATH_NAME = 'will_path.npy'
# PATH_NAME = 'path_complete.npy'
PATH_NAME = 'will_path_rrt_star.npy'

# here are some hardcoded paths to use if you want to develop l2_planning and this file in parallel
# TEMP_HARDCODE_PATH = [[2, 0, 0], [2.75, -1, -np.pi/2], [2.75, -4, -np.pi/2], [2, -4.4, np.pi]]  # almost collision-free
TEMP_HARDCODE_PATH = [[2, -.5, 0], [2.4, -1, -np.pi/2], [2.45, -3.5, -np.pi/2], [1.5, -4.4, np.pi]]  # some possible collisions

class PathFollower():
    def __init__(self):
        # time full path
        self.path_follow_start_time = rospy.Time.now()

        # use tf2 buffer to access transforms between existing frames in tf tree
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)
        rospy.sleep(1.0)  # time to get buffer running

        # constant transforms
        self.map_odom_tf = self.tf_buffer.lookup_transform('map', 'odom', rospy.Time(0), rospy.Duration(2.0)).transform
        # print(self.map_odom_tf)

        # subscribers and publishers
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.global_path_pub = rospy.Publisher('~global_path', Path, queue_size=1, latch=True)
        self.local_path_pub = rospy.Publisher('~local_path', Path, queue_size=1)
        self.collision_marker_pub = rospy.Publisher('~collision_marker', Marker, queue_size=1)

        # map
        map = rospy.wait_for_message('/map', OccupancyGrid)
        self.map_np = np.array(map.data).reshape(map.info.height, map.info.width)
        self.map_resolution = round(map.info.resolution, 5)
        self.map_origin = -utils.se2_pose_from_pose(map.info.origin)  # negative because of weird way origin is stored
        # print(self.map_origin)
        self.map_nonzero_idxes = np.argwhere(self.map_np)
        # print(map)


        # collisions
        self.collision_radius_pix = COLLISION_RADIUS / self.map_resolution
        self.collision_marker = Marker()
        self.collision_marker.header.frame_id = '/map'
        self.collision_marker.ns = '/collision_radius'
        self.collision_marker.id = 0
        self.collision_marker.type = Marker.CYLINDER
        self.collision_marker.action = Marker.ADD
        self.collision_marker.scale.x = COLLISION_RADIUS * 2
        self.collision_marker.scale.y = COLLISION_RADIUS * 2
        self.collision_marker.scale.z = 1.0
        self.collision_marker.color.g = 1.0
        self.collision_marker.color.a = 0.5

        # transforms
        self.map_baselink_tf = self.tf_buffer.lookup_transform('map', 'base_link', rospy.Time(0), rospy.Duration(2.0))
        self.pose_in_map_np = np.zeros(3)
        self.pos_in_map_pix = np.zeros(2)
        self.update_pose()

        # path variables
        cur_dir = os.path.dirname(os.path.realpath(__file__))

        # to use the temp hardcoded paths above, switch the comment on the following two lines
        self.path_tuples = np.load(os.path.join(cur_dir, PATH_NAME)).T
        # self.path_tuples = np.array(TEMP_HARDCODE_PATH)

        self.path_tuples[9][0] = 11.6
        self.path_tuples[11][0] = 11.6
        self.path_tuples[22][0] = 4.4
        self.path_tuples[23][0] = 4
        self.path_tuples[24][0] = 4
        self.path_tuples[34][0] = 26.5
        self.path_tuples[35][0] = 27
        self.path_tuples[41][1] = -44.62

        self.path = utils.se2_pose_list_to_path(self.path_tuples, 'map')
        self.global_path_pub.publish(self.path)

        # goal
        self.cur_goal = np.array(self.path_tuples[0])
        self.cur_path_index = 0

        # trajectory rollout tools
        # self.all_opts is a Nx2 array with all N possible combinations of the t and v vels, scaled by integration dt
        self.all_opts = np.array(np.meshgrid(TRANS_VEL_OPTS, ROT_VEL_OPTS)).T.reshape(-1, 2)

        # if there is a [0, 0] option, remove it
        all_zeros_index = (np.abs(self.all_opts) < [0.001, 0.001]).all(axis=1).nonzero()[0]
        if all_zeros_index.size > 0:
            self.all_opts = np.delete(self.all_opts, all_zeros_index, axis=0)
        self.all_opts_scaled = self.all_opts * INTEGRATION_DT

        self.num_opts = self.all_opts_scaled.shape[0]
        self.horizon_timesteps = int(np.ceil(CONTROL_HORIZON / INTEGRATION_DT))

        self.rate = rospy.Rate(CONTROL_RATE)

        rospy.on_shutdown(self.stop_robot_on_shutdown)
        self.follow_path()

    def follow_path(self):

        print(self.path_tuples[11])

        global OBS_DIST_MULT
        global CUMUL_ROT_DIST_MULT
        global FOLLOW_DIST_MULT
        global COLLISION_RADIUS

        while not rospy.is_shutdown():
            # timing for debugging...loop time should be less than 1/CONTROL_RATE
            tic = rospy.Time.now()

            self.update_pose()
            self.check_and_update_goal()

            
            # start trajectory rollout algorithm
            print("Propagate the trajectory forward")
            local_paths = np.zeros([self.horizon_timesteps + 1, self.num_opts, 3])
            local_paths[0] = np.atleast_2d(self.pose_in_map_np).repeat(self.num_opts, axis=0)

            """
            1) Trajectory rollout: propagate trajectory forward, storing resulting points in local_paths
            """
            print("1) Trajectory rollout")
            for t in range(1, self.horizon_timesteps + 1):
                pose_prev = local_paths[t-1]

                vel = self.all_opts[:, 0]
                rot_vel = self.all_opts[:, 1]

                local_paths[t, :, 0] = pose_prev[:, 0] + vel * np.cos(pose_prev[:, 2]) * INTEGRATION_DT
                local_paths[t, :, 1] = pose_prev[:, 1] + vel * np.sin(pose_prev[:, 2]) * INTEGRATION_DT
                local_paths[t, :, 2] = pose_prev[:, 2] + rot_vel * INTEGRATION_DT
            

            """
            2) Check for collisions
            """
            # check all trajectory points for collisions
            # first find the closest collision point in the map to each local path point
            local_paths_pixels = (self.map_origin[:2] + local_paths[:, :, :2]) / self.map_resolution
            valid_opts = range(self.num_opts)
            local_paths_lowest_collision_dist = np.ones(self.num_opts) * 50

            assert local_paths_pixels.shape[1] == len(valid_opts)
            assert self.horizon_timesteps+1 == local_paths_pixels.shape[0]

            print("2.1) Check the points in local_path_pixels for collisions")
            for opt in range(self.num_opts):
                for timestep in range(local_paths_pixels.shape[0]):
                    px = local_paths_pixels[timestep, opt, 0]
                    py = local_paths_pixels[timestep, opt, 1]

                    if self.map_nonzero_idxes.size > 0:
                        dists = np.sqrt(
                            (self.map_nonzero_idxes[:, 0] - py)**2 + (self.map_nonzero_idxes[:, 1] - px)**2
                        )
                        min_dist = np.min(dists)

                        if min_dist < local_paths_lowest_collision_dist[opt]:
                            local_paths_lowest_collision_dist[opt] = min_dist

            print("2.2) Remove trajectories with collisions")
            valid_opts = [opt for opt in range(self.num_opts) if local_paths_lowest_collision_dist[opt] > self.collision_radius_pix]

            """
            3) Calculate final cost and choose best control option
            """
            # calculate final cost and choose best option
            print("3) Calculate the final cost and choose the best control option!")

            print(f"cur path index: {self.cur_path_index}")

            if len(valid_opts) == 0:
                control = [-0.1, 0] # hardcoded recover if all options infeasible
            else:
                final_cost = []
                
                # if self.cur_path_index >= 6:
                #     OBS_DIST_MULT = .2
                #     CUMUL_ROT_DIST_MULT = .38
                # if self.cur_path_index >= 10:
                #     COLLISION_RADIUS = 0.27
                # if self.cur_path_index >= 23:
                #     OBS_DIST_MULT = .25
                    

                    
                for opt in valid_opts:

                    cur_path_goal = self.path_tuples[self.cur_path_index]

                    last_pose = local_paths[-1, opt, :]

                    trans_dist = np.linalg.norm(last_pose[:2] - self.cur_goal[:2])

                    if trans_dist < MIN_TRANS_DIST_TO_USE_ROT:
                    # if True:
                        abs_theta_diff = np.abs(last_pose[2] - self.cur_goal[2])
                        rot_dist = min(np.pi*2 - abs_theta_diff, abs_theta_diff)
                        # rot_error = np.arctan2(
                        #     self.cur_goal[1] - last_pose[1],
                        #     self.cur_goal[0] - last_pose[0]
                        # ) - last_pose[2]
                        # rot_error = (rot_error + np.pi) % (2*np.pi) - np.pi
                        # rot_dist = np.abs(rot_error)
                    else:
                        rot_dist = 0

                    obs_pen = OBS_DIST_MULT / (local_paths_lowest_collision_dist[opt] + 0.1) # 0.1 for numerical stability

                    cumul_rot_dist = 0
                    prev_pose = local_paths[0, opt, :]
                    for pose in local_paths[1:, opt]:
                        abs_theta_diff = np.abs(prev_pose[2] - pose[2])
                        incr_rot_dist = min(np.pi*2 - abs_theta_diff, abs_theta_diff)
                        cumul_rot_dist += incr_rot_dist
                        prev_pose = pose

                    follow_dist_cost = 0
                    for pose in local_paths[:, opt]:
                        dist = np.linalg.norm(pose[:2] - cur_path_goal[:2])
                        follow_dist_cost += dist

                    # cost = trans_dist + ROT_DIST_MULT * rot_dist + obs_pen + CUMUL_ROT_DIST_MULT * cumul_rot_dist
                    cost = trans_dist + ROT_DIST_MULT * rot_dist + obs_pen + CUMUL_ROT_DIST_MULT * cumul_rot_dist + FOLLOW_DIST_MULT * follow_dist_cost
                    final_cost.append(cost)
                
                final_cost = np.array(final_cost)
                best_i = final_cost.argmin()
                best_opt = valid_opts[best_i]
                control = self.all_opts[best_opt]
                self.local_path_pub.publish(
                    utils.se2_pose_list_to_path(local_paths[:, best_opt], "map")
                )

            # send command to robot
            self.cmd_pub.publish(utils.unicyle_vel_to_twist(control))

            # uncomment out for debugging if necessary
            # print("Selected control: {control}, Loop time: {time}, Max time: {max_time}".format(
            #     control=control, time=(rospy.Time.now() - tic).to_sec(), max_time=1/CONTROL_RATE))

            self.rate.sleep()

    def update_pose(self):
        # Update numpy poses with current pose using the tf_buffer
        self.map_baselink_tf = self.tf_buffer.lookup_transform('map', 'base_link', rospy.Time(0)).transform
        self.pose_in_map_np[:] = [self.map_baselink_tf.translation.x, self.map_baselink_tf.translation.y,
                                  utils.euler_from_ros_quat(self.map_baselink_tf.rotation)[2]]
        self.pos_in_map_pix = (self.map_origin[:2] + self.pose_in_map_np[:2]) / self.map_resolution
        self.collision_marker.header.stamp = rospy.Time.now()
        self.collision_marker.pose = utils.pose_from_se2_pose(self.pose_in_map_np)
        self.collision_marker_pub.publish(self.collision_marker)

    def check_and_update_goal(self):
        # iterate the goal if necessary
        dist_from_goal = np.linalg.norm(self.pose_in_map_np[:2] - self.cur_goal[:2])
        abs_angle_diff = np.abs(self.pose_in_map_np[2] - self.cur_goal[2])
        rot_dist_from_goal = min(np.pi * 2 - abs_angle_diff, abs_angle_diff)
        if dist_from_goal < TRANS_GOAL_TOL and rot_dist_from_goal < ROT_GOAL_TOL:
            rospy.loginfo("Goal {goal} at {pose} complete.".format(
                    goal=self.cur_path_index, pose=self.cur_goal))
            if self.cur_path_index == len(self.path_tuples) - 1:
                rospy.loginfo("Full path complete in {time}s! Path Follower node shutting down.".format(
                    time=(rospy.Time.now() - self.path_follow_start_time).to_sec()))
                rospy.signal_shutdown("Full path complete! Path Follower node shutting down.")
            else:
                self.cur_path_index += 1
                self.cur_goal = np.array(self.path_tuples[self.cur_path_index])
        else:
            rospy.logdebug("Goal {goal} at {pose}, trans error: {t_err}, rot error: {r_err}.".format(
                goal=self.cur_path_index, pose=self.cur_goal, t_err=dist_from_goal, r_err=rot_dist_from_goal
            ))

    def stop_robot_on_shutdown(self):
        self.cmd_pub.publish(Twist())
        rospy.loginfo("Published zero vel on shutdown.")

if __name__ == '__main__':
    try:
        rospy.init_node('path_follower', log_level=rospy.DEBUG)
        pf = PathFollower()
    except rospy.ROSInterruptException:
        pass