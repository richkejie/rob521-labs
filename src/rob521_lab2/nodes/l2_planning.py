#!/usr/bin/env python3
#Standard Libraries
import numpy as np
import yaml
import pygame
import time
import pygame_utils
import matplotlib.image as mpimg
from skimage.draw import disk
from scipy.linalg import block_diag


def load_map(filename):
    im = mpimg.imread("../maps/" + filename)
    if len(im.shape) > 2:
        im = im[:,:,0]
    im_np = np.array(im)  #Whitespace is true, black is false
    #im_np = np.logical_not(im_np)    
    return im_np

def load_map_yaml(filename):
    with open("../maps/" + filename, "r") as stream:
            map_settings_dict = yaml.safe_load(stream)
    return map_settings_dict

# easy bounds
class EasyBounds:
    def __init__(self, x, y, width, height):
        """
        x: float
        y: float
        width: float (positive)
        height: float (positive)
        """
        self.x = x
        self.y = y
        self.width = width
        self.height = height
    def bottom_left(self):
        return np.array([self.x, self.y])
    def bottom_right(self):
        return np.array([self.x + self.width, self.y])
    def top_right(self):
        return np.array([self.x + self.width, self.y + self.height])
    def top_left(self):
        return np.array([self.x, self.y + self.height])

#Node for building a graph
class Node:
    def __init__(self, point, parent_id, cost):
        self.point = point # A 3 by 1 vector [x, y, theta]
        self.parent_id = parent_id # The parent node id that leads to this node (There should only every be one parent in RRT)
        self.cost = cost # The cost to come to this node
        self.children_ids = [] # The children node ids of this node
        return

#Path Planner 
class PathPlanner:
    #A path planner capable of perfomring RRT and RRT*
    def __init__(self, map_filename, map_setings_filename, goal_point, stopping_dist, outer_easy_bounds, hyperparameters={}):

        # hyperparameters
        self.hp_duplicate_threshold = hyperparameters.get("duplicate_threshold", 0.05)
        self.hp_rrt_num_to_search_nearby = hyperparameters.get("rrt_num_to_search_nearby", 100)
        self.hp_rrt_nearby_easy_bounds_size = hyperparameters.get("rrt_nearby_easy_bounds_size", 6)
        self.hp_rrt_nearby_search_reset_on_found = hyperparameters.get("rrt_nearby_search_reset_on_found", True)
        self.hp_rrt_collision_reduce_nearby_search = hyperparameters.get("rrt_collision_reduce_nearby_search", 0)
        self.hp_ctrl_kpv = hyperparameters.get("ctrl_kpv", 1)
        self.hp_ctrl_kpw = hyperparameters.get("ctrl_kpw", 1)

        #Get map information
        self.occupancy_map = load_map(map_filename)
        self.map_shape = self.occupancy_map.shape
        self.map_settings_dict = load_map_yaml(map_setings_filename)

        # easy bounds
        self.outer_easy_bounds = outer_easy_bounds

        #Robot information
        self.robot_radius = 0.22 #m
        self.vel_max = 0.5 #m/s (Feel free to change!)
        self.rot_vel_max = 0.2 #rad/s (Feel free to change!)

        #Goal Parameters
        self.goal_point = goal_point #m
        self.stopping_dist = stopping_dist #m

        #Trajectory Simulation Parameters
        self.timestep = 1.0 #s
        self.num_substeps = 10

        #Planning storage
        self.nodes = [Node(np.zeros((3,1)), -1, 0)]

        #RRT* Specific Parameters
        self.lebesgue_free = np.sum(self.occupancy_map) * self.map_settings_dict["resolution"] **2
        self.zeta_d = np.pi
        self.gamma_RRT_star = 2 * (1 + 1/2) ** (1/2) * (self.lebesgue_free / self.zeta_d) ** (1/2)
        self.gamma_RRT = self.gamma_RRT_star + .1
        self.epsilon = 2.5
        
        #Pygame window for visualization
        self.window = pygame_utils.PygameWindow(
            "Path Planner", (1000, 1000), self.occupancy_map.shape, self.map_settings_dict, self.goal_point, self.stopping_dist)
        
        # draw easy bounds
        self.draw_easy_bounds(self.outer_easy_bounds)

        return

    def draw_easy_bounds(self, easy_bounds):
        self.window.add_line(easy_bounds.bottom_left(), easy_bounds.bottom_right(), color=(0,0,255))
        self.window.add_line(easy_bounds.bottom_right(), easy_bounds.top_right(), color=(0,0,255))
        self.window.add_line(easy_bounds.top_right(), easy_bounds.top_left(), color=(0,0,255))
        self.window.add_line(easy_bounds.top_left(), easy_bounds.bottom_left(), color=(0,0,255))

    #Functions required for RRT
    def sample_map_space(self, easy_bounds, GOAL_BIASING=True):
        """
        Return an [x, y] coordinate to drive the robot towards.
        Samples randomly within the map bounds, with a 5% chance to sample the goal.
        
        Returns:
            np.ndarray: 2 x 1 vector [x; y]
        """

        # goal biasing (will help converge faster)
        # 5% of the time, try to go straight to the goal
        if GOAL_BIASING:
            if np.random.rand() < 0.01:
                return self.goal_point
        
        # # regular random sampling
        random_x = np.random.uniform(easy_bounds.x, easy_bounds.x + easy_bounds.width)
        random_y = np.random.uniform(easy_bounds.y, easy_bounds.y + easy_bounds.height)

        sampled_point = np.array([[random_x], [random_y]])
        # print(sampled_point)

        return sampled_point
    
    def check_if_duplicate(self, point):
        """
        Check if point is a duplicate of an already existing node (or close enough).
        
        Args:
            point (np.ndarray): 2 x 1 vector [x; y]
        Returns:
            bool: True if a node already exists within a small threshold
        """
        threshold = self.hp_duplicate_threshold
        for node in self.nodes:
            node_xy = node.point[0:2, :]
            dist = np.linalg.norm(point-node_xy)
            if dist < threshold:
                print(f"point {point} is a duplicate")
                return True
        return False
    
    def closest_node(self, point):
        """
        Returns the index of the closest node in the tree to the sampled point.
        
        Args:
            point (np.ndarray): 2 x 1 vector [x; y]
        Returns:
            int: The index of the closest node in self.nodes
        """
        all_nodes_xy = np.hstack([node.point[0:2,:] for node in self.nodes])
        diff = all_nodes_xy - point # numpy broadcast
        dists = np.linalg.norm(diff, axis=0)
        closest_idx = np.argmin(dists)

        # print(f"closest node point to {point} is {self.nodes[closest_idx].point[0:2,:]}")

        return int(closest_idx)
    
    def simulate_trajectory(self, point_i, point_s):
        """
        Simulates the non-holonomic motion of the robot from node_i towards point_s.
        
        Args:
            point_i (np.ndarray): 3 by 1 vector [x; y; theta] (the starting pose)
            point_s (np.ndarray): 2 by 1 vector [x; y] (the sampled point)
            
        Returns:
            np.ndarray: 3 x num_substeps matrix representing the robot's path
        """
        # Get the control inputs (v, omega) based on the current state and target
        vel, rot_vel = self.robot_controller(point_i, point_s)

        # Roll out the trajectory starting from the current node's pose
        # We pass node_i as the x0 starting point
        robot_traj = self.trajectory_rollout(vel, rot_vel, point_i=point_i)
        
        # print_trajectory(robot_traj)

        return robot_traj
    
    def robot_controller(self, point_i, point_s):
        """
        Determines the velocities to move the robot from point_i towards point_s.
        
        Args:
            point_i (np.ndarray): 3 x 1 vector [x; y; theta] (current pose)
            point_s (np.ndarray): 2 x 1 vector [x; y] (target point)
        Returns:
            tuple: (vel: float, rot_vel: float) clamped by max limits
        """
        KP_V = self.hp_ctrl_kpv
        KP_W = self.hp_ctrl_kpw

        x_i, y_i, theta_i = point_i
        x_s, y_s = point_s

        dist_to_target = np.sqrt((x_s-x_i)**2 + (y_s-y_i)**2)

        angle_to_target = np.arctan2(y_s-y_i,x_s-x_i) - theta_i

        # normalize angle_to_target to [-pi, pi]
        angle_to_target = (angle_to_target + np.pi) % (2*np.pi) - np.pi

        # scale rotational velocity based on angle_to_target
        # apply max angular velocity at > pi/2
        if np.abs(angle_to_target) > np.pi/2:
            rot_vel = self.rot_vel_max
        else:
            rot_vel = KP_W * self.rot_vel_max * np.pi/2 * np.abs(angle_to_target)
        
        # linear velocity: proportional
        vel = KP_V * dist_to_target

        # enforce limits
        vel = min(vel, self.vel_max)
        rot_vel = min(np.abs(rot_vel), self.rot_vel_max)
        rot_vel *= np.sign(angle_to_target) # ensure correct direction

        return vel, rot_vel
    
    def trajectory_rollout(self, vel, rot_vel, point_i):
        """
        Given chosen velocities, determine the trajectory of the robot 
        for the given timestep using the unicycle model.
        
        Args:
            vel (float): Linear velocity (m/s)
            rot_vel (float): Angular velocity (rad/s)
        Returns:
            np.ndarray: 3 x num_substeps matrix of [x; y; theta] points
        """

        x0, y0, theta0 = point_i.flatten()

        t = np.linspace(0, self.timestep, self.num_substeps)

        if abs(rot_vel) < 1e-6:
            # omega = 0, straight line motion
            x = x0 + vel * np.cos(theta0) * t
            y = y0 + vel * np.sin(theta0) * t
            theta = np.full_like(t, theta0)
        else:
            # omega != 0, arc motion
            theta = theta0 + rot_vel * t
            x = x0 + (vel/rot_vel) * (np.sin(theta) - np.sin(theta0))
            y = y0 - (vel/rot_vel) * (np.cos(theta) - np.cos(theta0))

        traj = np.stack([x,y,theta], axis=1).T

        return traj
    
    def point_to_cell(self, point):
        """
        Convert a series of [x,y] points in the map to the indices for the corresponding cell in the occupancy map
        
        Args:
            point (np.ndarray): An 2 x N matrix of points of interest, where N is the number of points.

        Returns:
            np.ndarray: An array of cell indices [row,col] in the occupancy map corresponding to each input point.
        """
        assert point.ndim == 2
        assert point.shape[0] == 2

        # get values from map settings
        origin = np.array(self.map_settings_dict["origin"])[:2].reshape(2,1) # origin: [o_x, o_y, 0.000]
        resolution = self.map_settings_dict["resolution"]

        # Translate and Scale
        grid_coords = np.floor((point-origin) / resolution).astype(int)

        # grid_coords[0] is 'x' (column index)
        # grid_coords[1] is 'y' (row index)

        # Axis Inversion
        rows = (self.map_shape[0] - 1) - grid_coords[1,:] # ?
        cols = grid_coords[0,:]

        # Stack as [row, col]
        cell_indices = np.column_stack((rows, cols))

        return cell_indices

    def points_to_robot_circle(self, points):
        """
        Convert a series of [x,y] points to robot map footprints for collision detection

        Args:
            points (np.ndarray): 2 x N matrix of [x, y] world positions.
        Returns:
            tuple: (list of row arrays, list of col arrays) representing the footprints.
        """
        assert points.ndim == 2
        assert points.shape[0] == 2

        # get values from map settings
        resolution = self.map_settings_dict["resolution"]

        # Convert world points to map cells
        cells = self.point_to_cell(points)

        # Convert robot radius using resolution
        robot_radius = self.robot_radius / resolution

        # Use disk function to find circles
        footprint_rows = []
        footprint_cols = []

        for i in range(cells.shape[0]):
            row = cells[i, 0]
            col = cells[i, 1]

            rr, cc = disk((row, col), robot_radius, shape=self.map_shape)

            footprint_rows.append(rr)
            footprint_cols.append(cc)

        return footprint_rows, footprint_cols
    #Note: If you have correctly completed all previous functions, then you should be able to create a working RRT function

    #RRT* specific functions
    def ball_radius(self):
        #Close neighbor distance
        card_V = len(self.nodes)
        return min(self.gamma_RRT * (np.log(card_V) / card_V ) ** (1.0/2.0), self.epsilon)
    
    def connect_node_to_point(self, node_i, point_f):
        #Given two nodes find the non-holonomic path that connects them
        #Settings
        #node is a 3 by 1 node
        #point is a 2 by 1 point
        print("TO DO: Implement a way to connect two already existing nodes (for rewiring).")
        return np.zeros((3, self.num_substeps))
    
    def cost_to_come(self, trajectory_o):
        #The cost to get to a node from lavalle 
        print("TO DO: Implement a cost to come metric")
        return 0
    
    def update_children(self, node_id):
        #Given a node_id with a changed cost, update all connected nodes with the new cost
        print("TO DO: Update the costs of connected nodes after rewiring.")
        return

    #Planner Functions
    def rrt_planning(self, max_iters=5000):

        iter = 0
        iter_nearby = 0
        num_to_search_nearby = self.hp_rrt_num_to_search_nearby
        nearby_bounds_size = self.hp_rrt_nearby_easy_bounds_size
        searching_nearby = False
        nearby_bounds = None

        collision_reduce_nearby_factor = 1

        while True:
            #Sample map space
            
            if searching_nearby:
                point = self.sample_map_space(nearby_bounds)
                iter_nearby += 1
                if iter_nearby >= num_to_search_nearby:
                    searching_nearby = False
            else:
                point = self.sample_map_space(self.outer_easy_bounds)

            # check if duplicate
            if self.check_if_duplicate(point):
                continue

            ### SHOW POINT ###
            self.window.add_point(
                map_frame_point=np.array([point[0][0], point[1][0]]),
                radius=2,
                color=(255,0,0)
            )

            #Get the closest point
            closest_node_id = self.closest_node(point)
            closest_node = self.nodes[closest_node_id]

            #Simulate driving the robot towards the closest point
            trajectory_o = self.simulate_trajectory(closest_node.point, point)

            #Check for collisions
            footprint_rows, footprint_cols = self.points_to_robot_circle(trajectory_o[:2,:])

            collision_detected = False
            for r, c in zip(footprint_rows, footprint_cols):
                if np.any(self.occupancy_map[r, c] == 0):
                    collision_detected = True
                    break
            
            if not collision_detected:
                # create a new node at end of collision-free trajectory
                new_point = trajectory_o[:, -1].reshape(3,1)

                # compute cost
                segment_dist = np.linalg.norm(new_point[0:2] - closest_node.point[0:2])
                new_cost = closest_node.cost + segment_dist # cost not used for regular RRT

                new_node = Node(new_point, closest_node_id, new_cost)
                # print(new_node.point[0][0])
                self.nodes.append(new_node)
                
                # num_valid_points_found_in_grid[grid_num] += 1

                ### SHOW POINT ###
                self.window.add_point(
                    map_frame_point=np.array([new_node.point[0][0], new_node.point[1][0]]),
                    radius=2,
                    color=(0,255,0)
                )

                # update parent's children list
                new_node_id = len(self.nodes) - 1
                closest_node.children_ids.append(new_node_id)

                # check if goal has been reached
                dist_to_goal = np.linalg.norm(new_point[0:2] - self.goal_point)
                if dist_to_goal < self.stopping_dist:
                    print(f"Goal reached in {iter} iterations!")
                    return self.nodes
                
                if not searching_nearby or self.hp_rrt_nearby_search_reset_on_found:
                    nearby_bounds = EasyBounds(
                        max(new_node.point[0][0] - nearby_bounds_size/2, self.outer_easy_bounds.x),
                        max(new_node.point[1][0] - nearby_bounds_size/2, self.outer_easy_bounds.y),
                        nearby_bounds_size,
                        nearby_bounds_size
                    )
                    searching_nearby = True
                    iter_nearby = 0
            else:
                if self.hp_rrt_collision_reduce_nearby_search > 0:
                    iter_nearby = min(
                        (iter_nearby + self.hp_rrt_collision_reduce_nearby_search)*collision_reduce_nearby_factor, 
                         0
                    )
                if searching_nearby:
                    collision_reduce_nearby_factor += 3
                else:
                    collision_reduce_nearby_factor = max(1, collision_reduce_nearby_factor-1)

            print(f"iter {iter}; {'valid' if not collision_detected else 'collision'}; searching {'nearby' if searching_nearby else 'whole map'}")
            iter += 1

    def rrt_star_planning(self):
        #This function performs RRT* for the given map and robot        
        for i in range(1): #Most likely need more iterations than this to complete the map!
            #Sample
            point = self.sample_map_space()

            #Closest Node
            closest_node_id = self.closest_node(point)

            #Simulate trajectory
            trajectory_o = self.simulate_trajectory(self.nodes[closest_node_id].point, point)

            #Check for Collision
            print("TO DO: Check for collision.")

            #Last node rewire
            print("TO DO: Last node rewiring")

            #Close node rewire
            print("TO DO: Near point rewiring")

            #Check for early end
            print("TO DO: Check for early end")
        return self.nodes
    
    def recover_path(self, node_id = -1):
        path = [self.nodes[node_id].point]
        current_node_id = self.nodes[node_id].parent_id
        while current_node_id > -1:
            path.append(self.nodes[current_node_id].point)
            current_node_id = self.nodes[current_node_id].parent_id
        path.reverse()
        return path

def main():
    #Set map information
    map_filename = "willowgarageworld_05res.png"
    map_setings_filename = "willowgarageworld_05res.yaml"

    #robot information
    goal_point = np.array([[42.05], [-44]]) #m
    stopping_dist = 0.5 #m

    #RRT precursor
    if "willow" in map_filename:
            outer_easy_bounds = EasyBounds(-2, -48, 47, 60)
    elif "myhal" in map_filename:
        pass
    else:
        raise ValueError("Unknown map")

    path_planner = PathPlanner(map_filename, map_setings_filename, goal_point, stopping_dist, outer_easy_bounds)    
    nodes = path_planner.rrt_star_planning()
    node_path_metric = np.hstack(path_planner.recover_path())

    #Leftover test functions
    np.save("shortest_path.npy", node_path_metric)

def rrt_planning_test():
    #Set map information
    map_filename = "willowgarageworld_05res.png"
    map_setings_filename = "willowgarageworld_05res.yaml"

    #robot information
    goal_point = np.array([[42.05], [-44]]) #m
    stopping_dist = 0.5 #m

    #RRT precursor
    if "willow" in map_filename:
            outer_easy_bounds = EasyBounds(-1, -48, 45, 58)
    elif "myhal" in map_filename:
        pass
    else:
        raise ValueError("Unknown map")

    hyperparameters = {
        "duplicate_threshold": 0.05, # m
        "rrt_num_to_search_nearby": 80,
        "rrt_nearby_easy_bounds_size": 6,
        "rrt_nearby_search_reset_on_found": True,
        "rrt_collision_reduce_nearby_search": 2,
        "ctrl_kpv": 1,
        "ctrl_kpw": 1
    }

    path_planner = PathPlanner(map_filename, map_setings_filename, goal_point, stopping_dist, outer_easy_bounds, hyperparameters)
    nodes = path_planner.rrt_planning()
    node_path_metric = np.hstack(path_planner.recover_path())

    #Leftover test functions
    np.save("path.npy", node_path_metric)

    while True:
        pass

# --------------------- debuggers ---------------------
def print_nodes(nodes):
    for node in nodes:
        pt = node.point
        print(f"{pt[0][0]}, {pt[1][0]}, {pt[2][0]}")

def print_trajectory(traj):
    print("robot_traj:")
    print(f"num steps: {traj.shape[1]}")
    for i in range(traj.shape[1]):
        print(f"{traj[0][i]}, {traj[1][i]}, {traj[2][i]}")

if __name__ == '__main__':

    # set seed
    np.random.seed(42)

    # main()

    rrt_planning_test()
