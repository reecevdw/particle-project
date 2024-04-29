#!/usr/bin/env python3

import rospy

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Quaternion, Point, Pose, PoseArray, PoseStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header, String

import tf
from tf import TransformListener
from tf import TransformBroadcaster
from tf.transformations import quaternion_from_euler, euler_from_quaternion

import numpy as np
from numpy.random import random_sample
import math

from random import randint, random
import copy



def compute_prob_zero_centered_gaussian(dist, sd):
    """ Takes in distance from zero (dist) and standard deviation (sd) for gaussian
        and returns probability (likelihood) of observation """
    c = 1.0 / (sd * math.sqrt(2 * math.pi))
    prob = c * math.exp((-math.pow(dist,2))/(2 * math.pow(sd, 2)))
    return prob
    

def get_yaw_from_pose(p):
    """ A helper function that takes in a Pose object (geometry_msgs) and returns yaw"""

    yaw = (euler_from_quaternion([
            p.orientation.x,
            p.orientation.y,
            p.orientation.z,
            p.orientation.w])
            [2])

    return yaw




class Particle:

    def __init__(self, pose, w):

        # particle pose (Pose object from geometry_msgs)
        self.pose = pose

        # particle weight
        self.w = w



class ParticleFilter:


    def __init__(self):

        # once everything is setup initialized will be set to true
        self.initialized = False


        # initialize this particle filter node
        rospy.init_node('turtlebot3_particle_filter')

        # set the topic names and frame names
        self.base_frame = "base_footprint"
        self.map_topic = "map"
        self.odom_frame = "odom"
        self.scan_topic = "scan"

        # inialize our map
        self.map = OccupancyGrid()

        # the number of particles used in the particle filter
        self.num_particles = 5000

        # initialize the particle cloud array
        self.particle_cloud = []

        # initialize the estimated robot pose
        self.robot_estimate = Pose()

        # set threshold values for linear and angular movement before we preform an update
        self.lin_mvmt_threshold = 0.2
        self.ang_mvmt_threshold = (np.pi / 6)

        self.odom_pose_last_motion_update = None

        # Setup publishers and subscribers

        # publish the current particle cloud
        self.particles_pub = rospy.Publisher("particle_cloud", PoseArray, queue_size=10)

        # publish the estimated robot pose
        self.robot_estimate_pub = rospy.Publisher("estimated_robot_pose", PoseStamped, queue_size=10)

        # subscribe to the map server
        rospy.Subscriber(self.map_topic, OccupancyGrid, self.get_map)

        # subscribe to the lidar scan from the robot
        rospy.Subscriber(self.scan_topic, LaserScan, self.robot_scan_received)

        # enable listening for and broadcasting corodinate transforms
        self.tf_listener = TransformListener()
        self.tf_broadcaster = TransformBroadcaster()

        # initialize the likelihood field
        self.likeihood_field = self.compute_likelihood_field(self.map)

        # intialize the particle cloud
        self.initialize_particle_cloud()

        self.initialized = True


    def get_map(self, data):

        self.map = data
        # print(f"Map: {data}")
        print(f"Map Width: {data.info.width}")
        print(f"Map Height: {data.info.height}")
        print(f"Map Resolution: {data.info.resolution}")
        print(f"Map Width (m): {data.info.width * data.info.resolution}")
        print(f"Map Height (m): {data.info.height *data.info.resolution}")

    def compute_likelihood_field(self, occupancy_grid_map):
        # Initialize the likelihood field with infinity values
        likelihood_field = np.full((occupancy_grid_map.info.width, occupancy_grid_map.info.height), np.inf)

        # Iterate through every cell in the occupancy grid
        for mx in range(occupancy_grid_map.info.width):
            for my in range(occupancy_grid_map.info.height):
                # Convert map coordinates to world coordinates
                wx = mx * occupancy_grid_map.info.resolution + occupancy_grid_map.info.origin.position.x
                wy = my * occupancy_grid_map.info.resolution + occupancy_grid_map.info.origin.position.y
                # Check if the current cell is an obstacle
                min_dist = np.inf
                if occupancy_grid_map.data[my * occupancy_grid_map.info.width + mx] == 0: # Only compute for inside map
                    # For free cells, search for the closest obstacle
                    for ox in range(occupancy_grid_map.info.width):
                        for oy in range(occupancy_grid_map.info.height):
                            if occupancy_grid_map.data[oy * occupancy_grid_map.info.width + ox] == 100:  # Obstacle value
                                # Compute distance to this obstacle
                                dist = np.hypot(wx - (ox * occupancy_grid_map.info.resolution + occupancy_grid_map.info.origin.position.x),
                                                wy - (oy * occupancy_grid_map.info.resolution + occupancy_grid_map.info.origin.position.y))
                                # If this is the closest obstacle so far, update min_dist
                                if dist < min_dist:
                                    min_dist = dist
                # Update the likelihood field with the closest distance
                likelihood_field[mx, my] = min_dist
                #if min_dist != np.inf:
                    #print(f"likelihood_field:\tmx={mx}\tmy={my}: {likelihood_field[mx, my]}")

        return likelihood_field

    def initialize_particle_cloud(self):
        rospy.sleep(1)  # Wait for ROS nodes and topics to be ready
        free_space_indices = np.where(np.array(self.map.data) == 0)[0]
        self.particle_cloud = []
        for _ in range(self.num_particles):
            index = np.random.choice(free_space_indices)
            x = (index % self.map.info.width) * self.map.info.resolution + self.map.info.origin.position.x
            y = (index // self.map.info.width) * self.map.info.resolution + self.map.info.origin.position.y
            theta = np.random.uniform(0, 2*np.pi)
            q = quaternion_from_euler(0, 0, theta)
            pose = Pose(Point(x, y, 0), Quaternion(*q))
            self.particle_cloud.append(Particle(pose, 1.0 / self.num_particles))
        self.normalize_particles()
        self.publish_particle_cloud()


    def normalize_particles(self):
        #this ensure that the sum of the weights add to 1

        for particle in self.particle_cloud:
            if particle.w != 0:
                print(f"normalize\tOld particle weight {particle.w}")
        total_weight = sum(particle.w for particle in self.particle_cloud) # if np.isfinite(particle.w))
        print(f"normalize:\tTotal Weight: {total_weight}")
        if total_weight == 0:
            print("normalize:\tWarning: Total particle weight is zero. Assigning equal weights.")
            equal_weight = 1.0 / len(self.particle_cloud)
            for particle in self.particle_cloud:
                particle.w = equal_weight
        else:
            for particle in self.particle_cloud:
                particle.w = particle.w / total_weight
                print(f"normalize\tNew particle weight {particle.w}")
        
        print(f"normalize\tTotal Weight: {total_weight}")

        print("normalize\tSum of weights after normalization:", sum([particle.w for particle in self.particle_cloud]))

    def publish_particle_cloud(self):

        particle_cloud_pose_array = PoseArray()
        particle_cloud_pose_array.header = Header(stamp=rospy.Time.now(), frame_id=self.map_topic)
        particle_cloud_pose_array.poses

        for part in self.particle_cloud:
            particle_cloud_pose_array.poses.append(part.pose)

        self.particles_pub.publish(particle_cloud_pose_array)




    def publish_estimated_robot_pose(self):

        robot_pose_estimate_stamped = PoseStamped()
        robot_pose_estimate_stamped.pose = self.robot_estimate
        robot_pose_estimate_stamped.header = Header(stamp=rospy.Time.now(), frame_id=self.map_topic)
        self.robot_estimate_pub.publish(robot_pose_estimate_stamped)



    def resample_particles(self):

        # we need to create new cloud of particles so that there is no reference to previous particles

        temp_copy = copy.deepcopy(self.particle_cloud)
        particles = np.random.choice(temp_copy, self.num_particles, replace=True, p=[particle.w for particle in self.particle_cloud])

        self.particle_cloud = particles

        print(f"resample:\tnumber of particles:{len(self.particle_cloud)}")
        #print(f"Resample_Particles:\tdraw_random:{particle.w for particle in self.particle_cloud if np.isfinite(particle.w)}")

    def robot_scan_received(self, data):

        # wait until initialization is complete
        if not(self.initialized):
            return

        # we need to be able to transfrom the laser frame to the base frame
        if not(self.tf_listener.canTransform(self.base_frame, data.header.frame_id, data.header.stamp)):
            return

        # wait for a little bit for the transform to become avaliable (in case the scan arrives
        # a little bit before the odom to base_footprint transform was updated)
        self.tf_listener.waitForTransform(self.base_frame, self.odom_frame, data.header.stamp, rospy.Duration(0.5))
        if not(self.tf_listener.canTransform(self.base_frame, data.header.frame_id, data.header.stamp)):
            return

        # calculate the pose of the laser distance sensor
        p = PoseStamped(
            header=Header(stamp=rospy.Time(0),
                          frame_id=data.header.frame_id))

        self.laser_pose = self.tf_listener.transformPose(self.base_frame, p)

        # determine where the robot thinks it is based on its odometry
        p = PoseStamped(
            header=Header(stamp=data.header.stamp,
                          frame_id=self.base_frame),
            pose=Pose())

        self.odom_pose = self.tf_listener.transformPose(self.odom_frame, p)

        # we need to be able to compare the current odom pose to the prior odom pose
        # if there isn't a prior odom pose, set the odom_pose variable to the current pose
        if not self.odom_pose_last_motion_update:
            self.odom_pose_last_motion_update = self.odom_pose
            return


        if len(self.particle_cloud) > 0:

            # check to see if we've moved far enough to perform an update

            curr_x = self.odom_pose.pose.position.x
            old_x = self.odom_pose_last_motion_update.pose.position.x
            curr_y = self.odom_pose.pose.position.y
            old_y = self.odom_pose_last_motion_update.pose.position.y
            curr_yaw = get_yaw_from_pose(self.odom_pose.pose)
            old_yaw = get_yaw_from_pose(self.odom_pose_last_motion_update.pose)

            if (np.abs(curr_x - old_x) > self.lin_mvmt_threshold or
                np.abs(curr_y - old_y) > self.lin_mvmt_threshold or
                np.abs(curr_yaw - old_yaw) > self.ang_mvmt_threshold):

                # This is where the main logic of the particle filter is carried out

                print(f"\n******************\nNew Instance:\n******************\n")

                self.update_particles_with_motion_model()

                self.update_particle_weights_with_measurement_model(data)

                self.normalize_particles()

                self.resample_particles()

                self.update_estimated_robot_pose()

                self.publish_particle_cloud()
                self.publish_estimated_robot_pose()

                self.odom_pose_last_motion_update = self.odom_pose

    def update_estimated_robot_pose(self):
        x = np.mean([particle.pose.position.x for particle in self.particle_cloud])
        y = np.mean([particle.pose.position.y for particle in self.particle_cloud])
        orientations = np.array([get_yaw_from_pose(particle.pose) for particle in self.particle_cloud])
        avg_orientation = np.arctan2(np.sin(orientations).mean(), np.cos(orientations).mean())
        q = quaternion_from_euler(0, 0, avg_orientation)
        self.robot_estimate = Pose(Point(x, y, 0), Quaternion(*q))

    # def update_particle_weights_with_measurement_model(self, data):
    #     for particle in self.particle_cloud:
    #         # Assume we have a function that computes the expected measurements for a particle and compares them to actual measurements
    #         # For simplicity, let's assume all weights are updated by a fixed amount (in a real scenario, you would use sensor data)
    #         particle.w *= compute_prob_zero_centered_gaussian(0, 1)  # Example with a dummy distance and SD

    def update_particle_weights_with_measurement_model(self, data):
        for particle in self.particle_cloud:
            
            particle_weight = 1.0  #q
            #z_hit = 1
            sd = 1
            dist = np.inf

            #[::5]
            for k in range(len(data.ranges) - 1):  #loop through the lidar
                if k % 50:
                    continue
                #print(f"lidar: {k}, len:{len(data.ranges)-1}")
                max_lidar = 3
                z_k = data.ranges[k]
                if data.ranges[k] != max_lidar:
                    x = particle.pose.position.x
                    y = particle.pose.position.y
                    x_k_sense = 0
                    y_k_sense = 0
                    theta = get_yaw_from_pose(particle.pose)
                    theta_k_sense = np.radians(k)
                    x_z = x + x_k_sense * np.cos(theta) - y_k_sense * np.sin(theta) + z_k * np.cos(theta + theta_k_sense)
                    y_z = y + y_k_sense * np.cos(theta) - x_k_sense * np.sin(theta) + z_k * np.sin(theta + theta_k_sense)
                    #self.map.data
                    #map.info.origin.position.x or y
                    

                    #print(f"measurement model;\tk={k}\tx_z:{x_z}\ty_z:{y_z}")

                    x_w = round((x_z - self.map.info.origin.position.x) / self.map.info.resolution)
                    y_w = round((y_z - self.map.info.origin.position.y) / self.map.info.resolution) 

                    # Convert map coordinates to world coordinates

                    #print(f"measurement model;\tk={k}\tx_w:{x_w}\ty_w:{y_w}")
                    
                    field = self.likeihood_field

                    dist = field[x_w, y_w]

                    if dist == np.inf:
                        particle_weight *= 1e-10
                    else:
                        particle_weight *= (compute_prob_zero_centered_gaussian(dist,sd))
                    
                     
                    
                    #print(f"update_particle_weights_with_measurement_model:\n\tk={k}\n\t\tx={x}\n\t\ty={y}\n\t\tz_k={z_k}\n\t\ttheta={theta}\n\t\ttheta_k_sense={theta_k_sense}\n\t\tx_z={x_z}\n\t\ty_z={y_z}")
            
            #print(f"Particle Weight {particle} = {particle_weight}")
            particle.w = particle_weight
            #print(f"measurement:\tparticle_weight={particle_weight}")



    def update_particles_with_motion_model(self):

        # based on the how the robot has moved (calculated from its odometry), we'll  move
        # all of the particles correspondingly
        dx = self.odom_pose.pose.position.x - self.odom_pose_last_motion_update.pose.position.x
        dy = self.odom_pose.pose.position.y - self.odom_pose_last_motion_update.pose.position.y
        
        speed = math.sqrt(math.pow(dx,2) + math.pow(dy,2))
        
        dtheta = get_yaw_from_pose(self.odom_pose.pose) - get_yaw_from_pose(self.odom_pose_last_motion_update.pose)
        
        particles = copy.deepcopy(self.particle_cloud)
        self.particle_cloud = []


        for particle in particles:
            p = copy.deepcopy(particle)
            yaw = get_yaw_from_pose(p.pose)
            yaw += dtheta  + np.radians(np.random.randint(-20,20)) #+ np.random.normal(0, 0.005)  # Adding noise
            # dx_noisy = dx + np.random.normal(0, 0.002)  # Adding noise
            # dy_noisy = dy + np.random.normal(0, 0.002)  # Adding noise
            p.pose.position.x += speed * np.cos(yaw) + np.random.normal(0, 0.01)#*self.map.info.resolution
            p.pose.position.y += speed * np.sin(yaw) + np.random.normal(0, 0.01)#*self.map.info.resolution
            # particle.pose.position.x += dx_noisy * np.cos(yaw) - dy_noisy * np.sin(yaw)
            # particle.pose.position.y += dx_noisy * np.sin(yaw) + dy_noisy * np.cos(yaw)
            q = quaternion_from_euler(0, 0, yaw)
            p.pose.orientation = Quaternion(*q)

            self.particle_cloud.append(p)


if __name__=="__main__":


    pf = ParticleFilter()

    rospy.spin()
    