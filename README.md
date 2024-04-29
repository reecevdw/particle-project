# particle_filter_project

1. [Particle Filter Localization Implementation and Testing Plan](#implementation1)
   - [Overview](#overview)
   - [`initialize_particle_cloud`](#initialize)
   - [`update_particles_with_motion_model`](#motion_model)
   - [`update_particle_weights_with_measurement_model`](#measurement_mode)
   - [`normalize_particles`](#normalize)
   - [`resample_particles`](#resample)
   - [`update_estimated_robot_pose`](#update)
   - [Incorporate Noise into Particle Filter Localization](#noise)
   - [Timeline](#timeline)
2. [Objectives Description](#objectives)
3. [Final Submission High-level Description](#high_level)
4. [Steps Overview](#step_overview)
   - [Initialization of Particle Cloud](#initialization_of_particle_cloud)
   - [Creating a Likelihood Field](#fieldl)
   - [Movement Model](#movement_model)
   - [Measurement Model](#measurement_model)
   - [Resampling](#resampling)
   - [Incorporation of Noise](#incorporation_of_noise)
   - [Updating Estimated Robot Pose](#updating_estimated_robot_pose)
   - [Optimization of Parameters](#optimization_of_parameters)
7. [Challenges](#challenges)
8. [Future Work](#future_work)
9. [Takeaways](#take_aways)
10. [Final Videos](#final_videos)
11. [Videos](#videos)

# Particle Filter Localization Implementation and Testing Plan <a name="implementation1"></a>

The name of our team is **TheFew** which is composed of *Reece VanDeWeghe*, *Caleb Dalgleish*, and *Ryan Ziaee*. 

## Overview <a name="overview"></a>

This document outlines the team **TheFew**'s (*Reece VanDeWeghe*, *Ryan Ziaee*, and *Caleb Dalgleish*) plan for implementing and testing a particle filter localization system for a robot, particularly focusing on Robot State Estimation and Robot Localization. We leverage the Bayes filter algorithm's principles, sensor models, and Monte Carlo Localization (MCL) strategies discussed in Lecture 3 and 4.

## 1. Initialize Particle Cloud (`initialize_particle_cloud`) <a name="initialize"></a>

### Implementation
We will initialize the particle cloud by distributing particles randomly across the environment within Gazebo, reflecting the Bayes filter's prior belief about the robot's position. This approach ensures a comprehensive hypothesis space for the robot's initial location.

### Testing
Particle distribution will be visually inspected in Gazebo to confirm uniform coverage. For in-person tests at CSIL 5, manual verification will ensure the distribution matches theoretical expectations, ensuring a robust prior belief.

## 2. Update Particles with Motion Model (`update_particles_with_motion_model`) <a name="motion_model"></a>

### Implementation
Following the Bayes filter prediction step, particle positions will be updated based on robot movements (actions \(u\)) received via ROS topics. This update mimics the robot's motion, adjusting each particle's state to predict the robot's new location.

### Testing
Simulate robot movements in Gazebo, ensuring particle distribution shifts accurately reflect those movements. Real-world testing at CSIL 5 will validate simulation accuracy, emphasizing predictive modeling accuracy.

## 3. Update Particle Weights with Measurement Model (`update_particle_weights_with_measurement_model`) <a name="measurement_mode"></a>

### Implementation
Weights for each particle will be computed by comparing each particle's predicted sensor readings against the actual sensor data from the robot, adhering to sensor models discussed in class. This step aligns with the Bayes filter correction step, adjusting beliefs based on new evidence.

$$w_t^{[m]} = \frac{1}{ \sum _{i=0} \text{abs}(z_t[i] - z_t^{[m]}[i])}
$$

### Testing
Predefined scenarios in Gazebo with known outcomes will test weight computation accuracy. Real-world scenarios at CSIL 5 will compare simulation to physical sensor data, ensuring measurement models are accurately implemented.

## 4. Normalize and Resample Particles

### Implementation 

- **Normalize Particles (`normalize_particles`):** <a name="normalize"></a> Weights will be normalized to sum to one, preparing for resampling. $$\sum_{m} w_t^{[m]} =1$$ 

- **Resample Particles (`resample_particles`):** <a name="resample"></a> This step will focus the particle set on regions with higher probability, directly inspired by the Monte Carlo Localization exercise from class. $$X_t^{[m']} \sim \text{Categorical}\left(\frac{w_t^{[1]}}{W_t}, \frac{w_t^{[2]}}{W_t}, \ldots, \frac{w_t^{[M]}}{W_t}\right)$$ where each new particle $\( X_t^{[m']} \)$ is drawn with a probability proportional to the corresponding weight $\( w_t^{[m]} \)$. $M$ is the number of particles, and $W_t$ is the sum of all weights at time $t$. 

### Testing
Testing will involve assessing the concentration of particles around high-probability areas pre- and post-resampling in both Gazebo and real-world environments at CSIL 5.

## 5. Update Estimated Robot Pose (`update_estimated_robot_pose`) <a name="update"></a>

### Implementation
The robot's estimated pose will be derived from the weighted average of the particle set, incorporating lessons from the class on prediction and correction via sensor measurements.

### Testing
Estimated poses will be compared against actual robot positions in controlled Gazebo simulations and CSIL 5 experiments, measuring the localization system's accuracy.

## 6. Incorporate Noise into Particle Filter Localization <a name="noise"></a>

### Implementation
We will add Gaussian noise to particle states during initialization and after updating movements and measurements to simulate real-world uncertainties in robot positioning and sensor data accuracy.

### Testing
Noise impact on the localization will be evaluated through Gazebo simulations and real-world trials at CSIL 5, focusing on maintaining localization accuracy and adjusting noise parameters for optimal performance.

## Timeline <a name="timeline"></a>

- **(March 31st - April 2nd):** Finalize implementation plan and prepare for initial deliverable submission.

- **By Tuesday, April 2nd, 5:00pm CST:** 
  - **Implementation Plan Due:** Finalize and submit the detailed implementation plan.

- **(April 3rd - April 5th):** Focus on coding and testing the `initialize_particle_cloud` and `update_particles_with_motion_model` functions. Prepare and submit the intermediate deliverable including code, writeup, gif/video, and rosbag.

- **By Friday, April 5th, 5:00pm CST - Intermediate Deliverable: Particle Cloud Initialization & Movement**
  - **Code:** Complete the initialization of the particle cloud and motion model components (`initialize_particle_cloud()` and `update_particles_with_motion_model()`). Ensure that when the code is run, particles are initialized within the map boundaries and move in sync with the robot's movements.
  - **Writeup:** Complete the warm-up sections including objectives description, high-level description, and detailed information on code location & functions for both initialization of the particle cloud and the movement model.
  - **gif / Embedded Video:** Produce a gif or an embedded video showing the particles being initialized in RViz and moving in accordance with the robot's movements.
  - **rosbag:** Record a rosbag of the particle initialization and movement, capturing at least 5 movement updates.

- **(April 6th - April 12th):** Implement and test `update_particle_weights_with_measurement_model`, `normalize_particles`, and begin work on `resample_particles`. Continue documentation and start preparing for the final submission.

- **By Friday, April 12th, 5:00pm CST - Final Submission:**
  - Submit all components of the project including updated code, comprehensive writeup, additional gifs/videos showcasing further stages of the project, and multiple rosbags demonstrating different aspects of particle filter functionality.
  - Complete the Partner Contributions Survey to reflect the division of work and contributions of each team member.
 
# Objectives Description <a name="objectives"></a>

The objective of this project is to program a robot using ROS, enabling it to identify its location within an environment. By leveraging probabilistic methods and analyzing sensor data, the robot enhances its localization accuracy progressively. This hands-on experience aims to deepen understanding of robotics programming and the practical application of theoretical concepts in robot localization.

# Final Submission, High-level Description <a name="high_level"></a>

At a high-level, our solution to robot localization employs a particle filter algorithm, which is a probabilistic method that estimates the robot's position and orientation within an environment. The process begins with the initialization of a cloud of particles, each representing a potential state (position and orientation) of the robot. As the robot moves and receives sensor data, these particles are updated to reflect the new estimated states based on the motion model and the measurement model. The motion model predicts the next state of each particle based on the robot's movements, while the measurement model adjusts the particles' weights according to how well the sensor data matches the predicted state. To maintain a set of particles that closely represent the robot's actual state, we perform resampling, drawing a new set of particles from the existing set with probability proportional to their weights. This cycle of prediction, measurement, and resampling continues as the robot moves through its environment, allowing the particles to converge over time to the robot's actual location. The estimated position of the robot is then derived by aggregating the states of the particles, providing a robust estimate even in the presence of noise and uncertainty in sensor data and motion. This approach leverages ROS for robot control and sensor data integration, numpy for efficient mathematical computations, and custom algorithms for probabilistic reasoning and sensor data interpretation.

# Steps Overview <a name="step_overview"></a>

## Initialization of Particle Cloud <a name="initialization_of_particle_cloud"></a>
- **Code Location:** This step is implemented in the `initialize_particle_cloud` method of the `ParticleFilter` class.
- **Function Description:** This function initializes the particle cloud by distributing particles randomly across the free space of the map. Each particle is given an initial weight, and their positions and orientations are randomized within the map's bounds, leveraging the map's occupancy grid to ensure particles are placed in navigable spaces.
- **Lidar Compatibility:** This step does not incorporate lidar so it is compatible with both models.

## Creating a Likelihood Field <a name="fieldl"></a>
- **Code Location:** The likelihood field is generated within the 'compute_likelihood_field' method of the 'ParticleFilter' class.
- **Function Description:** This function constructs a field that represents the proximity of each grid cell to the nearest obstacle based on the map data. By iterating through each cell and calculating the minimum distance to an obstacle, it assigns a likelihood value to each cell that influences the weighting of particles during the measurement update phase. This field is essential for accurately assessing the probability of a particle's position relative to observed measurements.
- **Lidar Compatibility:** The likelihood field itself is independent of the specific LiDAR model used as it is based purely on the map data. However, it is needed for interpreting LiDAR data during the measurement model step, making it effectively compatible with any LiDAR model that provides distance measurements.

## Movement Model <a name="movement_model"></a>
- **Code Location:** The movement model is applied in the `update_particles_with_motion_model` method.
- **Function Description:** This method updates each particle's position based on the robot's movement, as detected through odometry. By calculating the difference in position and orientation (yaw) from the last update, it shifts all particles accordingly. This models how we expect the robot's movement to influence its probable new location.
- **Lidar Compatibility:** This step does not incorporate lidar so it is compatible with both models.

## Measurement Model <a name="measurement_model"></a>
- **Code Location:** Measurements are processed in the `update_particle_weights_with_measurement_model` function.
- **Function Description:** This function updates the weights of each particle based on how well the predicted measurements (based on the particle's position) match the actual sensor data from the robot's LiDAR. It's crucial for associating sensor observations with particle predictions.
- **Lidar Compatibility:** This step is compatible with LDS-01.

## Resampling <a name="resampling"></a>
- **Code Location:** Resampling is carried out in the `resample_particles` method.
- **Function Description:** This method generates a new set of particles from the current set, with the probability of each particle being chosen based on its weight. This step focuses on increasing the number of particles in regions of high likelihood, effectively concentrating the particle cloud around the most probable robot positions.
- **Lidar Compatibility:** This step does not incorporate lidar so it is compatible with both models.

## Incorporation of Noise <a name="incorporation_of_noise"></a>
- **Code Location:** Noise is incorporated within `update_particles_with_motion_model` and `update_particle_weights_with_measurement_model` methods.
- **Function Description:** When updating the particles' positions and weights, random Gaussian noise is added to account for the uncertainty in motion and measurement. This helps to maintain diversity in the particle cloud and prevents premature convergence.
- **Lidar Compatibility:** This step does not incorporate lidar so it is compatible with both models.

## Updating Estimated Robot Pose <a name="updating_estimated_robot_pose"></a>
- **Code Location:** This step is implemented in the `update_estimated_robot_pose` method.
- **Function Description:** The function calculates the mean position and orientation of all particles to estimate the robot's current pose. This aggregation provides a robust estimate of the robot's location by considering the collective prediction of all particles.
- **Lidar Compatibility:** This step does not incorporate lidar so it is compatible with both models.

## Optimization of Parameters <a name="optimization_of_parameters"></a>
- **Code Location:** Parameters are optimized across various functions but are primarily set in the `__init__` method of the `ParticleFilter` class.
- **Function Description:** Key parameters, such as the number of particles and thresholds for movement updates, are adjusted here to balance between computational efficiency and the accuracy of the localization. This ongoing adjustment is crucial for the particle filter's performance in different environments and robot models.
- **Lidar Compatibility:** We optimize the parameters to be compatible with LDS-01.

# Challenges <a name="challenges"></a>

One of the significant challenges we faced was getting the particles to update correctly according to the motion model. Initially, changes made to any particle's state during the motion update inadvertently affected the state of other particles. This was due to Python's handling of object references, where modifying one particle altered others because they were not independent copies. To resolve this, we implemented deep copies of particles before updating their states, ensuring that each particle's state is independent and modifications do not propagate unexpectedly. Another challenge was effectively utilizing the likelihood field. Accurately mapping sensor readings to the likelihood field and adjusting particle weights accordingly proved complex. We refined our method for calculating distances and improved the accuracy of our sensor model integration, leading to more reliable particle weight updates.

# Future Work <a name="future_work"></a>

With more time, we would focus on two main areas to enhance our particle filter localization using adaptive resampling and tuning the parameters further. Adaptive resampling would adjust the resampling rate based on the effective number of particles, which could help in reducing computational load and preventing particle deprivation. Tuning certain parameters could possibly be automated using machine learning techniques to dynamically adjust parameters like the noise levels and resampling thresholds based on the robot's performance in real-time. These improvements could significantly enhance the robustness and accuracy of our localization under varying environmental conditions.

# Takewaways <a name="take_aways"></a>

- Throughout this project, we learned the critical importance of thorough debugging and validation of each component of the particle filter. We realized that small errors, especially in handling particle references and likelihood calculations, could significantly affect the overall system's performance. This experience has taught us valuable lessons in the rigorous review and testing of code, especially in a probabilistic context where outcomes are not deterministic. Once we started using a ton of print statements to see real-time changes, it made the world of a difference.
- Understanding the root of the problem seems like it would be an obvious answer to any problem in a project like this, however, it took us awhile to realize that sometimes we were way over-thinking certain aspects to the code implementation. It took going to office hours or talking with the Prof to realize that there was a straightforward approach to the math behind (for example) computing the probabilty, updating the weights, etc. Thus, a valuable takeway with a project like this is to think out the implementation thoroughly before jumping into the code. 
- Working in groups on this project showed the value of effective collaboration, especially when dividing tasks and integrating individual components into a cohesive code base. Regular meetings and code reviews helped us identify discrepancies in our understanding and implementation which facilitated a more unified approach to solving the localization problem. These practices will be beneficial in future projects as our team members agree on objectives and methods.

# Final Videos <a name="final_videos"></a>

These videos were taken simultaneously:

https://github.com/Intro-Robotics-UChicago-Spring-2024/particle-filter-project-thefew/assets/114620452/97cbbe6f-d45c-4789-8758-4b3d71e98aeb

https://github.com/Intro-Robotics-UChicago-Spring-2024/particle-filter-project-thefew/assets/114620452/74509132-522d-400a-bd0f-f7f8f9321591

# Videos <a name="videos"></a>

## Intermediate Deliverable 

https://github.com/Intro-Robotics-UChicago-Spring-2024/particle-filter-project-thefew/assets/114620452/e50cf32e-e509-4165-87bb-8b9beba148a7

https://github.com/Intro-Robotics-UChicago-Spring-2024/particle-filter-project-thefew/assets/77137055/fd3c6105-a9cf-4757-aa7e-17be57cb64c2

https://github.com/Intro-Robotics-UChicago-Spring-2024/particle-filter-project-thefew/assets/114620452/25793503-22b7-4236-8554-8fce5040aac8


