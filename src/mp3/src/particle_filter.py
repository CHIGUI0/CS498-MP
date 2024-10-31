import numpy as np
from maze import Maze, Particle, Robot
import bisect
import rospy
from gazebo_msgs.msg import  ModelState
from gazebo_msgs.srv import GetModelState
import shutil
from std_msgs.msg import Float32MultiArray
from scipy.integrate import ode
import copy

import random

def vehicle_dynamics(t, vars, vr, delta):
    curr_x = vars[0]
    curr_y = vars[1] 
    curr_theta = vars[2]
    
    dx = vr * np.cos(curr_theta)*t
    dy = vr * np.sin(curr_theta)*t
    dtheta = delta
    return [dx,dy,dtheta]

class particleFilter:
    def __init__(self, bob, world, num_particles, sensor_limit, x_start, y_start):
        self.num_particles = num_particles  # The number of particles for the particle filter
        self.sensor_limit = sensor_limit    # The sensor limit of the sensor
        particles = list()

        ##### TODO:  #####
        # Modify the initial particle distribution to be within the top-right quadrant of the world, and compare the performance with the whole map distribution.
        for i in range(num_particles):

            # (Default) The whole map
            x = np.random.uniform(0, world.width)
            y = np.random.uniform(0, world.height)


            ## first quadrant
            # x = 
            # y =

            particles.append(Particle(x = x, y = y, maze = world, sensor_limit = sensor_limit))

        ###############

        self.particles = particles          # Randomly assign particles at the begining
        self.bob = bob                      # The estimated robot state
        self.world = world                  # The map of the maze
        self.x_start = x_start              # The starting position of the map in the gazebo simulator
        self.y_start = y_start              # The starting position of the map in the gazebo simulator
        self.modelStatePub = rospy.Publisher("/gazebo/set_model_state", ModelState, queue_size=1)
        self.controlSub = rospy.Subscriber("/gem/control", Float32MultiArray, self.__controlHandler, queue_size = 1)
        self.control = []                   # A list of control signal from the vehicle
        return

    def __controlHandler(self,data):
        """
        Description:
            Subscriber callback for /gem/control. Store control input from gem controller to be used in particleMotionModel.
        """
        tmp = list(data.data)
        self.control.append(tmp)

    def getModelState(self):
        """
        Description:
            Requests the current state of the polaris model when called
        Returns:
            modelState: contains the current model state of the polaris vehicle in gazebo
        """

        rospy.wait_for_service('/gazebo/get_model_state')
        try:
            serviceResponse = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            modelState = serviceResponse(model_name='polaris')
        except rospy.ServiceException as exc:
            rospy.loginfo("Service did not process request: "+str(exc))
        return modelState

    def weight_gaussian_kernel(self,x1, x2, std = 5000):
        if x1 is None: # If the robot recieved no sensor measurement, the weights are in uniform distribution.
            return 1./len(self.particles)
        else:
            tmp1 = np.array(x1)
            tmp2 = np.array(x2)
            return np.sum(np.exp(-((tmp2-tmp1) ** 2) / (2 * std)))


    def updateWeight(self, readings_robot):
        """
        Description:
            Update the weight of each particles according to the sensor reading from the robot 
        Input:
            readings_robot: List, contains the distance between robot and wall in [front, right, rear, left] direction.
        """

        ## TODO #####
        for particle in self.particles:
            
            weight = self.weight_gaussian_kernel(readings_robot,particle.read_sensor())
            particle.weight = weight

        # Step 2: Normalize weights
        weights = np.array([particle.weight for particle in self.particles])  # Extract weights into a NumPy array

        # Avoid division by zero
        if np.sum(weights) > 0:
            normalized_weights = weights / np.sum(weights)
        else:
            normalized_weights = weights  # This would keep the weights as is if they sum to 0

        # Step 3: Assign normalized weights back to particles
        for particle, norm_weight in zip(self.particles, normalized_weights):
            particle.weight = norm_weight

        assert sum([i.weight for i in self.particles]) > 0.98
        ###############
        # pass

    def resample_particles(self):
        # Define the weight threshold
        threshold = 1 / (2 * len(self.particles))

        # Extract weights from particles
        weights = np.array([particle.weight for particle in self.particles])

        # Set weights less than the threshold to zero
        weights[weights < threshold] = 0

        # Check if all weights are zero, and handle if needed (e.g., return original particles or raise an error)
        if np.all(weights == 0):
            print("All particle weights are zero; returning original particles.")
            return self.particles  # or handle as needed

        # Normalize weights to ensure they sum to 1
        cumulative_sum = np.cumsum(weights)
        cumulative_sum /= cumulative_sum[-1]

        # Prepare to store new particles
        particles_new = []

        # Resampling based on weights
        num_particles_to_resample = len(self.particles)
        for _ in range(num_particles_to_resample):
            # Draw a random number
            random_number = np.random.uniform(0, 1)
            
            # Find the index of the first cumulative weight that exceeds the random number
            index = np.searchsorted(cumulative_sum, random_number)
            
            # Create a deep copy of the selected particle
            new_particle = copy.deepcopy(self.particles[index])
            
            # Optionally perturb the position
            new_particle.x += np.random.uniform(-0.5, 0.5)
            new_particle.y += np.random.uniform(-0.5, 0.5)
            # new_particle.heading = np.random.uniform(0,2*np.pi)
            new_particle.heading += np.random.uniform(-5/180 * np.pi, 5/180 * np.pi)

            # Append the new particle to the list
            particles_new.append(new_particle)

        return particles_new


    def resampleParticle(self):
        """
        Description:
            Perform resample to get a new list of particles 
        """
        # ## TODO #####

        self.particles = self.resample_particles()

    def particleMotionModel(self):
        """
        Description:
            Estimate the next state for each particle according to the control input from actual robot 
            You can either use ode function or vehicle_dynamics function provided above
        """
        ## TODO #####
        for particle in self.particles:
            for c in self.control:
                vars = [particle.x, particle.y, particle.heading]
                vr, delta = c
                dx, dy, heading = vehicle_dynamics(0.01, vars, vr, delta)
                particle.x += dx
                particle.y += dy
                particle.heading += delta * 0.01
        self.control = []

        ###############
        # pass


    def runFilter(self):
        """
        Description:
            Run PF localization
        """
        count = 0 
        while True:
            ## TODO: (i) Implement Section 3.2.2. (ii) Display robot and particles on map. (iii) Compute and save position/heading error to plot. #####
            self.particleMotionModel()
            readings_robot = self.bob.read_sensor()
            self.updateWeight(readings_robot)
            self.resampleParticle()
            count += 1
            ###############

            # Show robot and particles
            self.world.clear_objects()
            self.world.show_particles(self.particles)
            self.world.show_estimated_location(self.particles)
            self.world.show_robot(self.bob)

