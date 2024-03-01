import numpy as np
from maze import Maze, Particle, Robot
import bisect
import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState
import shutil
from std_msgs.msg import Float32MultiArray
from scipy.integrate import ode

import random


def vehicle_dynamics(t, vars, vr, delta):
    curr_x = vars[0]
    curr_y = vars[1]
    curr_theta = vars[2]

    dx = vr * np.cos(curr_theta)
    dy = vr * np.sin(curr_theta)
    dtheta = delta
    return [dx, dy, dtheta]


class particleFilter:
    def __init__(self, bob, world, num_particles, sensor_limit, x_start, y_start):
        self.num_particles = (
            num_particles  # The number of particles for the particle filter
        )
        self.sensor_limit = sensor_limit  # The sensor limit of the sensor
        particles = list()

        ##### TODO:  #####
        # Modify the initial particle distribution to be within the top-right quadrant of the world, and compare the performance with the whole map distribution.
        for i in range(num_particles):
            # (Default) The whole map
            # x = np.random.uniform(0, world.width)
            # y = np.random.uniform(0, world.height)

            ## first quadrant
            x = np.random.uniform(world.width/2, world.width)
            y = np.random.uniform(world.height/2, world.height)

            particles.append(Particle(x=x, y=y, maze=world, sensor_limit=sensor_limit))

        ###############

        self.particles = particles  # Randomly assign particles at the begining
        self.bob = bob  # The estimated robot state
        self.world = world  # The map of the maze
        self.x_start = (
            x_start  # The starting position of the map in the gazebo simulator
        )
        self.y_start = (
            y_start  # The starting position of the map in the gazebo simulator
        )
        self.modelStatePub = rospy.Publisher(
            "/gazebo/set_model_state", ModelState, queue_size=1
        )
        self.controlSub = rospy.Subscriber(
            "/gem/control", Float32MultiArray, self.__controlHandler, queue_size=1
        )
        self.control = []  # A list of control signal from the vehicle
        return

    def __controlHandler(self, data):
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

        rospy.wait_for_service("/gazebo/get_model_state")
        try:
            serviceResponse = rospy.ServiceProxy(
                "/gazebo/get_model_state", GetModelState
            )
            modelState = serviceResponse(model_name="polaris")
        except rospy.ServiceException as exc:
            rospy.loginfo("Service did not process request: " + str(exc))
        return modelState

    def weight_gaussian_kernel(self, x1, x2, std=5000):
        if (
            x1 is None
        ):  # If the robot recieved no sensor measurement, the weights are in uniform distribution.
            return 1.0 / len(self.particles)
        else:
            tmp1 = np.array(x1)
            tmp2 = np.array(x2)
            return np.sum(np.exp(-((tmp2 - tmp1) ** 2) / (2 * std)))

    def updateWeight(self, readings_robot):
        """
        Description:
            Update the weight of each particles according to the sensor reading from the robot
        Input:
            readings_robot: List, contains the distance between robot and wall in [front, right, rear, left] direction.
        """

        ## TODO #####
        # print(readings_robot)
        
        total_weight = 0
        # max_w = 0
        # max_p = None
        # max_sensor_read = None
        for particle in self.particles:
            sensor_reading = particle.read_sensor()
            particle.weight = self.weight_gaussian_kernel(
                readings_robot, sensor_reading, 5000
            )
            # print(particle.weight)
            total_weight += particle.weight
            # if max_w < particle.weight:
            #     max_w = particle.weight
                # max_p = particle
                # max_sensor_read = sensor_reading
        # print(f"{particle.weight}\t{max_sensor_read}\t{readings_robot}")
        # Normalize the weight
        for particle in self.particles:
            particle.weight /= total_weight
        ###############
        # pass

    def resampleParticle(self):
        """
        Description:
            Perform resample to get a new list of particles
        """
        particles_new = list()
        distribution = []
        accu_weight = 0
        ## TODO #####

        ###############
        # for i in range(len(self.particles)):
            # multinomial
            # rand = random.uniform(0, 1)
            # total = 0
            # for particle in self.particles:
            #     total += particle.weight
            #     if total > rand:
            #         particles_new.append(particle)
            #         break

            # Mine resample
            # rand = random.sample(self.particles, 10)
            # x = 0
            # y = 0
            # weight = 0
            # for particle in rand:
            #     x += particle.x * particle.weight
            #     y += particle.y * particle.weight
            #     weight += particle.weight
            # # Particle(x=x, y=y, maze=self.world, sensor_limit=self.sensor_limit)
            # particles_new.append(
            #     Particle(
            #         x=x / weight,
            #         y=y / weight,
            #         maze=self.world,
            #         sensor_limit=self.sensor_limit,
            #     )
            # )

        # bisect
        for particle in self.particles:
            accu_weight += particle.weight            
            distribution.append(accu_weight)
        # print(distribution)
        
        for i in range(len(self.particles)):
            try:
                # selected = self.particles[bisect.bisect_left(distribution, random.uniform(0, 1))]
                selected = self.particles[bisect.bisect_left(distribution, random.uniform(0, distribution[-1]))]
                particles_new.append(Particle(x=selected.x,
                                              y=selected.y,
                                              heading=selected.heading,
                                              maze=self.world, 
                                              weight=selected.weight,
                                              sensor_limit=self.sensor_limit, 
                                              noisy=True))    
            except:
                particles_new.append(Particle(x=np.random.uniform(0, self.world.width),
                                              y=np.random.uniform(0, self.world.height),
                                            maze=self.world, sensor_limit=self.sensor_limit))

        self.particles = particles_new

    def particleMotionModel(self):
        """
        Description:
            Estimate the next state for each particle according to the control input from actual robot
        """
        

        ## TODO #####
        while len(self.control) != 0:
            control = self.control.pop(0)
            for particle in self.particles:
                r = ode(vehicle_dynamics)
                r.set_initial_value([particle.x, particle.y, particle.heading])
                r.set_f_params(control[0], control[1])
                val = r.integrate(r.t + 0.01)
                particle.x = val[0]
                particle.y = val[1]
                particle.heading = val[2]
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
            reading = self.bob.read_sensor()
            self.updateWeight(reading)
            self.resampleParticle()
            
            self.world.clear_objects()
            self.world.show_particles(self.particles)
            self.world.show_robot(self.bob)
            self.world.show_estimated_location(self.particles)
            ###############
