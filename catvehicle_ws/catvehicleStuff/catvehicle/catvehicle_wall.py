import rospy
import numpy
import time
import math
from gym import spaces
from openai_ros.robot_envs import catvehicle_env #from John's catvehicle
from gym.envs.registration import register
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header

# The path is __init__.py of openai_ros, where we import the TurtleBot2MazeEnv directly
timestep_limit_per_episode = 100 # Can be any Value

register(
        id='CatVehicleWall-v0',
        entry_point='openai_ros.task_envs.catvehicle.catvehicle_wall:CATVehicleWallEnv',
        max_episode_steps=timestep_limit_per_episode,
    )

class CATVehicleWallEnv(catvehicle_env.CATVehicleEnv): # change location
    def __init__(self):
        """
        This Task Env is designed for having the TurtleBot2 in some kind of maze.
        It will learn how to move around the maze without crashing.
        """
        
        # Only variable needed to be set here
        number_actions = rospy.get_param('/catvehicle/n_actions') #change the namespace in yaml file
        self.action_space = spaces.Discrete(number_actions) #may look at spaces in gym
        
        
        # We set the reward range, which is not compulsory but here we do it.
        self.reward_range = (-numpy.inf, numpy.inf)
        
        
        #number_observations = rospy.get_param('/turtlebot2/n_observations')
        """
        We set the Observation space for the 6 observations
        cube_observations = [
            round(current_disk_roll_vel, 0),
            round(y_distance, 1),
            round(roll, 1),
            round(pitch, 1),
            round(y_linear_speed,1),
            round(yaw, 1),
        ]
        """
        
        # Actions and Observations
        self.dec_obs = rospy.get_param("/catvehicle/number_decimals_precision_obs", 1)
        self.linear_forward_speed = rospy.get_param('/catvehicle/linear_forward_speed')
        self.linear_turn_speed = rospy.get_param('/catvehicle/linear_turn_speed')
        self.angular_speed = rospy.get_param('/catvehicle/angular_speed')
        self.init_linear_forward_speed = rospy.get_param('/catvehicle/init_linear_forward_speed')
        self.init_linear_turn_speed = rospy.get_param('/catvehicle/init_linear_turn_speed')
        
        
        self.n_observations = rospy.get_param('/catvehicle/n_observations')
        self.min_range = rospy.get_param('/catvehicle/min_range')
        self.max_sensor_value = rospy.get_param('/catvehicle/max_sensor_value')
        self.min_sensor_value = rospy.get_param('/catvehicle/min_sensor_value')
        self.min_angle_value = rospy.get_param('/catvehicle/min_angle_value')
        self.max_angle_value = rospy.get_param('/catvehicle/max_angle_value')
        
        
        #self.dist = 0
        #self.angle = 0
        
        # Here we will add any init functions prior to starting the MyRobotEnv
        super(CATVehicleWallEnv, self).__init__()
        
        # We create two arrays based on the binary values that will be assigned
        # In the discretization method.
        #laser_scan = self._check_laser_scan_ready()
        # laser_scan = self.get_laser_scan() we don't need laser scan, instead we use get_dist from John code
        #rospy.logdebug("laser_scan len===>"+str(len(laser_scan.ranges)))
        self.dist = self.get_dist()
        rospy.logdebug("dist len===>"+str(self.dist))
        
        # Laser data
        #self.laser_scan_frame = laser_scan.header.frame_id

        
        # come back 
        # Number of laser reading jumped
        #self.new_ranges = int(math.ceil(float(len(laser_scan.ranges)) / float(self.n_observations)))
        
        #rospy.logdebug("n_observations===>"+str(self.n_observations))
        #rospy.logdebug("new_ranges, jumping laser readings===>"+str(self.new_ranges))
        
        
        high = numpy.zeros(self.n_observations)
        high[0] = self.max_sensor_value
        high[1] = self.max_angle_value
        
        low = numpy.zeros(self.n_observations)
        low[0] = self.min_sensor_value
        low[1] = self.min_angle_value
        
        # We only use two integers
        self.observation_space = spaces.Box(low, high) # create n dimensional array
        
        
        rospy.logdebug("ACTION SPACES TYPE===>"+str(self.action_space))
        rospy.logdebug("OBSERVATION SPACES TYPE===>"+str(self.observation_space))
        
        # Rewards
        self.forwards_reward = rospy.get_param("/catvehicle/forwards_reward")  # change the namespace to catvehicle2 (yaml)
        self.turn_reward = rospy.get_param("/catvehicle/turn_reward")
        self.end_episode_points = rospy.get_param("/catvehicle/end_episode_points")

        self.cumulated_steps = 0.0

        self.laser_pub = rospy.Publisher('/catvehicle/laser/scan_filtered', LaserScan, queue_size=1)

    def _set_init_pose(self):
        """Sets the Robot in its init pose
        """
        self.move_base( self.init_linear_forward_speed,
                        self.init_linear_turn_speed,
                        epsilon=0.05,
                        update_rate=10,
                        min_laser_distance=-1)

        return True


    def _init_env_variables(self):
        """
        Inits variables needed to be initialised each time we reset at the start
        of an episode.
        :return:
        """
        # For Info Purposes
        self.cumulated_reward = 0.0
        # Set to false Done, because its calculated asyncronously
        self._episode_done = False
        
        # We wait a small ammount of time to start everything because in very fast resets, laser scan values are sluggish
        # and sometimes still have values from the prior position that triguered the done.
        time.sleep(1.0)
        
        # TODO: Add reset of published filtered laser readings
        #laser_scan = self.get_laser_scan() 
        #discretized_ranges = laser_scan.ranges
        #self.publish_filtered_laser_scan(   laser_original_data=laser_scan,
         #                                new_filtered_laser_range=discretized_ranges)
        self.dist = self.get_dist()
        self.angle = self.get_angle()


    def _set_action(self, action): # ok
        """
        This set action will Set the linear and angular speed of the turtlebot2
        based on the action number given.
        :param action: The action integer that set s what movement to do next.
        """
        
        rospy.logdebug("Start Set Action ==>"+str(action))
        # We convert the actions to speed movements to send to the parent class CubeSingleDiskEnv
        if action == 0: #FORWARD
            linear_speed = self.linear_forward_speed
            angular_speed = 0.0
            self.last_action = "FORWARDS"
        elif action == 1: #LEFT
            linear_speed = self.linear_turn_speed
            angular_speed = self.angular_speed
            self.last_action = "TURN_LEFT"
        else: #RIGHT
            linear_speed = self.linear_turn_speed
            angular_speed = -1*self.angular_speed
            self.last_action = "TURN_RIGHT"
        
        # We tell TurtleBot2 the linear and angular speed to set to execute
        self.move_base( linear_speed,
                        angular_speed,
                        epsilon=0.05,
                        update_rate=10,
                        min_laser_distance=self.min_range)
        
        rospy.logdebug("END Set Action ==>"+str(action)+", NAME="+str(self.last_action))

    def _get_obs(self):
        """
        Here we define what sensor data defines our robots observations
        To know which Variables we have acces to, we need to read the
        TurtleBot2Env API DOCS
        :return:
        """
        rospy.logdebug("Start Get Observation ==>")
        # We get the laser scan data
        #laser_scan = self.get_laser_scan()
        self.dist = self.get_dist()
        self.angle = self.get_angle()
        
        if self.dist < self.min_range:
            self._episode_done = True
        else:
            self._episode_done = False
        
        rospy.logdebug("BEFORE DISCRET _episode_done==>"+str(self._episode_done))
        
        #discretized_observations = self.discretize_observation( laser_scan,
         #                                                       self.new_ranges
          #                                                      )

        #rospy.logdebug("Observations==>"+str(discretized_observations))
        rospy.logdebug("Observation >>>"+str(self.dist)+ ", "+str(self.angle))
        rospy.logdebug("AFTER DISCRET_episode_done==>"+str(self._episode_done))
        rospy.logdebug("END Get Observation ==>")
        return [self.dist, self.angle]
        

    def _is_done(self, observations):
        
        if self._episode_done:
            rospy.logdebug("Catvehicle is Too Close to wall==>"+str(self._episode_done))
        else:
            rospy.logerr("Catvehicle is Ok ==>")

        return self._episode_done

    def _compute_reward(self, observations, done):

        if not done:
            if self.last_action == "FORWARDS":
                reward = self.forwards_reward
            else:
                reward = self.turn_reward
        else:
            reward = -1*self.end_episode_points


        rospy.logdebug("reward=" + str(reward))
        self.cumulated_reward += reward
        rospy.logdebug("Cumulated_reward=" + str(self.cumulated_reward))
        self.cumulated_steps += 1
        rospy.logdebug("Cumulated_steps=" + str(self.cumulated_steps))
        
        return reward
        
    """ # WORK IN PROGRESS: Create new reward function that defines reward as negative distance to behind wall
    def _compute_reward(self, observations, done):
       if not done:
           reward = 

    """

    # Internal TaskEnv Methods; These functions are some data processing
    '''
    def discretize_observation(self,data,new_ranges):
        """
        Discards all the laser readings that are not multiple in index of new_ranges
        value.
        """
        self._episode_done = False
        
        discretized_ranges = []
        filtered_range = []
        #mod = len(data.ranges)/new_ranges
        mod = new_ranges
        
        max_laser_value = data.range_max
        min_laser_value = data.range_min
        
        rospy.logdebug("data=" + str(data))
        rospy.logwarn("mod=" + str(mod))
        
        for i, item in enumerate(data.ranges):
            if (i%mod==0):
                if item == float ('Inf') or numpy.isinf(item):
                    #discretized_ranges.append(self.max_laser_value)
                    discretized_ranges.append(round(max_laser_value,self.dec_obs))
                elif numpy.isnan(item):
                    #discretized_ranges.append(self.min_laser_value)
                    discretized_ranges.append(round(min_laser_value,self.dec_obs))
                else:
                    #discretized_ranges.append(int(item))
                    discretized_ranges.append(round(item,self.dec_obs))
                    
                if (self.min_range > item > 0):
                    rospy.logerr("done Validation >>> item=" + str(item)+"< "+str(self.min_range))
                    self._episode_done = True
                else:
                    rospy.logwarn("NOT done Validation >>> item=" + str(item)+"< "+str(self.min_range))
                # We add last value appended
                filtered_range.append(discretized_ranges[-1])
            else:
                # We add value zero
                filtered_range.append(0.1)
                    
        rospy.logdebug("Size of observations, discretized_ranges==>"+str(len(discretized_ranges)))
        
        
        self.publish_filtered_laser_scan(   laser_original_data=data,
                                            new_filtered_laser_range=discretized_ranges)
        
        return discretized_ranges
        
    '''
    '''
    def publish_filtered_laser_scan(self, laser_original_data, new_filtered_laser_range):
        
        rospy.logdebug("new_filtered_laser_range==>"+str(new_filtered_laser_range))
        
        laser_filtered_object = LaserScan()

        h = Header()
        h.stamp = rospy.Time.now() # Note you need to call rospy.init_node() before this will work
        h.frame_id = laser_original_data.header.frame_id
        
        laser_filtered_object.header = h
        laser_filtered_object.angle_min = laser_original_data.angle_min
        laser_filtered_object.angle_max = laser_original_data.angle_max
        
        new_angle_incr = abs(laser_original_data.angle_max - laser_original_data.angle_min) / len(new_filtered_laser_range)
        
        #laser_filtered_object.angle_increment = laser_original_data.angle_increment
        laser_filtered_object.angle_increment = new_angle_incr
        laser_filtered_object.time_increment = laser_original_data.time_increment
        laser_filtered_object.scan_time = laser_original_data.scan_time
        laser_filtered_object.range_min = laser_original_data.range_min
        laser_filtered_object.range_max = laser_original_data.range_max
        
        laser_filtered_object.ranges = []
        laser_filtered_object.intensities = []
        for item in new_filtered_laser_range:
            if item == 0.0:
                laser_distance = 0.1
            else:
                laser_distance = item
            laser_filtered_object.ranges.append(laser_distance)
            laser_filtered_object.intensities.append(item)
        
        
        self.laser_filtered_pub.publish(laser_filtered_object)
    '''

