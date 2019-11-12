import rospy
import numpy
import time
import math
from gym import spaces
from openai_ros.robot_envs import catvehicle_env
from gym.envs.registration import register

# The path is __init__.py of openai_ros, where we import the CATVehicleWall directly
timestep_limit_per_episode = 1000 # Can be any Value

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
        
        # 1st and 2nd index represent x and y coordinate. 3rd index is 0 if reward avail, 1 if not avail.
        self.path_array = [[10, 0, 0], [15, 0, 0], [25, 0, 0], [30, 0, 0]]
        
        # Only variable needed to be set here
        number_actions = rospy.get_param('/catvehicle/n_actions') #change the namespace in yaml file
        self.action_space = spaces.Discrete(number_actions) #may look at spaces in gym
        
        
        # We set the reward range, which is not compulsory but here we do it.
        self.reward_range = (-numpy.inf, numpy.inf)
        
        # Actions and Observations
        self.dec_obs = rospy.get_param("/catvehicle/number_decimals_precision_obs", 1)
        self.linear_forward_speed = rospy.get_param('/catvehicle/linear_forward_speed')
        self.linear_turn_speed = rospy.get_param('/catvehicle/linear_turn_speed')
        self.angular_speed = rospy.get_param('/catvehicle/angular_speed')
        self.init_linear_forward_speed = rospy.get_param('/catvehicle/init_linear_forward_speed')
        self.init_linear_turn_speed = rospy.get_param('/catvehicle/init_linear_turn_speed')
        self.path_epsilon = rospy.get_param('/catvehicle/path_epsilon')
        self.path_reward = rospy.get_param('/catvehicle/get_reward')
        
        
        self.n_observations = rospy.get_param('/catvehicle/n_observations')
        self.min_range = rospy.get_param('/catvehicle/min_range')
        self.max_sensor_value = rospy.get_param('/catvehicle/max_sensor_value')
        self.min_sensor_value = rospy.get_param('/catvehicle/min_sensor_value')
        self.min_angle_value = rospy.get_param('/catvehicle/min_angle_value')
        self.max_angle_value = rospy.get_param('/catvehicle/max_angle_value')
        
        # Here we will add any init functions prior to starting the MyRobotEnv
        super(CATVehicleWallEnv, self).__init__()
        
        # Initialize values
        self.dist = self.get_dist()
        self.angle = self.get_angle()
        
        rospy.logdebug("dist len===>"+str(self.dist))
        rospy.logdebug("angle ===>"+str(self.angle))
        
        
        high = numpy.zeros(self.n_observations)
        high[0] = self.max_sensor_value
        high[1] = self.max_angle_value
        
        low = numpy.zeros(self.n_observations)
        low[0] = self.min_sensor_value
        low[1] = self.min_angle_value
        
        self.observation_space = spaces.Box(low, high) # create n dimensional array
        
        
        rospy.logdebug("ACTION SPACES TYPE===>"+str(self.action_space))
        rospy.logdebug("OBSERVATION SPACES TYPE===>"+str(self.observation_space))
        
        # Rewards
        self.forwards_reward = rospy.get_param("/catvehicle/forwards_reward")
        self.turn_reward = rospy.get_param("/catvehicle/turn_reward")
        self.end_episode_points = rospy.get_param("/catvehicle/end_episode_points")

        self.cumulated_steps = 0.0
        

    def _set_init_pose(self):
        """Sets the Robot in its init pose
        """
        self.move_car( self.init_linear_forward_speed,
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
        time.sleep(2.0)
        
        self.dist = self.get_dist()
        self.angle = self.get_angle()


    def _set_action(self, action): # ok
        """
        This set action will Set the linear and angular speed of the turtlebot2
        based on the action number given.
        :param action: The action integer that set s what movement to do next.
        """
        
        rospy.logdebug("Start Set Action ==>"+str(action))
        # We convert the actions to speed movements to send to the parent class CATVehicleEnv
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
        self.move_car( linear_speed,
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
        self.dist = self.get_dist()
        self.angle = self.get_angle()
        
        self._episode_done = self.has_crashed(self.min_range)
        
        rospy.logdebug("BEFORE DISCRET _episode_done==>"+str(self._episode_done))
        
        rospy.logdebug("Observation >>>"+str(self.dist)+ ", "+str(self.angle))
        rospy.logdebug("AFTER DISCRET_episode_done==>"+str(self._episode_done))
        rospy.logdebug("END Get Observation ==>")
        
        if (self.angle.data < self.min_angle_value) or (self.angle.data > self.max_angle_value):
            return [self.dist.data, 0]
        else:
            return [self.dist.data, self.angle.data]
        

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
            
        x = self.odom.pose.pose.position.x
        y = self.odom.pose.pose.position.y
        
        for i in range(len(self.path_array)):
            reward_x = self.path_array[i][0]
            reward_y = self.path_array[i][1]
            
            if (self.euclid_distance(reward_x, reward_y, x, y) < self.path_epsilon) and self.path_array[i][2] != 1:
                self.path_array[i][2] = 1
                reward += self.path_reward

        rospy.logdebug("reward=" + str(reward))
        self.cumulated_reward += reward
        rospy.logdebug("Cumulated_reward=" + str(self.cumulated_reward))
        self.cumulated_steps += 1
        rospy.logdebug("Cumulated_steps=" + str(self.cumulated_steps))
        
        return reward
''' # Alternative reward function and helper
    def _compute_reward(self, observations, done): # Make reward process iterative
        e_dist = self.euclid_distance(self.odom.pose.pose.position.x, self.odom.pose.pose.position.y,16.5,0)
	rospy.logdebug("reward=" + str(reward))
	    reward = e_dist - self.cumulated_reward # Reward is change in euclidean distance
        self.cumulated_reward += reward # There is no cumulated reward; we only consider euclidean distance to desired point
        rospy.logdebug("Cumulated_reward=" + str(self.cumulated_reward))
        self.cumulated_steps += 1
        rospy.logdebug("Cumulated_steps=" + str(self.cumulated_steps))    
        
        return -reward
'''
    def euclid_distance(self, x1, y1, x2, y2):
        return math.sqrt((x1 - x2)**2 + (y1 - y2)**2) 
