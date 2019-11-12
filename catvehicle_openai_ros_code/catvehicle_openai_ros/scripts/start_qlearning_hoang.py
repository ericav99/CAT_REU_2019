#!/usr/bin/env python

import gym
import numpy
import time
import qlearn
from gym import wrappers
# ROS packages required
import rospy
import rospkg
# import our training environment
from openai_ros.task_envs.catvehicle import catvehicle_wall

#Hoang import
from tensorflow.keras.models import Sequential      # One layer after the other
from tensorflow.keras.layers import Dense, Flatten  # Dense layers are fully connected layers, Flatten layers flatten out multidimensional inputs
from collections import deque            # For storing moves 
import random

# for any function that calls start_qlearning.py, this will be the main
if __name__ == '__main__':

    rospy.init_node('catvehicle_wall_qlearn', anonymous=True, log_level=rospy.WARN)

    # Create the Gym environment
    env = gym.make('CatVehicleWall-v0') # loads environment that was created in turtlebot2_maze.py
    rospy.loginfo("Gym environment done")

    # Set the logging system
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('catvehicle_openai_ros')
    outdir = pkg_path + '/training_results'
    env = wrappers.Monitor(env, outdir, force=True)
    rospy.loginfo("Monitor Wrapper started")

    last_time_steps = numpy.ndarray(0)
    
    '''
    # Loads parameters from the ROS param server
    # Parameters are stored in a yaml file inside the config directory
    # They are loaded at runtime by the launch file
    Alpha = rospy.get_param("/catvehicle/alpha")
    Epsilon = rospy.get_param("/catvehicle/epsilon")
    Gamma = rospy.get_param("/catvehicle/gamma")
    epsilon_discount = rospy.get_param("/catvehicle/epsilon_discount")
    nepisodes = rospy.get_param("/catvehicle/nepisodes")
    nsteps = rospy.get_param("/catvehicle/nsteps")

    running_step = rospy.get_param("/catvehicle/running_step")

    # Initialises the algorithm that we are going to use for learning
    qlearn = qlearn.QLearn(actions=range(env.action_space.n),
                           alpha=Alpha, gamma=Gamma, epsilon=Epsilon)
    initial_epsilon = qlearn.epsilon

    start_time = time.time()
    highest_reward = 0
    '''
    
    #Hoang NN
    # Create network. Input is two consecutive game states, output is Q-values of the possible moves.
    model = Sequential()
    model.add(Dense(20, input_shape=(2,) + env.observation_space.shape, activation='relu'))
    model.add(Flatten())       # Flatten input so as to have no problems with processing
    model.add(Dense(18, activation='relu'))
    model.add(Dense(10, activation='relu'))
    model.add(Dense(env.action_space.n, activation='linear'))    # Same number of outputs as possible actions

    model.compile(loss='mse', optimizer='adam', metrics=['accuracy'])
    
    
    # Parameters
    D = deque()                                # Register where the actions will be stored

    observetime = 1000                         # Number of timesteps we will be acting on the game and observing results
    epsilon = 0.7                              # Probability of doing a random move
    gamma = 0.9                                # Discounted future reward. How much we care about steps further in time
    mb_size = 50                               # Learning minibatch size
    
    
     #FIRST STEP: Knowing what each action does (Observing)

    observation = env.reset()                     # Game begins
    obs = numpy.expand_dims(observation, axis=0)     # (Formatting issues) Making the observation the first element of a batch of inputs
    rospy.logwarn(obs) 
    state = numpy.stack((obs, obs), axis=1)
    rospy.logwarn(state)
    done = False
    for t in range(observetime):
        rospy.logwarn("iteration: "+ str(t))
        if numpy.random.rand() <= epsilon:
            action = numpy.random.randint(0, env.action_space.n, size=1)[0]
        else:
            Q = model.predict(state)          # Q-values predictions
            action = numpy.argmax(Q)             # Move with highest Q-value is the chosen one
        observation_new, reward, done, info = env.step(action)     # See state of the game, reward... after performing the action
        obs_new = numpy.expand_dims(observation_new, axis=0)          # (Formatting issues)
        state_new = numpy.append(numpy.expand_dims(obs_new, axis=0), state[:, :1, :], axis=1)     # Update the input with the new state of the game
        D.append((state, action, reward, state_new, done))         # 'Remember' action and consequence
        state = state_new         # Update state
        if done:
            env.reset()           # Restart game if it's finished
            obs = numpy.expand_dims(observation, axis=0)     # (Formatting issues) Making the observation the first element of a batch of inputs 
            state = numpy.stack((obs, obs), axis=1)
    rospy.logwarn('Observing Finished')
    
    
    # SECOND STEP: Learning from the observations (Experience replay)

    minibatch = random.sample(D, mb_size)                              # Sample some moves

    inputs_shape = (mb_size,) + state.shape[1:]
    inputs = numpy.zeros(inputs_shape)
    targets = numpy.zeros((mb_size, env.action_space.n))

    for i in range(0, mb_size):
        state = minibatch[i][0]
        action = minibatch[i][1]
        reward = minibatch[i][2]
        state_new = minibatch[i][3]
        done = minibatch[i][4]
        
    # Build Bellman equation for the Q function
        inputs[i:i+1] = numpy.expand_dims(state, axis=0)
        targets[i] = model.predict(state)
        Q_sa = model.predict(state_new)
        
        if done:
            targets[i, action] = reward
        else:
            targets[i, action] = reward + gamma * numpy.max(Q_sa)

    # Train network to output the Q function
        model.train_on_batch(inputs, targets)
    rospy.logwarn('Learning Finished')
        
    
    # THIRD STEP: Play!

    #observation = env.reset()
    obs = numpy.expand_dims(observation, axis=0)
    state = numpy.stack((obs, obs), axis=1)
    done = False
    tot_reward = 0.0
    while not done:
        #env.render()                    # Uncomment to see game running
        Q = model.predict(state)        
        action = numpy.argmax(Q)         
        observation, reward, done, info = env.step(action)
        obs = numpy.expand_dims(observation, axis=0)
        state = numpy.append(numpy.expand_dims(obs, axis=0), state[:, :1, :], axis=1)    
        tot_reward += reward
    rospy.logwarn('Game ended! Total reward: {}'.format(reward))
    env.close()
    
'''
    # Starts the main training loop: the one about the episodes to do
    for x in range(nepisodes):
        rospy.logdebug("############### START EPISODE=>" + str(x))

        cumulated_reward = 0
        done = False
        if qlearn.epsilon > 0.05:
            qlearn.epsilon *= epsilon_discount

        # Initialize the environment and get first state of the robot
        observation = env.reset()
        state = ''.join(map(str, observation))

        # Show on screen the actual situation of the robot
        # env.render()
        # for each episode, we test the robot for nsteps
        for i in range(nsteps):
            rospy.logwarn("############### Start Step=>" + str(i))
            # Pick an action based on the current state
            action = qlearn.chooseAction(state)
            rospy.logwarn("Next action is:%d", action)
            # Execute the action in the environment and get feedback
            observation, reward, done, info = env.step(action)

            rospy.logwarn(str(observation) + " " + str(reward))
            cumulated_reward += reward
            if highest_reward < cumulated_reward:
                highest_reward = cumulated_reward

            nextState = ''.join(map(str, observation))

            # Make the algorithm learn based on the results
            rospy.logwarn("# state we were=>" + str(state))
            rospy.logwarn("# action that we took=>" + str(action))
            rospy.logwarn("# reward that action gave=>" + str(reward))
            rospy.logwarn("# episode cumulated_reward=>" + str(cumulated_reward))
            rospy.logwarn("# State in which we will start next step=>" + str(nextState))
            qlearn.learn(state, action, reward, nextState)

            if not (done):
                rospy.logwarn("NOT DONE")
                state = nextState
            else:
                rospy.logwarn("DONE")
                last_time_steps = numpy.append(last_time_steps, [int(i + 1)])
                break
            rospy.logwarn("############### END Step=>" + str(i))
            #raw_input("Next Step...PRESS KEY")
            # rospy.sleep(2.0)
        m, s = divmod(int(time.time() - start_time), 60)
        h, m = divmod(m, 60)
        rospy.logerr(("EP: " + str(x + 1) + " - [alpha: " + str(round(qlearn.alpha, 2)) + " - gamma: " + str(
            round(qlearn.gamma, 2)) + " - epsilon: " + str(round(qlearn.epsilon, 2)) + "] - Reward: " + str(
            cumulated_reward) + "     Time: %d:%02d:%02d" % (h, m, s)))

    rospy.loginfo(("\n|" + str(nepisodes) + "|" + str(qlearn.alpha) + "|" + str(qlearn.gamma) + "|" + str(
        initial_epsilon) + "*" + str(epsilon_discount) + "|" + str(highest_reward) + "| PICTURE |"))

    l = last_time_steps.tolist()
    l.sort()

    # print("Parameters: a="+str)
    rospy.loginfo("Overall score: {:0.2f}".format(last_time_steps.mean()))
    rospy.loginfo("Best 100 score: {:0.2f}".format(reduce(lambda x, y: x + y, l[-100:]) / len(l[-100:])))

    env.close()
'''
