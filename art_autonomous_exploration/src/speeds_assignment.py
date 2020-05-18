#!/usr/bin/env python

import rospy
import math
import time

from sensor_msgs.msg import Range
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

from sonar_data_aggregator import SonarDataAggregator
from laser_data_aggregator import LaserDataAggregator
from navigation import Navigation

# Class for assigning the robot speeds
class RobotController:

    # Constructor
    def __init__(self):

      # Debugging purposes
      self.print_velocities = rospy.get_param('print_velocities')

      # Where and when should you use this?
      self.stop_robot = False

      # Create the needed objects
      self.sonar_aggregation = SonarDataAggregator()
      self.laser_aggregation = LaserDataAggregator()
      self.navigation  = Navigation()

      self.linear_velocity  = 0
      self.angular_velocity = 0

      # Check if the robot moves with target or just wanders
      self.move_with_target = rospy.get_param("calculate_target")

      # The timer produces events for sending the speeds every 110 ms
      rospy.Timer(rospy.Duration(0.11), self.publishSpeeds)
      self.velocity_publisher = rospy.Publisher(\
              rospy.get_param('speeds_pub_topic'), Twist,\
              queue_size = 10)

    # This function publishes the speeds and moves the robot
    def publishSpeeds(self, event):

      # Produce speeds
      self.produceSpeeds()

      # Create the commands message
      twist = Twist()
      twist.linear.x = self.linear_velocity
      twist.linear.y = 0
      twist.linear.z = 0
      twist.angular.x = 0
      twist.angular.y = 0
      twist.angular.z = self.angular_velocity

      # Send the command
      self.velocity_publisher.publish(twist)

      # Print the speeds for debuggind purposes
      if self.print_velocities == True:
        print "[L,R] = [" + str(twist.linear.x) + " , " + \
            str(twist.angular.z) + "]"

    # Produces speeds from the laser
    def produceSpeedsLaser(self):
      scan = self.laser_aggregation.laser_scan

      ############################### NOTE QUESTION ############################
      # Check what laser_scan contains and create linear and angular speeds
      # for obstacle avoidance
      MAX_SPEED = 0.3
      MAX_DISTANCE = 5
      TWIST_SENSITIVITY = 0.7 # ratio
      EQUILIBRIUM = 0.06

      span = len(scan)

      front_obst = 0
      for i in scan[4*span/10 : 6*span/10]:
          front_obst += i
      front_obst /= (6*span/10 - 4*span/10) # avg
      #front_obst = MAX_SPEED * front_obst / MAX_DISTANCE  # normalization


      left_obst = 0
      for i in scan[6*span/10 : 7*span/10]:
          left_obst += i
      left_obst /= span/10
      left_obst = MAX_SPEED * left_obst / MAX_DISTANCE  # normalization

      right_obst = 0
      for i in scan[3*span/10 : 4*span/10]:
          right_obst += i
      right_obst /= span/10
      right_obst = MAX_SPEED * right_obst / MAX_DISTANCE  # normalization

      r_angular = MAX_SPEED - left_obst
      l_angular = MAX_SPEED - right_obst

      angular = (l_angular - r_angular) * TWIST_SENSITIVITY  # insert bias -> increase twisting sensitivity

      # print r_angular , l_angular
      #if front_obst < 1 :
      linear = front_obst ** 1.2
      linear = MAX_SPEED * linear / MAX_DISTANCE ** 1.2 # normalization
      #else:
      #linear = front_obst
      #linear = MAX_SPEED * linear / MAX_DISTANCE
      #print front_obst, linear

      if linear > MAX_SPEED :
          linear = MAX_SPEED
      if linear < -MAX_SPEED :
          linear = -MAX_SPEED


      if angular > MAX_SPEED :
          angular = MAX_SPEED
      if angular < -MAX_SPEED :
          angular = -MAX_SPEED

      # APPLY FORCE TO ESCAPE LOCAL MINIMUM
      if linear <= EQUILIBRIUM:
          angular = MAX_SPEED

      # RANDOM VALUE GENERATOR IN ORDER TO
      # GENERATE RANDOM TWISTS AND THUS
      # ESCAPE THE PREDIFINED CYLE PATH
      #rand_value = time.clock() % 0.3
      #if rand_value > 0.28:
          #print "RANDOM = ", rand_value
          #angular = MAX_SPEED

      #print round(linear, 2),  round(l_angular,2) , round(r_angular,2), round(angular, 2)
      #self.print_velocities = True

      ##########################################################################
      return [linear, angular]

    # Combines the speeds into one output using a motor schema approach
    def produceSpeeds(self):

      # Produce target if not existent
      if self.move_with_target == True and \
              self.navigation.target_exists == False:

        # Create the commands message
        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0

        # Send the command
        self.velocity_publisher.publish(twist)
        self.navigation.selectTarget()

      # Get the submodule's speeds
      [l_laser, a_laser] = self.produceSpeedsLaser()

      # You must fill these
      self.linear_velocity  = 0
      self.angular_velocity = 0

      if self.move_with_target == True:
        [l_goal, a_goal] = self.navigation.velocitiesToNextSubtarget()
        ############################### NOTE QUESTION ############################
        # You must combine the two sets of speeds. You can use motor schema,
        # subsumption of whatever suits your better.

        # MOTOR SCHEMA IMPLEMENTATION
        PI = 3.14

        scan = self.laser_aggregation.laser_scan
        resolution = (3*PI/2) / len(scan) # 3*pi/2 = 270
        lin_obs = 0
        ang_obs = 0

        for i,j in enumerate(scan):
            if resolution * i >= 3 * PI / 4:
                theta = resolution * i - 3 * PI / 4
            elif resolution * i < 3 * PI / 4:
                theta = resolution * i + 5 * PI / 4

            lin_obs += math.cos(theta) / j ** 2
            ang_obs += math.sin(theta) / j ** 2

            print j

        #lin_obs *= -1
        #ang_obs *= -1

        # @JOHN : Motor schema produces extremely high speeds on lin_obs / ang_obs

        print lin_obs, ang_obs
        self.linear_velocity = 0.9 * l_goal + 0.1 * l_laser
        self.angular_velocity = 0.9 * a_goal + 0.1 * a_laser
        ##########################################################################
      else:
        ############################### NOTE QUESTION ############################
        # Implement obstacle avoidance here using the laser speeds.
        # Hint: Subtract them from something constant
        self.linear_velocity = l_laser
        self.angular_velocity = a_laser
        pass

        ##########################################################################

    # Assistive functions
    def stopRobot(self):
      self.stop_robot = True

    def resumeRobot(self):
      self.stop_robot = False
