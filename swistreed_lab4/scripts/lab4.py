#!/usr/bin/env python
import rospy
import math
import numpy as np
import copy as cm
import tf
from copy import deepcopy
from nav_msgs.msg import Odometry
from kobuki_msgs.msg import BumperEvent
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import JointState

BUMPERS = ["LEFT", "CENTER", "RIGHT"]
STATUS = ["STOPPED", "STRAIGHT", "ROTATING", "ARC"]

# TURTLEBOT SPECS

MAX_LIN_VEL = .7
MAX_ROT_VEL = 3.14

BASE_DIAMETER = .23
WHEEL_RADIUS = .035
WHEEL_WIDTH = .021
TICKS_PER_REVOLUTION = 52
PULSES_PER_REVOLUTION = 13
TICK_TO_METER = .000085292090497737556558
METER_TO_TICK = 11724.41658029856624751591
MILLIMITER_TO_TICK = 11.72441658029856624751591
TICK_TO_RADIAN = .002436916871363930187454
DEG_TO_RAD = np.pi/180

# THRESHOLDS CHANGE THOSE AS YOU SEEM FIT
# x, y distance form target to consider correct
XY_THRES = .05
# same for rotation angle
ROT_THRES = 5
# same for arc movement
QUAT_THRES = .01


class RobotStatus:
    # helper class with possible robot states
    # not necessary for implementation
    STOPPED, STRAIGHT, ROTATING, ARC = range(4)

    def __init__(self):
        pass
    
class Position:
    """ Helper class that describes the positon of the turtlebot:
            x,y and theta 
    """
    # Position
    x = 0.0
    y = 0.0
    theta = 0.0
    
    # Orientation
    Clockwise = False


def normalize_angle(angle):
    """REDUCE ANGLES TO -pi pi"""
    angle %= np.pi*2
    angle = (angle + np.pi*2) % np.pi*2
    if angle > np.pi:
        angle -= np.pi*2
    return angle


class Lab4Solution:

    def vel_from_wheels(self, phi_right, phi_left, time=1):
        """takes in right and left wheel velocities and translates the to twist messages"""
        self.duration=time	
        
        # Solve for the desired linear velocity and yaw rate
        vel = (WHEEL_RADIUS/2)*(phi_right + phi_left)
        omega = WHEEL_RADIUS*(phi_right - phi_left)/(BASE_DIAMETER)
        
#        print 'v:{}, omega:{}'.format(vel,omega)
        self.twist_msg = Twist(Vector3(vel,0,0),Vector3(0,0,omega))
        
    def copy_odom(self, message=-1):
        """
            copy an odometry message (deepcopy did not work for me
            if you figure it out let me know :P).
            If no arguments are given it will the message the solution
            uses
        """
        if message == -1:
            message = self.odom
        ret = Odometry()
        ret.pose.pose.position.x = message.pose.pose.position.x
        ret.pose.pose.position.y = message.pose.pose.position.y
        ret.pose.pose.position.z = message.pose.pose.position.z
        ret.pose.pose.orientation.x = message.pose.pose.orientation.x
        ret.pose.pose.orientation.y = message.pose.pose.orientation.y
        ret.pose.pose.orientation.z = message.pose.pose.orientation.z
        ret.pose.pose.orientation.w = message.pose.pose.orientation.w
        ret.twist.twist.linear.x = message.twist.twist.linear.x
        ret.twist.twist.linear.y = message.twist.twist.linear.y
        ret.twist.twist.linear.z = message.twist.twist.linear.z
        ret.twist.twist.angular.x = message.twist.twist.angular.x
        ret.twist.twist.angular.y = message.twist.twist.angular.y
        ret.twist.twist.angular.z = message.twist.twist.angular.z
        return ret

    def euclidean_distance(self, x1, x2, y1, y2):
        """ Calculate euclidean distance between two points"""
        dist = np.sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2))
        # print dis
        return dist

    def rotation_distance(self, q1, q2):
        """ Calculate the difference in yaw between two quaternions. For the 
            turtlebot, the z and w are the only two quaternions.

            Inputs:
                q1 - self.odom.pose.pose.orientation.z
                q2 - self.odom.pose.pose.orientation.w
            
            Ouput:
                yaw - yaw of the turtlebot
        """
        # This should convert the quaternions to roll/pitch/yaw. We only have 
        #   quaternions z and w so the output is only yaw.
        rot = tf.transformations.euler_from_quaternion([0, 0, q1, q2])
        yaw = rot[2]
        return yaw

    def bumper_callback(self, bumper_msg):
        """ callback to handle bumper messages"""
        # Determine if the bumper is pressed
        if ( bumper_msg.state == BumperEvent.RELEASED ) :
            self.bumper_pressed=-1
        else:
            self.bumper_pressed=1 # Pressed

    def odom_callback(self, odom_msg):
        """ callback to handle odometry messages"""
#        # Get the odometry message
#        ret = self.copy_odom(odom_msg)
        
        # Set previous position
        self.prev_position = self.position
        
        # We only care about the x,y position and the z and w orientation
        self.odom.pose.pose.position.x = odom_msg.pose.pose.position.x
        self.odom.pose.pose.position.y = odom_msg.pose.pose.position.y
        self.odom.pose.pose.orientation.z = odom_msg.pose.pose.orientation.z
        self.odom.pose.pose.orientation.w = odom_msg.pose.pose.orientation.w
        
        # convert those into x/y/theta positions
        self.position.x = self.odom.pose.pose.position.x
        self.position.y = self.odom.pose.pose.position.y
        self.position.theta = normalize_angle(self.rotation_distance(self.odom.pose.pose.orientation.z,self.odom.pose.pose.orientation.w))
        
        if (self.position.theta > self.prev_position.theta):
            self.position.Clockwise = False
        elif (self.position.theta < self.prev_position.theta):
            self.position.Clockwise = True

    def encoder_callback(self, encoder_msg):
        """ Callback to handle joint states that could read individual wheel 
            velocities and positions.
        """
        # print encoder_msg
        self.right_wheel_vel = encoder_msg.something
        self.right_wheel_pos = encoder_msg.somethingelse
        self.left_wheel_vel = encoder_msg.somethingdifferent
        self.left_wheel_pos = encoder_msg.somethingtotallydifferent

    def clear_list(self):
        """
        You may want to implement a function that cancels given action(s)
        """
        self.command_list = []

    def cancel_goals(self):
        """
        You may want to implement a function that stops movement
        """
        
        rospy.logerr("Cancelling All Movement")
        self.status = RobotStatus.STOPPED
        self.goal_rotation = -1
        self.goal_distance = -1
        self.twist_msg = Twist()
        self.starting_position =self.position
        # self.command_list = []

    def drive_straight(self, speed, distance=0, time=.1):
        """ The function drives straight for a certain time or distance. If
            a distance is not inputed or is inputted as 0, the turtlebot will
            drive straight for time seconds. Otherwise it will travel for the 
            time it takes to go distance meters.
            
            Inputs: 
                speed - desired speed in m/s
                distance - desired distance in meters
                time - desired time in seconds
        """
        # Set the goal rotation, starting odom, and status on the first 
        #  time through the loop.
        if self.first_time:
            self.goal_distance = distance    
            self.status = RobotStatus.STRAIGHT
            self.first_time = False
        
        # Calculate the rotational speed using the formula: v = r*phi_dot
        phi = speed/WHEEL_RADIUS
        
        # If there is a inputed distance, then we need to calculate the time 
        #  needed to travel the desired distance using the formula: v=d/t
        if (distance != 0):
            time = distance/speed
        
        self.vel_from_wheels(phi,phi,time)

    def rotate(self, angle, time=0, Clockwise=False):
        """ Rotates the turtlebot in place. 
            
            Inputs:
                angle - desired angle (in radians)
                time - desired time to complete the rotation (in seconds)
                Clockwise - determines the direction of the rotation where:
                            True = Clockwise (left)
                            False = Counter-Clockwise (right)
                            
        """
        # Set the goal rotation, starting odom, and status on the first time
        #  through the loop.
        if self.first_time:
            self.goal_rotation = angle
            self.status = RobotStatus.ROTATING
            self.first_time = False
        
        # If time is not set assume a omega of 1 rad/sec
        if (time ==0):
            omega = 1
        # Otherwise calculate omega (in rad/sec) using the formula:
        #   omega = angle/time
        else:
            omega = angle/time/2
        
        # Calculate omega using the formula: 
        #   omega = r*(phi)/(2l)
        phi = omega*BASE_DIAMETER/WHEEL_RADIUS
        
        if (Clockwise):
            self.vel_from_wheels(-phi,phi,abs(time))
        else:
            self.vel_from_wheels(phi,-phi,abs(time))

    def drive_arc(self, radius, speed=0.1, angle=0):
        """ This function drives the turtlebot in an arc around the ICR
            
            Inputs:
                radius - desired distance to the ICR
                speed - desired speed (in meters/sec)
                angle - desired total angular distance traveled
        """
        # Set the goal rotation, starting odom, and status on the first time
        #  through the loop.
        if self.first_time:
            self.goal_arc = angle
            self.status = RobotStatus.ARC
#            self.starting_position =self.position
            self.first_time = False
#            self.starting_odom = self.odom
        
        ## Solve for phi_dot of each wheel using the equations:
        #   R = l((phi_dot_r + phi_dot_l)/(phi_dot_r - phi_dot_l))
        #   V = (r/2)(phi_dot_r + phi_dot_l)
        r = WHEEL_RADIUS
        l = BASE_DIAMETER/2
        
        # A = np.matrix([[(l-radius), (l+radius)],[1, 1]])
        # B = np.matrix([[0],[2*speed/r]])
        # Solve [X]=[A]-1[B]
        phi_dot_right =(speed/r)*(l+radius)/radius
        phi_dot_left =(speed/r)*(radius-l)/radius
        
        omega = r*(phi_dot_right - phi_dot_left)/(2*l)
        # Solve for the desired time
        time = abs(angle/omega)
        
        self.vel_from_wheels(phi_dot_right, phi_dot_left, time)
        
    def set_trajectory(self):
        """ This code sets the trajectory defined in the lab handout:
                - Drive forward 60cm
                - Turn right 90 degrees
                - Drive -180 degrees arc with a radius of 15cm
                - Turn left 135 degrees
                - Drive forward 42cm
        """
        
        self.command_list.append(['Straight',0.5, 0.6])
#        self.command_list.append(self.rotate(90*DEG_TO_RAD, 1, False))
#        self.command_list.append(self.drive_arc(0.15, 0.5, -180*DEG_TO_RAD))
#        self.command_list.append(self.rotate(135*DEG_TO_RAD, 1, True))
#        self.command_list.append(self.drive_straight(0.5, 0.42))
        
    def execute_trajectory(self):
        """ This code impliments the trajectory defined in the lab handout as 
            follows:
                - Drive forward 60cm
                - Turn right 90 degrees
                - Drive -180 degrees arc with a radius of 15cm
                - Turn left 135 degrees
                - Drive forward 42cm
        """   
        if len(self.command_list)>0:
#            print self.command_list[0]
            # Run the command that is first in the list
            if (self.command_list[0][0]== 'Straight'):
                self.drive_straight(self.command_list[0][1], self.command_list[0][2])
                self.cmd_vel.publish(self.twist_msg)
            # Check to see if the condition has been met
            if (not self.process_position()):  
                # If it has, remove it from the list of commands
                print "remove"
                self.command_list.pop(0)
                self.cancel_goals()
        else:
            print "empty"
            self.cancel_goals()
            self.trig = False
            
    def process_position(self):
        """ Check the progress of your actions.
        
            Outputs:
                Go_NoGo - Boolean that is true when the action is still in 
                            progress and false when the action is complete
        """
        Go_NoGo = True
        # Both the Rotating and Arc commands should check to see if we have
        #  reached the desired angle.
        if (self.status == RobotStatus.ROTATING) or (self.status == RobotStatus.ARC):
            ##### NEED TO CHECK WHAT DOMAIN THE ANGLE IS IN ####
            # Check to see if the desired angle has been met
            goal_angle = self.goal_rotation+self.process_position.theta
            if (goal_angle> self.position.theta) and (self.position.Clockwise):
                Go_NoGo = False
            elif (goal_angle < self.position.theta) and not (self.position.Clockwise):
                Go_NoGo = False
                
        elif (self.status == RobotStatus.STRAIGHT):
            # Check to see if the desired distance has been met
            dist_traveled = self.euclidean_distance(self.starting_position.x, self.position.x,
                                                    self.starting_position.y, self.position.y)
            print dist_traveled
            if (dist_traveled > self.goal_distance):
                Go_NoGo = False
        
        return Go_NoGo

    def shutdown(self):
        """ publish an empty twist message to stop the turtlebot"""
        rospy.loginfo("Stopping Turtlebot")
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)

    def __init__(self):
        """ STUFF YOU MAY WANT TO USE"""
        rospy.init_node('lab4')
        rospy.on_shutdown(self.shutdown)
        # odometry messages
        self.odom = Odometry()
        self.starting_odom = Odometry()
        # bumper event message
        self.bumper_msg = BumperEvent()
        # twist message
        self.twist_msg = Twist()
        # Positions (x, y, theta)
        self.position = Position()
        self.prev_position = Position()
        self.starting_position = Position()

        # transformer
        self.current_tf = tf.TransformerROS()
        self.current_transform = []
        self.current_rotation = []

        # transform listener
        self.odom_listener = tf.TransformListener()
        # transform broadcaster
        self.odom_tf_broadcaster = tf.TransformBroadcaster()

        # placeholder variables
        self.right_wheel_vel = 0.0
        self.right_wheel_pos = 0.0
        self.left_wheel_vel = 0.0
        self.left_wheel_pos = 0.0

        # things you might want to consider using
        self.bumper_pressed = -1
        self.goal_distance = -1
        self.goal_rotation = -1
        self.goal_arc = []
        self.command_list = []
        self.status = RobotStatus.STOPPED  # -1 to stop, 1 to move
        
        # loop rate
        r = rospy.Rate(50)

        # SUBSCRIBERS AND PUBLISHERS
        # you might need to change topics depending on
        # whether you use simulation or real robots
        # subscribe to bumper events
        self.bumper_sub = rospy.Subscriber("/mobile_base/events/bumper", BumperEvent, self.bumper_callback)
        
        # subscribe to odometry messages
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        
        # subscribe to encoder messages
#        self.encoder_sub = rospy.Subscriber("/", JointState, self.encoder_callback)
        
        # publisher for twist messages
        self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/teleop', Twist,queue_size=100)

        start_time = rospy.Time.now()
        
        # trigger to execute trajectory
        trig = True
        self.first_time = True
        self.final_test_start =True
        
        # Boolean for testing part 3 of the lab
        testing_pt3 = False
        
        # Set the desired trajectory defined by the lab handout
        self.set_trajectory()
        
        now = rospy.get_rostime().to_sec()
        
        # Start without any goals
        self.cancel_goals()
        while not rospy.is_shutdown():
            # IN THIS LOOP ROS SENDS AND RECEIVES
            if rospy.Time.now().to_sec() - start_time.to_sec() > 1:
                start_time = rospy.Time.now()
                # LOGGING MESSAGES FOR DEBUGGING
                rospy.logwarn(
                    "Starting Position %.2f %.2f %.2f" % (self.starting_odom.pose.pose.position.x,
                                                          self.starting_odom.pose.pose.position.y,
                                                          self.starting_odom.pose.pose.position.z))
                rospy.logwarn(
                    "Current Position %.2f %.2f %.2f" % (self.odom.pose.pose.position.x,
                                                         self.odom.pose.pose.position.y,
                                                         self.odom.pose.pose.position.z))
                # rospy.logwarn("Linear Velocity %f %f %f" % (self.odom.twist.twist.linear.x,
                #                                             self.odom.twist.twist.linear.y,
                #                                             self.odom.twist.twist.linear.z))
                # rospy.logwarn("Angular Velocity %f %f %f" % (self.odom.twist.twist.angular.x,
                #                                              self.odom.twist.twist.angular.y,
                #                                              self.odom.twist.twist.angular.z))
#                rospy.logwarn("Wheels Velocity R:%.2f L:%.2f" % (self.right_wheel_vel, self.left_wheel_vel))
#                rospy.logwarn("Wheels Position R:%.2f L:%.2f" % (self.right_wheel_pos, self.left_wheel_pos))
#                rospy.logwarn("Bumper Status %d " % self.bumper_pressed)
#                rospy.logwarn("Robot Status %d " % self.status)
#                rospy.logwarn("%d Commands to be executed" % len(self.command_list))
                
            # When your trigger is activated execute trajectory
            if trig:
                if (rospy.Time.now().to_sec()- now)>1:
                    self.execute_trajectory()
                    print 'current position {},{} starting position {},{}'.format(self.position.x,self.position.y, self.starting_position.x, self.starting_position.y)
            
            # Test to see if part 3 (the vel_from_wheels function) works
            if testing_pt3:
                self.rotate(-95*DEG_TO_RAD, 2)
                if(rospy.get_rostime().to_sec()-now< self.duration):                  
                    self.cmd_vel.publish(self.twist_msg)
                    print 'Theta: {}'.format(self.position.theta/DEG_TO_RAD)
                else:
                    testing_pt3 = False
            r.sleep()
        return


if __name__ == '__main__':
    try:
        Lab4Solution()
    except rospy.ROSInterruptException:
        pass
