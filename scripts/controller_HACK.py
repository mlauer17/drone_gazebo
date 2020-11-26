#!/usr/bin/env python2
from __future__ import print_function
import sys
import time

import rospy
import math as m
import numpy as np

from nav_msgs.msg import Odometry

from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Wrench

import tf
from tf.transformations import *

g = 9.8
mass = 1.5

# Max linear velocities
max_vert_vel = 5
max_hori_vel = 10

# Max angular rates
max_roll_rate = 1
max_pitch_rate = 1
max_yaw_rate = 0.5

# Max roll/pitch
max_roll = 25 * m.pi/180
max_pitch = 25 * m.pi/180

class PID():

    def __init__(self, kp, ki, kd, sat_min, sat_max, derivative_tau):

        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.sat_min = sat_min
        self.sat_max = sat_max

        self.derivative_tau = derivative_tau

        self.time_prev = None

        self.integral = 0
        self.error_prev = 0
        self.derivative_error_prev = 0

    def update(self, setpoint, measure, dt):

        error = setpoint - measure

        integral_error = self.ki * error * dt

        derivative_error = (self.kd * (error - self.error_prev) + self.derivative_tau * self.derivative_error_prev) / (dt + self.derivative_tau)

        output = self.kp * error + self.integral + integral_error + derivative_error


        # update variables
        self.error_prev = error
        self.derivative_error_prev = derivative_error

        # Saturate output
        if output > self.sat_max:
            output = self.sat_max
            if integral_error < 0.0:
                self.integral += integral_error

        elif output < self.sat_min:
            output = self.sat_min
            if integral_error > 0.0:
                self.integral += integral_error

        else:
            self.integral += integral_error

        return output


class Controller():
         
    def __init__(self):


        self.position_setpoint = Twist()
        self.position_setpoint.linear.z = 10

        # Publish the controller output to gazebo. (apply force and torque)
        self.controller_output_pub = rospy.Publisher('/drone/applied_forces', Wrench, queue_size=1)
        
        # Subscribe to ground truth position and orientation data
        self.state_sub = rospy.Subscriber('/drone/sensors/gps_hack', Odometry, self.newStateCB)

        # Subscribe to setpoints topic
        self.setpoint_sub = rospy.Subscriber('/drone/controller/setpoints', TwistStamped, self.newSetpointCB)

        self.time_prev = None
        self.state_prev = Odometry()


        self.x_vel_world = 0
        self.y_vel_world = 0
        self.z_vel_world = 0

        ##### CONTROLLERS #####

        # Initialize the PID controller
        self.position_pid_x = PID(0.5,0.1,0.5,-max_pitch,max_pitch,0)
        self.position_pid_y = PID(0.5,0.1,0.5,-max_roll,max_roll,0)
        self.position_pid_z = PID(1,0.1,1,-10,25,0)

        # PID for vertial and horizontal velocity
        self.x_vel_pid = PID(0.5,0,0.2,-max_hori_vel,max_hori_vel,0)
        self.y_vel_pid = PID(0.5,0,0.2,-max_hori_vel,max_hori_vel,0)
        self.z_vel_pid = PID(1,0,0,-max_vert_vel,max_vert_vel,0)

        # 
        self.roll_rate_pid = PID(1,0,0,-max_roll_rate,max_roll_rate,0)
        self.pitch_rate_pid = PID(1,0,0,-max_pitch_rate,max_pitch_rate,0)
        self.yaw_rate_pid = PID(1,0,0,-max_yaw_rate,max_yaw_rate,0)

        #
        self.attitude_pid_roll = PID(0.1,0,0.001,-1,1,0)
        self.attitude_pid_pitch = PID(0.1,0,0.001,-1,1,0)
        self.attitude_pid_yaw = PID(0.1,0,0.001,-1,1,0)
        


    def newSetpointCB(self,data):
        
        # Save the setpoint 
        self.position_setpoint = data.twist


    def newStateCB(self,data):
        
        
        ##### Remove data that comes at more than 1khz !!! #####
        # For some reason the plug in doesnt just stick to 100hz
        if self.time_prev == None:
            self.time_prev = data.header.stamp
            self.state_prev = data
            return

        time_now = data.header.stamp
        dt = (time_now - self.time_prev).nsecs*0.000000001
        self.time_prev = time_now

        #if dt < 0.001:
         #   return
        #########################################################

        self.state = data

        ##### Update the controller #####
        pid_out_z, pid_out_roll, pid_out_pitch, pid_out_yaw = self.updateController(dt)

        # Applies control effort to the drone and publishes it to gazebo
        self.applyForces(pid_out_z, pid_out_roll, pid_out_pitch, pid_out_yaw, data.pose.pose)

        # Save data for next iteration
        self.state_prev = data

    def updateController(self,dt):

        ###### linear velocities #####
        self.x_vel_world = (self.state.pose.pose.position.x - self.state_prev.pose.pose.position.x) /dt
        self.y_vel_world = (self.state.pose.pose.position.y - self.state_prev.pose.pose.position.y) /dt
        self.z_vel_world = (self.state.pose.pose.position.z - self.state_prev.pose.pose.position.z) /dt

        ##### Angular rates #####
        q = self.state.pose.pose.orientation
        (roll, pitch, yaw) = euler_from_quaternion ([q.x, q.y, q.z, q.w])

        q = self.state_prev.pose.pose.orientation
        (roll_prev, pitch_prev, yaw_prev) = euler_from_quaternion ([q.x, q.y, q.z, q.w])

        roll_rate = (roll - roll_prev) / dt
        pitch_rate = (pitch - pitch_prev) / dt
        yaw_rate = (yaw - yaw_prev) / dt

        ##### Linear velocity setpoints
        x_vel_sp = self.x_vel_pid.update(self.position_setpoint.linear.x, self.state.pose.pose.position.x, dt)
        y_vel_sp = self.y_vel_pid.update(self.position_setpoint.linear.y, self.state.pose.pose.position.y, dt)
        z_vel_sp = self.z_vel_pid.update(self.position_setpoint.linear.z, self.state.pose.pose.position.z, dt)


        ##### Convert X_world, Y_world to X_body, Y_body
        x_vel_body = m.cos(yaw) * self.x_vel_world + m.sin(yaw) * self.y_vel_world
        y_vel_body = -m.sin(yaw) * self.x_vel_world + m.cos(yaw) * self.y_vel_world

        x_vel_sp_body = m.cos(yaw) * x_vel_sp + m.sin(yaw) * y_vel_sp
        y_vel_sp_body = -m.sin(yaw) * x_vel_sp + m.cos(yaw) * y_vel_sp

        
        # Set the Yaw setpoint (controlled directly from the setpoint topic)
        yaw_setpoint = self.position_setpoint.angular.z


        # Update position pid
        pitch_setpoint = self.position_pid_x.update(x_vel_sp_body, x_vel_body, dt)
        roll_setpoint = -self.position_pid_y.update(y_vel_sp_body, y_vel_body, dt)
        pid_out_z = self.position_pid_z.update(z_vel_sp, self.z_vel_world, dt) + g*mass

        # Generate angular rates setpoints for the attitude controller
        roll_rate_sp = self.roll_rate_pid.update(roll_setpoint, roll, dt)
        pitch_rate_sp = self.roll_rate_pid.update(pitch_setpoint, pitch, dt)
        yaw_rate_sp = self.roll_rate_pid.update(yaw_setpoint, yaw, dt)        

        # Update the attitude controller        
        pid_out_roll = self.attitude_pid_roll.update(roll_rate_sp, roll_rate,dt)
        pid_out_pitch = self.attitude_pid_pitch.update(pitch_rate_sp, pitch_rate,dt)
        pid_out_yaw = self.attitude_pid_pitch.update(yaw_rate_sp, yaw_rate,dt)

        return pid_out_z, pid_out_roll, pid_out_pitch, pid_out_yaw

    def applyForces(self, lin_z, ang_x, ang_y, ang_z, pose):

        # heuristic drag. 
        drag_x = 0.1*self.x_vel_world**2
        drag_y = 0.1*self.y_vel_world**2
        drag_z = 0.3*self.z_vel_world**2

        # Apply forces
        msg = Wrench()

        # Extract the rotation matrix from quaternion
        q = pose.orientation
        rotation = quaternion_matrix([q.x,q.y,q.z,q.w])

        # Apply linear forces in world frame
        linear_forces_body = np.array([[0],[0],[lin_z],[1]])
        linear_force_world = np.dot(rotation, linear_forces_body)
        msg.force.x = linear_force_world[0]
        msg.force.y = linear_force_world[1]
        msg.force.z = linear_force_world[2]

        # Apply torques in the world frame
        torques_body = np.array([[ang_x],[ang_y],[ang_z],[1]])
        torques_world = np.dot(rotation,torques_body)
        msg.torque.x = torques_world[0]
        msg.torque.y = torques_world[1]
        msg.torque.z = torques_world[2]

        # Publish the msg
        self.controller_output_pub.publish(msg)

        



def main(args):
    controller_node = Controller()
    rospy.init_node('controller_node', anonymous=False)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)
