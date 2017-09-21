#!/usr/bin/env python

# Copyright (C) 2017 Electric Movement Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Float64
from mpmath import *
from sympy import symbols, cos, acos, sin, atan2, simplify, sqrt, pi
from sympy.matrices import Matrix
from math import floor, pi

pi_2 = pi/2.


class IK_server(object):
    def __init__(self, track_error=True):
        # initialize node and declare calculate_ik service
        rospy.init_node('IK_server')
        self.serv = rospy.Service('calculate_ik', CalculateIK, self.handle_calculate_IK)
        print "Ready to receive an IK request"

        # Pre calculate as much as possible
        # Create symbols
        q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
        d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
        a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
        alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')
        
        # Define Modified DH Transformation matrix
        s = {alpha0: 0,       a0: 0,      d1: 0.75,   q1: q1,
             alpha1: -pi_2,   a1: 0.35,   d2: 0,      q2: q2 - pi_2,
             alpha2: 0,       a2: 1.25,   d3: 0,      q3: q3,
             alpha3: -pi_2,   a3: -0.054, d4: 1.5,    q4: q4,
             alpha4:  pi_2,   a4: 0,      d5: 0,      q5: q5,
             alpha5: -pi_2,   a5: 0,      d6: 0,      q6: q6,
             alpha6: 0,       a6: 0,      d7: 0.303,  q7: 0}

        # Create individual transformation matrices
        T0_1 = self._DH_transform_matrix(alpha0, a0, d1, q1).subs(s)
        T1_2 = self._DH_transform_matrix(alpha1, a1, d2, q2).subs(s)
        T2_3 = self._DH_transform_matrix(alpha2, a2, d3, q3).subs(s)
        T3_4 = self._DH_transform_matrix(alpha3, a3, d4, q4).subs(s)
        T4_5 = self._DH_transform_matrix(alpha4, a4, d5, q5).subs(s)
        T5_6 = self._DH_transform_matrix(alpha5, a5, d6, q6).subs(s)
        T6_G = self._DH_transform_matrix(alpha6, a6, d7, q7).subs(s)

        # Combined transforms
        self.T0_3 = (T0_1 * T1_2 * T2_3) # base link to link 3
        self.T_total = (self.T0_3 * T3_4 * T4_5 * T5_6 * T6_G) # base link to gripper

        # Extract rotation matrices from the transformation matrices
        # Gripper orientation correction
        R_z = self._rot_matrix_z(pi)
        R_y = self._rot_matrix_y(-pi_2)
        self.R_corr = (R_z * R_y)

        # Flag for error tracking
        self.TRACK_ERROR = track_error
        self.error_pub = rospy.Publisher('/kuka_arm/EE_error', Float64, queue_size=10)

        rospy.spin()

    def _DH_transform_matrix(self, alpha, a, d, q):
        return Matrix([[            cos(q),           -sin(q),           0,             a],
                       [ sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
                       [ sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],
                       [                 0,                 0,           0,             1]])

    def _rot_matrix_x(self, theta):
        return Matrix([[ 1,           0,           0         ],
                       [ 0,           cos(theta), -sin(theta)],
                       [ 0,           sin(theta),  cos(theta)]])

    def _rot_matrix_y(self, theta):
        return Matrix([[ cos(theta),  0,           sin(theta)],
                       [ 0,           1,           0         ],
                       [-sin(theta),  0,           cos(theta)]])

    def _rot_matrix_z(self, theta):
        return Matrix([[ cos(theta), -sin(theta),  0],
                       [ sin(theta),  cos(theta),  0],
                       [ 0,           0,           1]])

    def _norm_angle(self, angle):
        """
        Normalise angle between -pi and pi.
        """
        width  = 2.*pi
        offset = angle + pi
        return (offset - (floor(offset/width)*width)) - pi

    def _track_error(self, thetas, request_EE_xyz):
        q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
        calc_EE = self.T_total.evalf(subs={q1: thetas[0], q2: thetas[1], q3: thetas[2], 
                                           q4: thetas[3], q5: thetas[4], q6: thetas[5]})

        error = sqrt((calc_EE[0,3] - request_EE_xyz[0])**2 
                    + (calc_EE[1,3] - request_EE_xyz[1])**2 
                    + (calc_EE[2,3] - request_EE_xyz[2])**2)

        # publish error
        print error
        # self.error_pub.publish(error)

    def handle_calculate_IK(self, req):
        rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
        if len(req.poses) < 1:
            print "No valid poses received"
            return -1

        # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

            # Extract end-effector position and orientation from request
            # px,py,pz = end-effector position
            # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])
     
            # Calculate joint angles using Geometric IK method
            # Final rotation
            Rrpy = self._rot_matrix_x(yaw) * self._rot_matrix_x(pitch) \
                    * self._rot_matrix_x(roll) * self.R_corr

            # Wrist center
            wx = px - (0 + 0.303)*Rrpy[0, 2]
            wy = py - (0 + 0.303)*Rrpy[1, 2]
            wz = pz - (0 + 0.303)*Rrpy[2, 2]
            
            # Trig
            wxy = sqrt(wx**2 + wy**2)
            Bxy = wxy-0.35
            Bz = wz-0.75
            B = sqrt(Bxy**2 + Bz**2)
            A = 1.50097     # sqrt(0.054**2 + 1.5**2)
            C = 1.25
            a = acos((B**2 + C**2 - A**2)/(2*B*C))
            b = acos((A**2 + C**2 - B**2)/(2*A*C))
            t3_offset = 0.03598     # atan(0.054/1.5)

            theta1 = atan2(wy, wx)
            theta2 = atan2(Bxy, Bz) - a
            theta3 = pi_2 - b - t3_offset

            # Extract rotation matrices from the transformation matrices
            q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
            R0_3 = (self.T0_3[0:3, 0:3]).evalf(subs={q1: theta1, q2: theta2, q3: theta3})
            # R3_6 = R0_3.inv() * Rrpy
            R3_6 = R0_3.transpose() * Rrpy  # Note transpose equivalent and faster than inverse

            theta5 = atan2(sqrt(R3_6[1,0]**2 + R3_6[1,1]**2), R3_6[1,2])
            if sin(theta5) < 0:
                theta4 = atan2(-R3_6[2,2], R3_6[0,2])
                theta6 = atan2(R3_6[1,1], -R3_6[1,0])
            else:
                theta4 = atan2(R3_6[2,2], -R3_6[0,2])
                theta6 = atan2(-R3_6[1,1], R3_6[1,0])

            # Populate response for the IK request
            thetas = map(self._norm_angle, [theta1, theta2, theta3, theta4, theta5, theta6])
            joint_trajectory_point.positions = thetas
            joint_trajectory_list.append(joint_trajectory_point)

            if self.TRACK_ERROR:
                self._track_error(thetas, (px, py, pz))

        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)
    

if __name__ == "__main__":
    try:
        IK_server()
    except rospy.ROSInterruptException:
        pass
