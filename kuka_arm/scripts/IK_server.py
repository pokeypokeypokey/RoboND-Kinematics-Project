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
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import symbols, cos, acos, sin, atan2, simplify, sqrt, pi
from sympy.matrices import Matrix
from math import floor, pi

pi_2 = pi/2.

def DH_transform_matrix(alpha, a, d, q):
    return Matrix([[            cos(q),           -sin(q),           0,             a],
                   [ sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
                   [ sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],
                   [                 0,                 0,           0,             1]])

def rot_matrix_x(theta):
    return Matrix([[ 1,           0,           0         ],
                   [ 0,           cos(theta), -sin(theta)],
                   [ 0,           sin(theta),  cos(theta)]])

def rot_matrix_y(theta):
    return Matrix([[ cos(theta),  0,           sin(theta)],
                   [ 0,           1,           0         ],
                   [-sin(theta),  0,           cos(theta)]])

def rot_matrix_z(theta):
    return Matrix([[ cos(theta), -sin(theta),  0],
                   [ sin(theta),  cos(theta),  0],
                   [ 0,           0,           1]])

def norm_angle(angle):
    """
    Normalise angle between -pi and pi.
    """
    width  = 2.*pi
    offset = angle + pi
    return (offset - (floor(offset/width)*width)) - pi

def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1

    ### Your FK code here
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
    T0_1 = DH_transform_matrix(alpha0, a0, d1, q1).subs(s)
    T1_2 = DH_transform_matrix(alpha1, a1, d2, q2).subs(s)
    T2_3 = DH_transform_matrix(alpha2, a2, d3, q3).subs(s)

    # Combined transforms
    T0_2 = (T0_1 * T1_2) # base link to link 2
    T0_3 = (T0_2 * T2_3) # base link to link 3

    # Extract rotation matrices from the transformation matrices
    # Gripper orientation correction
    R_z = rot_matrix_z(pi)
    R_y = rot_matrix_y(-pi_2)
    R_corr = (R_z * R_y)

    e1, e2, e3 = symbols('e1:4')
    Re_x = rot_matrix_x(e1)
    Re_y = rot_matrix_y(e2)
    Re_z = rot_matrix_z(e3)

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
        Rrpy = (Re_z * Re_y * Re_x * R_corr).evalf(subs={e1: roll, e2: pitch, e3: yaw})

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
        R0_3 = (T0_3[0:3, 0:3]).evalf(subs={q1: theta1, q2: theta2, q3: theta3})
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
        joint_trajectory_point.positions = map(norm_angle, [theta1, theta2, theta3, theta4, theta5, theta6])
        joint_trajectory_list.append(joint_trajectory_point)

    rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
    return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()
