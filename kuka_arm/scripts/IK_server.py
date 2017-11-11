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
from sympy import *
import numpy as np

def rot_x(q):
    R_x = Matrix([[ 1,              0,        0],
                  [ 0,         cos(q),  -sin(q)],
                  [ 0,         sin(q),  cos(q)]])
    
    return R_x
    
def rot_y(q):              
    R_y = Matrix([[ cos(q),        0,  sin(q)],
                  [      0,        1,       0],
                  [-sin(q),        0, cos(q)]])
    
    return R_y

def rot_z(q):    
    R_z = Matrix([[ cos(q),  -sin(q),       0],
                  [ sin(q),   cos(q),       0],
                  [      0,        0,       1]])
    
    return R_z


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
		
        ### Your FK code here
	# Create symbols
	q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
	d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
	a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
	alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')
	 
	# Create Modified DH parameters
	# Conversion Factors
	rtd = 180/np.pi
	dtr = np.pi/180

	a12=0.35
	a23=1.25
	a34=-0.054
	d12=0.75
	d45=1.5
	d67=0.303

	s = {alpha0: 0,       a0:   0, d1: d12, 
	     alpha1: -90*dtr, a1: a12, d2: 0,  
	     alpha2: 0,       a2: a23, d3: 0,
	     alpha3: -90*dtr, a3: a34, d4: d45,
	     alpha4: 90*dtr,  a4:   0, d5: 0,
	     alpha5: -90*dtr, a5:   0, d6: 0,
	     alpha6: 0,       a6:   0, d7: d67}
	#
	#            
	# Define Modified DH Transformation matrix
	#
	#
	# Create individual transformation matrices
	#### Homogeneous Transforms
	T0_1 = Matrix([[             cos(q1),            -sin(q1),            0,              a0],
		       [ sin(q1)*cos(alpha0), cos(q1)*cos(alpha0), -sin(alpha0), -sin(alpha0)*d1],
		       [ sin(q1)*sin(alpha0), cos(q1)*sin(alpha0),  cos(alpha0),  cos(alpha0)*d1],
		       [                   0,                   0,            0,               1]])
	T0_1 = T0_1.subs(s)

	T1_2 = Matrix([[             cos(q2-(90*dtr)),            -sin(q2-(90*dtr)),            0,              a1],
		       [ sin(q2-(90*dtr))*cos(alpha1), cos(q2-(90*dtr))*cos(alpha1), -sin(alpha1), -sin(alpha1)*d2],
		       [ sin(q2-(90*dtr))*sin(alpha1), cos(q2-(90*dtr))*sin(alpha1),  cos(alpha1),  cos(alpha1)*d2],
		       [                   0,                   0,            0,               1]])
	T1_2 = T1_2.subs(s)

	T2_3 = Matrix([[             cos(q3),            -sin(q3),            0,              a2],
		       [ sin(q3)*cos(alpha2), cos(q3)*cos(alpha2), -sin(alpha2), -sin(alpha2)*d3],
		       [ sin(q3)*sin(alpha2), cos(q3)*sin(alpha2),  cos(alpha2),  cos(alpha2)*d3],
		       [                   0,                   0,            0,               1]])
	T2_3 = T2_3.subs(s)

	T3_4 = Matrix([[             cos(q4),            -sin(q4),            0,              a3],
		       [ sin(q4)*cos(alpha3), cos(q4)*cos(alpha3), -sin(alpha3), -sin(alpha3)*d4],
		       [ sin(q4)*sin(alpha3), cos(q4)*sin(alpha3),  cos(alpha3),  cos(alpha3)*d4],
		       [                   0,                   0,            0,               1]])
	T3_4 = T3_4.subs(s)

	T4_5 = Matrix([[             cos(q5),            -sin(q5),            0,              a4],
		       [ sin(q5)*cos(alpha4), cos(q5)*cos(alpha4), -sin(alpha4), -sin(alpha4)*d5],
		       [ sin(q5)*sin(alpha4), cos(q5)*sin(alpha4),  cos(alpha4),  cos(alpha4)*d5],
		       [                   0,                   0,            0,               1]])
	T4_5 = T4_5.subs(s)

	T5_6 = Matrix([[             cos(q6),            -sin(q6),            0,              a5],
		       [ sin(q6)*cos(alpha5), cos(q6)*cos(alpha5), -sin(alpha5), -sin(alpha5)*d6],
		       [ sin(q6)*sin(alpha5), cos(q6)*sin(alpha5),  cos(alpha5),  cos(alpha5)*d6],
		       [                   0,                   0,            0,               1]])
	T5_6 = T5_6.subs(s)

	T6_7 = Matrix([[             cos(q7),            -sin(q7),            0,              a6],
		       [ sin(q7)*cos(alpha6), cos(q7)*cos(alpha6), -sin(alpha6), -sin(alpha6)*d7],
		       [ sin(q7)*sin(alpha6), cos(q7)*sin(alpha6),  cos(alpha6),  cos(alpha6)*d7],
		       [                   0,                   0,            0,               1]])
	T6_7 = T6_7.subs(s)

	# Transform from base link to end effector
	T0_7 = T0_1 * T1_2 * T2_3 * T3_4* T4_5* T5_6* T6_7
	#Calculate correction matrix
	R_corr=rot_z(180*dtr)*rot_y(-90*dtr)
	#Transform from base link to end effector after correction matrix
	T_corr = R_corr.row_join(Matrix([[0], [0], [0]]))
	T_corr = T_corr.col_join(Matrix([[0, 0, 0, 1]])) 
	T_base_gripper = T0_7*T_corr

        
        
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
	    #Rotational matrix using yaw,pitch & roll of the gripper
	    Rrpy=rot_z(yaw)*rot_y(pitch)*rot_x(roll)*R_corr
	    #Calculate the position of wrist center
	    l_gripper=0
	    WC_x=px-((d67+l_gripper)*Rrpy[0,2])
	    WC_y=py-((d67+l_gripper)*Rrpy[1,2])
	    WC_z=pz-((d67+l_gripper)*Rrpy[2,2])
	    #theta1 calculation
	    theta1=atan2(WC_y,WC_x)

	    #theta2 calculation
	    beta1=atan2(WC_z-d12,(sqrt((WC_x**2)+(WC_y**2))-a12))
	    
	    #d25:distance from joint 2 to WC
	    d25=sqrt(((WC_z-d12)**2)+((sqrt((WC_x**2)+(WC_y**2))-a12)**2))

	    #d35:distance from joint 3 to WC
	    d35=sqrt((d45**2)+(a34**2))
	    
	    beta2=acos(((d25**2)+(a23**2)-(d35**2))/(2*d25*a23))
	    
	    theta2=(np.pi/2)-beta1-beta2

	    #theta3 calculation

	    beta3=acos(((a23**2)+(d35**2)-(d25**2))/(2*d35*a23))
	    
	    beta4=atan2(a34,d45)
	    
	    theta3=(np.pi/2)-beta3-beta4
	    
	    #theta4,theta5,theta6 calculation

	    T0_3=T0_1 * T1_2 * T2_3 
	    T0_3=(T0_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3}))

	    R0_3=T0_3[0:3,0:3]
	    R3_6=R0_3.T * Rrpy

	    ### Euler Angles from Rotation Matrix
	    # sympy synatx for atan2 is atan2(y, x)
	    theta4 = atan2(R3_6[2,2], -R3_6[0,2])
	    theta5 = atan2(sqrt(R3_6[0,2]**2 + R3_6[2,2]**2), R3_6[1,2])
	    theta6 = atan2(-R3_6[1,1], R3_6[1,0])

	        
		
            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
	    joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
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
