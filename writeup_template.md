## Project: Kinematics Pick & Place
### Writeup Template: You can use this file as a template for your writeup if you want to submit it as a markdown file, but feel free to use some other method and submit a pdf if you prefer.

---


**Steps to complete the project:**  


1. Set up your ROS Workspace.
2. Download or clone the [project repository](https://github.com/udacity/RoboND-Kinematics-Project) into the ***src*** directory of your ROS Workspace.  
3. Experiment with the forward_kinematics environment and get familiar with the robot.
4. Launch in [demo mode](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/ae64bb91-e8c4-44c9-adbe-798e8f688193).
5. Perform Kinematic Analysis for the robot following the [project rubric](https://review.udacity.com/#!/rubrics/972/view).
6. Fill in the `IK_server.py` with your Inverse Kinematics code. 


[//]: # (Image References)

[image7]: ./misc_images/gripperframe.png
[image8]: ./misc_images/DH parameters.png
[image4]: ./misc_images/theta1.png
[image5]: ./misc_images/theta2.png
[image6]: ./misc_images/theta3.png



## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

i used the same axis assignment as explained in the lessons and derived the DH parameters

The figure below is a screenshot from the lesson explainning the assignment of axis for deriving DH parameters


   ![alt text][image7]


From the figure below i get the alpha values in DH parameters


   ![alt text][image8]


and using data in kr210.urdf.xacro file i get the other DH parameters as following: 
        
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





#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

Links | alpha(i-1) | a(i-1) | d(i-1) | theta(i)
--- | --- | --- | --- | ---
0->1 | 0 | 0 | d12 | q1
1->2 | - pi/2 | a12 | 0 | -pi/2 + q2
2->3 | 0 | a23 | 0 | q3
3->4 |  -pi/2 | a34 | d45 | q4
4->5 | pi/2 | 0 | 0 | q5
5->6 | -pi/2 | 0 | 0 | q6
6->EE | 0 | 0 | d67 | q7


#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

##### Steps
 
###### 1-Calculate wrist center poistion w.r.t base link from end effector poistion

![wrist postion equation](http://latex.codecogs.com/gif.latex?%5Clarge%20_%7B%7D%5E%7B0%7D%5Ctextrm%7Br%7D_%7BWC/0%7D%3D%7B%7D%5E%7B0%7D%5Ctextrm%7Br%7D_%7BEE/0%7D-d*_%7B6%7D%5E%7B0%7D%5Ctextrm%7BR%7D*%5Cbegin%7Bbmatrix%7D%200%5C%5C%200%5C%5C%201%5C%5C%20%5Cend%7Bbmatrix%7D)

where d is the position of wrist center w.r.t end efffector
and the rotation matrix is obtained from yaw , pitch & roll angles of end effector multiplied by correction matrix s0 the wrist position is obtained using equations below

	    #Rotational matrix using yaw,pitch & roll of the gripper
	    Rrpy=rot_z(yaw)*rot_y(pitch)*rot_x(roll)*R_corr
	    #Calculate the position of wrist center
	    l_gripper=0
	    WC_x=px-((d67+l_gripper)*Rrpy[0,2])
	    WC_y=py-((d67+l_gripper)*Rrpy[1,2])
	    WC_z=pz-((d67+l_gripper)*Rrpy[2,2])

###### 2-Calculate thata 1-3 using wrist position

###### a-theta 1 can be calculated from the figure below
     
      #theta1 calculation
      theta1=atan2(WC_y,WC_x)


![alt text][image4]

###### b-theta 2 can be calculated from the figure below

      #theta2 calculation
	    beta1=atan2(WC_z-d12,(sqrt((WC_x**2)+(WC_y**2))-a12))
	    
	    #d25:distance from joint 2 to WC
	    d25=sqrt(((WC_z-d12)**2)+((sqrt((WC_x**2)+(WC_y**2))-a12)**2))

	    #d35:distance from joint 3 to WC
	    d35=sqrt((d45**2)+(a34**2))
	    
	    beta2=acos(((d25**2)+(a23**2)-(d35**2))/(2*d25*a23))
	    
	    theta2=(np.pi/2)-beta1-beta2
      
 ![alt text][image5]
     
###### c-theta 3 can be calculated from the figure below


	    #theta3 calculation

	    beta3=acos(((a23**2)+(d35**2)-(d25**2))/(2*d35*a23))
	    
	    beta4=atan2(a34,d45)
	    
	    theta3=(np.pi/2)-beta3-beta4
	 
![alt text][image6]


##### 3-Calculate thata 4-6 using rotation matrices to get Euler angles

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





### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 


##### The code is divided into 2 parts:

###### In part 1 :Calculation of homogenous transform from base link to end effector using DH parameters

	#Create individual transformation matrices
	####Homogeneous Transforms
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

	#Transform from base link to end effector
	T0_7 = T0_1 * T1_2 * T2_3 * T3_4* T4_5* T5_6* T6_7
	#Calculate correction matrix
	R_corr=rot_z(180*dtr)*rot_y(-90*dtr)
	#Transform from base link to end effector after correction matrix
	T_corr = R_corr.row_join(Matrix([[0], [0], [0]]))
	T_corr = T_corr.col_join(Matrix([[0, 0, 0, 1]])) 
	T_base_gripper = T0_7*T_corr
	
###### In part 2 :Calculation of theta 1-6 using end effector position and orientation as disscussed above

I used IK_debug.py to debug my inverse kinematics code and that is the results i get 

Total run time to calculate joint angles from pose is 33.7475 seconds

Wrist error for x position is: 0.00002426
Wrist error for y position is: 0.00000562
Wrist error for z position is: 0.00006521
Overall wrist offset is: 0.00006980 units

Theta 1 error is: 3.14309971
Theta 2 error is: 0.27927962
Theta 3 error is: 1.94030206
Theta 4 error is: 3.11289370
Theta 5 error is: 0.03462632
Theta 6 error is: 6.20633831

**These theta errors may not be a correct representation of your code, due to the fact            
that the arm can have muliple positions. It is best to add your forward kinmeatics to            
confirm whether your code is working or not**
 

End effector error for x position is: 0.05348588
End effector error for y position is: 0.05381796
End effector error for z position is: 0.07685628
Overall end effector offset is: 0.10800000 units 










