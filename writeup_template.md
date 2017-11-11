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

[image1]: ./misc_images/misc1.png
[image2]: ./misc_images/misc3.png
[image3]: ./misc_images/misc2.png
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

Here is an example of how to include an image in your writeup.

![alt text][image1]

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


Here I'll talk about the code, what techniques I used, what worked and why, where the implementation might fail and how I might improve it if I were going to pursue this project further.  


And just for fun, another example image:
![alt text][image3]


