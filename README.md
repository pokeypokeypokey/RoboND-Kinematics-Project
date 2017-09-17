## Project: Kinematics Pick & Place

---

[//]: # (Image References)

[arm]: ./misc_images/arm.jpg

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points

---

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

![params][arm]

DH parameters (from .xacro file)

Links | alpha(i-1) | a(i-1) | d(i)  | theta(i)
---   | ---        | ---    | ---   | ---
0->1  | 0          | 0      | 0.75  | q1
1->2  | -pi/2      | 0.35   | 0     | -pi/2 + q2
2->3  | 0          | 1.25   | 0     | q3
3->4  | -pi/2      | -0.054 | 1.5   | q4
4->5  | pi/2       | 0      | 0     | q5
5->6  | -pi/2      | 0      | 0     | q6
6->EE | 0          | 0      | 0.303 | 0

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.




#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.
 

![alt text][image2]

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 

![alt text][image3]


