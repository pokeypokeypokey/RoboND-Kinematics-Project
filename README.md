## Project: Kinematics Pick & Place

---

[//]: # (Image References)

[arm]: ./misc_images/arm.jpg
[theta23]: ./misc_images/theta23.jpg
[theta456]: ./misc_images/theta456.gif

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

Matrices generated with the following code:

```
from sympy import symbols, Matrix

def DH_transform_matrix(alpha, a, d, q):
    return Matrix([[            cos(q),           -sin(q),           0,             a],
                   [ sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
                   [ sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],
                   [                 0,                 0,           0,             1]])

q1, q2, q3, q4, q5, q6 = symbols('q1:7')
T0_1  = DH_transform_matrix( 0,     0,      0.75,  q1)
T1_2  = DH_transform_matrix(-pi/2,  0.35,   0,     q2)
T2_3  = DH_transform_matrix( 0,     1.25,   0,     q3)
T3_4  = DH_transform_matrix(-pi/2, -0.054,  1.5,   q4)
T4_5  = DH_transform_matrix( pi/2,  0,      0,     q5)
T5_6  = DH_transform_matrix(-pi/2,  0,      0,     q6)
T6_EE = DH_transform_matrix( 0,     0,      0.303, 0)

T_total = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_EE
```

#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.
 
The wrist center is chosen at joint 5 (see image above). The first three joints (i.e. thetas 1-3) are used to move the wrist center to the correct position (inverse position kinematics) and the last three joints (i.e. thetas 4-6) are used to orient the end effector (inverse orientation kinematics).

Theta 1 is calculated by projecting the wrist center (WC) onto the XY plane. It can then be solved with `atan2(WCy, WCx)`.



Theta 2 & 3 are calculated by solving the following:

![params][theta456]

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 

![alt text][image3]


