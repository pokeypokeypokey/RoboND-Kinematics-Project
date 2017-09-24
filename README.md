## Project: Kinematics Pick & Place

---

[//]: # (Image References)

[arm]: ./misc_images/arm.jpg
[theta23]: ./misc_images/theta23.jpg
[theta456]: ./misc_images/theta456.gif
[fetch]: ./misc_images/fetch.png
[drop]: ./misc_images/drop.png
[final]: ./misc_images/final.png
[errors]: ./misc_images/errors.png

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points

---

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

![params][arm]

DH parameters (from `.xacro` file)

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

```python
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

Theta 2 is calculated by constructing a triangle between joint 2, joint 3 and the wrist center, as shown below:

![params][theta23]

The lengths of the sides are known, and the angles can be solved with the cosine law. Theta 2 plus `a` is `atan2(Bxy, Bz)` where `Bxy` and `Bz` are the components of `B` projected onto the xy plane and the z axis respectively. `a` can then be subtracted to get theta 2.

Theta 3 is simply `pi/2` minus `b`, with a constant offset as indicated.

Thetas 4, 5 & 6 are calculated by solving the following:

![params][theta456]

The right matrix was created with a composition rotation matrix from joint 3 to the end effector:

```python
R3_6_comp = simplify(T3_4[0:3, 0:3] * T4_5[0:3, 0:3] * T5_6[0:3, 0:3] * T6_G[0:3, 0:3])
```

Taking advantage of the trig rules `tan(x) = sin(x)/cos(x)` and `sin(x)**2 + cos(x)**2 = 1` allows the use of `atan`, which avoids quadrant issues:

```python
theta4 = atan2(r22, -r02)
theta5 = atan2(sqrt(r10**2 + r11**2), r12)
theta6 = atan2(-r11, r10)
```

The values of `rnm` in the left matrix come from substituting thetas 1-3 in the `R0_3` composite rotation matrix, inverting that and multiplying by the final rotation matrix:

```python
R0_3 = (T0_3[0:3, 0:3]).evalf(subs={q1: theta1, q2: theta2, q3: theta3})
R3_6 = R0_3.inv() * R_total
```

Where `R_total` was the final (target) rotation, including a correction factor:

```python 
R_corr   = rot_matrix_z(pi) * rot_matrix_y(-pi/2.)
R_target = rot_matrix_z(target_yaw) * rot_matrix_y(target_pitch) * rot_matrix_x(target_roll)
R_total = R_target * R_corr
```

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 

The node was converted to a class so that the transforms could be pre-calculated on node startup.

Inverse kinematics calculations were implemented as described with only minor adjustments:  Thetas 4 and 6 were wrapped between `-pi` and `pi` to try minimise rotations. Also, to take advantage of the fact that `R0_3` is orthogonal, the transpose was used to calculated the inverse, which is much faster and more reliable.

##### Results
Fetching a cylinder:
![params][fetch]

Dropping cylinder in the bucket:
![params][drop]

When the placement accuracy is a bit too good and the bucket fills prematurely :P 
![params][final]

##### Plotting EE error
To get the EE error, the full EE transform was applied to the calculated joint angles. The resulting position was then compared to the request, and the difference plotted:

![params][errors]

The red, green and blue traces show the errors in x, y and z end effector locations respectively, as the arm moved from the start position to the position shown.