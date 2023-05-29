# Drone Localization using the onboard camera
Estimation of the position,attitude and velocity of a Micro Aerial Vehicle using perspective projection and optical flow.

/EstimatePose estimates the position and attitude of an MAV using the camera onboard capturing an april tag mat on the ground (Run estimatePose.m)

/EstimateVelocity estimates the velocity of the MAV using optical flow between frames. (Run OpticalFlow.m)

## **Notes**

For localization using IMU data and Extended Kalman filtering, check [this project](https://github.com/ar6841/MAV-state-estimation-using-extended-Kalman-filter)

## Dependencies

1. MATLAB Computer Vision Toolbox
2. MATLAB Parallel Computing Toolbox

## Results

The tracking results are shown below:

### Part 1

Data set 1:

![image](https://user-images.githubusercontent.com/96152967/218155586-b6060c1d-17da-4984-80d9-bd16aee48fd7.png)

Data set 4:

![image](https://user-images.githubusercontent.com/96152967/218155659-7bfc1da9-a0ca-49a4-a82c-05d30fd295f5.png)

### Part 2 

#### Without RANSAC:

Data set 1:

![image](https://user-images.githubusercontent.com/96152967/218156278-1002ae65-74dc-4c8e-b70b-c74c13e2fedb.png)


Data set 4:

![image](https://user-images.githubusercontent.com/96152967/218156176-29ac2746-ad50-4710-abae-38af12ed4673.png)


#### With RANSAC:

Data set 1:

![image](https://user-images.githubusercontent.com/96152967/218156101-334e246b-103d-4566-8152-b1b1e4d8814a.png)

Data set 4:

![image](https://user-images.githubusercontent.com/96152967/218156144-b87ae616-28a0-4487-947f-42283040dc41.png)




A detailed explanation of the project and the code is given below:
## Project overview

This project entails vision-based estimation of pose, velocity and angular velocity of a drone. The data for this was collected using a Nano+ quadrotor that was either held by hand or flown through a prescribed trajectory over a mat of AprilTags, each of which has a unique ID. 

## April Tags grid layout

The tags are arranged in a 12 x 9 grid. The top left corner of the top left tag has been used as coordinate (0, 0) with the X coordinate going down the mat and the Y coordinate going to the right. Each tag is a 0.152 m square with 0.152 m between tags, except for the space between columns 3 and 4, and 6 and 7, which is 0.178 m.

In an ideal case with no offsets (ignore the 0.178 m for now), let d be the length of the square and distance between the squares (d=0.152). For some (i,j)∈N which denote increments of AprilTags along the positive X and Y axes respectively, the x and y distances for each point of all AprilTags form an arithmetic sequence. The world frame coordinates of points p0, p1, p2, p3 and p4 of all the AprilTags are computed in the getCorner function. Here the offset kicks into the y position when j>3 and j>6. A table of (x,y) positions is created where the columns signify the AprilTag ID and the rows signify the (x,y) coordinates stored in the order p0, p1, p2, p3. This table ‘res_table’ is a persistent variable and exists between function calls to improve performance and is only created the first time the function getCorner is used. Subsequently only the ID column is pulled out and given as the output.

The images below are a visualization of the grid layout:

![image](https://user-images.githubusercontent.com/96152967/219221651-0cff144d-0dd7-46dd-b66a-5210a7142e9d.png)


![image](https://user-images.githubusercontent.com/96152967/219221399-7427cd09-143d-4988-94dd-b5991f4d910c.png)


## Pose Estimation
The pose of the MAV describes the position and orientation of the MAV in the world frame. This is computed in the estimatePose() function. The first goal is to use the captured data which contains the AprilTag ID, image coordinates of the points p0, p1, p2, p3, p4 for that tag ID and compute the homography of the camera at time stamp (t). The homography can be computed because we know the world frame coordinates of all the points for all the AprilTags. The homography can be computed by solving the equation

$$
\left(\begin{array}{X}
x_i & y_i & 1 & 0 & 0 & 0 & -x_i^{\prime} x_i & -x_i^{\prime} y_i & -x_i^{\prime} \\
0 & 0 & 0 & x_i & y_i & 1 & -y_i^{\prime} x_i & -y_i^{\prime} y_i & -y_i^{\prime}
\end{array}{X}\right) h=0 \\
$$

$$
A h=0
$$

Where $h=\mathrm{stacked}(H)$. To do this we find the A matrix for all the IDs detected and solve the SVD $A=U S V^T$ ( $h$ will be the 9th column of $V$ ).

For each AprilTag ID; in the code p_w(2*j-1) is the world x coordinate of point p(j-1). data(t).(fns{j+7})(1,i) is the image x coordinate of point p(j-1). Similarly p_w(2*j) and data(t).(fns{j+7})(2,i) are the y coordinates of the point p(j-1) in the world and image frames respectively. h is then converted to a (3x3) homography matrix HL whose sign must first be corrected with respect to some point p and its coordinates in the world and image frames, and normalized to find the required homography H.

$\mathrm{H}$ contains the data for rotation and translation from the world to the camera frame. To get the transformation matrix we first find $K^{-1} H$ where $\mathrm{K}$ is the camera calibration matrix.

```math
\left(\begin{array}{lll}
\hat{R}_1 & \hat{R}_2 & \hat{T}
\end{array}\right)=\left(\begin{array}{lll}
\hat{r}_{11} & \hat{r}_{12} & \hat{t}_1 \\
\hat{r}_{21} & \hat{r}_{22} & \hat{t}_2 \\
\hat{r}_{31} & \hat{r}_{32} & \hat{t}_3
\end{array}\right)=\left(\begin{array}{ccc}
f & 0 & x_0 \\
0 & f & y_0 \\
0 & 0 & 1
\end{array}\right)^{-1}\left(\begin{array}{lll}
h_{11} & h_{12} & h_{13} \\
h_{21} & h_{22} & h_{23} \\
h_{31} & h_{32} & h_{33}
\end{array}\right)
```

As orthonormality bust be preserved we need to find a rotation matrix $\mathrm{R}$ by finding the SVD.

$$
\begin{aligned}
& \left(\hat{R}_1 \hat{R}_2 \hat{R}_1 \times \hat{R}_2\right)=U S V^T \\
& R=U\left(\begin{array}{llc}
1 & 0 & 0 \\
0 & 1 & 0 \\
0 & 0 & \mathrm{det}\left(U V^T\right)
\end{array}\right) V^T
\end{aligned}
$$

The translation $\hat{T}$ must also be scaled by $\mathrm{norm}(\hat{R})$.
$\mathrm{R}$ describes $\quad{ }^c R_w$ and $T$ describes ${ }^c T_w$. Now we have ${ }^B R_C=\mathrm{rotx}(180) * \mathrm{rotz}(45)$ and ${ }^c T_B=[-0.04 ; 0 ;-0.03]$ from the parameters and pictures of the MAV provided. Thus we can form the equations

$$
\begin{aligned}
{ }^c H_B & =\left[\begin{array}{ll}
{ }^c R_B & { }^c T_B \\
0 & 1
\end{array}\right] \\
{ }^c H_w & =\left[\begin{array}{cc}
{ }^c R_w & { }^c T_w \\
0 & 1
\end{array}\right] \\
{ }^w H_B & =\left(\begin{array}{cc}
\left.{ }^c H_W\right)^{-1} & { }^c H_B
\end{array}\right.
\end{aligned}
$$

${ }^{{ }^W} H_B$ gives us ${ }^{{ }^W} R_B$ which is a rotation matrix formed by $\mathrm{ZYX}$ Euler angle sequence. It can be converted using rotm2eul() or done manually using:

```math
\begin{aligned}
& \boldsymbol{\beta}=\tan _2^{-1}\left(-r_{31}, \sqrt{r_{11}^2+r_{21}^2}\right) \\
& \boldsymbol{\alpha}=\tan _2^{-1}\left(r_{21}, r_{11}\right) \\
& \boldsymbol{\gamma}=\tan _2^{-1}\left(r_{32}, r_{33}\right)
\end{aligned}
```

where $\boldsymbol{\alpha} \boldsymbol{\beta} \boldsymbol{\gamma}$ are ZYX Euler angles respectively
The fourth column of ${ }^{{ }^W} H_B$ gives us the translation (position) of the MAV in the world frame. The output order of orientation(1:3) is $\mathrm{Z}, \mathrm{Y}$ and $\mathrm{X}$ Euler angles.
The output plots for part1 of the project are given in figures

## Corner Extraction and tracking
Part2 of the project entails the calculation of velocity and angular velocity using the optical flow between frames.
To calculate the velocity and angular velocity of the MAV, features in the image must be identified to calculate optical flow. At some time stamp t, the features from the image at (t-1) were detected using the detectFASTFeatures() method provided in the MATLAB Computer Vision toolbox. The points with a metric greater than some cutoff_metric are then selected to ensure that the strongest points get used.  Then the features at the current timestamp t are detected using a vision.PointTracker object from the Computer Vision toolbox, this object utilizes the KLT algorithm to detect and return the pixel coordinates of the matching features. 
The pixel coordinates of the matching features of the images at timestamp (t) and (t-1) have now been established. 

The pixel coordinates of matched_points1 and matched_points2 need to be normalized using the camera calibration matrix $\mathrm{K} .(K)^{-1} *$ matched_points gives the normalized image coordinates.
Note: in the code ‘n’ represents the timestamp ‘t’.

## Optical Flow
The optical flow of the image can be calculated by finding the shift in normalized pixel coordinate vector and dividing by change in time between frames. 

$$
\left[\begin{array}{l}
\dot{x} \\
\dot{y}
\end{array}\right]=\frac{1}{\Delta t}\left[\begin{array}{l}
\Delta x \\
\Delta y
\end{array}\right]
$$

The $\Delta t$ is passed through a lowpass filter to smooth the results. The optical flow $\dot{\mathbf{p}}$ is represented by p_dot and opt_flow in the code.

## Velocity Estimation
To compute the velocity and angular velocity we must solve the equation

$$
\begin{gathered}
\boldsymbol{V}=\left(\begin{array}{l}
\mathbf{V} \\
\boldsymbol{\Omega}
\end{array}\right)=\left(\boldsymbol{H}^T \mathbf{H}\right)^{-\mathbf{1}} \boldsymbol{H}^T \dot{\mathbf{p}} \\
\\
\mathbf{H}=\left(\begin{array}{cc}
\frac{1}{Z_1} \boldsymbol{A}\left(\boldsymbol{p}_1\right) & \boldsymbol{B}\left(\boldsymbol{p}_1\right) \\
\vdots \\
\frac{1}{Z_n} \boldsymbol{A}\left(\boldsymbol{p}_n\right) & \boldsymbol{B}\left(\boldsymbol{p}_n\right)
\end{array}\right) \quad \dot{\mathbf{p}}=\left(\begin{array}{c}
\dot{\boldsymbol{p}}_1 \\
\vdots \\
\dot{\boldsymbol{p}}_n
\end{array}\right) \\
\\
\boldsymbol{A}\left(\boldsymbol{p}_1\right)=\left(\begin{array}{ccc}
-1 & 0 & x \\
0 & -1 & y
\end{array}\right) \text { and } \quad \boldsymbol{B}\left(\boldsymbol{p}_1\right)=\left(\begin{array}{ccc}
x y & -\left(1+x^2\right) & y \\
1+y^2 & -x y & -x
\end{array}\right)
\end{gathered}
$$

Where $\boldsymbol{V}$ is a $(6,1)$ vector containing the linear and angular velocities, $\dot{\mathbf{p}}$ is a (2x(number of corners used),1) vector and $\mathrm{H}$ is a (2x(number of corners used), 6) matrix. $Z_n$ is the depth of the corner detected.

![image](https://user-images.githubusercontent.com/96152967/218152799-e8397424-7536-4a6c-abd7-914f923c8238.png)

$$
\text { If the corner detected is at }\left(\begin{array}{l}
x_c \\
y_c \\
z_c
\end{array}\right) \text { in the camera frame, then we can form the vector sum: }
$$

$$
{ }^C P_W+{ }^C R_W\left(\begin{array}{l}
x_w \\
y_w \\
0
\end{array}\right)=\left(\begin{array}{l}
x_c \\
y_c \\
z_c
\end{array}\right) \text { where }
$$

${ }^C \boldsymbol{P}_W$ is the position of the origin of the world frame written in the camera frame 

$$
\left(\begin{array}{l}
x_c \\
y_c \\
z_c
\end{array}\right)=\lambda\left(\begin{array}{l}
x \\
y \\
1
\end{array}\right) \text { where }\left(\begin{array}{l}
x \\
y \\
1
\end{array}\right) \text { are the normalized image coordinates }
$$

Note: in the code ${ }^C P_W$ is represented by position_cam. When we expand ${ }^C R_W$ and simplify as a system of linear equations we get.

```math
\begin{aligned}
& \left(\begin{array}{lll}
r_{11} & r_{12} & -x \\
r_{21} & r_{22} & -y \\
r_{31} & r_{32} & -1
\end{array}\right)\left(\begin{array}{l}
x_w \\
y_w \\
\lambda
\end{array}\right)=-{ }^C \boldsymbol{P}_W \\
\\
& \left(\begin{array}{l}
x_w \\
y_w \\
\lambda
\end{array}\right)=-G^{-1}\quad{}^C\boldsymbol{P}_W 
\text{ where } G=\left(\begin{array}{lll}
r_{11} & r_{12} & -x \\
r_{21} & r_{22} & -y \\
r_{31} & r_{32} & -1
\end{array}\right) \\
&
\end{aligned}
```

As $\lambda$ is $Z_n$ of the corner we can now compute the H matrix for all the corners in the (t-1) frame, vertically concatenate them, and using the optical flow $\dot{\mathbf{p}}$, the velocity $\boldsymbol{V}$ in the equation 

$$
\left(\begin{array}{l}
\mathbf{V} \\
\boldsymbol{\Omega}
\end{array}\right)=
\mathrm{pinv}(\boldsymbol{H}) \dot{\mathbf{p}}
$$

Where pinv() is the pseudoinverse matrix function.

In the code, a for loop from 1 to length(matched points) is used to implement the steps above. This gives us the linear and angular velocity $\boldsymbol{V}$ of the camera expressed in the camera frame. To find the linear and angular velocity expressed in the world frame we can use:
Thus, we have calculated the linear and angular velocity of the MAV between two frames. The steps must be repeated between all the images collected in the dataset.

The final velocity estimatedVel has a bit of a jitter so it was passed through a lowpass() filter
with sampling rate 1000 and fs_pass 1 to smooth out the graph.

## RANSAC based outlier rejection
To improve the results from the previous phase, a flag in the parameter section, ransac_flag can be changed to true. This implements the RANSAC based 3 point outlier rejection in the code. When the flag is set to true, the function velocityRANSAC is called. 

The inputs to this function are the optical flow of the matched corners opt $\mathrm{V}$, the normalized image coordinates of the matched corners in frame (t-1) optPos, the depth of the corners in the camera frame Z, ${ }^C \boldsymbol{R}_W$ and the RANSAC hyper parameter $\epsilon$. The output of the function is the $(6,1)$ velocity vector $\boldsymbol{V}$ in the camera frame.



