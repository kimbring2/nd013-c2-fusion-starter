# Writeup: Track 3D-Objects Over Time

Please use this starter template to answer the following questions:

### 1. Write a short recap of the four tracking steps and what you implemented there (filter, track management, association, camera fusion). Which results did you achieve? Which part of the project was most difficult for you to complete, and why?

- Filter.py : In this step, I implement the state transition function F and process noise covariance Q for the Extended Kalman Filter for the tracking car. 

For 3D motion and velocity, the constant velocity system matrix is:

$\begin{aligned} \mathbf{F}(\Delta t) = \begin{bmatrix}1 & 0 & 0 & \Delta t & 0 & 0 \\ 
                                       0 & 1 & 0 & 0 & \Delta t & 0 \\
                                       0 & 0 & 1 & 0 & 0 & \Delta t \\
                                       0 & 0 & 0 & 1 & 0 & 0 \\
                                       0 & 0 & 0 & 0 & 1 & 0 \\
                                       0 & 0 & 0 & 0 & 0 & 1 \\
                                       \end{bmatrix} \end{aligned}$                                     

The state transition equation in 3D space is:

$\begin{bmatrix}{p_x} \\ {p_y} \\{p_z} \\{v_x} \\{v_y} \\{v_z} \\ \end{bmatrix} =    \begin{bmatrix}1 & 0 & 0 & \Delta t & 0 & 0 \\ 
               0 & 1 & 0 & 0 & \Delta t & 0 \\
               0 & 0 & 1 & 0 & 0 & \Delta t \\
               0 & 0 & 0 & 1 & 0 & 0 \\
               0 & 0 & 0 & 0 & 1 & 0 \\
               0 & 0 & 0 & 0 & 0 & 1 \\
               \end{bmatrix} \begin{bmatrix}{p_x} \\ {p_y} \\{p_z} \\{v_x} \\{v_y} \\{v_z} \\ \end{bmatrix} + \begin{bmatrix}{v_{p_x}} \\ {v_{p_y}} \\{v_{p_z}} \\ {v_{v_x}} \\{v_{v_y}} \\{v_{v_z}} \\ \end{bmatrix}$

$Q = E[vv^t]  = \begin{bmatrix}0 & 0 & 0 & 0 & 0 & 0 \\ 
                                0 & 0 & 0 & 0 & 0 & 0 \\
                                0 & 0 & 0 & 0 & 0 & 0 \\
                                0 & 0 & 0 & E[v_{x}^2] & 0 & 0 \\
                                0 & 0 & 0 & 0 & 0 & 0 \\
                                0 & 0 & 0 & 0 & 0 & 0 \\
                                \end{bmatrix} $

![](result_img/step1_graph.png)

### 2. Do you see any benefits in camera-lidar fusion compared to lidar-only tracking (in theory and in your concrete results)?

### 3. Which challenges will a sensor fusion system face in real-life scenarios? Did you see any of these challenges in the project?

### 4. Can you think of ways to improve your tracking results in the future?
