# Transformation Matrix Computation For Tf AprilTag Example 

## Description 

April tag can be used in Improving Mobile robots Localization , so the robot after reaching certain pose system can start listen to tag tf with respect to camera tha to the map
then send the roobot command to move until aligning the baselink for example with the tag frame 

so the first step is to find the tf from the tag frame to the map frame

we will practice this in both code and calculations 


![Example Result](images/tf.gif)



## Given:

- **`mapTbase`** (Map to Base Link):

$$
T_{base}^{map} =
\begin{bmatrix}
1 & 0 & 0 & 1 \\
0 & 1 & 0 & 2 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1
\end{bmatrix}
$$

- **`baseTcam`** (Base Link to Camera):

$$
T_{cam}^{base} =
\begin{bmatrix}
0 & -1 & 0 & 0.5 \\
1 & 0 & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1
\end{bmatrix}
$$

- **`camTtag`** (Camera to Apriltag):

$$
T_{tag}^{cam} =
\begin{bmatrix}
1 & 0 & 0 & 1 \\
0 & 1 & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1
\end{bmatrix}
$$

## Objective:

Compute the transformation from `apriltag` to `map`:

$$
T_{apriltag}^{map} = T_{base}^{map} \cdot T_{cam}^{base} \cdot T_{tag}^{cam}
$$

## Steps:

1. **Compute \( T_{baseTcam} \cdot T_{camTtag} \):**

$$
T_{baseTcam} \cdot T_{camTtag} =
\begin{bmatrix}
0 & -1 & 0 & 0.5 \\
1 & 0 & 0 & 1 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1
\end{bmatrix}
\cdot
\begin{bmatrix}
1 & 0 & 0 & 1 \\
0 & 1 & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1
\end{bmatrix}
=
\begin{bmatrix}
0 & -1 & 0 & 1.5 \\
1 & 0 & 0 & 3.0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1
\end{bmatrix}
$$

2. **Compute \( T_{mapTbase} \cdot \left( T_{baseTcam} \cdot T_{camTtag} \right) \):**

$$
T_{mapTbase} \cdot \left( T_{baseTcam} \cdot T_{camTtag} \right) =
\begin{bmatrix}
1 & 0 & 0 & 1 \\
0 & 1 & 0 & 2 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1
\end{bmatrix}
\cdot
\begin{bmatrix}
0 & -1 & 0 & 1.5 \\
1 & 0 & 0 & 3.0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1
\end{bmatrix}
=
\begin{bmatrix}
0 & -1 & 0 & 1.5 \\
1 & 0 & 0 & 3.0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1
\end{bmatrix}
$$

## Result:

The transformation matrix from `apriltag` to `map` is:

$$
T_{apriltag}^{map} =
\begin{bmatrix}
0 & -1 & 0 & 1.5 \\
1 & 0 & 0 & 3.0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1
\end{bmatrix}
$$

## Translation

The translation vector is:

$$
\text{Translation} = [1.500, 3.000, 0.000]
$$

## Rotation

The rotation matrix \( R \) is:

$$
Rotation =
\begin{bmatrix}
0 & -1 & 0 \\
1 & 0 & 0 \\
0 & 0 & 1
\end{bmatrix}
$$

### RPY Angles (in degrees):

- **Roll**: 0.000
- **Pitch**: 0.000
- **Yaw**: 90.000
