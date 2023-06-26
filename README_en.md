# 6DOF-KFS
Kinematics Forward Solver of 6 Degrees of Freedom

## Theoretical Background
#### Concept
Kinematics Forward Solver, in simple terms, refers to determining the position and orientation of the end effector of a robotic arm given the values of the 6 joint variables. In other words, given $j_1 - j_6$, we aim to find $x, y, z, rx, ry, rz$.

#### DH Parameters
It is not possible to directly convert between joint values and Cartesian coordinates without considering the specific robot configuration. This relationship is described by the DH parameter table, which outlines the relationships between the coordinate systems of each joint.

***Table Content***  
Link Length (length): The distance between two adjacent joint axes.  
Link Twist (angle): The angle between two adjacent joint axes.  
Link Offset (d): The distance between the X-axes of two consecutive joint coordinate systems.

**Example: UR5e**

- DH Parameter Table

| Joint No. | Length (mm) | d (mm) | angle (deg) |
| --------- | ----------- | ------ | ----------- |
| 1         | 0           | 162.5  | 90          |
| 2         | -425        | 0      | 0           |
| 3         | -392.2      | 0      | 0           |
| 4         | 0           | 133.3  | 90          |
| 5         | 0           | 99.7   | -90         |
| 6         | 0           | 99.6   | 0           |

### Calculation
Based on the DH parameter table and given $j_1 - j_6$, we establish 6 joint matrices $A_1 - A_6$ and compute transformation matrices $T_1 - T_6$. By multiplying $A_1 - A_6$, we obtain the matrix R.

$R = \begin{bmatrix} \text{rot}_{3 \times 3} & P_{3 \times 1} \\ 0_{1 \times 3} & 1 \end{bmatrix}$

$P_{3 \times 1} = (x, y, z)^T$

Thus, finding R allows us to determine $x, y, z$. The joint matrix $A_i$ is derived from the current joint value $j_i$ and the DH parameters, where $\beta$ represents the current value of $j_i$, length represents $l$, d represents $d$, and angle represents $\alpha$.

$A_i = \begin{bmatrix} \cos(\beta) & -\sin(\beta)\cos(\alpha) & \sin(\beta)\sin(\alpha) & l\cos(\beta) \\ \sin(\beta) & \cos(\beta)\cos(\alpha) & -\cos(\beta)\sin(\alpha) & l\sin(\beta) \\ 0 & \sin(\alpha) & \cos(\alpha) & d \\ 0 & 0 & 0 & 1 \end{bmatrix}$

$R = A_1A_2A_3A_4A_5A_6$

Next, we calculate $rx, ry, rz$:

$rot_{3 \times 3} = \begin{bmatrix} r_{00} & r_{01} & r_{02} \\ r_{10} & r_{11} & r_{12} \\ r_{20} & r_{21} & r_{22} \end{bmatrix}$

$rx = \text{atan2}(r_{12}, r_{22})$

$ry = \text{atan2}(r_{02}, \sqrt{r_{00}^2 + r_{01}^2})$

$rz = \text{atan2}(r_{01}, r_{00})$

### Validation (UR5e)
j.txt: 57.3 57.3 57.3 57.3 57.3 57.3

Output:
| x          | y          | z           | rx          | ry        | rz        |
| ---------- | ---------- | ----------- | ----------- | --------- | --------- |
| 174.032973 | -75.257828 | -464.848688 | -106.158882 | 64.782997 | 67.592110 |

Compared with RoboDK C# API results:

| x     | y     | z      | rx     | ry   | rz   |
| ----- | ----- | ------ | ------ | ---- | ---- |
| 174.0 | -75.3 | -464.9 | -106.2 | 64.8 | 67.6 |

## References:
[https://blog.csdn.net/weixin_37942267/article/details/78806448](https://blog.csdn.net/weixin_37942267/article/details/78806448)  
[https://blog.csdn.net/u013039705/article/details/88894743](https://blog.csdn.net/u013039705/article/details/88894743)  
[https://zhuanlan.zhihu.com/p/259999988](https://zhuanlan.zhihu.com/p/259999988)

