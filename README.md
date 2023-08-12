# 6DOF-KFS
Kinematics Forward· Solver of 6 Degrees of Freedom

## 理论部分
#### 概念
运动学正解，简而言之，就是给出6个关节变量，求得机械臂末端的位置和姿态
即给出j<sub>1</sub> - j<sub>6</sub>​，求x,y,z,rx,ry,rz​

#### DH参数
只单一地给出关节值或直角坐标值，是不能直接互相转化的，还与具体的机器人有关，这部分有关的内容可以用DH参数表来表示，其描述了机器人各关节坐标系之间的关系

***表中内容***  
连杆长度 (length) ：2个相邻关节轴线之间的距离
连杆扭角 (angle) ：2个相邻关节轴线之间的角度
连杆偏距 (d) ：2个关节坐标系的X轴之间的距离

**eg：UR5e**
- DH参数表

|关节编号	|legth(mm)	|d(mm)	|angle(deg)|
|--|--|--|--|
|1	|0	|162.5	|90|
|2|	-425|	0|	0|
|3	|-392.2	|0|	0|
|4|	0|	133.3|	90|
|5|	0|	99.7|	-90|
|6|	0|	99.6|	0|
### 计算
根据DH参数表以及j<sub>1</sub> - j<sub>6</sub>，建立6个关节矩阵A<sub>1</sub>-A<sub>6</sub>计算出转换矩阵T<sub>1</sub>-T<sub>6</sub>，计算A<sub>1</sub>-A<sub>6</sub>相乘得到矩阵R

$R = \begin{bmatrix} \text{rot}_{3\times3} & P_{3\times1} \\ 0_{1\times3} & 1 \end{bmatrix}$

$P_{3 \times 1} = (x, y, z)^T$

则求出R即求出x,y,z关节矩阵$A_i$由当前的关节的$j_i$和DH参数导出，设当前$j_i$为$\beta$，legth为$l$，d为$d$，angle为$\alpha$

$A_i = \begin{bmatrix} \cos(\beta) & -\sin(\beta)\cos(\alpha) & \sin(\beta)\sin(\alpha) & l\cos(\beta) \\ \sin(\beta) & \cos(\beta)\cos(\alpha) & -\cos(\beta)\sin(\alpha) & l\sin(\beta) \\ 0 & \sin(\alpha) & \cos(\alpha) & d \\ 0 & 0 & 0 & 1 \end{bmatrix}$

$R=A_1A_2A_3A_4A_5A_6$

然后再求rx,ry,rz  
$rot_{3 \times 3} = \begin{bmatrix} r_{00} & r_{01} & r_{02} \\ r_{10} & r_{11} & r_{12} \\ r_{20} & r_{21} & r_{22} \end{bmatrix}$

$rx = arctan(r[1][2], r[2][2])$

$ry = arctan(r[0][2], \sqrt{r[0][0] ^2 + r[0][1]^2})$

$rz = arctan(r[0][1], r[0][0])$

### 验证(UR5e)
j.txt: 57.3 57.3 57.3 57.3 57.3 57.3

输出：
|x     | y  |  z  |rx  |ry | rz|
|--|--|--|--|--|--|
|174.032973| -75.257828| -464.848688| -106.158882|64.782997| 67.592110|

robodk C#API结果：

|x     | y  |  z  |rx  |ry | rz|
|--|--|--|--|--|--|
|174.0| -75.3| -464.9| -106.2|64.8| 67.6|

## 参考资料：
[6轴机器人运动学正解,逆解1](https://blog.csdn.net/weixin_37942267/article/details/78806448?spm=1001.2014.3001.5502)  
[机器人导论 学习笔记2 - 运动学（正解](https://blog.csdn.net/u013039705/article/details/88894743)  
[欧拉角，四元数，旋转矩阵相互转化（c++, python)](https://zhuanlan.zhihu.com/p/259999988)
