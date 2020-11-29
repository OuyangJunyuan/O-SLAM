

# 视觉SLAM框架

# 前端VO

两个在优化中很重要的导数，一个是变换后像素对李代数导数，一个是变换后三维坐标对李代数的导数。

## 特征点法——特征提取提取

在其他SLAM中又叫路标，在视觉SLAM中路标就是位置xi时候图像Ii里的特征点测量量像素坐标Zi。

图片特征有特征点、特征线、特征面。其中特征点最稳定和好得到。



一般说起特征点(多为角点)，就包含两个部分：关键点u,v和描述子des(u,v)。关键点u,v是图片中关键点的坐标。描述子是对关键点周围像素信息的描述。

好的特征点应该具有几个属性：

* 可重复性：相同的区域可以在不同图片中 找到
* 可区别性：不同的区域在同一个图片中有不同的描述
* 高效性：特征点远远小于图片总像素大小
* 本地性：特征只与一小片图像区域有关

此外，我们希望特征有：

* 尺度不变：否则远处是关键点，近处看就不是关键点了。

* 旋转不变形：显然。



常用特征点：

* SIFT：尺度不变特征变换。对光照、尺度、旋转等变化具有适应能力。但是计算量巨大。5000ms
* SURF：速度比上面快。200ms
* ORB：旋转不变形FAST与旋转BRIEF描述子。15ms
* FAST：贼快，但是没有尺度和旋转不变形。认为灰度变化剧烈的点为角点



ORB：先进行FAST角点提取(周围半径3圆周上的16个像素点，判断是否大量像素点连续亮度高)。计算几何重心和灰度质心方向。计算BREIF描述子。



综上：特征提取与匹配有几个通用步骤

* 两幅图像单独提取特征点

* 两幅图像单独对自己每个特征点计算对应描述子

* 对特征进行匹配(可以认为描述子欧式距离近的为相识度)

* 对以匹配点进行赛选，得出max+0.2*(max-min)或 max(2\*min,30)距离之内的点作为良好匹配点。

  （对于错匹配可以用**RANSAC**算法进行剔除，否则无法认为描述子距离小就是相近，且不同算法中需要各自优化）

  注释：RANSAC主要解决样本中的外点问题，最多可处理50%的外点情况。

  * ​	https://zhuanlan.zhihu.com/p/45532306
  * ​    https://blog.csdn.net/robinhjwy/article/details/79174914
  * ​    https://www.jianshu.com/p/4e426f0c48cc

* 良好匹配点就是 路标在两幅图像中的不同像素坐标。



### 估计相机运动

可以参考：https://blog.csdn.net/xiaoxiaowenqiang/article/details/79278884

已有匹配好的两帧之间的点对，就可以进行两帧相机的姿态估计。

* 2D像素点对(单目)
  
  对极几何：求两帧之间的相机运动，存在初始化，纯旋转，尺度不定的问题
  
  * 本质矩阵基础矩阵——八点法：归一化平面坐标与归一化平面坐标关系
  * 单应矩阵——3D点共面——四点法：像素坐标与像素坐标关系
  
* 3D点对(rgb-d)

  ICP：一般认为ICP估计变换Isometry是最准的，因为他利用到了最多的(3D)信息。

* 3D(世界坐标)+2D对：多用在单目初始化后，第一二帧的尺度确定，第二帧中3D点相机坐标配合三角测量确定。之后就用新帧的2D点配对旧一帧的3D做PnP

  如3D点为世界坐标值则得到的R,t是相机姿态。

  如果3D点是第一帧的像极坐标系下坐标，则R,t是相机运动(第一帧与第二帧之间的位姿)。

  * nPn：求解世界坐标系下3D点在归一化坐标系下(或像素坐标系)的坐标。进而求解相机位姿。

  * BA优化：已知世界坐标系下3D点，按估计相机姿态重(虚拟)投影回像素坐标值，对误差平方进行最小化来得到优化的相机姿态估计

#### 2d2d对极几何——本质矩阵

极平面、极点、极线、基线等名词。
$$
s_1p_1=KP    \\ s_2p_2=K(RP+t)\\x为归一化平面上的点，式子在乘任意非零常数下相等。\longrightarrow:\\x_2=Rx_1+t,
$$

$$
左右同时叉积t:t\hat{ }x_2=t\hat{}(Rx_1+t)=t\hat{}Rx_1,
\\左右同时内积x_2:x_2^Tt\hat{ }x_2=0=x_2^Tt\hat{}Rx_1
\\带入x_2与x_1关系式\\
对极约束：p_2^TK^{-T}t\hat{}RK^{-1}p_1=0\\
记本质矩阵(包含旋转和平移)E=t\hat{}R\\
记基础矩阵F=K^{-T}EK^{-1}\\
于是：\\
x_2^TEx_1=0\\
p_2^TFp_1=(p_2^TK^{-T})E(K^{-1}p_1)=0
$$




由于K可得知，因此我们讨论本质矩阵E性质：

* E是在对极约束为0等式的约束下，所以E乘任意整数后都是对极约束依旧成立，所以**<u>不同尺度下的E具有等价性</u>**。

* 本质矩阵的奇异值必定是：3x3矩阵是本质矩阵充要条件是前两奇异值相等，第三个奇异值等于0：
  $$
  [\sigma ,\sigma,0]^T
  $$
  奇异值介绍：

   <img src="https://bkimg.cdn.bcebos.com/pic/95eef01f3a292df533a5239eb6315c6035a873d3?x-bce-process=image/resize,m_lfit,w_268,limit_1/format,f_jpg" alt="img" style="zoom:50%;" />

  * https://zhuanlan.zhihu.com/p/36546367

* 包含3D旋转和平移又包含一个尺度等价性，**<u>故自由度为3+3-1=5</u>**。





**<u>求解E</u>**：E为五自由度，因此至少需要5个点对构成5个等式来求解。

但是E的内在属性对求解有很大约束。因此我们一般可以只考虑其尺度等价性，用8点对来估计E——经典的**<u>八点法</u>**。且是线性的等式可以线性代数框架下求解。

八点法：

* https://blog.csdn.net/kokerf/article/details/72630863
* https://github.com/opencv/opencv/blob/3.1.0/modules/calib3d/src/fundam.cpp#L548
* https://blog.csdn.net/WeskerXR/article/details/105651865



算法过程：p'是和p是一组匹配点的**<u>归一化坐标</u>**
$$
(u_1,v_1,1) 
\begin{bmatrix} 

e1& e2 &  e3\\e4&e5 &e6\\e7 &e8& e9  

\end{bmatrix}

\left(
\begin{matrix} 

u_1'\\v_1'\\1

\end{matrix}
\right)

=0
$$


写成矩阵形式
$$
(u_1u_1',u_1v_1',u_1,v_1u_1',v_1v_1',v_1,u_1',v_1',1)·e=0
$$
合并n=8个点的方程为
$$
线性方程组Ax=
\left(
\begin{matrix} 

u_1u_1'&u_1v_1'&u_1&v_1u_1'&v_1v_1'&v_1&u_1'&v_1'&1\\
u_2u_2'&u_2v_2'&u_2&v_2u_2'&v_2v_2'&v_2&u_2'&v_2'&1\\
.&.&.&.&.&.&.&.&.\\
.&.&.&.&.&.&.&.&.\\
u_nu_n'&u_nv_n'&u_n&v_nu_n'&v_nv_n'&v_n&u_n'&v_n'&1\\
\end{matrix}
\right)
\left(
\begin{matrix} 

e_1\\
e_2\\
e_3\\
e_4\\
e_5\\
e_6\\
e_7\\
e_8\\
e_9\\

\end{matrix}
\right)
=0
$$
n=8时，矩阵A是8x9的如果满秩rank=8，则显然A的零空间是1维的是一条直线，与E尺度等价性一致。

如果n大于9可以使用**<u>最小二乘</u>**来估计E。



从E中恢复R、t：使用SVD：
$$
E=U\sum V^T，可以调整为\Sigma=[1\ \  1\  \ 0或[\cfrac{\sigma_1+\sigma_2}{2}\ \ \cfrac{\sigma_1+\sigma_2}{2} \ \ 0]\\
t_1=UR_Z(\pi/2)\sum U^T,
t_2=UR_Z(-\pi/2)\sum U^T\\
R_1=UR^T_Z(\pi/2)V^T,R_2=UR^T_Z(-\pi/2)V^T\\
$$
由于E和-E等价，故有4种可行解。

<img src="https://images2015.cnblogs.com/blog/532915/201704/532915-20170404171300738-530280294.png" alt="image" style="zoom:50%;" />

只需要带入一个点，判断在两个相机下的z都是正的，则该R和t就是对的。

#### 2d2d对极集合——单应矩阵

如果匹配的点对，是同一个平面上的多个点在不同相机下的投影，则可以单应矩阵描述。

同一个平面上的点$P$满足：同一个平面上点在其所在平面法向量上的投影值为常数且相同
$$
n^TP+d=0
$$
又由于在相差一个尺度的意义下有如下等式：$p_2为第二帧像素齐次坐标,P_1是空间坐标$
$$
p_2=K(RP_1+t)\\
p_2=K(RP_1+t*(-\cfrac{n^TP_1}{d}))\\
p_2=K(R-\cfrac{tn^T}{d})P_1\\
p_2=K(R-\cfrac{tn^T}{d})K^{-1}p_1\\
p_2=Hp_1
$$
其中H也是在3x3的矩阵，且等式是在乘不为零常数下成立的。故H自由度最多为8，**<u>因此零H最有一个值为1</u>**。
$$
\left(
\begin{matrix}
u_2\\v_2\\1
\end{matrix}
\right)
=
\left(
\begin{matrix}
h_1&h_2&h_3\\
h_4&h_5&h_6\\
h_7&h_8&1
\end{matrix}
\right)
\left(
\begin{matrix}
u_1\\v_1\\1
\end{matrix}
\right)
$$
一对点对可得2个约束方程故8自由度只需要4点法：
$$
h_1u_1+h_2v_1+h_3-h_7u_1u_2-h_8v_1u_2=u_2\\
h_4u_1+h_5v_1+h_6-h_7u_1v_2-h_8v_1v_2=v_2\\
\ \\

\left(
\begin{matrix}
u_1&v_1&1&0&0&0&-u_1u_2&-v_1u_2\\
0&0&0&u_1&v_1&1&-u_1v_2&-v_1v_2\\
.&.&.&.&.&.&.&.\\
.&.&.&.&.&.&.&.
\end{matrix}
\right)

\left(
\begin{matrix}
h_1\\h_2\\h_3\\h_4\\h_5\\h_6\\h_7\\h_8
\end{matrix}
\right)
=

\left(
\begin{matrix}
u_2\\
v_2\\
.\\
.\\
.\\
.\\
.\\
.\\
\end{matrix}
\right)
$$

单应矩阵恢复R,t方法也差不多和上面一样，具体可以看

* 数值法：
  * 《Motion and structure from motion ina piecewise planar environment》
  * 《3d reconstruction based on homography mapping》
* 解析法：
  * 《deeper understanding of the homography decomposition for vision-based control》

#### 3d-2d——PnP

pnp->perspective n point 求解3D到2D点对运动的方法。

<u>描述了当知道n个3D空间点**世界坐标**和其投影2D点**归一化平面齐此坐标**时，求解相机坐标系下的3D坐标。求相机坐标系下3D坐标后**再用3D3D点对进行恢复Rt**</u>

一般这里的点是地图上固定的点，且已经直到其世界坐标了，否则RGB-D也得不到世界坐标系下的3D点呀是相机坐标系下的。

最少只需要3个点对就可以实现估计相机运动。

在双目和RGB-D相机可以直接得到3D点，所以可以直接使用PnP，而单目视觉里程计需要初始化后才能使用PnP。

常用方法：P3P、DLT(直接线性变换法)、EPnP(效率PnP)、UPnP、Bundl Ajustment



---

直接线性变换法：看书158页。比较简单，就是列坐标变换矩阵式子，展开成分量式子，整理未知数为向量形式进行最小二乘求解。





---

P3P：缺点在于只能利用到3个点的信息，多余的点很难利用到。所以出现了一些EPnP uPnP迭代优化消除噪声影响。

大写ABC是空间中3个点的世界坐标，abc是图像前归一化平面上的齐次坐标

<img src="https://img-blog.csdnimg.cn/2019050515493252.png" alt="img" style="zoom:67%;" />

对O-ABC四面体三棱锥的3个侧面，以底边为第三遍进行余弦定理
$$
OA^2+OB^2-2OA·OB·\cos<a,b>-AB^2=0\\
OB^2+OC^2-2OB·OC·\cos<b,c>-BC^2=0\\
OA^2+OC^2-2OA·OC·\cos<a,c>-AC^2=0
$$
上式除$OC^2$，带1式入2,3式子，令
$$
x=\frac{OA}{OC}\\
y=\frac{OB}{OC}\\
v=\frac{AB^2}{OC^2}\\
u=\frac{BC^2}{AB^2}\\
w=\frac{AC^2}{AB^2}\\

得到\\
(1-u)y^2-ux^2-cos<b,c>y+2uxycos<a,b>+1=0\\
(1-w)x^2-wy^2-cos<a,c>x+2wxycos<a,b>+1=0\\
由于ABC世界坐标系已知,abc在归一化坐标系下已知：\\
u，w，cos<·,·>已知\\
x,y未知且随着相机的位姿变化而变化
$$
故上式是$x,y$的二元二次方程。
$$
ay^2-cx^2-dy+exy+1=0\\
fx^2-gy^2-hx+ixy+1=0
$$

---

Bundle Ajustmen(BA优化)t：是一个**<u>最小化重投影误差的问题</u>**，定义在李代数上的最小二乘问题，将相机姿态和空间点位置都作为优化变量。甚至可以直接把2帧的特征点都直接丢进去，不知道3D坐标和变换，直接优化求解得到3D坐标和相机之间变换。

一般用于对PnP和ICP结果进行优化。可以用对极几何求解两帧之间R,t，三角测量得到归一化坐标系中特征点坐标，用BA求解两帧之间相机运动的优化。

也可以是世界坐标下的特征点坐标与相机位姿的优化。



​		已知n个空间点世界坐标$P_i$与其像素坐标$u_i$。
$$
s_i
\left(
\begin{matrix}
u_i\\v_i\\1
\end{matrix}
\right)
=
K\exp(\xi\hat{})
\left(
\begin{matrix}
X_i\\Y_i\\Z_i\\1
\end{matrix}
\right)
$$


构建最小二乘问题，$P点作为特征点像素坐标为u_i,其在相机位姿估计值\xi 下的虚拟(重)投影值u_i'=K\exp(\xi\hat{})P_i/s_i，误差项u_i-u_i'最后一维是1，故是二维误差项$
$$
\xi^*=\arg\  \min_\xi \cfrac{1}{2}\sum_{i=1}^n\| u_i-u_i'\|^2_2=\arg\  \min_\xi \cfrac{1}{2}\sum_{i=1}^n\| e(x_i)\|^2_2
$$


我们需要知道误差关于优化变量的导数，这样就可以用无约束优化用高斯牛顿、列文伯格-马夸尔特方法求解。

即列出增量式方程：$e(x+\Delta x)\approx e(x)+J\Delta x$也就是一阶线性化

自变量是6维，因变量是2维。故其中$J$是雅克比矩阵，维度是2x6。 
$$
P'=K\exp(\xi\hat{})P_{1:3}=[X',Y',Z']^T\\
\ \\
u_i-u_i'=u_i-f_x\frac{X'}{Z'}+c_x\\
v_i-v_i'=v_i-f_y\frac{Y'}{Z'}+c_y\\

而\\
\cfrac{\part e}{\part \delta\xi}=\lim_{\delta \xi\to0}\cfrac{e(\delta \xi \oplus\xi )}{\delta\xi}=\cfrac{\part e}{\part P'} \cfrac{\part P'}{\part \delta \xi}
\\
\cfrac{\part e}{\part P'}=-
\begin{bmatrix}
\cfrac{\part u}{\part X'}&\cfrac{\part u}{\part Y'}&\cfrac{\part u}{\part Z'}\\
\cfrac{\part v}{\part X'}&\cfrac{\part v}{\part Y'}&\cfrac{\part v}{\part Z'}
\end{bmatrix}_{2\times6}
=-
\begin{bmatrix}
\cfrac{f_x}{Z'}&0&-\cfrac{ f_xX'}{ Z'^2}\\
0&\cfrac{f_y}{Z'}&-\cfrac{ f_yY'}{ Z'^2}
\end{bmatrix}_{2\times6}
\\
\cfrac{\part (TP)}{\part \delta\xi}=(TP)^\odot=
\begin{bmatrix}
I&-P'^\hat{}\\
 0^T &0^T
\end{bmatrix}_{6\times6}\\

J=\cfrac{\part e}{\part \delta \xi}=
\begin{bmatrix}
\cfrac{ f_x}{Z'}&0&-\cfrac{\ f_xX'}{Z'^2}&-\cfrac{ f_xX'Y'}{Z'^2}&f_x+\cfrac{f_xX^2}{ Z'^2}&-\cfrac{f_x Y'}{Z'}\\
0&\cfrac{ f_y}{Z'}&-\cfrac{ f_yY'}{Z'^2}&-f_y-\cfrac{f_yY^2}{ Z'^2}&\cfrac{ f_yX'Y'}{Z'^2}&-\cfrac{ f_yX'}{Z'}
\end{bmatrix}_{2\times6}
$$


而优化点空间点的位置时需要求得误差对于世界坐标P的导数
$$
\frac{\part e}{\part P}=\frac{\part e}{\part P'}\frac{\part P'}{\part P}\\
\cfrac{\part e}{\part P'}=-
\begin{bmatrix}
\cfrac{f_x}{Z'}&0&-\cfrac{ f_xX'}{ Z'^2}\\
0&\cfrac{f_y}{Z'}&-\cfrac{ f_yY'}{ Z'^2}
\end{bmatrix}_{2\times6}
\\
\cfrac{\part P'}{\part P}=\part(RP+t)/\part P=R
$$
**<u>以上两个导数可以指导迭代时候的梯度方向，使得得到局部最小值。</u>**



可以参考：

ba

* https://www.cnblogs.com/Jessica-jie/p/7739775.html

g20

* https://www.cnblogs.com/gaoxiang12/p/5304272.html

<img src="https://upload-images.jianshu.io/upload_images/15852708-693e77bf73da3445.png?imageMogr2/auto-orient/strip%7CimageView2/2/w/1240" alt="image" style="zoom:50%;" />

由图中我们可以看出，我们需要
* 构建一个稀疏优化器(sparseOptimizer),起始就是构建一起超图，这个图中有一些边和定点。
  * 为迭代策略设置矩阵求解器(BlockSolver)
   * 选择迭代策略(GN,LM,DG)
```c++
/* --- 创建优化(图) --- */
g2o::SparseOptimizer    optimizer;
/* --- 创建矩阵求解器 --- */
//设置矩阵求解器求解方法
 g2o::BlockSolver_6_3::LinearSolverType* linearSolver = new  g2o::LinearSolverCholmod<g2o::BlockSolver_6_3::PoseMatrixType> ();
//创建矩阵求解器
g2o::BlockSolver_6_3* block_solver = new g2o::BlockSolver_6_3( linearSolver );
/* --- 迭代方法 --- */
//构建矩阵求解器构建迭代方法(迭代时候求解矩阵的方法)
 g2o::OptimizationAlgorithmLevenberg* algorithm = new g2o::OptimizationAlgorithmLevenberg( block_solver );
/* --- 设置优化器(图)的算法(迭代算法) --- */
optimizer.setAlgorithm( algorithm );
```



在视觉SLAM的BA中用到以下节点和边

* 结点1：相机位姿结点：g2o::VertexSE3Expmap，来自<g2o/types/sba/types_six_dof_expmap.h>；
* 结点2：特征点空间坐标结点：g2o::VertexSBAPointXYZ，来自<g2o/types/sba/types_sba.h>；
* 边：重投影误差：g2o::EdgeProjectXYZ2UV，来自<g2o/types/sba/types_six_dof_expmap.h>；

书上的P167例题是两帧之间的优化，默认第一帧位姿全为0，所以不考虑，只有一个节点1。

而参考博客中用了两个位姿节点，是考虑到第一帧不是世界坐标的情况。

```c++
//总结： g2o::VertexSE3Expmap* v = new g2o::VertexSE3Expmap();
// 然后：  v->setId(i);
//然后：   v->setEstimate( g2o::SE3Quat() );
//然后： 添加到图中 optimizer.addVertex( v );

//  书上
       g2o::VertexSE3Expmap* v = new g2o::VertexSE3Expmap();
        v->setId(i);
        if ( i == 0)
            v->setFixed( true ); // 第一个点固定为零
        // 预设值为单位Pose，因为我们不知道任何信息
        v->setEstimate( g2o::SE3Quat() );
        optimizer.addVertex( v );

//博客 
  for ( int i=0; i<2; i++ )
    {
        g2o::VertexSE3Expmap* v = new g2o::VertexSE3Expmap();
        v->setId(i);
        if ( i == 0)
            v->setFixed( true ); // 第一个点固定为零
        // 预设值为单位Pose，因为我们不知道任何信息
        v->setEstimate( g2o::SE3Quat() );
        optimizer.addVertex( v );
    }
```

特征点节点

```c++
for ( size_t i=0; i<pts1.size(); i++ )
    {
        g2o::VertexSBAPointXYZ* v = new g2o::VertexSBAPointXYZ();
        v->setId(1+i );
        v->setEstimate( Eigen::Vector3d(pts1[i].x,pts1[i].y,pts1[i].z) );
        v->setMarginalized(true);
        optimizer.addVertex( v );
    }

    // 准备相机参数
    g2o::CameraParameters* camera = new g2o::CameraParameters( fx, Eigen::Vector2d(cx, cy), 0 );
    camera->setId(0);
    optimizer.addParameter( camera );
```

准备边(成像)

```c++
// 第二帧到每个特征点
for ( size_t i=0; i<pts1.size(); i++ )
{
    g2o::EdgeProjectXYZ2UV*  edge = new g2o::EdgeProjectXYZ2UV();
    edge->setVertex( 0, dynamic_cast<g2o::VertexSBAPointXYZ*>   (optimizer.vertex(i+1)) );
    edge->setVertex( 1, dynamic_cast<g2o::VertexSE3Expmap*>     (optimizer.vertex(0)) );
    edge->setMeasurement( Eigen::Vector2d(p2d[i].x, p2d[i].y ) );
    edge->setParameterId(0, 0);
    edge->setInformation( Eigen::Matrix2d::Identity() );//协方差之逆

    optimizer.addEdge( edge );
}
```
启动优化

```c++
optimizer.setVerbose(true);//打印优化详情
optimizer.initializeOptimization();//初始化图优化
optimizer.optimize(100);//迭代次数

//输出结果。由于都是动态申请的，没有销毁。且优化是传址的，可以在优化完后直接访问其估计值。
Eigen::Isometry3d( pose->estimate() ).matrix();
```



#### 3d-3d——ICP

ICP(iterative closet point，迭代最近点)，用在已知两帧图像中分别配对的相机坐标系(pnp的3D是世界坐标系下的)下3D坐标(RGB-D相机)。

起始一般是用在两朵不同位形下点云的变换矩阵Isometry计算。

一般认为ICP估计变换Isometry是最准的，因为他利用到了最多的(3D)信息。

---

线性代数求解法：SVD

看书，很简单，直接建立 3D点经过变换后的误差平方和得到优化变量，最小化这个优化变量即可，而且

![image-20200925002849225](/home/ou/.config/Typora/typora-user-images/image-20200925002849225.png)

经过去质心距离计算来减小优化问题，最终可以变成比较简单的问题，通过SVD得到解析解。

---

非线性优化方法：

用李代数表示位姿后进行无约束非线性优化。由于每项误差对位姿的导数已经推导过了

![image-20200925003231594](/home/ou/.config/Typora/typora-user-images/image-20200925003231594.png)

可以证明，ICP问题是有全局最优值的，所以初值可以乱选。

但是这是在点对已经预先匹配好的情况，在RGB-D中常见。

**<u>但是有时候有的点RGB-D测不到深度，就需要结合ICP和PNP了，对于未知深度点，用3D-2D重投影。对于已知深度的，用3D-3D ICP。于是，所有误差都可以放到图优化中了。</u>**

但g2o没有3D-3D的边所以需要自己定义一个这样的边，并实现其求导的接口











### 估计路标空间位置

对于单目相机，得到相机的运动后，需要依靠两张相关图片进行三角测量才能得到像素的深度信息。

#### 三角测量

假设图像$I_1$中像素点$p_1，光心O_1$.$I_2$中像素点$p_2，光心O_2$。理论上$O_1p_	1与O_2p_2相交于空间中的一点P即路标的空间位置，而P在两图中的归一化平面梭镖为x_1,x_2$
$$
z_2x_2=z_1Rx_1+t\\

为了求解z_1可以两边乘x_2\hat{}\\
得\ \ \ 0=z_1x_2\hat{}Rx_2+x_2\hat{}t\\
求得s_2后带入方程可得s_1。\\
注意：由于噪声，我们一般不取0解，而是最小二乘解。
$$


三角化的矛盾：**<u>没有平移量就没有三角，对极约束永远满足。平移量太小，则三角化精度太低，平移量太大，导致匹配失败</u>**。







 

## 直接法



特征点法有自身固有缺点

* 需要计算特征点和匹配特征点，占用SLAM性能60%时间。
* 使用特征点，丢掉了其他万千像素点的信息，只能构建稀疏地图。
* 特征点在特征少的地方比如白墙时可能无足够特征点来计算相机运动。
* 特征点法具误匹配处理的问题。而光流法一般只有跟丢的问题，不存在误匹配。

改进

* 计算关键点，但是不计算描述子，用光流法(光流跟踪帧间像素对应位置)，在用对极几何、PnP、ICP求相机运动。
* 计算关键点，但是不计算描述子，而是使用直接法。
* 不计算关键点，也不计算描述子，直接依靠像素灰度信息来得出相机运动。

其中

* 第一种为光流法，和特征点法一样，依靠重投影误差优化。
* 第二三种为直接法，依靠最小光度误差来优化。



根据使用的像素，直接法还分为

* 稀疏：依靠稀疏关键点，只能稀疏重构
* 半稠密：只用部分关键点，由下面介绍的误差对位姿雅克比可知，对于梯度为0的点对计算增量运动没有贡献。所以一般取带有梯度的像素点，比如边沿点。
* 稠密：计算所有像素点(几十万几百万个)，需要使用GPU加速。由于梯度为零的点对计算贡献不大，在重构的时候位置也难以估计。



下面介绍一下光流法

### 光流Optical Flow

只要有灰度变化的地方就可以使用光流，而不必具有梯度(渐变的也可)。

根据计算的像素个数，计算部分像素为稀疏光流、全部像素为稠密光流



其中稀疏光流以Lucas-Kanade (LK)光流最为有名。

下面介绍一下光流的原理。

首先有3个假设：

* 灰度不变假设：同一个物体在不同时间出现在不同图片位置时他的灰度是一样的。
* 微小运动假设：物体在图片中的位置变化不大。
* LK光流的假设——同质性：同一个局部窗口中的像素运动方向一致。

这样就可以得出
$$
I(x+dt,y+dy,t+dt)=I(x,y,z)+\frac{\partial{I}}{\partial{x}}dx+\frac{\partial{I}}{\partial{y}}dy+\frac{\partial{I}}{\partial{t}}dt\\
由于灰度不变故 \frac{\partial{I}}{\partial{x}}dx+\frac{\partial{I}}{\partial{y}}dy+\frac{\partial{I}}{\partial{t}}dt=0\\
\frac{\partial{I}}{\partial{x}}\frac{dx}{dt}+\frac{\partial{I}}{\partial{y}}\frac{dy}{dt}=-\frac{\partial{I}}{\partial{t}}\\
\frac{dx}{dt}\triangleq u,\frac{dy}{dt}\triangleq v,\triangledown I=I_x+I_y+I_t\\
I_xu+I_yv=-I_t\\

写得紧凑一点\\
[I_x,I_y]
\begin{bmatrix}
u\\
v
\end{bmatrix}
=-I_t
$$
由于这个方程是是二元一次的，所以无法求解u,v。需要更多约束。

于是LK光流的解法就是假设局部窗口w*w大小窗口内的$w^2$个像素都具有相同运动速度。
$$
\begin{bmatrix}
[I_{x1},I_{y1}]\\
...\\
[I_{xk},I_{yk}]
\end{bmatrix}



\begin{bmatrix}
u\\
v
\end{bmatrix}
=-\begin{bmatrix}
I_{t1}\\
...\\
I_{tk}
\end{bmatrix}\\
这是一个超定方程，A\begin{bmatrix}
u\\
v
\end{bmatrix}=-b常用最小二乘解\\
\begin{bmatrix}
u\\
v
\end{bmatrix}=-(A^TA)^{-1}A^Tb
$$




OpenCV中有可以直接调用的API——calcOpticalFlowPyrLK(...)



### 直接法



假设P的世界坐标(以第一帧为参考)为$[X,Y,Z]$则它在两个相机上的成像对应非齐次像素坐标为$p_1,p_2$

我们要求取第一个相机到第二个相机的相对位姿变换$[R,t] \longrightarrow \xi$，相机内参为$K$。
$$
p_1=
\begin{bmatrix}
u\\v\\1
\end{bmatrix}_1
=\frac{1}{Z_1}KP\\
p_2=
\begin{bmatrix}
u\\v\\1
\end{bmatrix}_2
=\frac{1}{Z_2}K(RP+t)=\frac{1}{Z_2}K(exp(\xi\hat{} )P)_{1:3}
$$
由于特征点法中，我们直到p1和p2是同一个物体在两个相机下的成像，且可以配对得到两者的像素坐标，因此可以计算重投影误差。

而直接法中没有特征检测和匹配，我们无法直到p1和哪个p2配对。

**直接法的思路**：构建一个优化问题，已知第一帧下3D点及其在第一帧下投影像素坐标，优化所求T位姿使得此3D点在第二帧下投影处亮度和第一帧投影处亮度差异最小，使得寻找到的p2和p1更相似(灰度不变假设)，这就是光度误差(像素亮度的差)。
$$
e=I_1(p_1)-I_2(p_2)\\
\min_{\xi} J(\xi)=\sum_{i=1}^{N}e_i^Te_i,e_i =e=I_1(p_{1,i})-I_2(p_{2,i})
$$


为了求解这个问题，我们需要直到$e$是如何随着$\xi$变化的，即分析他们的导数关系。

对$\exp(\xi)$左乘$扰动\exp(\delta\xi)$
$$
e(\xi\oplus\delta\xi)=I_1(\frac{1}{Z_1}KP)-I_2(\frac{1}{Z_2}K\exp(\delta\xi\hat{})\exp(\xi\hat{})P)\\
\thickapprox I_1(\frac{1}{Z_1}KP)-I_2(\frac{1}{Z_2}K(1+\delta\xi\hat{})\exp(\xi\hat{})P)\\
=I_1(\frac{1}{Z_1}KP)-I_2(\frac{1}{Z_2}K\exp(\xi\hat{})P+\frac{1}{Z_2}K\delta\xi\hat{}\exp(\xi\hat{}))\\
q\triangleq\delta\xi\hat{}\exp(\xi\hat{})\\
u\triangleq \frac{1}{Z_2}Kq	\ \ \ 是扰动在第二个相机下的像素坐标。
$$



对其Taylor展开

这里留个坑，仔细去看李扰动模型P76 P111。

* 这里一直不理解为何要求$\frac{\partial q}{\partial \delta\xi}$而不是$\frac{\partial q}{\partial \xi}$。重温后发现，高斯牛顿法这种优化算法都是需要求目标函数对$\Delta x$的导数的，而不是求对$x$的导数。
  * 两种想法：1.是用对$\delta\xi $求导来近似对李代数求导，这样方便计算。2.是因为是对自变量增量$\delta\xi$求导而不是$\xi$求导
* 当然如果能求对$x$的导数的话，对$\Delta x$求导结果可以用$\frac{df}{dx}(对x求导)表示$。但是在李代数中对$\xi $求导需要计算左或右雅克比，比较麻烦。

$$
e(\xi\oplus\delta\xi)=e(\xi)+\triangledown_\xi\xi\\
=e(\xi)+\frac{\partial e(\xi)}{\partial\delta\xi}\delta\xi \\
而\frac{\partial e(\xi)}{\partial\delta\xi}=-\frac{\partial I_2}{\partial u}\frac{\partial u}{\partial q}\frac{\partial q}{\partial \delta\xi}
$$



* 稀疏直接法：对参考帧进行关键点识别，然后在后续帧中直接法跟踪关键点。

  <img src="/home/ou/.config/Typora/typora-user-images/image-20200926174750942.png" alt="image-20200926174750942" style="zoom:33%;" />

* 半稠密直接法：对参考帧中梯度大于某阈值(边沿点)的像素进行直接法跟踪。

  <img src="/home/ou/.config/Typora/typora-user-images/image-20200926174842918.png" alt="image-20200926174842918" style="zoom:33%;" />

#### 直接法的讨论

直接法是依靠图像的梯度来引导位姿李代数增量的。比如$P点在I_1$中像素点亮度比较高，而在初始位形$\xi下P点在I_2中的灰度变低了$。那么计算$I_2中对应的这个像素点附近的梯度$，使得更新量$\exp(\delta\xi)$后，在$I_2$中的像素往更亮的地方移动。如果综合许多像素点对位姿增量的建议，那么可以想象，最终期望误差会不断下降并收敛。



但是并不是沿着梯度方向就一定能找到最优值。因为图片是具有强非凸性的函数。**只有相机运动很小的时候，最优值就在初始位置附近的时候梯度是可靠的，如果运动太大，最优值可能在隔壁山谷里。**这里的微小运动假设和光流中的原因是不同的（为了偏导=0），不要搞混了。



由于上述描述是对单一像素进行计算误差的，但是单一像素并没有什么特点，附近可能有好多和他很像的。**所以后续我们会说用小的图块(patch)，计算他们的归一化相关性(NCC——没错，就是模板匹配中的那个NCC)，这个NCC不但不收线性光照变化影响**，而且可以表示一小块区域和一小块区域的相似性，由于微小运动假设，相机运动后，这些小块可以认为内部像素分布结构(外形)是不变的，比如字母A还是字母A。那么就可以用小图块之间的相似性来代替前面的像素灰度差异了。这是更鲁棒的做法。



直接法的优点很多：不计算特征点描述子、省时间。只需要像素有变化就可以，不需要有特征点。可以构建半稠密或稠密地图。

缺点也有：图像具有强非凸性，容易陷入局部极小值，只能假设微小运动。单个像素区分度不大，往往像素对相机运动的改变建议不同。灰度不变是很强的假设，以上说的简单的直接法求光度模型是无法在自动曝光的相机下运作的。







### VO

#### VO中的数据结构

* Camera：
  * 相机内参、深度信息尺度。
  *  
  * 定义构造函数、智能指针本类类型。
  * 世界、相机、像素坐标系的两两变换共六个。

* Frame：一帧RGB-D数据，包含
  * id、时间戳
  * 相机、世界到相机的变换(世界系在相机系中的位形)
  * 彩色图、深度图
  *  
  * 通过以上3行属性构造Frame的构造方法
  * 定义本类的智能指针，用工厂函数来动态创建并通过智能指针管理
  * 查询深度、获取本帧时相机中心在世界坐标下位置、判断一个3D点是否在当前帧视野中。

* MapPoint：特征点
  * id、描述
  * 世界坐标系下位置、到相机的角度
  * 该特征点被匹配的次数、观测的次数
  *  
  * 构造函数、本类智能指针定义
  * 工厂函数
* Map：slam建立的地图
  * 两个通过id做key值的散列：路标散列，关键帧散列。
  *  
  * 插入关键帧、插入路标点的方法
* Config：配置类，单列模式
  * 静态成员本类只能指针
  * opencv 文件读取数据结构filestorage
  *  
  * 静态工场函数来构造静态成员。
  * 静态函数来访问静态成员获取参数。



* VisualOdom:
  *   
  *  
  *  
  *  



### 流程图

带地图VO











## 后端1:

### H矩阵稀疏性



分块对角矩阵求逆

病态矩阵在求Ax=b的时候 对结果x的精度影响



为什么需要鲁棒核函







## 后端2

### 位姿图

Pose graph

原因：BA需要优化大量的关键点。即使利用稀疏性，主流CPU也只能实时计算几万个点左右的BA。限制了SLAM的规模。

又因为BA在几轮之后。特征点的优化就不怎么变了，基本上就停在一个点上了，再去优化他就是让费。



所以考虑只有关键帧轨迹之间位姿、两帧之间匹配计算特征点空间初始值作为约束的位姿图。要么就得用滑动窗口来抛弃一些历史数据。