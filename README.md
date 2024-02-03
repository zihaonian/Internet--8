## 项目概述                                                                                                                                                                         

随着我国道路交通事业的飞速发展和人们生活水平的逐渐提高以及各大城市道路交通系统的不断完善，交通事故猛增成了交通管理所面临的严重问题，如图。  针对酒后驾驶和超速驾驶我国已经从法律层面进行了规范和治理，但对于分心驾驶和疲劳驾驶至今仍然缺乏有效的预防和治理措施。

<div align=center><img  src ="https://github.com/zihaonian/Ti_Cup_NUEDC-2022-10-B/blob/main/res/Reversing into storage.jpg"/></div>

本项目设计一套基于Raspberry Pi和STM32微控制器驾驶员驾驶状态及心率血氧检测系统。在Raspberry Pi里植入Linux系统，Linux系统中搭建Opencv和Dlib环境，STM32微控制器作为Raspberry Pi的协处理器。  车辆行驶过程中，Raspberry Pi通过摄像头获取每帧图像，计算出驾驶员头部估计值（计算头部在三维空间中偏转角度），嘴部的开合度（MAR）和眼部的闭合程度（EAR）对驾驶员进行状况监测（如图2）。同时Raspberry Pi与 STM32通信，接收MAX30100模块采集的心率信息。该系统可实时对非正常驾驶中的驾驶员进行震动，语音和RGB灯光提醒。实现优势互补的“融合检测”。该装置完成了接触式设备和非接触式设备的融合监测，在驾驶员安全驾驶领域具有较为良好的应用前景。  

## 文件结构

```
Ti_Cup_NUEDC-2022-10-B/
│
├── Internet+_PPT/                       # 互联网+大赛汇报PPT
│   └── Internet+.pptx
├── res/                                 # 存放readme.md图表文件
│
├── Internet+_商业计划书/                  # 存放互联网+大赛商业计划书
│   └── 安驾——行车安全监测系统_计划书.pdf
│
├── Internet+_演讲稿/                     # 存放互联网+大赛演讲稿				
│   ├── 互联网+演讲稿.docx   	
│   ├── 手稿.docx
│   └── 互联网+演讲稿.pdf
│
├── 创新训练项目成果支撑材料/                 # 存放创新训练项目成果支撑材料
│   ├── 作品实物效果图.doc   
│   ├── 成果支撑材料.pdf 
│   └── 作品实物效果图.pdf
│
├── 创新训练项目结题报告/                    # 存放创新训练项目结题报告
│   ├── 结题报告书.doc   
│   └── 结题报告书.pdf
│
├── 创新训练项目申报书/                      # 存放创新训练项目申报书
│   ├── 创新训练项目申报书.docx  
│   └── 创新训练项目申报书.pdf
│
├── 创新训练项目作品演示PPT/                 # 存放创新训练项目申报书
│   ├── 作品演示.pptx    
│   └── 作品演示PPT.pdf
│
└── README.md                 # 本文档，提供测试方案与结果、入库过程图、系统总体设计以及文件结构等信息
```

## **项目方案**

### 项目综述

本项目是基于机器视觉和心率变异性的行车安全监测装置，在Raspberry Pi中使用Linux系统，在Linux系统中搭建Opencv和Dlib库环境，调用face­recognition模块与Dlib库中HOG特征对驾驶员眼部特征点标注，计算驾驶员眼睛闭合程度，开始实时进行驾驶员驾驶状态监测,如图所示：

<div align=center><img  src ="https://github.com/zihaonian/Ti_Cup_NUEDC-2022-10-B/blob/main/res/Side parking.jpg"/></div>

同时,STM32开始进行驾驶员心率的监测，当测定的疲劳值达到一定阈值时，Raspberry Pi与STM32通过串口通信，控制震动模块逐渐加大震动并通过声音播报提醒驾驶员。同时本项目所设计的装置在原有的开源项目上进行创新和优化，如：增加多维度的特征点来提高识别精度，不仅仅对疲劳驾驶进行识别，还对分心驾驶进行检测。在原有项目基础上进行优化，为了降低世界坐标系投影到相机二维图像的距离偏差，引入头部姿势估计算法，相似计算三维空间中真实投影的欧拉角。

### 系统硬件介绍

为了精确检测到车库位置，对摄像头采集到的视频进行实时的色块检测。对视频图像进行二值化处理并设定一个30×140的ROI区域，小车前进过程中不断检测并统计该区域中黑色像素点个数，当像素数达到设定阈值即表示检测到了车库交接处，通过IO口发送高电平信号给单片机，完成车库检测。车库检测方法示意图，如图所示。

<div align=center><img  src ="https://github.com/zihaonian/Ti_Cup_NUEDC-2022-10-B/blob/main/res/Schematic diagram of garage inspection.jpg"/></div>

#### STM32	

STM32系统原理图如图，系统板搭载STM32，其最高的工作频率可以达72MHz；具有串行的单线调试和JTAG接口，2个USART接口可进行全双工通用同步/异步串行收发，STM32连接有监测驾驶员心率模块和震动提醒模块。                  



<div align=center><img  src ="https://github.com/zihaonian/Ti_Cup_NUEDC-2022-10-B/blob/main/res/Schematic diagram of module composition.png"/></div>

#### MAX-30100

MAX-30100是一种非侵入式集成的心率和血氧饱和监测模块，其依靠两个发光二极管和一个光检测器，驾驶员只需将手指紧贴传感器之上，便可使系统通过IIC协议随时读取驾驶员心率。

<div align=center><img  src ="https://github.com/zihaonian/Ti_Cup_NUEDC-2022-10-B/blob/main/res/Schematic diagram of module composition.png"/></div>

该模块中红光、红外光都用来测量驾驶员血液中的氧含量。含氧血液中传递更 多的红光并吸收更多的红外光，而脱氧血液中传递更多的红外光并 吸收红光，读取两个光源的吸收电平，通过测量心脏向外泵的血液 中的氧合血红蛋白增加和减少之间的时间，确定脉搏率（心率）。

#### Raspberry Pi


Raspberry Pi 4B采用官方的Raspbian操作系统，搭载Opencv环境，大(主)芯片为Broadcom的BCM2711BO，最高主频1.5GHz，4个CortexA72内核，支持64位。Raspberry Pi4B配备500万像素的广角摄像头，其静止图像分辨率可达2592*1944px，并支持720p与1080p视频。

 <div align=center><img  src ="https://github.com/zihaonian/Ti_Cup_NUEDC-2022-10-B/blob/main/res/Schematic diagram of the drive module.png"/></div>                              

### 系统算法介绍

#### 头部姿态估计（Head Pose Estimation）

通过一幅面部图像来获得头部的姿态角. （示意图：图表9）在3D空间中，表示物体的旋转可以由三个欧拉角(EulerAngle)来表示：分别计算pitch(围绕X轴旋转)，yaw(围绕Y轴旋转)和roll(围绕Z轴旋转)，分别学名俯仰角、偏航角和滚转角，通俗是抬头、摇头和转头。

  <div align=center><img  src ="https://github.com/zihaonian/Ti_Cup_NUEDC-2022-10-B/blob/main/res/Schematic diagram of a stand-alone button module.png"/></div>   

HeadPoseEstimation算法的步骤为：

1. 2D人脸关键点检测
2. 3D人脸模型匹配
3. 求解3D点和对应2D点的转换关系
4. 根据旋转矩阵求解欧拉角  

 一个物体相对于相机的姿态可以使用旋转矩阵和平移矩阵来表示：

- **平移矩阵**：物体相对于相机的空间位置关系矩阵，用**t**表示
- **旋转矩阵**：物体相对于相机的空间姿态关系矩阵，用**R**表示

坐标系转换分析，分别是：世界坐标系***（U,V,W）***,相机坐标系***（X,Y,Z）***,  图像中心坐标系***（U,V）***  和像素坐标系  ***（X,Y）***，如下图：

 <div align=center><img  src ="https://github.com/zihaonian/Ti_Cup_NUEDC-2022-10-B/blob/main/res/There is a diagram of the test results of the reversing of the car in the adjacent warehouse.jpg"/></div> 

如图，***O***是相机的中心，图中所示的平面是图像平面。我们找出控制3D点***P***在图像平面上的投影的方程。假设我们知道3D点***P***在世界坐标中的位置。如果我们知道世界坐标相对于相机坐标的旋转***R***（一个3×3矩阵）和平移***t***（一个3×1向量），可以使用以下等式计算该点P在相机坐标系中的位置***（X,Y,Z）***。    

 <div align=center><img  src ="https://github.com/zihaonian/Ti_Cup_NUEDC-2022-10-B/blob/main/res/There is a diagram of the test results of the reversing of the car in the adjacent warehouse.jpg"/></div>                            

扩展形式：

<div align=center><img  src ="https://github.com/zihaonian/Ti_Cup_NUEDC-2022-10-B/blob/main/res/There is a diagram of the test results of the reversing of the car in the adjacent warehouse.jpg"/></div>     

知道足够数量的点对应关系***（U,V,W）*** 和***（X,Y,Z）***，便可以求解未知数。

同理：可以得到相机坐标系到像素坐标系的关系

<div align=center><img  src ="https://github.com/zihaonian/Ti_Cup_NUEDC-2022-10-B/blob/main/res/There is a diagram of the test results of the reversing of the car in the adjacent warehouse.jpg"/></div>     

联立上述公式可得到像素坐标系（摄像头捕捉的图像像素）到世界坐标系（场景）的变换关系：

<div align=center><img  src ="https://github.com/zihaonian/Ti_Cup_NUEDC-2022-10-B/blob/main/res/There is a diagram of the test results of the reversing of the car in the adjacent warehouse.jpg"/></div>   

确定pose是：确定从3Dmodel到图片中人脸的仿射变换矩阵，包括旋转和平移的信息；其实OpenCV已经给我们提供了求解PnP问题的函数 `solvePnp()` ，其输出结果包括旋转向量**roatationvector**和平移向量**translationvector**，本项目只需要关心旋转信息，所以主要将对旋转向量进行操作。通过引入四元数处理旋转向量就可以得到欧拉角。

#### Eye Aspect Ratio

计算眼睛长宽比**（EAR）**,当人眼睁开时，**EAR**在某个值上下波动，当人眼闭合时，**EAR**迅速下降，理论上会接近于零，当时人脸检测模型还没有这么精确。所以我们认为当EAR低于某个阈值时，眼睛处于闭合状态。为检测眨眼次数，需要设置同一次眨眼的连续帧数。眨眼速度比较快，一般1~3帧就完成了眨眼动作。两个阈值都要根据实际情况设置，本项目加入了自动调参算法，解决了因人眼型差异而导致的识别精确度不高问题。

<div align=center><img  src ="https://github.com/zihaonian/Ti_Cup_NUEDC-2022-10-B/blob/main/res/There is a diagram of the test results of the reversing of the car in the adjacent warehouse.jpg"/></div>   

<div align=center><img  src ="https://github.com/zihaonian/Ti_Cup_NUEDC-2022-10-B/blob/main/res/There is a diagram of the test results of the reversing of the car in the adjacent warehouse.jpg"/></div>   

通过计算P**38**、P**39**、P**42**、P**41**的纵坐标、P**37**、P**40**的横坐标来计算眼睛的睁开度。

通过一个阈值确定眼睛是睁开还是闭上。也可以将这个值与初始的值的比值作为睁开度，根据不同程度来进行比较。睁开度从大到小为进入闭眼期，从小到大为进入睁眼期，计算最长闭眼时间（可用视频帧数来代替）闭眼次数为进入闭眼、进入睁眼的次数：

<div align=center><img  src ="https://github.com/zihaonian/Ti_Cup_NUEDC-2022-10-B/blob/main/res/There is a diagram of the test results of the reversing of the car in the adjacent warehouse.jpg"/></div>   

目前，PERCLO方法有三种判断疲劳的不同准则，分别E准则、P70准则、P80准则。其具体含义如下：

**EM准则**：瞳孔被眼睑覆盖超50%的面积，则认为眼睛是闭合的

**P70准则**：瞳孔被眼睑覆盖超70%的面积，则认为眼睛是闭合的

**P80准则**：瞳孔被眼睑覆盖超过80%的面积，则认为眼睛是闭合的

当人注意力特别集中或处在沉思状态时可能也会有眼睑覆盖瞳孔超过50%甚至70%的可能，所以系统采用的是P80准则。

统计表明，人在一分钟之内要眨十次左右的眼睛，每次需要0.3—0.4秒左右。然而，由于驾驶员工作性质的不同，需要其在工作中注意力高度集中，所以眨眼次数略少，约5~10次。眼睛闭合的频率以及闭合时间的长短与疲劳有密切联系，如果连续监测到驾驶员的PERCLOS>30%且平均闭眼时长>0.25s，就判定驾驶员处于非正常驾驶状态，并发出报警。

截至目前，国内外针对疲劳驾驶检测系统的设计与开发方法可分为主观检测法和客观检测法。主观检测法是通过对驾驶员进行人工询问、填写调查表、主观评价等方式获取在不同时间段下驾驶员的心理、驾驶动作以及面部表情等信息，然后通过对获取到的信息进行分析从而得出驾驶员的疲劳状态。

####  心率值（MAX-30100）

该模块中红光、红外光都用来测量驾驶员血液中的氧含量。含氧血液中传递更多的红光并吸收更多的红外光，而脱氧血液中传递更多的红外光并吸收红光，读取两个光源的吸收电平，通过测量心脏向外泵的血液中的氧合血红蛋白增加和减少之间的时间，确定脉搏率（心率）。由于人体的皮肤、骨骼、肌肉、脂肪等对于光的反射是固定值，而毛细血管和动脉、静脉由于随着脉搏容积不停变大变小，所以对光的反射值是波动值，而这个波动值正好与心率一致，所以光电容积法正是通过这个波动的频率来确定使用者的心率数据。
***spo2***计算公式：

<div align=center><img  src ="https://github.com/zihaonian/Ti_Cup_NUEDC-2022-10-B/blob/main/res/There is a diagram of the test results of the reversing of the car in the adjacent warehouse.jpg"/></div>  

该模块利用IIC通信协议与STM32交换数据，读取raw IR Value（红外FIFO数据）、raw Red Value（红光FIFO数据）进行处理计算驾驶员心率血氧数据。

系统主要监测指标及人体正常情况下的数据,如表1所列：

| 健康指标   |                           正常范围                           |
| ---------- | :----------------------------------------------------------: |
| 心率       |                        60-100次/分钟                         |
| 血氧饱和度 |                           95%-100%                           |
| 脉搏       | 新生儿130-140次/分钟 3-5岁儿童100-120次/分钟10岁左右儿童90-100次/分 老年人约55-60次/分钟 |

## **项目特色与创新**

- **引入头部估计算法，降低世界坐标系投影到相机二维图像的距离偏差**

计算驾驶员在三维空间的三个垂直方向的偏转角度，修正EAR与MAR，提高识别可信度。汽车驾驶员从根本上受到人们在任何时候都能观察到的视野的限制。当一个人没有注意到他的环境发生变化时，如果驾驶员被警告出现看不见的危险，则可能会减轻危及生命的碰撞的可能性。

- **实现接触式设备和非接触式设备的融合监测**

与传统的纯计算机视觉设备相比，本系统增加了接触式设备，实现对驾驶员心率的测量，从而更高效准确的监测驾驶员驾驶状态和提醒驾驶员。与传统的接触式方案相比，其接触式方案更简单，无需繁杂的操作和过多的接触式设备，减少了该设备对驾驶员的影响。

- **增加多维度的特征点提高识别精度**

不仅仅对疲劳驾驶进行识别，还可实现分心驾驶检测。调用Face­-Recognition模块与Dlib库中HOG特征对驾驶员面部特征点标注。计算驾驶员EAR与MAR值，通过头部估计算法计算Pitch(围绕X轴旋转)，Yaw(围绕Y轴旋转)和Roll(围绕Z轴旋转)，修正EAR与MAR值，提高识别精度。
