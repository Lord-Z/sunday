# sunday功能包使用介绍以及开源



sunday我给自己机械臂的命名，原型是innfos的gluon机械臂。通过sw模型文件转urdf。Sunday项目主要由六个功能包sunday_description、sunday_gazebo、sunday_moveit_config、yolov5_ros、vacuum_plugin、realsense_ros_gazebo组成，下面我将介绍这六个功能包。

## 个人环境

首先介绍个人使用环境，博主使用的是Ubuntu18.04+gtx1660ti显卡+ros_melodic+cuda10.2+pytorch1.8+yolov5-6.1。环境配置的教程详见上一篇博客：
[https://blog.csdn.net/qq_48427527/article/details/129201676?spm=1001.2014.3001.5502](https://blog.csdn.net/qq_48427527/article/details/129201676?spm=1001.2014.3001.5502)



## sunday_description

sunday_description是由sw插件导出的功能包，我主要对sunday_description/urdf中的urdf文件进行修改，并配置xacro文件。

```
sunday_description/launch/sunday_rviz.launch
```

文件用于观察我所配置的xacro文件的模型状态。



## sunday_moveit_config

基于sunday_description/urdf/sunday.xacro对moveit进行配置修改方面还是参考古月的课程进行的修改。

修改过的文件：

```
sunday_moveit_config/config/controllers_gazebo.yaml

sunday_moveit_config/launch/moveit_rviz.launch

sunday_moveit_config/launch/moveit_planning_execution.launch
```



## sunday_gazebo

该功能包主要用于配置机械臂操作的gazebo仿真环境，具体配置也是参考古月的课程，几个launch文件分别用于发布机械臂关节的状态以及与moveit做对接。

```
sunday_gazebo/launch/sunday_bringup_moveit.launch
```

该launch文件里面包含了所有节点，包括yolov5节点。如果你还未配置好yolov5环境，也可以使用以下不包含yolov5的launch文件打开gazebo环境。

```
sunday_gazebo/launch/sunday_bringup_moveit_origin.launch
```



## yolov5_ros

该功能包是我在csdn上边找到的，封装了yolov5，能以launch文件启动，并发布话题，个人感觉还不错。地址：
[YoloV5 的ros功能包](https://blog.csdn.net/Chris121345/article/details/122563536?spm=1001.2101.3001.6650.5&amp;utm_medium=distribute.pc_relevant.none-task-blog-2~default~CTRLIST~default-5-122563536-blog-123269882.pc_relevant_multi_platform_whitelistv1_exp2&amp;depth_1-utm_source=distribute.pc_relevant.none-task-blog-2~default~CTRLIST~default-5-122563536-blog-123269882.pc_relevant_multi_platform_whitelistv1_exp2&amp;utm_relevant_index=8)

```
git clone https://github.com/qq44642754a/Yolov5_ros.git
```


具体使用过程就是将训练好的权重放置进功能包中对应launch文件的路径下边，在launch文件中修改话题名称以及其它参数即可。具体权重文件，在下面的开源链接中。



## vacuum_plugin

vacuum_plugin是一个吸盘的插件，在urdf中插入该插件即可在gazebo中实现吸盘的功能。

```
git clone https://github.com/tatsuya-s/gazebo_ros_vacuum_gripper_debugger
```

在sunday_description/urdf/sunday.xacro第490行添加插件

```
  <!-- vacuum_gripper plugin -->
  <gazebo>
    <plugin name="gazebo_ros_vacuum_gripper" filename="libvacuum_plugin.so">
      <robotNamespace>/sunday/vacuum_gripper</robotNamespace>
      <bodyName>link_6</bodyName>
      <topicName>grasping</topicName>
      <maxDistance>0.05</maxDistance>
      <minDistance>0.03</minDistance>
    </plugin>
</gazebo>
```



## realsense_ros_gazebo

realsense_ros_gazebo功能包是realsense的gazebo功能包，其仿真效果与实物相同，且其中包含多款realsense型号的模型文件，其中便包含本项目中使用的realsenseD435i摄像头，可以将原urdf中的摄像头删除，调用该功能包的模型进行替换。

```
git clone https://github.com/nilseuropa/realsense_ros_gazebo.git
```

在sunday_description/urdf/sunday.xacro第454行添加代码

```
<xacro:include filename="$(find realsense_ros_gazebo)/xacro/depthcam.xacro"/>
<xacro:realsense_d435 sensor_name="camera" parent_link="link_5" rate="30">
    <origin xyz="-0.00068847 -0.06 -0.13" rpy="-3.14 0 -1.5708"/>
</xacro:realsense_d435>
```



## gripper_model

gripper_model并不是一个功能包，这个包里有四个模型文件，对应视频中的三个多边形体以及，喂食过程中的人物张嘴闭嘴展板。模型文件的制作以及贴图的教程可以看我之前写的博客。博客地址：

[https://blog.csdn.net/qq_48427527/article/details/124477608?spm=1001.2014.3001.5502](https://blog.csdn.net/qq_48427527/article/details/124477608?spm=1001.2014.3001.5502)

将四个模型拷贝放置到.gazebo/models路径，注意不能将整个包拷贝，仅将四个模型文件拷贝即可。



## 使用教程

现将几个模型文件gripper_model中的几个模型文件放到.gazebo/models路径中

将sunday.zip解压置于xx_ws/src路径下

```
cd xx_ws

catkin_make

source devel/setup.bash

conda activate 之前创建的虚拟环境

roslaunch sunday_gazebo sunday_bringup_moveit.launch（该launch包含yolov5节点，因此需要在conda环境中运行）

新开一个终端

roscd sunday_gazebo/scripts

python grasp&feed.py(该脚本无需进入conda环境中运行)
在终端中输入想要拾取物体的名称，从而实现物体的拾取以及喂食

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

视频第一部分pick&place

conda activate 之前创建的虚拟环境
roslaunch sunday_gazebo sunday_bringup_moveit.launch
将gazebo中的展板挪远一点

新开终端
roscd sunday_gazebo/scripts
python grasp.py
在终端中输入想要拾取物体的名称，从而实现物体的拾取
```

由于用于训练的多边形样本照片过少，如果出现展板挡住多边形模型出现阴影从而无法准确识别的情况，可以拖动或者旋转多边形模型通过调整位姿实现物体识别。



## 开源地址
github无法上传开源的三个功能包，因此我将权重放置在百度网盘里面
链接: https://pan.baidu.com/s/1zS2sZkfdtcU0EuDZvVxjQg  密码: sg0v
--来自百度网盘超级会员V3的分享

另外三个开源的功能包，已经配置好权重，放进Sunday一同进行编译
链接: https://pan.baidu.com/s/1N2T7TLAK9QIbBxWjepwJ6w  密码: uvkv
--来自百度网盘超级会员V3的分享
在放进百度云盘之前，博主将yolov5_ros/weights路径下的best.pt重命名为了polygon.pt用于方便区分权重，给各位造成一定的困扰，请各位在使用时将launch文件中的路径名自行修改。


github:https://github.com/Lord-Z/sunday



## 资料
1.[YoloV5 的ros功能包](https://blog.csdn.net/Chris121345/article/details/122563536?spm=1001.2101.3001.6650.5&amp;utm_medium=distribute.pc_relevant.none-task-blog-2~default~CTRLIST~default-5-122563536-blog-123269882.pc_relevant_multi_platform_whitelistv1_exp2&amp;depth_1-utm_source=distribute.pc_relevant.none-task-blog-2~default~CTRLIST~default-5-122563536-blog-123269882.pc_relevant_multi_platform_whitelistv1_exp2&amp;utm_relevant_index=8)
2.[realsense_ros_gazebo](https://github.com/nilseuropa/realsense_ros_gazebo.git)
3.[vacuum_plugin](https://github.com/tatsuya-s/gazebo_ros_vacuum_gripper_debugger)
