# ur5-gazebo-grasping

ur5+robotiq_85_gripper gazebo grasping

程序包中还缺少UR5机械臂的驱动包，下载方式

git clone https://github.com/ros-industrial/universal_robot.git

运行（run）：

roslaunch ur5_single_arm_tufts ur5_single_arm_gazebo.launch

rosrun rosrun ur5_manipulation UR_test1

描述：

在GAZEBO环境下，搭建了世界环境，并且添加了深度相机和带手抓的UR5机械臂，利用moveit_commander实现简单抓取操作。

参考网址：

https://blog.csdn.net/harrycomeon/article/details/107073020
