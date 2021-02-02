// Test2: Get to the points to create environment
#include<moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <vector>
 
int main(int argc, char **argv)
{
    ros::init(argc, argv, "UR5_test2");
    //多线程
    ros::AsyncSpinner spinner(1);
    //开启新的线程
    spinner.start();
 
    //初始化需要使用move group控制的机械臂中的arm group
    moveit::planning_interface::MoveGroupInterface arm("manipulator");
    moveit::planning_interface::MoveGroupInterface gripper("gripper");
    
   //允许误差
    arm.setGoalJointTolerance(0.001);
   //允许的最大速度和加速度
    arm.setMaxAccelerationScalingFactor(0.2);
    arm.setMaxVelocityScalingFactor(0.2);
 
    // Point "1"
    arm.setNamedTarget("home_j");
    arm.move();
    sleep(1);

    
    geometry_msgs::PoseStamped pose_target = arm.getCurrentPose();
    // Block point
    pose_target.pose.position.x = 0.4;
    pose_target.pose.position.y = 0.0;
    pose_target.pose.position.z = pose_target.pose.position.z+0.35;
    // Point "3"
    arm.setPoseTarget(pose_target);
    arm.move();
    sleep(1);

    ros::shutdown();
 
    return 0;
}