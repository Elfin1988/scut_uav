/*
 * 修改后的多航点控制示例
 */
#include <thread>
#include <chrono>
#include <vector>

#include <Eigen/Core>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "waypoint_example");
  ros::NodeHandle nh;
  ros::Publisher trajectory_pub =
      nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
          mav_msgs::default_topics::COMMAND_TRAJECTORY, 10);
  ROS_INFO("Started waypoint example.");

  // 取消Gazebo暂停
  std_srvs::Empty srv;
  bool unpaused = ros::service::call("/gazebo/unpause_physics", srv);
  for (unsigned int i = 0; i < 10 && !unpaused; ++i) {
    ROS_INFO("Waiting for Gazebo to start...");
    std::this_thread::sleep_for(std::chrono::seconds(1));
    unpaused = ros::service::call("/gazebo/unpause_physics", srv);
  }
  if (!unpaused) {
    ROS_FATAL("Failed to start Gazebo");
    return -1;
  }

  // 定义航点列表（x + y + z）
  std::vector<Eigen::Vector3d> waypoints = {
      {0.0, 0.0, 1.0},   
      
      {0.0, 7.0, 1.2},   
  };

  std::vector<double> yaws = {0, 0};//航向角
  std::vector<double> durations = {1, 100}; // 持续时间，最后一个点悬停更久

  // 等待初始稳定
  ros::Duration(2.0).sleep();

  // 发布航点
  for (size_t i = 0; i < waypoints.size(); ++i) {
    trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
    mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(
        waypoints[i], yaws[i], &trajectory_msg);
    
    trajectory_msg.header.stamp = ros::Time::now();
    trajectory_pub.publish(trajectory_msg);
    
    ROS_INFO("waypoints: [%.1f, %.1f, %.1f], yaw: %.1f",
             i+1, waypoints[i].x(), waypoints[i].y(), waypoints[i].z(), yaws[i]);
    
    ros::Duration(durations[i]).sleep();
    ros::spinOnce();
  }

  ROS_INFO("All waypoints completed");
  ros::shutdown();
  return 0;
}
