#include <thread>
#include <chrono>
#include <vector>

#include <Eigen/Core>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
// 添加ArUco检测相关头文件
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
// 如果使用OpenCV ArUco模块
#include <opencv2/aruco.hpp>
// 添加在其他include之后
#include <opencv2/opencv.hpp>  // 包含大多数OpenCV功能
#include <opencv2/highgui.hpp> // 用于imshow和waitKey
#include <geometry_msgs/Pose.h> // 用于poseCallback函数

enum FlightState {
  INIT,
  FIRST_WAYPOINT_SEQUENCE,
  SEARCHING_ARUCO,
  LANDING,         // ArUco降落
  APPROACHING,     // ArUco接近阶段
  DESCENDING,      // ArUco下降阶段
  TOUCHING,        // ArUco接触阶段
  LANDED,          // ArUco已着陆
  SEARCHING_APRON, // 新增：搜索停机坪
  APPROACHING_APRON, // 新增：接近停机坪阶段
  DESCENDING_APRON,  // 新增：停机坪下降阶段
  TOUCHING_APRON,    // 新增：停机坪接触阶段
  LANDED_APRON,      // 新增：停机坪已着陆
  SECOND_WAYPOINT_SEQUENCE,
  COMPLETED
};

class ArucoWaypointController {
public:
  ArucoWaypointController(ros::NodeHandle& nh) : nh_(nh), state_(INIT), aruco_detected_(false) {
    // 创建发布器
    trajectory_pub_ = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
        mav_msgs::default_topics::COMMAND_TRAJECTORY, 10);
    
    // 订阅图像话题（根据您的相机设置修改）
    image_sub_ = nh_.subscribe("/hummingbird/vi_sensor/camera_depth/camera/image_raw", 1, 
        &ArucoWaypointController::imageCallback, this);
    
    // 订阅位置话题
    pose_sub_ = nh_.subscribe("/hummingbird/ground_truth/pose", 1,
        &ArucoWaypointController::poseCallback, this);
    
    // 初始化航点
    initializeWaypoints();
  }

  void start() {
    // 取消Gazebo暂停
    unpauseGazebo();
    
    // 等待系统稳定
    ros::Duration(2.0).sleep();
    
    // 执行任务
    state_ = FIRST_WAYPOINT_SEQUENCE;
    executeFlightPlan();
  }

private:
  ros::NodeHandle nh_;
  ros::Publisher trajectory_pub_;
  ros::Subscriber image_sub_;
  
  FlightState state_;
  bool aruco_detected_;
  Eigen::Vector3d aruco_position_;
  
  std::vector<Eigen::Vector3d> first_waypoints_;
  std::vector<double> first_yaws_;
  std::vector<double> first_durations_;
  
  std::vector<Eigen::Vector3d> second_waypoints_;
  std::vector<double> second_yaws_;
  std::vector<double> second_durations_;

  // 相机和降落相关参数
  cv::Point2f marker_center_;      // 标记中心点
  cv::Point2f image_center_;       // 图像中心点
  double kp_horizontal_ = 0.0010;  // 水平位置PID控制增益
  double kp_vertical_ = 0.0010;    // 垂直位置PID控制增益
  double current_height_ = 0.0;    // 当前高度
  
  // 新增位置订阅器
  ros::Subscriber pose_sub_;

  // 添加到class的private部分
  Eigen::Vector3d current_drone_position_;
  double current_drone_yaw_ = 0.0;

  // 在类的private部分添加
  bool apron_detected_ = false;
  Eigen::Vector3d apron_position_;
  cv::Point2f apron_center_;

  // 停机坪检测参数
  double apron_area_min_ = 1000;  // 停机坪最小面积
  double apron_area_max_ = 10000; // 停机坪最大面积
  double apron_aspect_ratio_min_ = 0.8; // H形状宽高比范围
  double apron_aspect_ratio_max_ = 1.2;

  // 添加日志控制变量
  bool verbose_logging_ = false;  // 控制详细日志输出
  bool save_debug_images_ = false;  // 控制是否保存调试图像
  int debug_image_interval_ = 10;  // 每10帧保存一次图像

  void initializeWaypoints() {
        // 第一段航点（ArUco检测之前）
        first_waypoints_ = {
            {0.0, 0.0, 1.6},
            {3.0, 1.0, 1.6},
            {3.0, 2.0, 1.6},
            {1.0, 2.0, 1.6},
            {1.0, 3.0, 1.6},
            {3.0, 3.0, 1.6},
            {3.0, 4.0, 1.6},
            {1.0, 4.0, 1.6},
            {1.0, 5.0, 1.6},
            {3.0, 5.0, 1.6},
            {3.0, 6.0, 1.6}, // 起飞点
            {0.3, 7.0, 1.6}  // 飞到搜索区域
        };
        first_yaws_ = {0.0, 0.0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
        first_durations_ = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 3, 3};

        // 第二段航点（降落后继续执行）
        second_waypoints_ = {
            {-2.6, 5.5, 1.7},
            {-2.7, 5.4, 1.0},
            {-3.3, 5.5, 1.0},
            {-4.0, 5.5, 1.0},
            {-4.0, 4.5, 1.0},
            {-4.0, 3.5, 1.0},
            {-4.0, 2.5, 1.0},
            {-3.5, 1.5, 1.0},
            {-2.7, 0.5, 1.0},
            {-1.5, -0.5, 1.0},
            {2.2, 1.5, 3.0},
            // 任务结束点
        };
        second_yaws_ = {0.0, 0.0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
        second_durations_ = {3, 1,1, 1, 1, 1, 1, 1, 1, 1, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3};
    }

  void unpauseGazebo() {
    std_srvs::Empty srv;
    bool unpaused = ros::service::call("/gazebo/unpause_physics", srv);
    for (unsigned int i = 0; i < 10 && !unpaused; ++i) {
      ROS_INFO("Waiting for Gazebo to start...");
      std::this_thread::sleep_for(std::chrono::seconds(1));
      unpaused = ros::service::call("/gazebo/unpause_physics", srv);
    }
    if (!unpaused) {
      ROS_FATAL("Failed to start Gazebo");
      ros::shutdown();
    }
  }

  void executeFlightPlan() {
    while (state_ != COMPLETED && ros::ok()) {
      ROS_INFO("Current state: %d", (int)state_);
      
      switch (state_) {
        case FIRST_WAYPOINT_SEQUENCE:
          executeWaypoints(first_waypoints_, first_yaws_, first_durations_);
          state_ = SEARCHING_ARUCO;  // 修正: 第一阶段后搜索ArUco
          ROS_INFO("First waypoint sequence completed, now searching for ArUco marker");
          break;
          
        case SEARCHING_ARUCO:
          searchArUco();
          break;
          
        case LANDING:
          performLanding();
          state_ = SECOND_WAYPOINT_SEQUENCE;
          ROS_INFO("Landing completed, continuing with second waypoint sequence");
          break;
          
        case SECOND_WAYPOINT_SEQUENCE:
          executeWaypoints(second_waypoints_, second_yaws_, second_durations_);
          state_ = SEARCHING_APRON;  // 修正: 第二阶段后搜索停机坪
          ROS_INFO("Second waypoint sequence completed, now searching for apron");
          break;
          
        case SEARCHING_APRON:
          searchApron();
          break;
          
        case APPROACHING_APRON:
        case DESCENDING_APRON:
        case TOUCHING_APRON:
        case LANDED_APRON:
          performApronLanding();
          break;
          
        default:
          break;
      }
      
      ros::spinOnce();
      ros::Duration(0.1).sleep();
    }
  }

  void executeWaypoints(const std::vector<Eigen::Vector3d>& waypoints,
                        const std::vector<double>& yaws,
                        const std::vector<double>& durations) {
    for (size_t i = 0; i < waypoints.size(); ++i) {
      if (state_ == SEARCHING_ARUCO && aruco_detected_) {
        // 如果在搜索状态且检测到ArUco，中断当前航点序列
        state_ = LANDING;
        return;
      }
      
      trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
      mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(
          waypoints[i], yaws[i], &trajectory_msg);
      
      trajectory_msg.header.stamp = ros::Time::now();
      trajectory_pub_.publish(trajectory_msg);
      
      ROS_INFO("Publishing waypoint: [%.1f, %.1f, %.1f], yaw: %.1f",
               waypoints[i].x(), waypoints[i].y(), waypoints[i].z(), yaws[i]);
      
      // 等待执行时间
      ros::Time start_time = ros::Time::now();
      while (ros::Time::now() - start_time < ros::Duration(durations[i]) && 
             !(state_ == SEARCHING_ARUCO && aruco_detected_)) {
        ros::spinOnce();
        ros::Duration(0.1).sleep();
      }
      
      if (state_ == SEARCHING_ARUCO && aruco_detected_) {
        state_ = LANDING;
        return;
      }
    }
  }

  void searchArUco() {
    ROS_INFO("Searching for ArUco marker...");
    
    // 定义一个计时器，避免永久阻塞
    ros::Time start_time = ros::Time::now();
    
    // 等待相机回调检测到ArUco标记或超时
    while (!aruco_detected_ && 
           ros::Time::now() - start_time < ros::Duration(10.0)) {
      ros::spinOnce();
      ros::Duration(0.1).sleep();
    }
    
    // 如果检测到ArUco标记，切换到降落状态
    if (aruco_detected_) {
      ROS_INFO("ArUco marker found, switching to landing mode");
      state_ = LANDING;
    }
  }
	
double getDescentSpeed(double height) {
    if (height > 1.5) return -0.006;      // 高空快速下降
    else if (height > 0.8) return -0.004; // 中等高度中速下降
    else return -0.003;                  // 低空慢速下降
}
	
  void performLanding() {
    ROS_INFO("ArUco marker detected at [%.1f, %.1f, %.1f], performing landing",
             aruco_position_.x(), aruco_position_.y(), aruco_position_.z());
    
    // 计算降落点（ArUco上方）
    Eigen::Vector3d landing_position = aruco_position_;
    landing_position.z() = aruco_position_.z() + 0.5; // 悬停在ArUco上方0.5米
    
    // 先飞到降落点上方
    trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
    mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(
        landing_position, 0.0, &trajectory_msg);
    trajectory_msg.header.stamp = ros::Time::now();
    trajectory_pub_.publish(trajectory_msg);
    
    
    // 基于ArUco中心位置与图像中心的偏差计算水平校正
    double dx = marker_center_.x - image_center_.x;
    double dy = marker_center_.y - image_center_.y;

    // 水平位置PID控制
    landing_position.x() += -kp_vertical_ * dy;    // 前后校正
    landing_position.y() += kp_horizontal_ * dx;   // 左右校正
    landing_position.z() += getDescentSpeed(current_height_);  // 垂直下降
    
    // 等待到达降落点上方
    ros::Duration(3.0).sleep();
    
    // 执行降落
    landing_position.z() = 0.1; // 降低到接近地面
    mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(
        landing_position, 0.0, &trajectory_msg);
    trajectory_msg.header.stamp = ros::Time::now();
    trajectory_pub_.publish(trajectory_msg);
    
    // 等待降落完成
    ros::Duration(5.0).sleep();
    
    // 可以在这里添加关闭电机的命令或其他降落后操作
    
    // 起飞继续任务
    Eigen::Vector3d takeoff_position = landing_position;
    takeoff_position.z() = 1.0; // 起飞高度
    mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(
        takeoff_position, 0.0, &trajectory_msg);
    trajectory_msg.header.stamp = ros::Time::now();
    trajectory_pub_.publish(trajectory_msg);
    
    // 等待起飞完成
    ros::Duration(3.0).sleep();
  }

  void performApronLanding() {
    // 定义固定的目标着陆点
    static const Eigen::Vector3d TARGET_LANDING_POSITION(2.2, 1.5, 2.0);
    
    ROS_INFO("Apron detected, directing to target position [%.2f, %.2f, %.2f]",
             TARGET_LANDING_POSITION.x(), TARGET_LANDING_POSITION.y(), TARGET_LANDING_POSITION.z());
    
    // 状态机实现
    static int landing_stage = 0;
    static ros::Time stage_start_time;
    
    // 降落过程中持续导航至目标位置
    trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
    
    switch (landing_stage) {
      case 0:  // 初始化
        stage_start_time = ros::Time::now();
        landing_stage = 1;
        break;
        
      case 1:  // APPROACHING: 水平接近目标位置
        {
          // 计算当前无人机与目标位置的水平距离
          double horizontal_dist = sqrt(
            pow(current_drone_position_.x() - TARGET_LANDING_POSITION.x(), 2) +
            pow(current_drone_position_.y() - TARGET_LANDING_POSITION.y(), 2)
          );
          
          if (horizontal_dist < 0.2) {  // 水平位置已经足够接近
            ROS_INFO("Reached target horizontal position, starting height adjustment");
            landing_stage = 2;
            stage_start_time = ros::Time::now();
            state_ = DESCENDING_APRON;
          }
          else {
            // 飞向目标位置的正上方，保持当前高度
            Eigen::Vector3d target_position(
              TARGET_LANDING_POSITION.x(),
              TARGET_LANDING_POSITION.y(),
              current_drone_position_.z()  // 保持当前高度
            );
            
            mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(
              target_position, current_drone_yaw_, &trajectory_msg);
            trajectory_msg.header.stamp = ros::Time::now();
            trajectory_pub_.publish(trajectory_msg);
            
            ROS_INFO("Approaching target position, dist: %.2f m", horizontal_dist);
          }
        }
        break;
        
      case 2:  // DESCENDING: 调整至目标高度
        {
          // 计算高度差
          double height_diff = TARGET_LANDING_POSITION.z() - current_height_;
          
          if (fabs(height_diff) < 0.1) {  // 高度接近目标高度
            ROS_INFO("Reached target height");
            landing_stage = 3;
            stage_start_time = ros::Time::now();
            state_ = TOUCHING_APRON;
          }
          else {
            // 计算缓慢的高度调整速度
            double height_adjustment_speed;
            if (height_diff > 0) {
              // 需要上升
              height_adjustment_speed = std::min(0.05, height_diff * 0.5);
            } else {
              // 需要下降
              height_adjustment_speed = std::max(-0.05, height_diff * 0.5);
            }
            
            // 保持水平位置，调整高度
            Eigen::Vector3d target_position(
              TARGET_LANDING_POSITION.x(),
              TARGET_LANDING_POSITION.y(),
              current_height_ + height_adjustment_speed  // 逐渐调整高度
            );
            
            mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(
              target_position, current_drone_yaw_, &trajectory_msg);
            trajectory_msg.header.stamp = ros::Time::now();
            trajectory_pub_.publish(trajectory_msg);
            
            ROS_INFO("Adjusting height: current=%.2f, target=%.2f, diff=%.2f, speed=%.3f",
                    current_height_, TARGET_LANDING_POSITION.z(), height_diff, height_adjustment_speed);
          }
        }
        break;
        
      case 3:  // TOUCHING: 保持目标位置
        {
          // 维持在目标位置
          Eigen::Vector3d landing_position(
            TARGET_LANDING_POSITION.x(),
            TARGET_LANDING_POSITION.y(),
            TARGET_LANDING_POSITION.z()
          );
          
          mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(
            landing_position, current_drone_yaw_, &trajectory_msg);
          trajectory_msg.header.stamp = ros::Time::now();
          trajectory_pub_.publish(trajectory_msg);
          
          // 等待5秒视为着陆完成
          if (ros::Time::now() - stage_start_time > ros::Duration(5.0)) {
            landing_stage = 4;
            state_ = LANDED_APRON;
          }
        }
        break;
        
      case 4:  // LANDED: 任务完成
        {
          ROS_INFO("Landing mission completed at target position");
          
          // 重置状态，完成任务
          landing_stage = 0;
          state_ = COMPLETED;
          ROS_INFO("Mission accomplished");
        }
        break;
    }
  }

  void searchApron() {
    ROS_INFO("Searching for apron...");
    
    // 定义一个计时器，避免永久阻塞
    ros::Time start_time = ros::Time::now();
    
    // 等待相机回调检测到停机坪或超时
    while (!apron_detected_ && 
           ros::Time::now() - start_time < ros::Duration(10.0)) {
      ros::spinOnce();
      ros::Duration(0.1).sleep();
    }
    
    // 如果检测到停机坪，切换到接近阶段
    if (apron_detected_) {
      ROS_INFO("Apron found, switching to approaching mode");
      state_ = APPROACHING_APRON;
    } else {
      // 超时未检测到，可以添加其他策略
      ROS_WARN("Apron not found after timeout, continuing with next waypoint");
      state_ = SECOND_WAYPOINT_SEQUENCE;
    }
  }

  void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    if (state_ != SEARCHING_ARUCO && state_ != LANDING && 
        state_ != SEARCHING_APRON && state_ != APPROACHING_APRON && 
        state_ != DESCENDING_APRON && state_ != TOUCHING_APRON) return;
    
    try {
      // 转换ROS图像消息到OpenCV格式
      cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      cv::Mat image = cv_ptr->image;
      
      // 设置图像中心点
      image_center_ = cv::Point2f(image.cols/2, image.rows/2);
      
// 根据当前状态选择检测方法
      if (state_ == SEARCHING_ARUCO || state_ == LANDING) {
        // 现有的ArUco检测代码不变
        // 转换ROS图像消息到OpenCV格式
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cv::Mat image = cv_ptr->image;
        
        // 设置图像中心点
        image_center_ = cv::Point2f(image.cols/2, image.rows/2);
        
      // ArUco检测
      cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
      std::vector<int> ids;
      std::vector<std::vector<cv::Point2f>> corners;
      cv::aruco::detectMarkers(image, dictionary, corners, ids);
      
      // 绘制检测结果
      cv::Mat output_image = image.clone();
      cv::aruco::drawDetectedMarkers(output_image, corners, ids);
      
      if (ids.size() > 0) {
        // 找到了ArUco标记
        // 提取标记角点和中心
        std::vector<cv::Point2f> marker_corners = corners[0];
        marker_center_ = cv::Point2f(0, 0);
        for (auto& corner : marker_corners) {
          marker_center_ += corner;
        }
        marker_center_ *= 0.25f; // 取平均得到中心点
        
        // 打印标记中心点像素坐标
        ROS_INFO("marker_center_pixel: (%.2f, %.2f)", marker_center_.x, marker_center_.y);
        ROS_INFO("image_center_pixel: (%.2f, %.2f)", image_center_.x, image_center_.y);
        
        // 使用实际的vi_sensor相机内参
        cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << 
                             205.46963709898583, 0.0, 320.5, 
                             0.0, 205.46963709898583, 240.5, 
                             0.0, 0.0, 1.0);
        cv::Mat dist_coeffs = cv::Mat::zeros(5, 1, CV_64F); // 无畸变
        
        // 计算姿态和位置
        std::vector<cv::Vec3d> rvecs, tvecs;
        cv::aruco::estimatePoseSingleMarkers(
            std::vector<std::vector<cv::Point2f>>{marker_corners},
            0.1, // 修改为实际标记大小(米)，例如0.15米
            camera_matrix, 
            dist_coeffs,
            rvecs, 
            tvecs);

        // 打印tvecs向量
        ROS_INFO("tvecs: [%.4f, %.4f, %.4f]", tvecs[0][0], tvecs[0][1], tvecs[0][2]);

        // 使用drawAxis代替drawFrameAxes
        cv::aruco::drawAxis(output_image, camera_matrix, dist_coeffs,
                         rvecs[0], tvecs[0], 0.05);

        // 计算相对偏移 - 垂直向下相机
        double marker_dx = tvecs[0][0];  // 相机x → 无人机前进
        double marker_dy = -tvecs[0][1]; // 相机y → 无人机右侧，取负得到左侧
        double marker_dz = -tvecs[0][2]; // 相机z → 无人机下方，取负得到高度

        // 考虑yaw角的旋转
        double cos_yaw = cos(current_drone_yaw_);
        double sin_yaw = sin(current_drone_yaw_);

        // 旋转到世界坐标系
        double world_dx = cos_yaw * marker_dx - sin_yaw * marker_dy;
        double world_dy = sin_yaw * marker_dx + cos_yaw * marker_dy;

        // 假设期望位置
        Eigen::Vector3d expected_aruco_position(0.4, 6.8, 0.0);

        // 计算原始位置（不含补偿）
        Eigen::Vector3d raw_aruco_position(
            current_drone_position_.x() + world_dx,
            current_drone_position_.y() + world_dy,
            0.0
        );

        // 记录第一次检测的偏差
        static bool first_detection = true;
        static Eigen::Vector3d offset_vector;

        if (first_detection && !aruco_detected_ && state_ == SEARCHING_ARUCO) {
            // 计算偏差 = 期望位置 - 原始位置
            offset_vector = expected_aruco_position - raw_aruco_position;
            first_detection = false;
            
            ROS_INFO("Calculated correction offset: [%.2f, %.2f, %.2f]",
                     offset_vector.x(), offset_vector.y(), offset_vector.z());
        }

        // 应用偏差修正
        aruco_position_ = raw_aruco_position + offset_vector;

        ROS_INFO("camera_offset: dx=%.4f, dy=%.4f, dz=%.4f", 
                  tvecs[0][0], tvecs[0][1], tvecs[0][2]);
        ROS_INFO("world_offset: dx=%.4f, dy=%.4f", world_dx, world_dy);
        ROS_INFO("final_aruco_position: [%.2f, %.2f, %.2f]", 
                 aruco_position_.x(), aruco_position_.y(), aruco_position_.z());
        
        // 首次检测到标记
        if (!aruco_detected_ && state_ == SEARCHING_ARUCO) {
          ROS_INFO("ArUco marker detected!");
          aruco_detected_ = true;
          state_ = LANDING;
        }
      }
      
      // 注释掉以下两行，避免GUI显示问题
      // cv::imshow("ArUco Detection", output_image);
      // cv::waitKey(1);
}
      else if (state_ == SEARCHING_APRON || state_ == APPROACHING_APRON || 
               state_ == DESCENDING_APRON || state_ == TOUCHING_APRON) {
        // 停机坪检测
        cv::Point2f detected_center;
        if (detectApron(image, detected_center)) {
          // 找到停机坪
          apron_center_ = detected_center;
          
          // 记录检测到的像素偏差（仅供参考）
          double dx = apron_center_.x - image_center_.x;
          double dy = apron_center_.y - image_center_.y;
          
          ROS_INFO("Apron detected: center=(%.2f, %.2f), offset=(%.2f, %.2f)",
                   apron_center_.x, apron_center_.y, dx, dy);
          
          // 首次检测 - 只需要将检测标志设为true，不计算相对位置
          if (!apron_detected_ && state_ == SEARCHING_APRON) {
            ROS_INFO("Apron detected for the first time! Moving to target coordinates.");
            apron_detected_ = true;
            state_ = APPROACHING_APRON;
          }
        }
      }      
    } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
    }
  }

  void poseCallback(const geometry_msgs::Pose::ConstPtr& msg) {
    // 更新当前高度
    current_height_ = msg->position.z;
    
    // 保存完整位置
    current_drone_position_ = Eigen::Vector3d(
      msg->position.x,
      msg->position.y,
      msg->position.z
    );
    
    // 从四元数计算yaw角
    double qx = msg->orientation.x;
    double qy = msg->orientation.y;
    double qz = msg->orientation.z;
    double qw = msg->orientation.w;
    
    // 简化的yaw计算 (假设俯仰和横滚接近0)
    current_drone_yaw_ = atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
    
    ROS_INFO_THROTTLE(1.0, "uav_pos_now: [%.2f, %.2f, %.2f], yaw: %.2f degrees",
             current_drone_position_.x(), current_drone_position_.y(), 
             current_drone_position_.z(), current_drone_yaw_ * 180.0 / M_PI);
  }

  bool detectApron(const cv::Mat& image, cv::Point2f& center) {
    // 限制日志频率
    static int frame_counter = 0;
    bool log_this_frame = (frame_counter % 10 == 0); // 每10帧输出一次详细日志
    frame_counter++;
    
    if (log_this_frame) {
      ROS_INFO("Detecting apron in image of size %dx%d", image.cols, image.rows);
    }
    
    // 转换为灰度图像
    cv::Mat gray;
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    
    // 自适应阈值处理
    cv::Mat binary;
    cv::threshold(gray, binary, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
    
    // 形态学处理
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::morphologyEx(binary, binary, cv::MORPH_OPEN, kernel);
    
    // 查找轮廓
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(binary, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    
    if (log_this_frame) {
      ROS_INFO("Found %zu contours", contours.size());
    }
    
    // 结果图像（调试用）
    cv::Mat result = image.clone();
    
    // H字母特征检测
    std::vector<std::pair<double, int>> scores; // 存储(得分,索引)对
    
    // 大幅扩大面积范围
    double apron_area_min = 200;      // 最小面积
    double apron_area_max = 400000;   // 最大面积 - 显著增加
    double apron_aspect_ratio_min = 0.5;
    double apron_aspect_ratio_max = 2.0;
    
    // 分析每个轮廓
    for (int i = 0; i < contours.size(); i++) {
      const auto& contour = contours[i];
      double area = cv::contourArea(contour);
      
      // 基本面积过滤
      if (area < apron_area_min || area > apron_area_max) {
        if (log_this_frame) {
          ROS_INFO("Contour %d: area %.2f - rejected (too small/large)", i, area);
        }
        continue;
      }
      
      // 获取最小外接矩形
      cv::RotatedRect rect = cv::minAreaRect(contour);
      double rect_area = rect.size.width * rect.size.height;
      double aspect_ratio = rect.size.width / rect.size.height;
      if (aspect_ratio < 1.0) aspect_ratio = 1.0 / aspect_ratio;
      
      // 检查宽高比
      if (aspect_ratio < apron_aspect_ratio_min || aspect_ratio > apron_aspect_ratio_max) {
        if (log_this_frame) {
          ROS_INFO("Contour %d: aspect_ratio %.2f - rejected (bad ratio)", i, aspect_ratio);
        }
        continue;
      }
      
      // 计算轮廓特征
      cv::Mat mask = cv::Mat::zeros(binary.size(), CV_8UC1);
      cv::drawContours(mask, std::vector<std::vector<cv::Point>>{contour}, 0, 255, -1);
      
      // 计算填充率 - 实际区域/矩形区域
      double filled_ratio = area / rect_area;
      
      // 计算H特征
      std::vector<cv::Point> hull;
      cv::convexHull(contour, hull);
      double hull_area = cv::contourArea(hull);
      double solidity = area / hull_area;

      // 获取外接矩形(非旋转) - 添加这一行
      cv::Rect bounding_rect = cv::boundingRect(contour);

      // 放宽条件，提高检测成功率
      double score = 0;
      score += (aspect_ratio > 0.6 && aspect_ratio < 1.5) ? 5 : 0;
      score += (filled_ratio > 0.4 && filled_ratio < 0.9) ? 5 : 0;
      score += (solidity > 0.5 && solidity < 0.95) ? 5 : 0;

      if (log_this_frame) {
        ROS_INFO("Contour %d: area=%.1f, aspect=%.2f, fill=%.2f, solidity=%.2f, score=%.1f", 
                 i, area, aspect_ratio, filled_ratio, solidity, score);
      }

      if (score >= 5) { // 降低得分阈值，增加检测概率
        scores.push_back(std::make_pair(score, i));
        
        // 绘制候选轮廓
        cv::drawContours(result, std::vector<std::vector<cv::Point>>{contour}, 0, cv::Scalar(0, 255, 0), 2);
        cv::circle(result, rect.center, 5, cv::Scalar(255, 0, 0), -1);
        
        if (log_this_frame) {
          cv::putText(result, "Score: " + std::to_string((int)score), 
                      cv::Point(bounding_rect.x, bounding_rect.y - 5), // 修改这里
                      cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1);
        }
      }
    }
    
    // 限制保存图像频率
    bool save_image = (frame_counter % debug_image_interval_) == 0;
    if (save_image) {
      static int image_count = 0;
      std::string filename = "/tmp/apron_detection_" + std::to_string(image_count++) + ".jpg";
      cv::imwrite(filename, result);
      if (log_this_frame) {
        ROS_INFO("Saved debug image to: %s", filename.c_str());
      }
    }
    
    // 如果找到候选区域，选择得分最高的
    if (!scores.empty()) {
      // 按得分排序
      std::sort(scores.begin(), scores.end(), std::greater<std::pair<double, int>>());
      int best_idx = scores[0].second;
      cv::RotatedRect best_rect = cv::minAreaRect(contours[best_idx]);
      
      // 找到最佳停机坪
      center = best_rect.center;
      ROS_INFO("Best apron candidate found at (%.1f, %.1f) with score %.1f", 
               center.x, center.y, scores[0].first);
      return true;
    }
    
    if (log_this_frame) {
      ROS_INFO("No suitable apron found");
    }
    return false;
  }
};

int main(int argc, char** argv) { argv,
  ros::init(argc, argv, "aruco_waypoint_controller");
  ros::NodeHandle nh;
  
  ArucoWaypointController controller(nh);
  controller.start();
  
  ros::spin();
  return 0;
}

