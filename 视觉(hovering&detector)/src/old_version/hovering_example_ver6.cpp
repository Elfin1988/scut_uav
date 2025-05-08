#include <thread>
#include <chrono>
#include <vector>
#include <Eigen/Core>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <geometry_msgs/Pose.h>

// 状态枚举定义整合
enum FlightState {
    INIT,
    FIRST_WAYPOINT_SEQUENCE,
    SEARCHING_ARUCO,
    LANDING,
    SECOND_WAYPOINT_SEQUENCE,
    SEARCHING_APRON,
    APPROACHING_APRON,
    DESCENDING_APRON,
    TOUCHING_APRON,
    LANDED_APRON,
    THIRD_WAYPOINT_SEQUENCE,  // 新增：前往第二个停机坪
    SEARCHING_SECOND_APRON,   // 新增：寻找第二个停机坪
    APPROACHING_SECOND_APRON, // 新增：接近第二个停机坪
    DESCENDING_SECOND_APRON,  // 新增：第二次降落过程
    TOUCHING_SECOND_APRON,    // 新增：第二次降落完成
    COMPLETED
};

class ArucoWaypointController {
public:
    ArucoWaypointController(ros::NodeHandle& nh) : nh_(nh), state_(INIT), aruco_detected_(false) {
        // 创建发布器和订阅器
        trajectory_pub_ = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
            mav_msgs::default_topics::COMMAND_TRAJECTORY, 10);
        image_sub_ = nh_.subscribe("/hummingbird/vi_sensor/camera_depth/camera/image_raw", 1, 
            &ArucoWaypointController::imageCallback, this);
        pose_sub_ = nh_.subscribe("/hummingbird/ground_truth/pose", 1,
            &ArucoWaypointController::poseCallback, this);
        
        initializeWaypoints();
        
        // 初始化配置参数
        kp_horizontal_ = 0.0010;
        kp_vertical_ = 0.0010;
        
        // 停机坪检测参数
        debug_image_interval_ = 20;  // 增大间隔，减少文件保存
        
        // 目标着陆位置
        target_landing_position_ = Eigen::Vector3d(2.2, 1.5, 2.0);
    }

    void start() {
        unpauseGazebo();
        ros::Duration(2.0).sleep();
        state_ = FIRST_WAYPOINT_SEQUENCE;
        executeFlightPlan();
    }

private:
    // 核心ROS组件
    ros::NodeHandle nh_;
    ros::Publisher trajectory_pub_;
    ros::Subscriber image_sub_;
    ros::Subscriber pose_sub_;
    
    // 状态变量
    FlightState state_;
    bool aruco_detected_ = false;
    bool apron_detected_ = false;
    
    // 位置信息
    Eigen::Vector3d aruco_position_;
    Eigen::Vector3d apron_position_;
    Eigen::Vector3d current_drone_position_;
    Eigen::Vector3d target_landing_position_;
    
    // 航点序列
    std::vector<Eigen::Vector3d> first_waypoints_;
    std::vector<double> first_yaws_;
    std::vector<double> first_durations_;
    std::vector<Eigen::Vector3d> second_waypoints_;
    std::vector<double> second_yaws_;
    std::vector<double> second_durations_;
    
    // aruco滤波器相关变量
    bool is_position_initialized_ = false;
    Eigen::Vector3d filtered_aruco_position_;
    double filter_alpha_ = 0.2; // 滤波系数(0-1)，越小滤波越强
    int anomaly_count_ = 0; // 异常计数
    const int MAX_ANOMALY_COUNT = 3; // 最大允许异常次数
    double position_jump_threshold_ = 0.5; // 位置突变阈值(米)

    // 允许的ArUco ID列表
    std::vector<int> allowed_aruco_ids_ = {43, 19}; // 只跟踪这些ID的标记

    // 相机和控制参数
    cv::Point2f marker_center_;
    cv::Point2f apron_center_;
    cv::Point2f image_center_;
    double kp_horizontal_;
    double kp_vertical_;
    double current_height_ = 0.0;
    double current_drone_yaw_ = 0.0;
    
    // 调试参数
    int debug_image_interval_;
    ros::Time last_apron_time_; // 记录最后一次检测到停机坪的时间

    // 方法实现
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
            {3.0, 6.0, 1.6},
            {0.3, 7.0, 1.6}
        };
        first_yaws_ = {0.0, 0.0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
        first_durations_ = {0.5, 0.6, 0.6, 0.6, 0.6, 0.6, 0.6, 0.6, 0.6, 0.6, 0.6, 1.5};

        // 第二段航点
        second_waypoints_ = {
            {-2.6, 5.5, 1.8},
            {-2.7, 5.4, 1.0},
            {-3.3, 5.5, 1.0},
            {-4.0, 5.5, 1.0},
            {-4.0, 4.5, 1.0},
            {-4.0, 3.5, 1.0},
            {-4.0, 2.5, 1.0},
            {-3.5, 1.5, 1.0},
            {-2.7, 0.5, 1.0},
            {-1.5, -0.5, 1.0},
            {1.7, 3.6, 3.2}
        };
        second_yaws_ = {0.0, 0.0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
        second_durations_ = {2, 0.6, 0.6, 0.4, 0.4, 0.4, 0.4, 0.4, 0.8, 1.2, 1.8};
    }

    void executeFlightPlan() {
        while (state_ != COMPLETED && ros::ok()) {
            ROS_INFO_THROTTLE(1.0, "Current state: %d", (int)state_);
            
            switch (state_) {
                case FIRST_WAYPOINT_SEQUENCE:
                    {
                        ROS_INFO("Executing first waypoint sequence");
                        executeWaypoints(first_waypoints_, first_yaws_, first_durations_);
                        state_ = SEARCHING_ARUCO;
                        ROS_INFO("First waypoint sequence completed, searching for ArUco marker");
                    }
                    break;
                    
                case LANDING:
                    {
                        // 只调用performLanding()但不改变状态
                        // 状态变更将在performLanding()内部完成
                        performLanding();
                    }
                    break;
                    
                case SECOND_WAYPOINT_SEQUENCE:
                    {
                        ROS_INFO("Executing second waypoint sequence");
                        executeWaypoints(second_waypoints_, second_yaws_, second_durations_);
                        state_ = SEARCHING_APRON;
                        ROS_INFO("Second waypoint sequence completed, searching for apron");
                    }
                    break;
                    
                case THIRD_WAYPOINT_SEQUENCE:
                    {
                        // 前往第二个停机坪上方
                        ROS_INFO("First landing successful, flying above second landing zone");
                        Eigen::Vector3d target_position(-1.5, -1.0, 1.5);
                        
                        // 发布目标位置
                        publishTrajectory(target_position, current_drone_yaw_);
                        
                        // 等待足够时间到达目标位置
                        ros::Duration(2.0).sleep();
                        
                        // 检查是否接近目标位置
                        double dist_to_target = (current_drone_position_ - target_position).norm();
                        if (dist_to_target < 0.5) {
                            ROS_INFO("Reached second landing zone, position: [%.2f, %.2f, %.2f]", 
                                    current_drone_position_.x(), current_drone_position_.y(), current_drone_position_.z());
                            state_ = SEARCHING_SECOND_APRON;
                            apron_detected_ = false; // 重置停机坪检测标志
                        } else {
                            ROS_WARN("Failed to reach second landing zone, retrying...");
                            // 可以考虑增加重试计数或容错逻辑
                        }
                    }
                    break;
                    
                case SEARCHING_SECOND_APRON:
                    if (searchTarget(20.0, apron_detected_)) {
                        state_ = APPROACHING_SECOND_APRON;
                        ROS_INFO("Second apron found, starting approach");
                    } else {
                        ROS_WARN("Second apron not found after timeout, mission aborted");
                        state_ = COMPLETED;
                    }
                    break;
                    
                case APPROACHING_APRON:
                case DESCENDING_APRON:
                case TOUCHING_APRON:
                    performApronLanding();
                    break;
                    
                case APPROACHING_SECOND_APRON:
                case DESCENDING_SECOND_APRON:
                case TOUCHING_SECOND_APRON:
                    performApronLanding();
                    break;
                    
                // 其他状态处理...
                
                default:
                    break;
            }
            
            ros::spinOnce();
            ros::Duration(0.1).sleep();
        }
    }

    bool searchTarget(double timeout, bool& detected_flag) {
        ros::Time start_time = ros::Time::now();
        while (!detected_flag && ros::Time::now() - start_time < ros::Duration(timeout)) {
            ros::spinOnce();
            ros::Duration(0.1).sleep();
        }
        return detected_flag;
    }

// 修改后的performApronLanding函数 - 简化水平对齐逻辑
void performApronLanding() {
    static int landing_stage = 0;
    static ros::Time stage_start_time;
    static ros::Time align_start_time;
    static bool alignment_stable = false;
    static int stable_frames = 0;
    static const int REQUIRED_STABLE_FRAMES = 10; // 连续10帧稳定才算真正对齐
    static const double ALIGNMENT_THRESHOLD = 10.0; // 对齐阈值，像素单位
    
    bool is_first_landing = (state_ == APPROACHING_APRON || state_ == DESCENDING_APRON || 
                          state_ == TOUCHING_APRON || state_ == LANDED_APRON);
    bool is_second_landing = (state_ == APPROACHING_SECOND_APRON || state_ == DESCENDING_SECOND_APRON || 
                           state_ == TOUCHING_SECOND_APRON);
    
    const double TARGET_HEIGHT_FIRST = 1.9;  // 第一次降落目标高度
    const double TARGET_HEIGHT_SECOND = 0.0; // 第二次降落目标高度
    const double HEIGHT_TOLERANCE = 0.1;     // 高度误差容忍
    
    double target_height = is_first_landing ? TARGET_HEIGHT_FIRST : TARGET_HEIGHT_SECOND;
    
    switch (landing_stage) {
        case 0:  // 初始化
            if (is_first_landing) {
                ROS_INFO("First apron landing initiated");
            } else {
                ROS_INFO("Second apron landing initiated");
            }
            stage_start_time = ros::Time::now();
            align_start_time = ros::Time::now();
            alignment_stable = false;
            stable_frames = 0;
            landing_stage = 1;
            break;
            
        case 1:  // 水平对齐阶段 - 强制要求稳定对齐
            {
                // 检查是否能检测到停机坪
                if (!apron_detected_) {
                    ROS_WARN_THROTTLE(1.0, "Apron not detected, hovering and waiting");
                    publishTrajectory(current_drone_position_, current_drone_yaw_);
                    stable_frames = 0; // 重置稳定计数
                    alignment_stable = false;
                    break;
                }
                
                // 计算图像中心与停机坪中心的偏差
                double dx = apron_center_.x - image_center_.x; // 水平偏差
                double dy = apron_center_.y - image_center_.y; // 垂直偏差
                
                // 显示当前偏差和稳定帧数
                ROS_INFO_THROTTLE(0.5, "Alignment: dx=%.1f, dy=%.1f, stable_frames=%d/%d", 
                                 dx, dy, stable_frames, REQUIRED_STABLE_FRAMES);
                
                // 检查是否在阈值范围内
                bool is_aligned = (std::abs(dx) < ALIGNMENT_THRESHOLD && std::abs(dy) < ALIGNMENT_THRESHOLD);
                
                // 如果对齐，增加稳定帧计数，否则重置
                if (is_aligned) {
                    stable_frames++;
                    if (!alignment_stable && stable_frames >= REQUIRED_STABLE_FRAMES) {
                        alignment_stable = true;
                        ROS_INFO("!!! TARGET ALIGNMENT STABLE !!! Frames: %d", stable_frames);
                        
                        // 给操作员一个清晰的视觉反馈
                        for (int i = 0; i < 3; i++) {
                            ROS_INFO("=== CENTERED AND STABLE: PROCEEDING TO DESCENT ===");
                        }
                        
                        // 记录稳定位置，进入下一阶段
                        landing_stage = 2;
                        stage_start_time = ros::Time::now();
                        
                        if (is_first_landing) {
                            state_ = DESCENDING_APRON;
                        } else {
                            state_ = DESCENDING_SECOND_APRON;
                        }
                        break;
                    }
                } else {
                    // 不对齐，重置稳定帧计数
                    stable_frames = 0;
                    alignment_stable = false;
                }
                
                // --- 更精细的调整逻辑 ---
                
                // 根据偏差大小计算步长
                const double MAX_STEP = 0.5; // 控制微调的最大步长50厘米
                
                // 计算移动方向和大小（比例控制）
                double move_x = 0.0;
                double move_y = 0.0;
                
                // X方向移动 (前后) - 与dy相关
                // X方向移动 (前后) - 与dy相关 - 修正方向
                if (std::abs(dy) > ALIGNMENT_THRESHOLD/3) {
                    double step = std::min(MAX_STEP, std::abs(dy) * 0.002);
                    // 修正：如果标记在图像下方(dy > 0)，无人机向前移动；如果标记在图像上方(dy < 0)，无人机向后移动
                    move_x = (dy > 0) ? -step : step;  // 修正方向关系
                }

                // 修正Y方向移动 (左右) - 与dx相关
                if (std::abs(dx) > ALIGNMENT_THRESHOLD/3) {
                    double step = std::min(MAX_STEP, std::abs(dx) * 0.002);
                    // 修正：如果标记在图像右侧(dx > 0)，无人机向右移动；如果标记在图像左侧(dx < 0)，无人机向左移动
                    move_y = (dx > 0) ? -step : step;  // 修正方向关系
                }

                // 计算新位置 - 添加这个代码块
                Eigen::Vector3d target_position(
                    current_drone_position_.x() + move_x,
                    current_drone_position_.y() + move_y,
                    current_drone_position_.z()
                );

                // 记录移动信息 - 添加明确的方向提示
                std::string move_str = "";
                if (std::abs(move_x) < 0.001 && std::abs(move_y) < 0.001) {
                    move_str = "保持位置";
                } else {
                    if (move_x > 0) move_str += "前进(标记在下方) ";
                    if (move_x < 0) move_str += "后退(标记在上方) ";
                    if (move_y > 0) move_str += "左移(标记在左侧) ";
                    if (move_y < 0) move_str += "右移(标记在右侧) ";
                }
                
                // 发布轨迹
                publishTrajectory(target_position, current_drone_yaw_);
                
                ROS_INFO_THROTTLE(0.5, "移动: %s [dx=%.1f, dy=%.1f] 稳定帧: %d/%d", 
                                move_str.c_str(), dx, dy, stable_frames, REQUIRED_STABLE_FRAMES);
            }
            break;
            
        case 2:  // 垂直移动阶段 - 继续保持水平对齐
            {
                // 检查是否已达到目标高度(±0.1误差范围)
                if (std::abs(current_height_ - target_height) <= HEIGHT_TOLERANCE) {
                    ROS_INFO("Reached target height: %.2f m (target: %.2f ± %.2f)",
                            current_height_, target_height, HEIGHT_TOLERANCE);
                    landing_stage = 3;
                    stage_start_time = ros::Time::now();
                    if (is_first_landing) {
                        state_ = TOUCHING_APRON;
                    } else {
                        state_ = TOUCHING_SECOND_APRON;
                    }
                    break;
                }
                
                // 计算垂直移动量
                double height_diff = current_height_ - target_height;
                double move_z = 0;
                
                // 自适应速率控制,控制不同海拔下的降落速度
                //50厘米/次，快速接近
                if (std::abs(height_diff) > 1.0) {
                    move_z = (height_diff > 0) ? -0.5 : 0.5;
                } else if (std::abs(height_diff) > 0.5) {
                    move_z = (height_diff > 0) ? -0.5 : 0.5; 
                } else if (std::abs(height_diff) > 0.2) {
                    move_z = (height_diff > 0) ? -0.5 : 0.5; 
                } else {
                    move_z = (height_diff > 0) ? -0.5 : 0.5; 
                }
                
                // 构建新位置向量
                Eigen::Vector3d target_position = current_drone_position_;
                target_position.z() += move_z;
                
                // 同时保持水平对齐
                if (apron_detected_) {
                    double dx = apron_center_.x - image_center_.x;
                    double dy = apron_center_.y - image_center_.y;
                    
                    // 下降时在偏差较大时进行修正，一般用不到
                    if (std::abs(dx) > ALIGNMENT_THRESHOLD) {
                        double step = std::min(0.03, std::abs(dx) * 0.0003);
                        target_position.y() += (dx > 0) ? -step : step;
                    }
                    
                    if (std::abs(dy) > ALIGNMENT_THRESHOLD) {
                        double step = std::min(0.03, std::abs(dy) * 0.0003);
                        target_position.x() += (dy > 0) ? -step : step;
                    }
                    
                    ROS_INFO_THROTTLE(1.0, "降落过程中保持对齐: dx=%.1f, dy=%.1f", dx, dy);
                } else {
                    ROS_WARN_THROTTLE(1.0, "降落过程中停机坪丢失！继续按当前位置下降");
                }
                
                // 发布轨迹
                publishTrajectory(target_position, current_drone_yaw_);
                
                ROS_INFO_THROTTLE(0.5, "%s中: 当前=%.2f, 目标=%.2f, 差=%.2f", 
                                (height_diff > 0) ? "下降" : "上升",
                                current_height_, target_height, height_diff);
            }
            break;
            
        case 3:  // 完成降落
            {
                // 短暂稳定在当前位置
                publishTrajectory(current_drone_position_, current_drone_yaw_);
                
                // 等待稳定期
                if (ros::Time::now() - stage_start_time > ros::Duration(1.0)) {
                    if (is_first_landing) {
                        ROS_INFO("First apron landing completed!");
                        landing_stage = 0; // 重置状态便于下次使用
                        state_ = THIRD_WAYPOINT_SEQUENCE; // 第一次降落后进入第三航段
                    } else {
                        ROS_INFO("Second apron landing completed!");
                        landing_stage = 0; // 重置状态便于下次使用
                        state_ = COMPLETED; // 第二次降落后任务完成
                        ROS_INFO("Full mission completed successfully!");
                    }
                }
            }
            break;
    }
}

    void publishTrajectory(const Eigen::Vector3d& position, double yaw) {
        trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
        mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(position, yaw, &trajectory_msg);
        trajectory_msg.header.stamp = ros::Time::now();
        trajectory_pub_.publish(trajectory_msg);
    }

    void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        
        cv::Mat image = cv_ptr->image;
        image_center_ = cv::Point2f(image.cols / 2.0, image.rows / 2.0);
        
        // 根据当前状态选择检测方法
        if (state_ == SEARCHING_ARUCO || state_ == LANDING) {
            // 创建调试图像
            cv::Mat debug_image = image.clone();
            
            // ArUco 检测代码
            cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
            std::vector<int> ids;
            std::vector<std::vector<cv::Point2f>> corners;
            cv::aruco::detectMarkers(image, dictionary, corners, ids);
            
            // 绘制检测结果
            cv::aruco::drawDetectedMarkers(debug_image, corners, ids);
            
            // 绘制中心参考线 (十字线)
            cv::line(debug_image, 
                    cv::Point(image_center_.x, 0), 
                    cv::Point(image_center_.x, image.rows), 
                    cv::Scalar(255, 255, 0), 1);
            cv::line(debug_image, 
                    cv::Point(0, image_center_.y), 
                    cv::Point(image.cols, image_center_.y), 
                    cv::Scalar(255, 255, 0), 1);
            
            if (ids.size() > 0) {
                // 找到了ArUco标记，但需要检查是否是允许的ID
                int valid_marker_idx = -1;
                
                for (size_t i = 0; i < ids.size(); i++) {
                    // 检查当前ID是否在允许列表中
                    bool is_allowed = false;
                    for (int allowed_id : allowed_aruco_ids_) {
                        if (ids[i] == allowed_id) {
                            is_allowed = true;
                            break;
                        }
                    }
                    
                    if (is_allowed) {
                        valid_marker_idx = i;
                        ROS_INFO_THROTTLE(1.0, "Found allowed ArUco marker ID: %d", ids[i]);
                        break;
                    }
                }
                if (valid_marker_idx >= 0) {
                    // 提取标记角点和中心
                    std::vector<cv::Point2f> marker_corners = corners[valid_marker_idx];
                    marker_center_ = cv::Point2f(0, 0);
                    for (auto& corner : marker_corners) {
                        marker_center_ += corner;
                    }
                    marker_center_ *= 0.25f; // 取平均得到中心点
                
                // 绘制标记中心
                cv::circle(debug_image, marker_center_, 5, cv::Scalar(0, 0, 255), -1);

                // 在图像上显示ID - 添加ID显示
                cv::putText(debug_image, 
                    "ID: " + std::to_string(ids[valid_marker_idx]),  // 使用valid_marker_idx
                    cv::Point(marker_center_.x + 10, marker_center_.y),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
                
                // 打印标记中心点像素坐标
                ROS_INFO_THROTTLE(1.0, "marker_center_pixel: (%.2f, %.2f)", marker_center_.x, marker_center_.y);
                ROS_INFO_THROTTLE(1.0, "image_center_pixel: (%.2f, %.2f)", image_center_.x, image_center_.y);
                
                // 计算并显示偏差
                double dx = marker_center_.x - image_center_.x;
                double dy = marker_center_.y - image_center_.y;
                cv::putText(debug_image, 
                        cv::format("ArUco offset: dx=%.1f, dy=%.1f", dx, dy),
                        cv::Point(20, 60),
                        cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
                
                // 增加ArUco位置与无人机位置的关系显示
                cv::putText(debug_image, 
                            cv::format("无人机位置: [%.2f, %.2f, %.2f]", 
                                    current_drone_position_.x(), current_drone_position_.y(), current_height_),
                            cv::Point(20, 150),
                            cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 128, 0), 2);
                           
                // 状态信息
                if (state_ == SEARCHING_ARUCO) {
                    cv::putText(debug_image, "SEARCHING ARUCO", cv::Point(20, 30),
                               cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
                } else {
                    cv::putText(debug_image, "LANDING ON ARUCO", cv::Point(20, 30),
                               cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
                }

                 // 添加方向指示器 - 显示移动方向的箭头
                if (state_ == LANDING) {
                    cv::Point2f center(image.cols/2, image.rows/2);
                    cv::Point2f direction(center.x + dx/3, center.y + dy/3); // 反向，因为要向标记移动
                    cv::arrowedLine(debug_image, center, direction, cv::Scalar(0, 255, 255), 2);
                    
                    cv::putText(debug_image, "Moving direction", direction - cv::Point2f(20, 20),
                            cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 1);
                }
                
                // 使用实际的vi_sensor相机内参
                cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << 
                                     205.46963709898583, 0.0, 320.5, 
                                     0.0, 205.46963709898583, 240.5, 
                                     0.0, 0.0, 1.0);
                cv::Mat dist_coeffs = cv::Mat::zeros(5, 1, CV_64F); // 无畸变
                
                // 计算姿态和位置
                std::vector<cv::Vec3d> rvecs, tvecs;
                cv::aruco::estimatePoseSingleMarkers(
                    std::vector<std::vector<cv::Point2f>>{marker_corners},  // 使用正确处理过的corners
                    0.1,  // 标记大小(米)
                    camera_matrix, 
                    dist_coeffs,
                    rvecs, 
                    tvecs);
    
                // 使用drawAxis可视化姿态
                cv::aruco::drawAxis(debug_image, camera_matrix, dist_coeffs,
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
    
    
                // 计算原始位置（不含补偿）
                Eigen::Vector3d raw_aruco_position(
                    current_drone_position_.x() + world_dx,
                    current_drone_position_.y() + world_dy,
                    current_height_ - marker_dz // 使用当前高度减去标记深度
                );

                // 应用指数滤波和异常值检测
                if (!is_position_initialized_) {
                    // 第一次检测，直接初始化
                    filtered_aruco_position_ = raw_aruco_position;
                    is_position_initialized_ = true;
                    anomaly_count_ = 0;
                } else {
                    // 检查是否为异常值(位置突变)
                    double position_diff = (raw_aruco_position - filtered_aruco_position_).norm();
                    
                    if (position_diff > position_jump_threshold_) {
                        // 可能是异常值
                        anomaly_count_++;
                        ROS_WARN("Possible ArUco position anomaly detected: %.2f m jump", position_diff);
                        
                        // 如果连续异常超过阈值，则接受新值，但使用更强的滤波
                        if (anomaly_count_ > MAX_ANOMALY_COUNT) {
                            filtered_aruco_position_ = filtered_aruco_position_ * 0.7 + raw_aruco_position * 0.3;
                            anomaly_count_ = 0;
                            ROS_WARN("Accepting new ArUco position after multiple anomalies");
                        }
                        // 否则保持使用旧值
                    } else {
                        // 正常值，应用指数滤波
                        filtered_aruco_position_ = filtered_aruco_position_ * (1 - filter_alpha_) + 
                                                raw_aruco_position * filter_alpha_;
                        anomaly_count_ = 0;
                    }
                }

                // 使用滤波后的位置
                aruco_position_ = filtered_aruco_position_;

                // 添加调试信息 - 显示原始位置和滤波后位置
                cv::putText(debug_image, 
                    cv::format("Raw pos: [%.2f, %.2f, %.2f]", 
                            raw_aruco_position.x(), raw_aruco_position.y(), raw_aruco_position.z()),
                    cv::Point(20, 180),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 255), 2);
                           
                // 距离信息 - 用红色显示
                cv::putText(debug_image, 
                        cv::format("Distance: %.2f m", marker_dz),
                        cv::Point(20, 120),
                        cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 255), 2);
                
                // 首次检测到标记
                if (!aruco_detected_ && state_ == SEARCHING_ARUCO) {
                  ROS_INFO("ArUco marker detected!");
                  aruco_detected_ = true;
                  state_ = LANDING;
                }
                
                // 保存调试图像
                static int image_count = 0;
                if (image_count++ % 10 == 0) {
                    cv::imwrite("/tmp/aruco_detection_" + std::to_string(image_count/10) + ".jpg", debug_image);
                }

                }
                
            }
            else {
                // 没有检测到ArUco
                cv::putText(debug_image, "NO ARUCO DETECTED", cv::Point(20, 30),
                           cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
            }
            
            // 显示调试图像
            cv::imshow("ArUco Detection", debug_image);
            cv::waitKey(1);
        } 
        else if (state_ == SEARCHING_APRON || state_ == APPROACHING_APRON || 
                 state_ == DESCENDING_APRON || state_ == TOUCHING_APRON ||
                 state_ == SEARCHING_SECOND_APRON || state_ == APPROACHING_SECOND_APRON || 
                 state_ == DESCENDING_SECOND_APRON || state_ == TOUCHING_SECOND_APRON) {
      
            // 统一的停机坪检测逻辑，适用于两个停机坪
            cv::Point2f detected_center;
            if (detectApron(image, detected_center)) {
                // 找到停机坪 - 更新中心点
                apron_center_ = detected_center;
                
                // 计算并显示偏差（仅供调试）
                double dx = apron_center_.x - image_center_.x;
                double dy = apron_center_.y - image_center_.y;
                
                ROS_INFO_THROTTLE(1.0, "Detected ArUco position: [%.2f, %.2f, %.2f]",
                    aruco_position_.x(), aruco_position_.y(), aruco_position_.z());
                
                // 首次检测到停机坪
                if (!apron_detected_) {
                  if (state_ == SEARCHING_APRON) {
                    ROS_INFO("First apron detected! Beginning approach");
                    apron_detected_ = true;
                    state_ = APPROACHING_APRON;
                  } 
                  else if (state_ == SEARCHING_SECOND_APRON) {
                    ROS_INFO("Second apron detected! Beginning approach");
                    apron_detected_ = true;
                    state_ = APPROACHING_SECOND_APRON;
                  }
                }
            } else {
                // 未检测到停机坪
                if ((state_ != SEARCHING_APRON && state_ != SEARCHING_SECOND_APRON) && 
                    ros::Time::now() - last_apron_time_ > ros::Duration(2.0)) {
                  ROS_WARN_THROTTLE(1.0, "Lost apron for over 2 seconds");
                  apron_detected_ = false;
                }
            }
            
            // 记录最后一次检测到停机坪的时间
            if (apron_detected_) {
                last_apron_time_ = ros::Time::now();
            }
        }
    }

    void poseCallback(const geometry_msgs::Pose::ConstPtr& msg) {
        current_drone_position_ = Eigen::Vector3d(msg->position.x, msg->position.y, msg->position.z);
        
        // 从四元数计算 yaw 角
        double qx = msg->orientation.x;
        double qy = msg->orientation.y;
        double qz = msg->orientation.z;
        double qw = msg->orientation.w;
        current_drone_yaw_ = atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
        
        current_height_ = msg->position.z;
    }

    void unpauseGazebo() {
        ros::ServiceClient unpause_client = nh_.serviceClient<std_srvs::Empty>("/gazebo/unpause_physics");
        std_srvs::Empty srv;
        unpause_client.call(srv);
    }

    void executeWaypoints(const std::vector<Eigen::Vector3d>& waypoints, const std::vector<double>& yaws, const std::vector<double>& durations) {
        for (size_t i = 0; i < waypoints.size(); ++i) {
            publishTrajectory(waypoints[i], yaws[i]);
            ros::Duration(durations[i]).sleep();
        }
    }

    void performLanding() {
        static int landing_stage = 0;
        static ros::Time stage_start_time;
        static ros::Time align_start_time;
        static bool alignment_stable = false;
        static int stable_frames = 0;
        static const int REQUIRED_STABLE_FRAMES = 10;
        static const double ALIGNMENT_THRESHOLD = 15.0;
        
        // 修改降落目标高度
        const double TARGET_HEIGHT = 0.0;  // 理论上的目标高度（地面）
        const double SAFE_LANDING_HEIGHT = 0.4;  // 实际安全降落高度
        const double HEIGHT_TOLERANCE = 0.1;
        
        switch (landing_stage) {
            case 0:  // 初始化
                ROS_INFO("ArUco marker landing initiated");
                ROS_INFO("ArUco marker detected at [%.1f, %.1f, %.1f], performing landing",
                         aruco_position_.x(), aruco_position_.y(), aruco_position_.z());
                
                stage_start_time = ros::Time::now();
                align_start_time = ros::Time::now();
                alignment_stable = false;
                stable_frames = 0;
                landing_stage = 1;
                break;
                
            case 1:  // 水平对齐阶段 - 强制要求稳定对齐
                {
                    // 检查是否能检测到ArUco标记
                    if (!aruco_detected_) {
                        ROS_WARN_THROTTLE(1.0, "ArUco marker not detected, hovering and waiting");
                        publishTrajectory(current_drone_position_, current_drone_yaw_);
                        stable_frames = 0; // 重置稳定计数
                        alignment_stable = false;
                        break;
                    }
                    
                    // 计算图像中心与ArUco标记中心的偏差
                    double dx = marker_center_.x - image_center_.x; // 水平偏差
                    double dy = marker_center_.y - image_center_.y; // 垂直偏差
                    
                    // 显示当前偏差和稳定帧数
                    ROS_INFO_THROTTLE(0.5, "ArUco Alignment: dx=%.1f, dy=%.1f, stable_frames=%d/%d", 
                                     dx, dy, stable_frames, REQUIRED_STABLE_FRAMES);
                    
                    // 检查是否在阈值范围内
                    bool is_aligned = (std::abs(dx) < ALIGNMENT_THRESHOLD && std::abs(dy) < ALIGNMENT_THRESHOLD);
                    
                    // 如果对齐，增加稳定帧计数，否则重置
                    if (is_aligned) {
                        stable_frames++;
                        if (!alignment_stable && stable_frames >= REQUIRED_STABLE_FRAMES) {
                            alignment_stable = true;
                            ROS_INFO("!!! ARUCO TARGET ALIGNMENT STABLE !!! Frames: %d", stable_frames);
                            
                            // 给操作员一个清晰的视觉反馈
                            for (int i = 0; i < 3; i++) {
                                ROS_INFO("=== ARUCO CENTERED AND STABLE: PROCEEDING TO DESCENT ===");
                            }
                            
                            // 记录稳定位置，进入下一阶段
                            landing_stage = 2;
                            stage_start_time = ros::Time::now();
                            break;
                        }
                    } else {
                        // 不对齐，重置稳定帧计数
                        stable_frames = 0;
                        alignment_stable = false;
                    }
                    
                    // --- 修正的调整逻辑 ---
                    
                    // 根据偏差大小计算步长
                    const double MAX_STEP = 0.07; // 控制微调的最大步长1厘米
                    
                    // 计算移动方向和大小（比例控制）
                    double move_x = 0.0;
                    double move_y = 0.0;
                    
                     // X方向移动 (前后) - 与dy相关 - 修正方向
                    if (std::abs(dy) > ALIGNMENT_THRESHOLD/3) {
                        double step = std::min(MAX_STEP, std::abs(dy) * 0.04);
                        // 修正：如果标记在图像下方(dy > 0)，无人机向前移动；如果标记在图像上方(dy < 0)，无人机向后移动
                        move_x = (dy > 0) ? -step : step;  // 修正方向关系
                    }

                    // 修正Y方向移动 (左右) - 与dx相关
                    if (std::abs(dx) > ALIGNMENT_THRESHOLD/3) {
                        double step = std::min(MAX_STEP, std::abs(dx) * 0.04);
                        // 修正：如果标记在图像右侧(dx > 0)，无人机向右移动；如果标记在图像左侧(dx < 0)，无人机向左移动
                        move_y = (dx > 0) ? -step : step;  // 修正方向关系
                    }
                    
                    // 计算新位置 - 添加这个关键代码块
                    Eigen::Vector3d target_position(
                        current_drone_position_.x() + move_x,
                        current_drone_position_.y() + move_y,
                        current_drone_position_.z()
                    );
                    
                    // 记录移动信息 - 添加明确的方向提示
                    std::string move_str = "";
                    if (std::abs(move_x) < 0.001 && std::abs(move_y) < 0.001) {
                        move_str = "保持位置";
                    } else {
                        if (move_x > 0) move_str += "前进(标记在下方) ";
                        if (move_x < 0) move_str += "后退(标记在上方) ";
                        if (move_y > 0) move_str += "左移(标记在左侧) ";
                        if (move_y < 0) move_str += "右移(标记在右侧) ";
                    }
                    
                    // 发布轨迹
                    publishTrajectory(target_position, current_drone_yaw_);
                    
                    ROS_INFO("ArUco对齐: %s 图像偏差[dx=%.1f, dy=%.1f] 移动量[%.3f, %.3f] 稳定帧=%d/%d", 
                            move_str.c_str(), dx, dy, move_x, move_y, stable_frames, REQUIRED_STABLE_FRAMES);
                }
                break;
                
                case 2:  // 垂直移动阶段
                {
                    // 当前高度已经低于安全降落高度时，直接进入完成阶段
                    if (std::abs(current_height_ - SAFE_LANDING_HEIGHT) <= HEIGHT_TOLERANCE) {
                        ROS_INFO("Reached target height: %.2f m (target: %.2f ± %.2f)",
                                current_height_, SAFE_LANDING_HEIGHT, HEIGHT_TOLERANCE);
                        landing_stage = 3;
                        stage_start_time = ros::Time::now();
                        break;
                    }
                    
                    // 计算垂直移动量 - 始终往下移动
                    double move_z = -0.3;  // 更温和的下降速率
                    
                    // 接近安全高度时减速
                    if (current_height_ < SAFE_LANDING_HEIGHT + 0.3) {
                        move_z = -0.2;  // 更慢的下降速率
                    }
                    
                    // 构建新位置向量 - 关键修改：主要使用当前位置进行垂直下降
                    Eigen::Vector3d target_position = current_drone_position_;
                    target_position.z() += move_z;
                    
                    // 同时保持水平对齐 - 但只进行最小必要的水平校正
                    if (aruco_detected_) {
                        double dx = marker_center_.x - image_center_.x;
                        double dy = marker_center_.y - image_center_.y;
                        
                        // 只在偏移较大时进行微小校正
                        if (std::abs(dx) > ALIGNMENT_THRESHOLD) {
                            double step = std::min(0.02, std::abs(dx) * 0.0002); // 减小校正步长
                            target_position.y() += (dx > 0) ? -step : step;
                        }
                        
                        if (std::abs(dy) > ALIGNMENT_THRESHOLD) {
                            double step = std::min(0.02, std::abs(dy) * 0.0002); // 减小校正步长
                            target_position.x() += (dy > 0) ? -step : step;
                        }
                        
                        ROS_INFO_THROTTLE(1.0, "ArUco垂直降落中，保持最小对齐: dx=%.1f, dy=%.1f", dx, dy);
                    }
                    
                    // 发布轨迹
                    publishTrajectory(target_position, current_drone_yaw_);
                    
                    ROS_INFO_THROTTLE(0.5, "ArUco垂直降落中: 当前高度=%.2f m, 目标安全高度=%.2f m, 剩余距离=%.2f m", 
                                    current_height_, SAFE_LANDING_HEIGHT, 
                                    current_height_ - SAFE_LANDING_HEIGHT);
                }
                break;
                
            case 3:  // 完成降落
                {
                    // 短暂稳定在当前位置
                    publishTrajectory(current_drone_position_, current_drone_yaw_);
                    
                    // 等待稳定期
                    if (ros::Time::now() - stage_start_time > ros::Duration(1.0)) {
                        ROS_INFO("ArUco marker landing completed!");
                        landing_stage = 0; // 重置状态便于下次使用
                        
                        // 起飞继续任务
                        Eigen::Vector3d takeoff_position = current_drone_position_;
                        takeoff_position.z() = 1.0; // 起飞高度
                        publishTrajectory(takeoff_position, current_drone_yaw_);
                        
                        // 等待起飞完成
                        ros::Duration(0.5).sleep();
                        
                        // 在这里改变状态 - 这是关键修改
                        state_ = SECOND_WAYPOINT_SEQUENCE;
                        ROS_INFO("ArUco landing completed, proceeding to second waypoint sequence");
                    }
                }
                break;
        }
    }

// 全新的停机坪识别函数
bool detectApron(const cv::Mat& image, cv::Point2f& center) {
    static cv::Point2f last_valid_center;
    static bool last_detection_valid = false;
    static ros::Time last_valid_time = ros::Time::now();
    
    // 创建调试图像
    cv::Mat debug_image = image.clone();
    
    // ---------- 方法1: 模板匹配法 ----------
    // 创建"H"形状模板 (动态生成黑底白H)
    static cv::Mat h_template;
    if (h_template.empty()) {
        h_template = cv::Mat::zeros(100, 100, CV_8UC1);
        // 画"H"形状
        cv::rectangle(h_template, cv::Rect(15, 10, 20, 80), cv::Scalar(255), -1); // 左竖
        cv::rectangle(h_template, cv::Rect(65, 10, 20, 80), cv::Scalar(255), -1); // 右竖
        cv::rectangle(h_template, cv::Rect(15, 40, 70, 20), cv::Scalar(255), -1); // 中横
        cv::imshow("H Template", h_template);
    }
    
    // 转换为灰度
    cv::Mat gray;
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    
    // 创建不同尺寸的模板用于多尺度匹配
    std::vector<cv::Mat> templates;
    std::vector<double> template_scales = {0.5, 0.75, 1.0, 1.5, 2.0};
    for (double scale : template_scales) {
        cv::Mat scaled_template;
        cv::resize(h_template, scaled_template, cv::Size(), scale, scale);
        templates.push_back(scaled_template);
    }
    
    // 多尺度模板匹配
    double best_match_val = 0;
    cv::Point best_match_loc;
    cv::Size best_template_size;
    
    for (const auto& templ : templates) {
        cv::Mat result;
        cv::matchTemplate(gray, templ, result, cv::TM_CCOEFF_NORMED);
        
        double min_val, max_val;
        cv::Point min_loc, max_loc;
        cv::minMaxLoc(result, &min_val, &max_val, &min_loc, &max_loc);
        
        // 记录最佳匹配
        if (max_val > best_match_val) {
            best_match_val = max_val;
            best_match_loc = max_loc;
            best_template_size = templ.size();
        }
    }
    
    // ---------- 方法2: 自适应颜色分割 ----------
    // HSV色彩空间更好地分离白色
    cv::Mat hsv, white_mask;
    cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);
    
    // 白色范围在HSV中的定义 (低饱和度、高亮度)
    cv::inRange(hsv, cv::Scalar(0, 0, 200), cv::Scalar(180, 30, 255), white_mask);
    
    // 形态学操作强化白色区域
    cv::Mat morph_kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
    cv::morphologyEx(white_mask, white_mask, cv::MORPH_OPEN, morph_kernel);
    cv::morphologyEx(white_mask, white_mask, cv::MORPH_CLOSE, morph_kernel);
    
    // 找到候选区域
    std::vector<std::vector<cv::Point>> color_contours;
    cv::findContours(white_mask, color_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    
    // 在调试图像上绘制轮廓
    cv::drawContours(debug_image, color_contours, -1, cv::Scalar(0, 0, 255), 1);
    cv::imshow("White Mask", white_mask);
    
    // ---------- 方法3: 双边阈值自适应分割 ----------
    // 对灰度图应用自适应阈值
    cv::Mat binary;
    cv::adaptiveThreshold(gray, binary, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, 
                         cv::THRESH_BINARY, 15, -2);
    
    // 显示二值化结果
    cv::imshow("Adaptive Binary", binary);
    
    // 寻找轮廓
    std::vector<std::vector<cv::Point>> adaptive_contours;
    cv::findContours(binary, adaptive_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    
    // ---------- 决策融合 ----------
    // 模板匹配结果可信度检查
    bool template_match_valid = (best_match_val > 0.65); // 匹配度阈值
    
    if (template_match_valid) {
        // 有效的模板匹配结果
        cv::Point center_pt(best_match_loc.x + best_template_size.width/2,
                          best_match_loc.y + best_template_size.height/2);
        
        // 绘制模板匹配结果
        cv::rectangle(debug_image, 
                    cv::Rect(best_match_loc, best_template_size),
                    cv::Scalar(0, 255, 0), 2);
                    
        cv::putText(debug_image, 
                  cv::format("Match: %.2f", best_match_val),
                  cv::Point(20, 90),
                  cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
                  
        // 记录结果
        center = cv::Point2f(center_pt);
        last_valid_center = center;
        last_detection_valid = true;
        last_valid_time = ros::Time::now();
        
        // 绘制其他调试信息
        cv::circle(debug_image, center_pt, 5, cv::Scalar(255, 0, 0), -1);
        
        // 添加中心点对齐指示线
        cv::Point2f image_center(image.cols/2.0f, image.rows/2.0f);
        cv::line(debug_image, 
                cv::Point(image_center.x, 0), 
                cv::Point(image_center.x, image.rows), 
                cv::Scalar(255, 255, 0), 1);
        cv::line(debug_image, 
                cv::Point(0, image_center.y), 
                cv::Point(image_center.y, image_center.y), 
                cv::Scalar(255, 255, 0), 1);
        
        // 显示偏移
        double dx = center.x - image_center.x;
        double dy = center.y - image_center.y;
        cv::putText(debug_image, 
                   cv::format("dx: %.1f, dy: %.1f", dx, dy),
                   cv::Point(20, 60),
                   cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
        
        // 状态文本
        cv::putText(debug_image, "APRON DETECTED (TEMPLATE)", cv::Point(20, 30),
                   cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
                   
        // 保存调试图像
        static int image_count = 0;
        if (image_count++ % 10 == 0) {
            cv::imwrite("/tmp/apron_detection_" + std::to_string(image_count/10) + ".jpg", debug_image);
        }
        
        // 显示调试图像
        cv::imshow("Apron Detection", debug_image);
        cv::waitKey(1);
        
        return true;
    }
    
    // 如果模板匹配失败，使用轮廓分析
    // 合并两种方法的轮廓
    std::vector<std::vector<cv::Point>> all_contours;
    all_contours.insert(all_contours.end(), color_contours.begin(), color_contours.end());
    all_contours.insert(all_contours.end(), adaptive_contours.begin(), adaptive_contours.end());
    
    // 分析轮廓找出最像H形状的
    int best_contour_idx = -1;
    double best_score = 0;
    
    for (size_t i = 0; i < all_contours.size(); i++) {
        double area = cv::contourArea(all_contours[i]);
        if (area < 100) continue; // 忽略太小的轮廓
        
        // 计算边界框和旋转矩形
        cv::Rect bbox = cv::boundingRect(all_contours[i]);
        cv::RotatedRect rot_rect = cv::minAreaRect(all_contours[i]);
        
        // 计算轮廓特征
        double aspect_ratio = std::max(bbox.width, bbox.height) / (double)std::min(bbox.width, bbox.height);
        double extent = area / (bbox.width * bbox.height);
        
        // 计算轮廓中心距图像中心的距离
        cv::Point2f contour_center(bbox.x + bbox.width/2.0f, bbox.y + bbox.height/2.0f);
        cv::Point2f img_center(image.cols/2.0f, image.rows/2.0f);
        double dist_to_center = cv::norm(contour_center - img_center);
        
        // 评分函数 - 优先考虑近似方形(aspect_ratio接近1)、面积适中且接近图像中心的轮廓
        double shape_score = (1.0 / (1.0 + std::abs(aspect_ratio - 1.5))) * 
                          (1.0 / (1.0 + std::abs(extent - 0.55))) * 
                          (1.0 / (1.0 + 0.01 * dist_to_center)) * 
                          std::min(area, 5000.0) / 5000.0;
        
        // 如果有上次有效位置，优先考虑接近的轮廓
        if (last_detection_valid && (ros::Time::now() - last_valid_time).toSec() < 2.0) {
            double dist_to_last = cv::norm(contour_center - last_valid_center);
            shape_score *= 1.0 + 1.0 / (1.0 + 0.01 * dist_to_last);
        }
        
        // 将得分显示在图像上
        cv::putText(debug_image, 
                  cv::format("S:%.2f", shape_score),
                  cv::Point(bbox.x, bbox.y - 5),
                  cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 0, 255), 1);
        
        if (shape_score > best_score) {
            best_score = shape_score;
            best_contour_idx = i;
        }
    }
    
    // 处理最佳轮廓
    if (best_contour_idx >= 0 && best_score > 0.1) {
        cv::Rect bbox = cv::boundingRect(all_contours[best_contour_idx]);
        cv::Point2f contour_center(bbox.x + bbox.width/2.0f, bbox.y + bbox.height/2.0f);
        
        // 绘制选中的轮廓
        cv::drawContours(debug_image, all_contours, best_contour_idx, cv::Scalar(0, 255, 255), 2);
        cv::rectangle(debug_image, bbox, cv::Scalar(255, 0, 255), 2);
        cv::circle(debug_image, contour_center, 5, cv::Scalar(255, 0, 255), -1);
        
        // 添加中心点对齐指示线
        cv::Point2f image_center(image.cols/2.0f, image.rows/2.0f);
        cv::line(debug_image, 
                cv::Point(image_center.x, 0), 
                cv::Point(image_center.x, image.rows), 
                cv::Scalar(255, 255, 0), 1);
        cv::line(debug_image, 
                cv::Point(0, image_center.y), 
                cv::Point(image.cols, image_center.y), 
                cv::Scalar(255, 255, 0), 1);
        
        // 显示偏移
        double dx = contour_center.x - image_center.x;
        double dy = contour_center.y - image_center.y;
        cv::putText(debug_image, 
                   cv::format("dx: %.1f, dy: %.1f", dx, dy),
                   cv::Point(20, 60),
                   cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 255), 2);
        
        // 状态文本
        cv::putText(debug_image, 
                   cv::format("APRON DETECTED (CONTOUR) %.2f", best_score),
                   cv::Point(20, 30),
                   cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 2);
        
        // 保存调试图像
        static int image_count = 0;
        if (image_count++ % 10 == 0) {
            cv::imwrite("/home/slz/image_debug_camera/apron_detection_contour_" + std::to_string(image_count/10) + ".jpg", debug_image);
        }
        
        // 显示调试图像
        cv::imshow("Apron Detection", debug_image);
        cv::waitKey(1);
        
        // 保存结果
        center = contour_center;
        last_valid_center = center;
        last_detection_valid = true;
        last_valid_time = ros::Time::now();
        
        return true;
    }
    
    // 如果短时间内丢失目标，使用上次位置
    if (last_detection_valid && (ros::Time::now() - last_valid_time).toSec() < 2.0) {
        center = last_valid_center;
        
        // 添加警告标识
        cv::putText(debug_image, "TRACKING LAST POSITION", cv::Point(20, 30),
                   cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 165, 255), 2);
        cv::circle(debug_image, center, 5, cv::Scalar(0, 165, 255), -1);
        
        // 保存并显示调试图像
        cv::imshow("Apron Detection", debug_image);
        cv::waitKey(1);
        
        ROS_WARN_THROTTLE(1.0, "Using last valid apron position (%.1f seconds old)", 
                         (ros::Time::now() - last_valid_time).toSec());
        return true;
    }
    
    // 添加错误提示
    cv::putText(debug_image, "NO APRON DETECTED", cv::Point(20, 30),
               cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
    cv::imshow("Apron Detection", debug_image);
    cv::waitKey(1);
    
    // 未检测到停机坪
    last_detection_valid = false;
    return false;
}
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "aruco_waypoint_controller");
    ros::NodeHandle nh;
    
    ArucoWaypointController controller(nh);
    controller.start();
    
    ros::spin();
    return 0;
}
