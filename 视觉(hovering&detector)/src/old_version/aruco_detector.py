Aruco_detector.py
#!/usr/bin/env python
import rospy
import cv2
import cv2.aruco as aruco
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Pose, Twist
from std_msgs.msg import Empty


class ArUcoLandingController:
    def __init__(self):
        rospy.init_node("aruco_landing_controller", anonymous=True)
        self.bridge = CvBridge()

        # 核心参数
        self.target_id = 43                  # 目标标记ID
        self.platform_height = 0.5           # 降落平台高度（米）
        self.descent_speed_high = -0.01      # 高空下降速度（>1.5m）
        self.descent_speed_mid = -0.008       # 中空下降速度（1.5m~0.8m）
        self.descent_speed_low = -0.005       # 低空下降速度（<0.8m）
        self.kp_horizontal = 0.0010          # 水平位置增益
        self.kp_vertical = 0.0010            # 垂直位置增益

        # 状态与时间参数
        self.detected_time = None            # 首次检测到标记的时间戳
        self.wait_duration = 2.0             # 等待时间（秒）
        self.is_waiting = False              # 等待状态标志
        self.is_landing = False              # 降落进行中标志

        # 订阅与发布
        self.image_sub = rospy.Subscriber(
            "/hummingbird/vi_sensor/camera_depth/camera/image_raw",
            Image,
            self.image_callback,
            queue_size=10
        )
        self.trajectory_pub = rospy.Publisher(
            "/hummingbird/command/trajectory",
            MultiDOFJointTrajectory,
            queue_size=10
        )
        self.stop_waypoints_pub = rospy.Publisher(
            "stop_waypoints",
            Empty,
            queue_size=10
        )
        self.pose_sub = rospy.Subscriber(
            "/hummingbird/ground_truth/pose",
            Pose,
            self.pose_callback,
            queue_size=10
        )

        # ArUco参数
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
        self.detector = aruco.ArucoDetector(self.aruco_dict)
        self.marker_length = 0.1              # 标记边长（米）
        self.camera_matrix = np.array([       # 相机内参（根据实际配置）
            [600.0, 0.0, 320.0],
            [0.0, 600.0, 240.0],
            [0.0, 0.0, 1.0]
        ], dtype=np.float32)
        self.dist_coeffs = np.zeros((4, 1), dtype=np.float32)
        self.current_height = 0.0
        rospy.loginfo(f"【控制器】初始化完成，目标标记ID:{self.target_id}，平台高度:{self.platform_height}m...")

    def pose_callback(self, msg):
        """获取并更新当前高度"""
        self.current_height = msg.position.z
        rospy.logdebug(f"【高度】当前高度：{self.current_height:.3f} m")

    def image_callback(self, data):
        """图像回调：检测标记、等待逻辑、发送控制指令"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except Exception as e:
            rospy.logerr(f"【错误】图像转换失败：{e}")
            return

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)

        if ids is not None and self.target_id in ids:
            target_idx = np.where(ids == self.target_id)[0][0]
            target_corner = corners[target_idx]

            # 首次检测到标记，记录时间并进入等待
            if not self.is_waiting and not self.is_landing:
                self.detected_time = rospy.get_time()
                self.is_waiting = True
                rospy.loginfo(f"【检测】发现目标标记ID:{self.target_id}，等待 {self.wait_duration} 秒后开始降落")

            # 等待结束，执行降落
            if self.is_waiting and rospy.get_time() - self.detected_time >= self.wait_duration:
                self.is_waiting = False
                self.is_landing = True
                self.stop_waypoints_pub.publish(Empty())  # 停止多航点运动
                rospy.loginfo("【开始】等待时间结束，启动降落程序")

                # 发送零速度指令稳定无人机
                self.send_trajectory([Twist()])
                rospy.sleep(0.5)  # 短暂延迟

                self.process_target_marker(target_corner, cv_image)
                self.draw_debug_info(cv_image, target_corner)

        else:
            # 标记丢失时重置状态
            self.is_waiting = False
            self.is_landing = False

        cv2.imshow("Aruco Detection", cv_image)
        cv2.waitKey(1)

    def process_target_marker(self, target_corner, cv_image):
        """处理目标标记：计算并发送控制指令"""
        if not self.is_landing:
            return  # 未进入降落状态时跳过

        image_size = cv_image.shape[:2]
        marker_center = np.mean(target_corner[0], axis=0).astype(np.float32)
        image_center = np.array([image_size[0]/2, image_size[1]/2], dtype=np.float32)
        dx = marker_center[0] - image_center[0]  # 水平偏移（像素，右正左负）
        dy = marker_center[1] - image_center[1]  # 垂直偏移（像素，下正上负）

        # 分级下降速度
        if self.current_height > 1.5:
            descent_speed = self.descent_speed_high
        elif self.current_height > 0.8:
            descent_speed = self.descent_speed_mid
        else:
            descent_speed = self.descent_speed_low

        # 平台保护：距离平台0.05m时停止下降
        if self.current_height <= self.platform_height + 0.05:
            descent_speed = 0.0
            rospy.loginfo(f"【着陆】已到达平台高度（{self.current_height:.2f}m），停止下降")

        # 构造速度指令（无人机坐标系：X前Y右Z上）
        twist = Twist()
        twist.linear.x = self.kp_vertical * dy     # 前后移动（垂直偏移控制）
        twist.linear.y = -self.kp_horizontal * dx  # 左右移动（水平偏移控制）
        twist.linear.z = descent_speed             # 上下移动（下降速度）

        self.send_trajectory([twist])  # 发送轨迹指令

    def draw_debug_info(self, cv_image, target_corner):
        """绘制目标标记边框和ID（修正ids格式为numpy数组）"""
        # 构造符合OpenCV要求的ids格式（形状为(1,1)的numpy数组）
        target_ids = np.array([[self.target_id]], dtype=np.int32)  
        cv2.aruco.drawDetectedMarkers(cv_image, [target_corner], target_ids, (0, 255, 0))  # 绘制绿色边框

        # 添加ID文字标注
        cv2.putText(cv_image, f"Target ID:{self.target_id}",
                    (int(target_corner[0][0][0]), int(target_corner[0][0][1] - 10)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)  # 绿色文字

    def send_trajectory(self, twists):
        """发送轨迹消息（确保非空检查）"""
        if not twists:
            rospy.logwarn("【警告】Twist列表为空，跳过发布")
            return
        trajectory_msg = MultiDOFJointTrajectory()
        point = MultiDOFJointTrajectoryPoint()
        point.velocities = twists  # 填充速度指令
        trajectory_msg.header.stamp = rospy.Time.now()
        trajectory_msg.points.append(point)
        self.trajectory_pub.publish(trajectory_msg)
        rospy.loginfo(
            f"【指令】X={twists[0].linear.x:.4f}m/s, Y={twists[0].linear.y:.4f}m/s, Z={twists[0].linear.z:.4f}m/s"
        )

    def shutdown_hook(self):
        """节点关闭时停止运动"""
        self.send_trajectory([Twist()])  # 发送零速度指令
        cv2.destroyAllWindows()


if __name__ == "__main__":
    controller = ArUcoLandingController()
    rospy.on_shutdown(controller.shutdown_hook)
rospy.spin()





Hovering_example.cpp
/*
 * 多航点控制示例（仅初始化位置，检测到标记后由降落代码接管）
 */
#include <ros/ros.h>                // ROS 核心头文件
#include <std_msgs/Empty.h>         // 停止信号消息类型
#include <std_srvs/Empty.h>         // Gazebo 服务类型
#include <Eigen/Core>               // Eigen 坐标表示
#include <mav_msgs/conversions.h>   // MAVROS 轨迹转换
#include <mav_msgs/default_topics.h>// MAVROS 默认话题
#include <trajectory_msgs/MultiDOFJointTrajectory.h> // 轨迹消息类型
#include <thread>                   // 添加该头文件以使用 std::this_thread

// 全局变量：轨迹发布者和停止标志
ros::Publisher trajectory_pub;
bool stop_waypoint_publishing = false;

// 停止信号回调：检测到标记后停止
void stopCallback(const std_msgs::Empty::ConstPtr& msg) {
    stop_waypoint_publishing = true;
    ROS_INFO("[Waypoint Example] Received marker detection signal, stopping waypoint publishing.");
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "waypoint_example");
    ros::NodeHandle nh;
    
    // 初始化轨迹发布者：使用 MAVROS 的默认轨迹话题
    trajectory_pub = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
        mav_msgs::default_topics::COMMAND_TRAJECTORY, 10
    );
    
    // 订阅停止信号话题（与降落代码的发布话题一致）
    ros::Subscriber stop_sub = nh.subscribe("stop_waypoints", 10, stopCallback);

    ROS_INFO("[Waypoint Example] Starting, flying to initial position (waiting for marker detection)...");

    // 取消 Gazebo 物理模拟暂停
    std_srvs::Empty srv;
    bool unpaused = ros::service::call("/gazebo/unpause_physics", srv);
    for (unsigned int i = 0; i < 10 && !unpaused; ++i) {
        ROS_INFO("[Waypoint Example] Waiting for Gazebo to start... (Attempt %u)", i + 1);
        std::this_thread::sleep_for(std::chrono::seconds(1));
        unpaused = ros::service::call("/gazebo/unpause_physics", srv);
    }
    if (!unpaused) {
        ROS_FATAL("[Waypoint Example] Failed to start Gazebo, exiting.");
        return -1;
    }

    // 定义初始航点（双精度坐标，便于检测标记）
    std::vector<Eigen::Vector3d> waypoints = {
        {0.0, 0.0, 1.0},
        {0.0, 7.0, 1.1}  // 初始高度 1.3米（根据实际标记位置调整）
    };
    std::vector<double> yaws = {0.0, 0.0};          // 航向角（0°，无旋转）
    std::vector<double> durations = {3, 10};      // 悬停 5秒，等待标记检测

    // 发布初始航点
    for (size_t i = 0; i < waypoints.size() && !stop_waypoint_publishing; ++i) {
        trajectory_msgs::MultiDOFJointTrajectory msg;
        mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(
            waypoints[i], yaws[i], &msg
        );
        msg.header.stamp = ros::Time::now();
        trajectory_pub.publish(msg);
        ROS_INFO("[Waypoint Example] Flying to waypoint (x=%.1f, y=%.1f, z=%.1f)m", waypoints[i].x(), waypoints[i].y(), waypoints[i].z());
        ros::Duration(durations[i]).sleep();
        ros::spinOnce();  // 处理停止信号回调
    }

    ROS_INFO("[Waypoint Example] Reached initial position, waiting for landing code to take over.");
    ros::shutdown();
    return 0;
}   
