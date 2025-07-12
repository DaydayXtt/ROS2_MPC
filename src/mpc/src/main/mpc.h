#ifndef MPC_H
#define MPC_H

#include <string>
#include <functional>
using std::placeholders::_1;

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
// #include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/accel.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
using geometry_msgs::msg::Point;
// using geometry_msgs::msg::PointStamped;
using geometry_msgs::msg::Accel;
using geometry_msgs::msg::PoseArray;
using geometry_msgs::msg::PoseStamped;
using nav_msgs::msg::Odometry;
using nav_msgs::msg::Path;

#include "mpc_controller.h"
#include "config_reader.h"

namespace OsqpMPC
{
    std::vector<Point> generate_random_points(int num_points, Point min_b, Point max_b);

    class MPC_Node : public rclcpp::Node
    {
    public:
        MPC_Node();
        std::string ns;
        RobotStruct robot_params;

    private:
        std::unique_ptr<ConfigReader> params_config_; // 规划总流程的一个配置

        // 二阶积分器odom
        Odometry quad_odom_;
        bool flag_init_completed_;
        rclcpp::Subscription<Odometry>::SharedPtr odom_suber_;
        void odom_callback(const Odometry::SharedPtr msg);

        // 期望轨迹
        Path desire_trajectory_;
        Path desire_trajectory_current_;
        Path generate_trajectory();
        void generateCircleTrajectory(PoseStamped &pose, int index);
        void generateHelixTrajectory(PoseStamped &pose, int index);
        void generateLineTrajectory(PoseStamped &pose, int index);
        void generateDefaultTrajectory(PoseStamped &pose, int index);
        rclcpp::Publisher<Path>::SharedPtr desire_trajectory_puber_;
        rclcpp::Publisher<Path>::SharedPtr desire_trajectory_current_puber_;
        rclcpp::TimerBase::SharedPtr desire_trajectory_timer_;
        void desire_trajectory_callback();

        // 控制器
        double timestamp_, timestamp_last_, dt_; // 时间戳
        int N, ct_;                              // 控制时间步数
        MPCController mpc_controller_;
        rclcpp::TimerBase::SharedPtr motion_planning_timer_;
        void motion_planning_callback();
        rclcpp::Publisher<Accel>::SharedPtr accel_puber_;

        // std::vector<Point> obstacles;
        // Point goal_;
        // double fov_;
        // rclcpp::Publisher<PoseStamped>::SharedPtr quadpose_puber;
        // rclcpp::Publisher<PoseStamped>::SharedPtr goal_pose_puber_;
        // rclcpp::Publisher<PoseArray>::SharedPtr obstacles_puber_;
        // rclcpp::TimerBase::SharedPtr env_info_timer_;
        // void env_info_callback();

        // VoroCell buffered_cell_;
        // rclcpp::Publisher<VoroCell>::SharedPtr buffer_cell_puber_; // 维诺划分发布器
    };

}
#endif // MPC_H