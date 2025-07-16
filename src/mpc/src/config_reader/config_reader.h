#ifndef CONFIG_READER_H_
#define CONFIG_READER_H_

#include "rclcpp/rclcpp.hpp"
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <unordered_map>

// 设置namespace的好处：防止和别的模块中的函数、变量命名冲突
namespace OsqpMPC
{
    struct RobotStruct // Robot
    {
        int id_ = 0; // ID
        std::string frame_ = "";
        double pose_x_ = 0.0;
        double pose_y_ = 0.0;
        double radius_ = 0.0;
    };

    struct MPCStruct // MPC
    {
        int horizon_ = 0; // 滑动窗口长度
        double dt_ = 0.0; // 时间步长
        double gamma_ = 0.0; // CBF约束系数
    };

    struct TrajectoryStruct // 期望轨迹
    {
        std::string type_ = "circle"; // circle、helix、line
        int publish_rate_ = 0;        // 发布频率
        int length_ = 1001;

        double circle_radius_ = 3.0;      // 圆形轨迹半径
        double circle_center_x_ = -0.2;   // 圆形轨迹圆心x坐标
        double circle_center_y_ = 0.5;    // 圆形轨迹圆心y坐标
        double circle_z_ = 0.2;           // 圆形轨迹高度
        double circle_speed_ = 0.005; // 圆形轨迹频率

        double helix_radius_ = 3.0;   // 螺旋线轨迹半径
        double helix_center_x_ = 0.0; // 螺旋线轨迹圆心x坐标
        double helix_center_y_ = 0.0; // 螺旋线轨迹圆心y坐标
        double helix_speed_ = 0.3;    // 螺旋线轨迹速度

        double line_start_x_ = 1.0;  // 直线起点x坐标
        double line_start_y_ = -0.5; // 直线起点y坐标
        double line_start_z_ = 0.1;  // 直线起点y坐标
        double line_speed_ = 0.1;
    };

    class ConfigReader // 配置文件读取器
    {
    public:
        ConfigReader();

        // 读取车辆的配置参数
        void read_robot_config(RobotStruct &robot, const std::string &name);
        void read_robots_config();
        inline RobotStruct robot() const { return robot_; }   // const作用：不允许修改{}内的变量
        inline RobotStruct robot0() const { return robot0_; } // const作用：不允许修改{}内的变量
        inline RobotStruct robot1() const { return robot1_; }
        inline RobotStruct robot2() const { return robot2_; }
        inline RobotStruct robot3() const { return robot3_; }

        // 控制器
        void read_mpc_config();
        inline MPCStruct mpc() const { return mpc_; }

        // 期望轨迹
        void read_trajectory_config();
        inline TrajectoryStruct trajectory() const { return trajectory_; }

    private:
        YAML::Node params; // 配置文件数据，写成员函数的实现时用到

        // Agent
        RobotStruct robot_;
        RobotStruct robot0_;
        RobotStruct robot1_;
        RobotStruct robot2_;
        RobotStruct robot3_;
        std::unordered_map<int, RobotStruct> robots_list_;

        MPCStruct mpc_; // 控制器

        TrajectoryStruct trajectory_; // 期望轨迹
    };
} // namespace Planning
#endif // CONFIG_READER_H_
