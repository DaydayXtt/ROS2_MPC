#include "config_reader.h"

namespace OsqpMPC
{
    ConfigReader::ConfigReader() // 配置文件读取器
    {
        // 获取 ${workspaceFolder}/install/planning/share/mpc/目录路径
        std::string planning_share_directory = ament_index_cpp::get_package_share_directory("mpc");
        // 获取配置文件
        // std::cout << "dir: " << planning_share_directory + "/config/params.yaml" << std::endl;
        params = YAML::LoadFile(planning_share_directory + "/config/params.yaml");
        // read_robots_config();
        // read_mpc_config();
    }

    /****************** robot ******************/
    void ConfigReader::read_robot_config(RobotStruct &robot, const std::string &name)
    {
        // 检查 YAML 结构和字段
        if (params.IsNull())
        {
            RCLCPP_ERROR(rclcpp::get_logger("read_robot_config"), "YAML params is empty!");
            return;
        }
        if (!params["robot"])
        {
            RCLCPP_ERROR(rclcpp::get_logger("read_robot_config"), "Missing 'robot' key in YAML");
            return;
        }
        if (!params["robot"][name])
        {
            RCLCPP_ERROR(rclcpp::get_logger("read_robot_config"), "Robot '%s' not found in YAML", name.c_str());
            return;
        }
        // std::cout << params << std::endl; // 打印整个 YAML 内容

        // auto robot_node = params["robot"][name];
        robot.id_ = params["robot"][name]["id"].as<int>();
        robot.frame_ = params["robot"][name]["frame"].as<std::string>();
        robot.pose_x_ = params["robot"][name]["pose_x"].as<double>();
        robot.pose_y_ = params["robot"][name]["pose_y"].as<double>();
        robot.radius_ = params["robot"][name]["radius"].as<double>();
        // robots_list_.emplace(robot.id_, robot);
    }

    void ConfigReader::read_robots_config()
    {
        try // 捕获异常
        {
            for (auto it = robots_list_.begin(); it != robots_list_.end(); ++it)
            {
                std::string name = "robot" + std::to_string(it->first);
                read_robot_config(it->second, name);
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(rclcpp::get_logger("config"), "Failed to load robots config: %s", e.what());
            // std::cerr << e.what() << '\n';
        }
    }

    /****************** MPC控制器 ******************/
    void ConfigReader::read_mpc_config()
    {
        try // 捕获异常
        {
            mpc_.horizon_ = params["MPC"]["horizon"].as<int>();
            mpc_.dt_ = params["MPC"]["dt"].as<double>();
        }
        catch (const YAML::Exception &e)
        {
            RCLCPP_ERROR(rclcpp::get_logger("config"), "Failed to load MPC config: %s", e.what());
            // std::cerr << e.what() << '\n';
        }
    }

    void ConfigReader::read_trajectory_config()
    {
        try // 捕获异常
        {
            trajectory_.type_ = params["Trajectory"]["type"].as<std::string>();
            trajectory_.publish_rate_ = params["Trajectory"]["publish_rate"].as<int>();
            trajectory_.length_ = params["Trajectory"]["length"].as<int>();

            trajectory_.circle_radius_ = params["Trajectory"]["circle_radius"].as<double>();
            trajectory_.circle_center_x_ = params["Trajectory"]["circle_center_x"].as<double>();   // 圆形轨迹圆心x坐标
            trajectory_.circle_center_y_ = params["Trajectory"]["circle_center_y"].as<double>();   // 圆形轨迹圆心y坐标
            trajectory_.circle_z_ = params["Trajectory"]["circle_z"].as<double>();                 // 圆形轨迹高度
            trajectory_.circle_speed_ = params["Trajectory"]["circle_speed"].as<double>(); // 圆形轨迹频率
            
            trajectory_.helix_radius_ = params["Trajectory"]["helix_radius"].as<double>();         // 螺旋线轨迹半径
            trajectory_.helix_center_x_ = params["Trajectory"]["helix_center_x"].as<double>();     // 螺旋线轨迹圆心x坐标
            trajectory_.helix_center_y_ = params["Trajectory"]["helix_center_y"].as<double>();     // 螺旋线轨迹圆心y坐标
            trajectory_.helix_speed_ = params["Trajectory"]["helix_speed"].as<double>();           // 螺旋线轨迹速度

            trajectory_.line_start_x_ = params["Trajectory"]["line_start_x"].as<double>();         // 直线起点x坐标
            trajectory_.line_start_y_ = params["Trajectory"]["line_start_y"].as<double>();         // 直线起点y坐标
            trajectory_.line_start_z_ = params["Trajectory"]["line_start_z"].as<double>();         // 直线起点y坐标
            trajectory_.line_speed_ = params["Trajectory"]["line_speed"].as<double>();
        }
        catch (const YAML::Exception &e)
        {
            RCLCPP_ERROR(rclcpp::get_logger("config"), "Failed to load Trajectory config: %s", e.what());
            // std::cerr << e.what() << '\n';
        }
    }

}