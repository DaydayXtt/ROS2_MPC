#include "mpc.h"

namespace OsqpMPC
{
    MPC_Node::MPC_Node() : Node("mpc_node")
    {
        // 设置输出精度
        std::cout << std::fixed << std::setprecision(6);
        ns = this->get_namespace();
        std::string robot_name = ns.substr(1, ns.length() - 1);
        RCLCPP_INFO(this->get_logger(), "I am %s", robot_name.c_str());

        params_config_ = std::make_unique<ConfigReader>();
        // 生成期望轨迹
        params_config_->read_trajectory_config(); // 初始化参数
        desire_trajectory_ = generate_trajectory();
        // RCLCPP_INFO(this->get_logger(), "Size of desire_trajectory: %ld", desire_trajectory_.poses.size());
        desire_trajectory_puber_ = create_publisher<Path>("desire_trajectory", 10);
        desire_trajectory_current_puber_ = create_publisher<Path>("desire_trajectory_current", 10);
        // 创建定时器
        int publish_rate = params_config_->trajectory().publish_rate_;
        desire_trajectory_timer_ = create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate)),
            std::bind(&MPC_Node::desire_trajectory_callback, this));

        // odom
        flag_init_completed_ = false;
        odom_suber_ = this->create_subscription<Odometry>(
            "odom", 1,
            std::bind(&MPC_Node::odom_callback, this, _1));

        // MPC轨迹跟踪控制
        params_config_->read_mpc_config(); // 初始化参数
        N = params_config_->mpc().horizon_;
        RCLCPP_INFO(this->get_logger(), "MPC horizon: %d", N);
        dt_ = params_config_->mpc().dt_;
        ct_ = 0;

        gamma_ = params_config_->mpc().gamma_;

        motion_planning_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(dt_ * 1000.0)),
            std::bind(&MPC_Node::motion_planning_callback, this));
        accel_puber_ = this->create_publisher<Accel>("input_accel", 10);
        // 随机生成障碍物
        // Point min_b, max_b;
        // min_b.x = -10.0;
        // min_b.y = -10.0;
        // min_b.z = 0.0;
        // max_b.x = 10.0;
        // max_b.y = 10.0;
        // max_b.z = 4.0;
        // obstacles = generate_random_points(30, min_b, max_b);
        // fov_ = 3.0;

        // quadpose_puber = this->create_publisher<PoseStamped>("/quad_pose", 10);

        // obstacles_puber_ = this->create_publisher<PoseArray>("/obstacles", 10);
        // goal_pose_puber_ = this->create_publisher<PoseStamped>("/goal_pose", 10);

        // env_info_timer_ = this->create_wall_timer(
        //     std::chrono::milliseconds(10),
        //     std::bind(&MPC_Node::env_info_callback, this));

        // buffer_cell_puber_ = this->create_publisher<VoroCell>("/buffer_cell", 10);
    }

    // void MPC_Node::env_info_callback()
    // {
    //     // RCLCPP_INFO(this->get_logger(), "Updating env...");
    //     PoseArray obstacles_msg;
    //     obstacles_msg.header.frame_id = "map";
    //     obstacles_msg.header.stamp = this->now();
    //     for (const Point &p : obstacles)
    //     {
    //         PoseStamped ps;
    //         ps.header.frame_id = "map";
    //         ps.header.stamp = this->now();
    //         ps.pose.position = p;
    //         obstacles_msg.poses.push_back(ps.pose);
    //     }
    //     obstacles_puber_->publish(obstacles_msg);
    //     PoseStamped goal_msg;
    //     goal_msg.header.frame_id = "map";
    //     goal_msg.header.stamp = this->now();
    //     goal_msg.pose.position = goal_;
    //     goal_pose_puber_->publish(goal_msg);

    //     if (quad_pose_.header.frame_id == "map")
    //         quadpose_puber->publish(quad_pose_);
    // }

    void MPC_Node::motion_planning_callback()
    {
        // 防止未收到odom数据
        if (quad_odom_.header.frame_id.empty() || quad_odom_.child_frame_id.empty())
        {
            RCLCPP_WARN(this->get_logger(), "Odometry not received yet.");
            return;
        }
        mpc_controller_.init(N, quad_odom_, desire_trajectory_, dt_, flag_init_completed_);
        // RCLCPP_INFO(this->get_logger(), "HERE!");
        // 计时
        auto tic = std::chrono::high_resolution_clock::now();
        timestamp_ = this->now().nanoseconds();
        if (timestamp_last_ != 0.0)
            dt_ = (timestamp_ - timestamp_last_) / 1e9;
        timestamp_last_ = timestamp_;

        // 更新MPC的信息
        RCLCPP_INFO(this->get_logger(), "Current Pose: %f, %f",
                    mpc_controller_.get_x0()(0), mpc_controller_.get_x0()(1));
        mpc_controller_.update_info(quad_odom_, ct_);

        // ROS2话题消息更新
        desire_trajectory_current_.header.stamp = this->now();
        desire_trajectory_current_.header.frame_id = "map";
        desire_trajectory_current_.poses.clear();
        for (int i = 0; i < mpc_controller_.desire_traj_inter_.size(); i++)
        {
            PoseStamped pose;
            pose.header.stamp = this->now();
            pose.header.frame_id = "map";
            pose.pose = mpc_controller_.desire_traj_inter_[i].pose;
            desire_trajectory_current_.poses.push_back(pose);
        }

        // 求解控制量（ax、ay）
        mpc_controller_.solveQP(ct_);

        // 发布至odom
        Accel accel_msg;
        accel_msg.linear.x = mpc_controller_.ctr.x();
        accel_msg.linear.y = mpc_controller_.ctr.y();
        accel_msg.linear.z = 0.0;
        accel_puber_->publish(accel_msg);

        ct_++;

        auto toc = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(toc - tic).count();
        // std::cout << "Calculate time: " << duration << " microseconds" << std::endl;
        RCLCPP_INFO(this->get_logger(), "Calculate time: %ld microseconds", duration);
    }

    void MPC_Node::odom_callback(const Odometry::SharedPtr msg)
    {
        // RCLCPP_INFO(this->get_logger(), "Heard from odom.");
        quad_odom_ = *msg;
    }

    // std::vector<Point> generate_random_points(int num_points, Point min_b, Point max_b)
    // {
    //     std::vector<Point> points;
    //     for (int i = 0; i < num_points; i++)
    //     {
    //         Point p;
    //         p.x = min_b.x + (max_b.x - min_b.x) * rand() / RAND_MAX;
    //         p.y = min_b.y + (max_b.y - min_b.y) * rand() / RAND_MAX;
    //         p.z = min_b.z + (max_b.z - min_b.z) * rand() / RAND_MAX;
    //         points.push_back(p);
    //         while (p.z > 0.1)
    //         {
    //             p.z -= 0.5;
    //             points.push_back(p);
    //         }
    //     }
    //     return points;
    // }

    Path MPC_Node::generate_trajectory()
    {
        Path path;
        // int length = get_parameter("length").as_int();
        int length = params_config_->trajectory().length_;
        // std::string type = get_parameter("type").as_string();
        std::string type = params_config_->trajectory().type_;

        // RCLCPP_INFO(this->get_logger(), "namespace: %s", ns.c_str());
        if (ns == "/robot0")
        {
            type = "circle";
        }
        else if (ns == "/robot1")
        {
            type = "helix";
        }
        else if (ns == "/robot2")
        {
            type = "line";
        }

        for (int i = 0; i < length; ++i)
        {
            PoseStamped pose;
            pose.header.stamp = now();
            pose.header.frame_id = "map";

            if (type == "circle")
                generateCircleTrajectory(pose, i);
            else if (type == "helix")
                generateHelixTrajectory(pose, i);
            else if (type == "line")
                generateLineTrajectory(pose, i);
            else
                generateDefaultTrajectory(pose, i);

            path.poses.push_back(pose);
        }

        return path;
    }

    void MPC_Node::desire_trajectory_callback()
    {
        // desire_trajectory_ = generate_trajectory();
        desire_trajectory_.header.stamp = now();
        desire_trajectory_.header.frame_id = "map";
        desire_trajectory_puber_->publish(desire_trajectory_);
        if (desire_trajectory_current_.header.frame_id == "map")
            desire_trajectory_current_puber_->publish(desire_trajectory_current_);
    }

    void MPC_Node::generateCircleTrajectory(PoseStamped &pose, int index)
    {
        // double radius = get_parameter("circle_radius").as_double();
        double radius = params_config_->trajectory().circle_radius_;
        // RCLCPP_INFO(this->get_logger(), "radius: %f", radius);
        // double center_x = get_parameter("circle_center_x").as_double();
        double center_x = params_config_->trajectory().circle_center_x_;
        // double center_y = get_parameter("circle_center_y").as_double();
        double center_y = params_config_->trajectory().circle_center_y_;
        // double z = get_parameter("circle_z").as_double();
        double z = params_config_->trajectory().circle_z_;
        // double speed = get_parameter("circle_speed").as_double();
        double speed = params_config_->trajectory().circle_speed_;
        double theta = speed * index;

        pose.pose.position.x = radius * std::cos(theta) + center_x;
        pose.pose.position.y = radius * std::sin(theta) + center_y;
        pose.pose.position.z = z;

        // 简单的方向设置，面向运动方向
        double yaw = theta + M_PI_2;
        pose.pose.orientation.w = std::cos(yaw * 0.5);
        pose.pose.orientation.z = std::sin(yaw * 0.5);
    }

    void MPC_Node::generateHelixTrajectory(PoseStamped &pose, int index)
    {
        // double radius = get_parameter("helix_radius").as_double();
        double radius = params_config_->trajectory().helix_radius_;
        // double center_x = get_parameter("helix_center_x").as_double();
        double center_x = params_config_->trajectory().helix_center_x_;
        // double center_y = get_parameter("helix_center_y").as_double();
        double center_y = params_config_->trajectory().helix_center_y_;
        // double speed = get_parameter("helix_speed").as_double();
        double speed = params_config_->trajectory().helix_speed_;
        double theta = speed * index;

        pose.pose.position.x = radius * std::cos(theta) + center_x;
        pose.pose.position.y = radius * std::sin(theta) + center_y;
        pose.pose.position.z = theta / (2.0 * M_PI); // 每转一圈z增加1

        // 简单的方向设置
        double yaw = theta + M_PI_2;
        pose.pose.orientation.w = std::cos(yaw * 0.5);
        pose.pose.orientation.z = std::sin(yaw * 0.5);
    }

    void MPC_Node::generateLineTrajectory(PoseStamped &pose, int index)
    {
        // double start_x = get_parameter("line_start_x").as_double();
        double start_x = params_config_->trajectory().line_start_x_;
        // double start_y = get_parameter("line_start_y").as_double();
        double start_y = params_config_->trajectory().line_start_y_;
        // double start_z = get_parameter("line_start_z").as_double();
        double start_z = params_config_->trajectory().line_start_z_;
        // double speed = get_parameter("line_speed").as_double();
        double speed = params_config_->trajectory().line_speed_;
        double theta = speed * index;
        pose.pose.position.x = start_x + index * speed * 0.5;
        pose.pose.position.y = start_y + index * speed * 0.3;
        pose.pose.position.z = start_z + index * speed * 0.1;
        // 固定方向
        pose.pose.orientation.w = 1.0;
    }

    void MPC_Node::generateDefaultTrajectory(PoseStamped &pose, int index)
    {
        pose.pose.position.x = index * 0.1;
        pose.pose.position.y = 0.0;
        pose.pose.position.z = 0.0;
        pose.pose.orientation.w = 1.0;
    }

}