#include "mpc.h"

int main(int argc, char const *argv[])
{
    // rclcpp::init_options::InitOptions options;
    // options.use_intra_process_comms(false); // 禁用多线程通信
    // rclcpp::init(argc, argv, options);
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OsqpMPC::MPC_Node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
