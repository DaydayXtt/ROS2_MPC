#include "mpc.h"

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OsqpMPC::MPC_Node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
