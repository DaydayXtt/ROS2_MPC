#ifndef MPC_CONTROLLER_H
#define MPC_CONTROLLER_H
#include <OsqpEigen/OsqpEigen.h>
#include <Eigen/Dense>
#include <iostream>

#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
using nav_msgs::msg::Odometry;
using nav_msgs::msg::Path;

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::PoseStamped;

using namespace Eigen;

namespace OsqpMPC
{
    class MPCController
    {
    public:
        MPCController();
        void init(int state_dim, int input_dim, int horizon,
                  const Odometry &odom, const Path &desire_traj, double dt,
                  double cbf_gamma, double slack_rho, bool &flag_init_completed);
        VectorXd ctr;
        void setupQP();
        bool solveQP_demo();
        bool solveQP(int ct);
        void update_info(const Odometry &odom, int ct); // 更新初始状态
        void update_x0(const Odometry &odom);           // 更新初始状态
        void update_desire_traj(int ct);                // 更新期望轨迹(N个时刻)
        std::vector<PoseStamped> desire_traj_inter_;    // 每个回合的期望轨迹区间
        void update_gradient();

        inline VectorXd get_x0() { return x0; }
        inline VectorXd get_state_prev() { return state_prev; }

    private:
        int n; // 状态维度
        int m; // 控制维度
        int N; // 预测时域

        double gamma;   // CBF约束系数
        double rho_;    // slack惩罚系数

        VectorXd x_obs; // 障碍物的位置（单个）
        double r_obs;   // 障碍物的半径（单个）

        VectorXd x_ref, x0;
        VectorXd state_prev, input_prev;

        Path desire_traj_; // 总期望轨迹

        // 系统矩阵
        MatrixXd A_system;
        MatrixXd B_system;
        double dt; // 时间步长
        void setDynamicsMatrices();

        // 约束
        MatrixXd x_min, x_max;
        MatrixXd u_min, u_max;
        void setInequalityConstraints();

        // 权重矩阵
        MatrixXd Q;  // 状态权重
        MatrixXd R;  // 控制权重
        MatrixXd Qf; // 终端权重
        void setWeightMatrices();

        // hessian矩阵 —— min 1/2 * x' * H * x + f' * x 的 H
        SparseMatrix<double> hessianMatrix;
        void castMPCToQPHessian();

        // 梯度向量 —— min 1/2 * x' * H * x + f' * x 的 f
        VectorXd gradient;
        void castMPCToQPGradient(const MatrixXd &xRef);

        // 约束矩阵
        SparseMatrix<double> constraintMatrix;
        void castMPCToQPConstraintMatrix();
        VectorXd lowerBound;
        VectorXd upperBound;
        void castMPCToQPConstraintVectors(const MatrixXd &x0);
        void updateConstraintVectors(const MatrixXd &x0);

        double getErrorNorm(const MatrixXd &x,
                            const MatrixXd &xRef);

        // OSQP求解器
        OsqpEigen::Solver solver;
    };
}
#endif // MPC_CONTROLLER_H