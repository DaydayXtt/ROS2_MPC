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
        void init(int horizon, const Odometry &odom, const Path &desire_traj, double dt, bool &flag_init_completed);
        Vector2d ctr;
        void setupQP();
        bool solveQP_demo();
        bool solveQP(int ct);
        void update_info(const Odometry &odom, int ct); // 更新初始状态
        void update_x0(const Odometry &odom);           // 更新初始状态
        void update_desire_traj(int ct);                // 更新期望轨迹(N个时刻)
        std::vector<PoseStamped> desire_traj_inter_;    // 每个回合的期望轨迹区间
        void update_gradient();
        void update_CBF_constraints();
        
        inline Vector4d get_x0() { return x0; }
    private:
        int n; // 状态维度
        int m; // 控制维度
        int N; // 预测时域

        double gamma; // CBF约束系数
        VectorXd x_obs; // 障碍物的位置（单个）
        double r_obs;   // 障碍物的半径（单个）

        Vector4d x_ref, x0;

        Path desire_traj_; // 总期望轨迹

        // 系统矩阵
        Matrix<double, 4, 4> A_system;
        Matrix<double, 4, 2> B_system;
        double dt; // 时间步长
        void setDynamicsMatrices();

        // 约束
        Matrix<double, 4, 1> x_min, x_max;
        Matrix<double, 2, 1> u_min, u_max;
        void setInequalityConstraints();

        // 权重矩阵
        DiagonalMatrix<double, 4> Q;  // 状态权重
        DiagonalMatrix<double, 2> R;  // 控制权重
        DiagonalMatrix<double, 4> Qf; // 终端权重
        void setWeightMatrices();

        // hessian矩阵 —— min 1/2 * x' * H * x + f' * x 的 H
        SparseMatrix<double> hessianMatrix;
        void castMPCToQPHessian();

        // 梯度向量 —— min 1/2 * x' * H * x + f' * x 的 f
        VectorXd gradient;
        void castMPCToQPGradient(const Matrix<double, 4, 1> &xRef);

        // 约束矩阵
        SparseMatrix<double> constraintMatrix;
        void castMPCToQPConstraintMatrix();
        VectorXd lowerBound;
        VectorXd upperBound;
        void castMPCToQPConstraintVectors(const Matrix<double, 4, 1> &x0);
        void updateConstraintVectors(const Matrix<double, 4, 1> &x0);

        double getErrorNorm(const Matrix<double, 4, 1> &x,
                            const Matrix<double, 4, 1> &xRef);

        // OSQP求解器
        OsqpEigen::Solver solver;
    };
}
#endif // MPC_CONTROLLER_H