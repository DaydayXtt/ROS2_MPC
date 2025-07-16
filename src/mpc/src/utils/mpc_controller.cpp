/*
TODO:
1. 添加 CBF Affine 约束
    int cbfRowStart = 2*n*(N+1) + m*N;
    约束矩阵 A_c 再加 N 行 (x_1, x_2, ..., x_N)
    约束矩阵 A_c 的状态列 (n*i)：\nabla h(x_k)^\mathsf{T}(A−I)
    约束矩阵 A_c 的输入列 (n*(N+1) + m*i)：\nabla h(x_k)^\mathsf{T}B
    l：-\gamma * h(x_k)
    u：OsqpEigen::INFTY
2. 将矩阵构造部分使用 k 循环赋值的替换为eigen矩阵整体赋值
*/
#include "mpc_controller.h"
using namespace Eigen;

namespace OsqpMPC
{
    MPCController::MPCController()
    {
    }

    void MPCController::init(int horizon, const Odometry &odom, const Path &desire_traj, double dt_, bool &flag_init_completed)
    {
        if (flag_init_completed)
            return;
        update_x0(odom);             // 初始状态
        x_ref << 0.5, 5.0, 0.0, 0.0; // 参考状态
        desire_traj_ = desire_traj;
        n = 4;
        m = 2;
        N = horizon;

        dt = dt_;

        setupQP();
        flag_init_completed = true;
        std::cout << "MPCController init completed." << std::endl;
    }

    void MPCController::setupQP()
    {
        setDynamicsMatrices();
        // std::cout << "Dynamics Matrices: " << std::endl
        //           << A_system << std::endl
        //           << B_system << std::endl;
        setInequalityConstraints();

        setWeightMatrices();

        castMPCToQPHessian();

        castMPCToQPGradient(x_ref); // 假设x_ref为0
        castMPCToQPConstraintMatrix();
        castMPCToQPConstraintVectors(x0);

        // 初始化OSQP求解器
        solver.settings()->setVerbosity(false);
        solver.settings()->setWarmStart(true);
        // solver.settings()->setPolish(true);

        solver.data()->setNumberOfVariables(n * (N + 1) + m * N);
        solver.data()->setNumberOfConstraints(2 * n * (N + 1) + m * N);

        if (!solver.data()->setHessianMatrix(hessianMatrix))
            return;
        if (!solver.data()->setGradient(gradient))
            return;
        if (!solver.data()->setLinearConstraintsMatrix(constraintMatrix))
            return;
        if (!solver.data()->setBounds(lowerBound, upperBound))
            return;

        if (!solver.initSolver())
        {
            std::cerr << "OSQP solver init failed!" << std::endl;
            return;
        }
    }
    void MPCController::update_info(const Odometry &odom, int ct)
    {
        update_x0(odom);        // 初始状态
        update_desire_traj(ct); // 期望轨迹
    }

    void MPCController::update_x0(const Odometry &odom)
    {
        auto pos = odom.pose.pose.position;
        auto vel = odom.twist.twist.linear;
        x0 << pos.x, pos.y, vel.x, vel.y; // 初始状态
        updateConstraintVectors(x0);
    }

    void MPCController::update_desire_traj(int ct) // 更新期望轨迹(N个时刻)
    {
        // 构造当前时刻的期望轨迹区间（N个）
        desire_traj_inter_.clear();
        int start_index = std::min(ct, static_cast<int>(desire_traj_.poses.size()) - 1);
        int end_index = std::min(ct + N, static_cast<int>(desire_traj_.poses.size()));
        // std::cout << "期望轨迹区间: " << start_index << " - " << end_index << std::endl;
        desire_traj_inter_ =
            std::vector<PoseStamped>(desire_traj_.poses.begin() + start_index,
                                     desire_traj_.poses.begin() + end_index);
        // std::cout << "期望轨迹长度: " << desire_traj_inter_.size() << std::endl;
        desire_traj_inter_.resize(N, desire_traj_.poses.back());

        // N = desire_traj_inter_.size();
        // std::cout << "期望轨迹：" << std::endl;
        // for (auto pose : desire_traj_inter_)
        //     std::cout << "x: " << pose.pose.position.x << " y: " << pose.pose.position.y << std::endl;

        // 更新梯度向量
        update_gradient(); // 梯度向量
    }

    void MPCController::update_gradient()
    {
        // Vector4d
        // 先构造向量形式的xRef
        DiagonalMatrix<double, Dynamic> Q_expanded(n * (N + 1));
        // 将 Q 的对角元素沿对角线扩展 2 倍
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < N; ++j)
                Q_expanded.diagonal()[i + j * 4] = Q.diagonal()[i];
        for (int i = 0; i < 4; ++i)
            Q_expanded.diagonal()[i + N * 4] = Qf.diagonal()[i];

        VectorXd xRef_expanded = VectorXd::Zero(n * (N + 1));
        for (int i = 0; i < N; ++i)
        {
            auto pose = desire_traj_inter_[i];
            double px = pose.pose.position.x;
            double py = pose.pose.position.y;
            double vx = 0.0;
            double vy = 0.0;
            // 处理参考速度
            if (i == 0) // 初
            {
                vx = (desire_traj_inter_[1].pose.position.x - px) / dt;
                vy = (desire_traj_inter_[1].pose.position.y - py) / dt;
            }
            else if (i == N - 1) // 末
            {
                vx = (px - desire_traj_inter_[N - 2].pose.position.x) / dt;
                vy = (py - desire_traj_inter_[N - 2].pose.position.y) / dt;
            }
            else // 中间
            {
                vx = (desire_traj_inter_[i + 1].pose.position.x -
                      desire_traj_inter_[i - 1].pose.position.x) /
                     (2 * dt);
                vy = (desire_traj_inter_[i + 1].pose.position.y -
                      desire_traj_inter_[i - 1].pose.position.y) /
                     (2 * dt);
            }
            xRef_expanded.segment(i * n, n) << px, py, vx, vy;
        }
        // 终端
        auto pose_terminal = desire_traj_inter_[N - 1];
        double px_terminal = pose_terminal.pose.position.x;
        double py_terminal = pose_terminal.pose.position.y;
        xRef_expanded.segment(N * n, n) << px_terminal, py_terminal, 0.0, 0.0;

        // Vector4d Qx_ref = Q * (-x_ref);
        auto Qx_ref_expanded = Q_expanded * (-xRef_expanded);
        // std::cout << "Qx_ref_expanded: " << Qx_ref_expanded.size() << std::endl;
        gradient = VectorXd::Zero(n * (N + 1) + m * N);
        gradient.segment(0, n * (N + 1)) = Qx_ref_expanded;

        // 填充梯度向量 (仅状态部分)
        // for (int i = 0; i < n * (N + 1); i++)
        // {
        //     int posQ = i % n;
        //     gradient(i) = Qx_ref(posQ);
        // }
    }

    bool MPCController::solveQP(int ct)
    {
        // 更新
        if (!solver.updateLinearConstraintsMatrix(constraintMatrix))
        {
            std::cerr << "更新约束失败!" << std::endl;
            return false;
        }
        // 更新
        if (!solver.updateHessianMatrix(hessianMatrix))
        {
            std::cerr << "更新Hessian失败!" << std::endl;
            return false;
        }

        // 更新约束
        if (!solver.updateBounds(lowerBound, upperBound))
        {
            std::cerr << "更新约束失败!" << std::endl;
            return false;
        }
        // 更新梯度向量
        if (!solver.updateGradient(gradient))
        {
            std::cerr << "更新梯度失败!" << std::endl;
            return false;
        }
        if (solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError)
        {
            std::cerr << "OSQP求解失败!" << std::endl;
            return false;
        }
        auto status = solver.getStatus();
        // std::cerr << static_cast<int>(status) << std::endl;
        if (status != OsqpEigen::Status::Solved)
        {
            std::cerr << "求解状态异常: " << static_cast<int>(status) << std::endl;
            return false;
        }

        // 获取控制输入
        // std::cout << "第" << ct + 1 << "步控制" << std::endl;
        const Eigen::VectorXd &QPSolution = solver.getSolution();
        if (QPSolution.size() == 0 || !QPSolution.allFinite())
        {
            std::cerr << "OSQP 解非法!" << std::endl;
            return false;
        }
        // std::cerr << "HERE in [solveQP]." << std::endl;
        ctr = QPSolution.segment(n * (N + 1), m);
        // std::cout << "u: " << ctr(0) << " " << ctr(1) << std::endl;
        return true;
    }

    void MPCController::setDynamicsMatrices()   
    {
        // 状态矩阵
        A_system << 1.0, 0.0, dt, 0.0,
            0.0, 1.0, 0.0, dt,
            0.0, 0.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 1.0;

        // 控制矩阵 (加速度输入)
        B_system << 0.5 * dt * dt, 0.0,
            0.0, 0.5 * dt * dt,
            dt, 0.0,
            0.0, dt;
    }
    void MPCController::setInequalityConstraints()
    {
        // 状态约束 (位置和速度限制)
        x_min << -OsqpEigen::INFTY, // x下限
            -OsqpEigen::INFTY,      // y下限
            -2.0,                   // vx下限 (m/s)
            -2.0;                   // vy下限 (m/s)

        x_max << OsqpEigen::INFTY, // x上限
            OsqpEigen::INFTY,      // y上限
            2.0,                   // vx上限 (m/s)
            2.0;                   // vy上限 (m/s)

        // 控制输入约束 (加速度限制)
        u_min << -1.5, -1.5; // ax, ay下限 (m/s^2)
        u_max << 1.5, 1.5;   // ax, ay上限 (m/s^2)
    }
    void MPCController::setWeightMatrices()
    {
        // 状态权重 (位置误差权重更高)
        Q.diagonal() << 10.0, // x权重
            10.0,             // y权重
            1.0,              // vx权重
            1.0;              // vy权重

        // 控制输入权重
        R.diagonal() << 0.1, // ax权重
            0.1;             // ay权重

        // 终端权重 (这里和Q一样)
        Qf.diagonal() << 10.0, // x权重
            10.0,              // y权重
            1.0,               // vx权重
            1.0;               // vy权重
    }
    void MPCController::castMPCToQPHessian()
    {
        hessianMatrix.resize(n * (N + 1) + m * N,
                             n * (N + 1) + m * N);
        // 填充Hessian矩阵 (Q和R的对角元素)
        for (int i = 0; i < n * N; i++)
        {
            int posQ = i % n;
            float value = Q.diagonal()[posQ];
            if (value != 0)
                hessianMatrix.insert(i, i) = value;
        }
        for (int i = n * N; i < n * (N + 1); i++)
        { // 终端代价
            int posQf = i % n;
            float value = Qf.diagonal()[posQf];
            if (value != 0)
                hessianMatrix.insert(i, i) = value;
        }
        for (int i = 0; i < m * N; i++)
        {
            int posR = i % m;
            float value = R.diagonal()[posR];
            if (value != 0)
                hessianMatrix.insert(i + n * (N + 1), i + n * (N + 1)) = value;
        }
        hessianMatrix.makeCompressed();
    }
    void MPCController::castMPCToQPGradient(const Matrix<double, 4, 1> &xRef)
    {
        Matrix<double, 4, 1> Qx_ref = Q * (-xRef);
        gradient = VectorXd::Zero(n * (N + 1) + m * N);

        // 填充梯度向量 (仅状态部分)
        for (int i = 0; i < n * (N + 1); i++)
        {
            int posQ = i % n;
            gradient(i) = Qx_ref(posQ);
        }
    }
    void MPCController::castMPCToQPConstraintMatrix()
    {
        constraintMatrix.resize(n * (N + 1) + n * (N + 1) + m * N,
                                n * (N + 1) + m * N);

        // 初始状态约束
        for (int i = 0; i < n * (N + 1); i++)
            constraintMatrix.insert(i, i) = -1.0;

        // 动力学约束 x_{k+1} = A x_k + B u_k
        for (int i = 0; i < N; i++)
            for (int j = 0; j < n; j++)
                for (int k = 0; k < n; k++)
                {
                    double value = A_system(j, k);
                    if (value != 0)
                        constraintMatrix.insert(n * (i + 1) + j, n * i + k) = value;
                }
        for (int i = 0; i < N; i++)
            for (int j = 0; j < n; j++)
                for (int k = 0; k < m; k++)
                {
                    float value = B_system(j, k);
                    if (value != 0)
                        constraintMatrix.insert(n * (i + 1) + j, m * i + k + n * (N + 1)) = value;
                }
        
        // 状态和输入边界
        for (int i = 0; i < n * (N + 1) + m * N; i++)
            constraintMatrix.insert(i + (N + 1) * n, i) = 1;
        constraintMatrix.makeCompressed();
    }
    void MPCController::castMPCToQPConstraintVectors(const Matrix<double, 4, 1> &x0)
    {
        // 不等式约束向量
        VectorXd lowerInequality = VectorXd::Zero(n * (N + 1) + m * N);
        VectorXd upperInequality = VectorXd::Zero(n * (N + 1) + m * N);

        // 状态约束
        for (int i = 0; i < N + 1; i++)
        {
            lowerInequality.segment(i * n, n) = x_min;
            upperInequality.segment(i * n, n) = x_max;
        }

        // 控制输入约束
        for (int i = 0; i < N; i++)
        {
            lowerInequality.segment((N + 1) * n + i * m, m) = u_min;
            upperInequality.segment((N + 1) * n + i * m, m) = u_max;
        }

        // 等式约束 (初始状态)
        VectorXd lowerEquality = VectorXd::Zero(n * (N + 1));
        VectorXd upperEquality = VectorXd::Zero(n * (N + 1));
        lowerEquality.segment(0, n) = -x0;
        upperEquality.segment(0, n) = -x0;

        // 合并约束
        lowerBound.resize(2 * n * (N + 1) + m * N);
        upperBound.resize(2 * n * (N + 1) + m * N);
        lowerBound << lowerEquality, lowerInequality;
        upperBound << upperEquality, upperInequality;
    }
    void MPCController::updateConstraintVectors(const Matrix<double, 4, 1> &x0)
    {
        lowerBound.segment(0, n) = -x0;
        upperBound.segment(0, n) = -x0;
    }

    double MPCController::getErrorNorm(const Matrix<double, 4, 1> &x,
                                       const Matrix<double, 4, 1> &xRef)
    {
        return (x - xRef).norm();
    }
}