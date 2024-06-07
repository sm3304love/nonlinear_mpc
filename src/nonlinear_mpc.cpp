#include <nonlinear_mpc/nonlinear_mpc.hpp>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(nonlinear_mpc::NonlinearMPC, nonlinear_mpc::NonlinearMPCInterface)

namespace nonlinear_mpc
{
NonlinearMPC::NonlinearMPC()
{
    Q_trans.diagonal() << 800, 800, 800;
    Q_ori.diagonal() << 300, 300, 300;
    Q_vel.diagonal() << 10, 10, 10, 10, 10, 10;
    R.diagonal() << 0.01, 0.01, 0.01, 0.01, 0.01, 0.01;
    Qf_trans.diagonal() << 1600, 1600, 1600;
    Qf_ori.diagonal() << 600, 600, 600;
    last_cmd.setZero();

    urdf_filename = package_path + "/urdf/ur20.urdf";

    pinocchio::urdf::buildModel(urdf_filename, model);
    data = pinocchio::Data(model);

    frame_id = model.getFrameId("tool0");

    x_lb << -6.283185307179586, -6.283185307179586, -3.141592653589793, -6.283185307179586, -6.283185307179586,
        -6.283185307179586, -2.0943951023931953, -2.0943951023931953, -2.6179938779914944, -3.6651914291880923,
        -3.6651914291880923, -3.6651914291880923;

    x_Ub << 6.283185307179586, 6.283185307179586, 3.141592653589793, 6.283185307179586, 6.283185307179586,
        6.283185307179586, 2.0943951023931953, 2.0943951023931953, 2.6179938779914944, 3.6651914291880923,
        3.6651914291880923, 3.6651914291880923;
}

void NonlinearMPC::set_dynamics(const mpc::cvec<num_states> &x)
{

    A = Eigen::MatrixXd::Zero(num_states, num_states);
    A.block<6, 6>(0, 6) = Eigen::MatrixXd::Identity(6, 6);

    B = Eigen::MatrixXd::Zero(num_states, num_inputs);
    B.block<6, 6>(6, 0) = Eigen::MatrixXd::Identity(6, 6);

    auto stateEq = [&](mpc::cvec<num_states> &dx, const mpc::cvec<num_states> &x, const mpc::cvec<num_inputs> &u,
                       const unsigned int &) { dx = A * x + B * u; };

    mpc_solver.setStateSpaceFunction(stateEq);
}

bool NonlinearMPC::initialize(ros::NodeHandle &nh, double dt)
{
    ts = dt;
    mpc_solver.setDiscretizationSamplingTime(ts);
    mpc_solver.setLoggerLevel(mpc::Logger::log_level::NORMAL);
    mpc_solver.setLoggerPrefix("ROBOT");
    ROS_INFO("MPC initial setup");

    // init state
    mpc::cvec<num_states> x0;
    x0 << 0.0, -1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

    set_dynamics(x0);

    set_obj();
    set_constraints();

    ROS_INFO("MPC initial linearization");

    mpc::NLParameters params;
    params.maximum_iteration = 150;
    params.relative_ftol = 1e-4;
    params.relative_xtol = 1e-6;

    mpc_solver.setOptimizerParameters(params);

    ROS_INFO("MPC initialization completed");

    return true;
}

mpc::cvec<num_inputs> NonlinearMPC::computeCommand(mpc::cvec<num_states> x) // was problem
{
    mpc::cvec<num_states> x0;
    mpc::cvec<num_inputs> u0;

    x0 = x;
    u0 = last_cmd;

    mpc::Result<num_inputs> r;

    r = mpc_solver.optimize(x0, u0);

    last_cmd = r.cmd;

    return last_cmd;
}

void NonlinearMPC::set_obj()
{
    mpc_solver.setObjectiveFunction([&](const mpc::mat<pred_hor + 1, num_states> &x,
                                        const mpc::mat<pred_hor + 1, num_outputs> &,
                                        const mpc::mat<pred_hor + 1, num_inputs> &u, const double &) {
        double cost = 0;
        for (int i = 0; i < pred_hor; i++)
        {
            Eigen::VectorXd q = x.row(i).head(6);
            Eigen::VectorXd q_dot = x.row(i).tail(6);

            pinocchio::framesForwardKinematics(model, data, q);
            const pinocchio::SE3 &effector_pose = data.oMf[frame_id];
            Eigen::Quaterniond ee_ori(effector_pose.rotation());
            Eigen::Vector3d ee_pos = effector_pose.translation();

            Eigen::VectorXd pose_error = ee_pos - ee_pos_ref;
            Eigen::VectorXd ori_error = (ee_ori_ref * ee_ori.inverse()).vec();
            double pose_cost = pose_error.transpose() * Q_trans * pose_error;
            double ori_cost = ori_error.transpose() * Q_ori * ori_error;
            double vel_cost = q_dot.transpose() * Q_vel * q_dot;
            double input_cost = u.row(i).dot(R * u.row(i).transpose());

            cost += (pose_cost + ori_cost + vel_cost + input_cost);
        }

        // // final terminal cost
        Eigen::VectorXd q_final = x.row(pred_hor).head(6);

        pinocchio::framesForwardKinematics(model, data, q_final);
        const pinocchio::SE3 &effector_pose_final = data.oMf[frame_id];

        Eigen::Quaterniond ee_ori_final(effector_pose_final.rotation());
        Eigen::Vector3d ee_pos_final = effector_pose_final.translation();

        Eigen::VectorXd pose_error_final = ee_pos_final - ee_pos_ref;
        Eigen::VectorXd ori_error_final = (ee_ori_ref * ee_ori_final.inverse()).vec();

        cost += pose_error_final.dot(Qf_trans * pose_error_final) + ori_error_final.dot(Qf_ori * ori_error_final);
        return cost;
    });
}

void NonlinearMPC::set_constraints()
{
    mpc_solver.setIneqConFunction([&](mpc::cvec<ineq_c> &in_con, const mpc::mat<pred_hor + 1, num_states> &x,
                                      const mpc::mat<pred_hor + 1, num_outputs> &,
                                      const mpc::mat<pred_hor + 1, num_inputs> &u, const double &) {
        int index = 0;

        for (int i = 0; i < pred_hor + 1; i++)
        {
            for (size_t j = 0; j < num_inputs; j++)
            {
                in_con(index++) = u(i, j) - 1.0;  // u <= u_max
                in_con(index++) = -u(i, j) - 1.0; // u_min <= u
            }
            for (int j = 0; j < num_states; j++) //
            {
                in_con(index++) = x(i, j) - x_Ub(j);  // q <= q_max
                in_con(index++) = -x(i, j) + x_lb(j); // q_min <= q
            }
        }
    });
}

void NonlinearMPC::setEePosRef(const Eigen::VectorXd &pos)
{
    ee_pos_ref = pos;
}

Eigen::VectorXd NonlinearMPC::getEePosRef() const
{
    return ee_pos_ref;
}

void NonlinearMPC::setEeOriRef(const Eigen::Quaterniond &ori)
{
    ee_ori_ref = ori;
}

Eigen::Quaterniond NonlinearMPC::getEeOriRef() const
{
    return ee_ori_ref;
}

} // namespace nonlinear_mpc
