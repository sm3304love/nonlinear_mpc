#include <nonlinear_mpc/nonlinear_mpc.hpp>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(nonlinear_mpc::NonlinearMPC, nonlinear_mpc::NonlinearMPCInterface)

namespace nonlinear_mpc
{
NonlinearMPC::NonlinearMPC()
{
    Q_trans.diagonal() << 160, 160, 160;
    Q_ori.diagonal() << 60, 60, 60;
    Q_vel.diagonal() << 2.5, 2.5, 2.5, 2.5, 2.5, 2.5, 2.5;
    R.diagonal() << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1;
    Qf_trans.diagonal() << 320, 320, 320;
    Qf_ori.diagonal() << 120, 120, 120;

    last_cmd.setZero();

    // auto? yaml?
    x_lb << -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973, -2.175, -2.175, -2.175, -2.175, -2.61, -2.61,
        -2.61;

    x_Ub << 2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973, 2.175, 2.175, 2.175, 2.175, 2.61, 2.61, 2.61;

    urdf_filename = package_path + "/urdf/velocity_panda_arm_hand.urdf";

    pinocchio::urdf::buildModel(urdf_filename, model);
    data = pinocchio::Data(model);
    pinocchio::urdf::buildGeom(model, urdf_filename, pinocchio::COLLISION, geomModel);
    geomData = pinocchio::GeometryData(geomModel);

    obs = std::make_shared<hpp::fcl::CollisionObject>(std::make_shared<hpp::fcl::Sphere>(0.05));

    frame_id = model.getFrameId("panda_link8"); // end effector frame
}

void NonlinearMPC::set_dynamics(const mpc::cvec<num_states> &x)
{

    A = Eigen::MatrixXd::Zero(num_states, num_states);
    A.block<dof, dof>(0, dof) = Eigen::MatrixXd::Identity(dof, dof);

    B = Eigen::MatrixXd::Zero(num_states, num_inputs);
    B.block<dof, dof>(dof, 0) = Eigen::MatrixXd::Identity(dof, dof);

    auto stateEq = [&](mpc::cvec<num_states> &dx, const mpc::cvec<num_states> &x, const mpc::cvec<num_inputs> &u,
                       const unsigned int &) { dx = A * x + B * u; };

    mpc_solver.setStateSpaceFunction(stateEq);
}

bool NonlinearMPC::initialize(ros::NodeHandle &nh, double dt)
{
    ts = dt;
    mpc_solver.setContinuosTimeModel(ts);
    mpc_solver.setLoggerLevel(mpc::Logger::log_level::NORMAL);
    mpc_solver.setLoggerPrefix("ROBOT");
    ROS_INFO("MPC initial setup");

    // init state
    mpc::cvec<num_states> x0;
    x0 << 0.0, -0.7854, 0.0, -2.1, 0.0, 1.5708, 0.7854, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

    set_dynamics(x0);
    set_obj();
    set_constraints();

    ROS_INFO("MPC initial linearization");

    mpc::NLParameters params;
    params.maximum_iteration = 150;
    params.relative_ftol = 1e-5;
    params.relative_xtol = 1e-6;
    params.hard_constraints = false;

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

    r = mpc_solver.step(x0, u0);

    last_cmd = r.cmd;

    return last_cmd;
}

void NonlinearMPC::set_obj()
{
    mpc_solver.setObjectiveFunction([&](const mpc::mat<pred_hor + 1, num_states> &x,
                                        const mpc::mat<pred_hor + 1, num_outputs> &,
                                        const mpc::mat<pred_hor + 1, num_inputs> &u, const double &slack) {
        double cost = 0;

        for (int i = 0; i < pred_hor; i++) // inf?
        {
            Eigen::VectorXd q = x.row(i).head(dof);
            Eigen::VectorXd q_dot = x.row(i).tail(dof);

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
            // std::cout << "slack : " << slack << std::endl;
        }

        // // final terminal cost
        Eigen::VectorXd q_final = x.row(pred_hor).head(dof);

        pinocchio::framesForwardKinematics(model, data, q_final);
        const pinocchio::SE3 &effector_pose_final = data.oMf[frame_id];

        Eigen::Quaterniond ee_ori_final(effector_pose_final.rotation());
        Eigen::Vector3d ee_pos_final = effector_pose_final.translation();

        Eigen::VectorXd pose_error_final = ee_pos_final - ee_pos_ref;
        Eigen::VectorXd ori_error_final = (ee_ori_ref * ee_ori_final.inverse()).vec();

        cost += pose_error_final.dot(Qf_trans * pose_error_final) + ori_error_final.dot(Qf_ori * ori_error_final) +
                100 * slack + 0.5 * slack * slack;
        return cost;
    });
}

void NonlinearMPC::set_constraints()
{
    mpc_solver.setIneqConFunction([&](mpc::cvec<ineq_c> &in_con, const mpc::mat<pred_hor + 1, num_states> &x,
                                      const mpc::mat<pred_hor + 1, num_outputs> &,
                                      const mpc::mat<pred_hor + 1, num_inputs> &u, const double &slack) {
        int index = 0;

        obs_transform.setTranslation(hpp::fcl::Vec3f(obs_pos(0), obs_pos(1), obs_pos(2)));
        obs_transform.setQuatRotation(hpp::fcl::Quaternion3f(obs_ori.w(), obs_ori.x(), obs_ori.y(), obs_ori.z()));
        obs->setTransform(obs_transform);

        for (int i = 0; i < pred_hor + 1; i++)
        {
            in_con(index++) = 0 - slack; // x = 0
            for (size_t j = 0; j < num_inputs; j++)
            {
                in_con(index++) = u(i, j) - 5.0;  // u <= u_max
                in_con(index++) = -u(i, j) - 5.0; // u_min <= u
            }
            for (int j = 0; j < num_states; j++) //
            {
                in_con(index++) = x(i, j) - x_Ub(j);  // q <= q_max
                in_con(index++) = -x(i, j) + x_lb(j); // q_min <= q
            }

            Eigen::VectorXd q = x.row(i).head(dof);
            pinocchio::updateGeometryPlacements(model, data, geomModel, geomData, q);

            for (int j = 0; j < collision_link; j++) // can't find solution
            {
                const pinocchio::GeometryObject &go = geomModel.geometryObjects[j];
                hpp::fcl::Transform3f go_transform;
                go_transform.setTranslation(hpp::fcl::Vec3f(geomData.oMg[j].translation()));
                go_transform.setQuatRotation(hpp::fcl::Quaternion3f(geomData.oMg[j].rotation()));

                auto go_collision_object = std::make_shared<hpp::fcl::CollisionObject>(go.geometry, go_transform);

                hpp::fcl::DistanceRequest request;
                hpp::fcl::DistanceResult result;
                hpp::fcl::distance(obs.get(), go_collision_object.get(), request, result);
                in_con(index++) = -result.min_distance + 0.05 - slack;
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

void NonlinearMPC::setEePosObs(const Eigen::VectorXd &pos)
{
    obs_pos = pos;
}

Eigen::VectorXd NonlinearMPC::getEePosObs() const
{
    return obs_pos;
}

void NonlinearMPC::setEeOriObs(const Eigen::Quaterniond &ori)
{
    obs_ori = ori;
}

Eigen::Quaterniond NonlinearMPC::getEeOriObs() const
{
    return obs_ori;
}

} // namespace nonlinear_mpc

// Eigen::VectorXd q = x0.head(dof);
// pinocchio::updateGeometryPlacements(model, data, geomModel, geomData, q);

// for (int j = 0; j < collision_link; j++)
// {
//     const pinocchio::GeometryObject &go = geomModel.geometryObjects[j];
//     hpp::fcl::Transform3f go_transform;
//     go_transform.setTranslation(hpp::fcl::Vec3f(geomData.oMg[j].translation()));
//     go_transform.setQuatRotation(hpp::fcl::Quaternion3f(geomData.oMg[j].rotation()));

//     auto go_collision_object = std::make_shared<hpp::fcl::CollisionObject>(go.geometry, go_transform);

//     hpp::fcl::DistanceRequest request;
//     hpp::fcl::DistanceResult result;
//     hpp::fcl::distance(obs.get(), go_collision_object.get(), request, result);
//     std::cout << "Distance" << result.min_distance << std::endl;
// }