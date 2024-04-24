#include <nonlinear_mpc/nonlinear_mpc.hpp>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(nonlinear_mpc::NonlinearMPC, nonlinear_mpc::NonlinearMPCInterface)

namespace nonlinear_mpc
{
NonlinearMPC::NonlinearMPC()
{
    Q_trans.diagonal() << 80, 80, 80;
    Q_ori.diagonal() << 30, 30, 30;
    R.diagonal() << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.8, 0.8, 0.8; // velocity weight?
    Qf_trans.diagonal() << 160, 160, 160;
    Qf_ori.diagonal() << 60, 60, 60;
    last_cmd.setZero();

    x_lb << -6.283185307179586, -6.283185307179586, -3.141592653589793, -6.283185307179586, -6.283185307179586,
        -6.283185307179586, -100, -100, -3.14159;

    x_Ub << 6.283185307179586, 6.283185307179586, 3.141592653589793, 6.283185307179586, 6.283185307179586,
        6.283185307179586, 100, 100, 3.14159;

    V_arm_lb << -2.0943951023931953, -2.0943951023931953, -2.6179938779914944, -3.6651914291880923, -3.6651914291880923,
        -3.6651914291880923;

    V_arm_Ub << 2.0943951023931953, 2.0943951023931953, 2.6179938779914944, 3.6651914291880923, 3.6651914291880923,
        3.6651914291880923;

    V_base_lb << -1, -1, -0.1;
    V_base_Ub << 1, 1, 0.1;

    urdf_filename = package_path + "/urdf/ur20.urdf";

    pinocchio::urdf::buildModel(urdf_filename, model);
    data = pinocchio::Data(model);

    pinocchio::urdf::buildGeom(model, urdf_filename, pinocchio::COLLISION, geomModel); // mobile manipulator꺼로?
    geomData = pinocchio::GeometryData(geomModel);

    obs = std::make_shared<hpp::fcl::CollisionObject>(std::make_shared<hpp::fcl::Sphere>(0.05));
    mobile_collision = std::make_shared<hpp::fcl::CollisionObject>(std::make_shared<hpp::fcl::Box>(1.0, 1.0, 0.5));

    frame_id = model.getFrameId("tool0");

    T_w_b = Eigen::Affine3d::Identity();
    T_b_arm = Eigen::Affine3d::Identity();
    T_b_arm.translation() << 0.0, 0.0, 0.5;
}

void NonlinearMPC::set_dynamics(const mpc::cvec<num_states> &x)
{

    A = Eigen::MatrixXd::Zero(num_states, num_states);

    B = Eigen::MatrixXd::Identity(num_states, num_inputs);

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
    x0 << 0.0, -1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

    set_dynamics(x0);

    set_obj();
    set_constraints();

    ROS_INFO("MPC initial linearization");

    mpc::NLParameters params;
    params.maximum_iteration = 100;
    params.relative_ftol = 1e-4;
    params.relative_xtol = 1e-6;
    params.hard_constraints = true;

    mpc_solver.setOptimizerParameters(params);

    ROS_INFO("MPC initialization completed");

    return true;
}

mpc::cvec<num_inputs> NonlinearMPC::computeCommand(mpc::cvec<num_states> x)
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
                                        const mpc::mat<pred_hor + 1, num_inputs> &u, const double &) {
        double cost = 0;
        for (int i = 0; i < pred_hor; i++)
        {
            Eigen::VectorXd q = x.row(i).head(dof);
            Eigen::VectorXd x_base = x.row(i).tail(3);

            T_w_b.linear() = Eigen::AngleAxisd(x_base(2), Eigen::Vector3d::UnitZ()).toRotationMatrix();
            T_w_b.translation() << x_base(0), x_base(1), 0.0;

            pinocchio::framesForwardKinematics(model, data, q);
            const pinocchio::SE3 &effector_tf = data.oMf[frame_id];
            Eigen::Affine3d T_w_ee;
            T_w_ee.matrix() = T_w_b.matrix() * T_b_arm.matrix() * effector_tf.toHomogeneousMatrix();

            Eigen::Quaterniond ee_ori(T_w_ee.rotation());
            Eigen::Vector3d ee_pos = T_w_ee.translation();

            Eigen::VectorXd pose_error = ee_pos - ee_pos_ref;
            Eigen::VectorXd ori_error = (ee_ori_ref * ee_ori.inverse()).vec();

            // Modified weight of input (velocity)
            Eigen::Affine3d T_w_arm;
            T_w_arm.matrix() = T_w_b.matrix() * T_b_arm.matrix();

            if ((T_w_arm.translation() - ee_pos_ref).norm() > 1.75)
            {
                R.diagonal() << 10, 10, 10, 10, 10, 10, 0.5, 0.5, 0.5;
            }
            else
            {
                R.diagonal() << 1, 1, 1, 1, 1, 1, 10, 10, 1;
            }

            double pose_cost = pose_error.transpose() * Q_trans * pose_error;
            double ori_cost = ori_error.transpose() * Q_ori * ori_error;
            double input_cost = u.row(i).dot(R * u.row(i).transpose());

            cost += (pose_cost + ori_cost + input_cost);
        }

        // // final terminal cost
        Eigen::VectorXd q_final = x.row(pred_hor).head(dof);
        Eigen::VectorXd x_base_final = x.row(pred_hor).tail(3);

        T_w_b.linear() = Eigen::AngleAxisd(x_base_final(2), Eigen::Vector3d::UnitZ()).toRotationMatrix();
        T_w_b.translation() << x_base_final(0), x_base_final(1), 0.0;

        pinocchio::framesForwardKinematics(model, data, q_final);
        const pinocchio::SE3 &effector_tf_final = data.oMf[frame_id];
        Eigen::Affine3d T_w_ee;
        T_w_ee.matrix() = T_w_b.matrix() * T_b_arm.matrix() * effector_tf_final.toHomogeneousMatrix();

        Eigen::Quaterniond ee_ori_final(T_w_ee.rotation());
        Eigen::Vector3d ee_pos_final = T_w_ee.translation();

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

        obs_transform.setTranslation(hpp::fcl::Vec3f(obs_pos(0), obs_pos(1), obs_pos(2)));
        obs_transform.setQuatRotation(hpp::fcl::Quaternion3f(obs_ori.w(), obs_ori.x(), obs_ori.y(), obs_ori.z()));
        obs->setTransform(obs_transform);

        for (int i = 0; i < pred_hor + 1; i++)
        {

            for (size_t j = 0; j < dof; j++)
            {
                in_con(index++) = u(i, j) - V_arm_Ub(j);  // u <= u_max
                in_con(index++) = -u(i, j) + V_arm_lb(j); // u_min <= u
            }
            for (size_t j = dof; j < num_inputs; j++)
            {
                in_con(index++) = u(i, j) - V_base_Ub(j - dof);  // u <= u_max
                in_con(index++) = -u(i, j) + V_base_lb(j - dof); // u_min <= u
            }
            for (int j = 0; j < num_states; j++) //
            {
                in_con(index++) = x(i, j) - x_Ub(j);  // q <= q_max
                in_con(index++) = -x(i, j) + x_lb(j); // q_min <= q
            }

            Eigen::VectorXd q = x.row(i).head(dof);
            Eigen::VectorXd x_base = x.row(i).tail(3);

            pinocchio::updateGeometryPlacements(model, data, geomModel, geomData, q);

            for (int j = 0; j < num_arm_links; j++)
            {
                const pinocchio::GeometryObject &go = geomModel.geometryObjects[j];
                hpp::fcl::Transform3f arm_transform;
                arm_transform.setTranslation(hpp::fcl::Vec3f(geomData.oMg[j].translation()));
                arm_transform.setQuatRotation(hpp::fcl::Quaternion3f(geomData.oMg[j].rotation()));

                hpp::fcl::Transform3f world_base_transform;
                world_base_transform.setTranslation(hpp::fcl::Vec3f(x_base[0], x_base[1], 0.5));
                world_base_transform.setQuatRotation(
                    hpp::fcl::Quaternion3f(Eigen::AngleAxisd(x_base[2], Eigen::Vector3d::UnitZ())));

                hpp::fcl::Transform3f world_transform = world_base_transform * arm_transform;

                auto go_collision_object = std::make_shared<hpp::fcl::CollisionObject>(go.geometry, world_transform);

                hpp::fcl::DistanceRequest arm_request;
                hpp::fcl::DistanceResult arm_result;
                hpp::fcl::distance(obs.get(), go_collision_object.get(), arm_request, arm_result);
                in_con(index++) = -arm_result.min_distance + 0.1;
            }

            mobile_collision->setTranslation(hpp::fcl::Vec3f(x_base[0], x_base[1], 0.25));
            hpp::fcl::Quaternion3f quat_mobile(Eigen::AngleAxisd(x_base[2], Eigen::Vector3d::UnitZ()));
            mobile_collision->setRotation(quat_mobile.toRotationMatrix());

            hpp::fcl::DistanceRequest mobile_request;
            hpp::fcl::DistanceResult mobile_result;

            if (obs_pos(2) > 0.5)
            {
                mobile_result.min_distance = 100;
            }
            else
            {
                auto obs_copy = obs;
                auto mobile_collision_copy = mobile_collision;
                obs_copy->setTranslation(hpp::fcl::Vec3f(obs->getTranslation()[0], obs->getTranslation()[1], 0));
                mobile_collision_copy->setTranslation(
                    hpp::fcl::Vec3f(mobile_collision->getTranslation()[0], mobile_collision->getTranslation()[1], 0));

                hpp::fcl::distance(obs_copy.get(), mobile_collision_copy.get(), mobile_request, mobile_result);
            }

            in_con(index++) = -mobile_result.min_distance + 0.3;

            for (size_t i = 1; i < geomModel.geometryObjects.size(); ++i)
            {
                for (size_t j = i + 1; j < geomModel.geometryObjects.size(); ++j)
                {
                    // Skip collision check between adjacent links
                    if (model.parents[i] == j || model.parents[j] == i)
                    {
                        continue;
                    }

                    const pinocchio::GeometryObject &go1 = geomModel.geometryObjects[i];
                    const pinocchio::GeometryObject &go2 = geomModel.geometryObjects[j];

                    hpp::fcl::Transform3f transform1;
                    transform1.setTranslation(hpp::fcl::Vec3f(geomData.oMg[i].translation()));
                    transform1.setQuatRotation(hpp::fcl::Quaternion3f(geomData.oMg[i].rotation()));

                    hpp::fcl::Transform3f transform2;
                    transform2.setTranslation(hpp::fcl::Vec3f(geomData.oMg[j].translation()));
                    transform2.setQuatRotation(hpp::fcl::Quaternion3f(geomData.oMg[j].rotation()));

                    std::shared_ptr<hpp::fcl::CollisionObject> obj1(
                        new hpp::fcl::CollisionObject(go1.geometry, transform1));
                    std::shared_ptr<hpp::fcl::CollisionObject> obj2(
                        new hpp::fcl::CollisionObject(go2.geometry, transform2));

                    hpp::fcl::DistanceRequest arm_self_collision_request;
                    hpp::fcl::DistanceResult arm_self_collision_result;
                    hpp::fcl::distance(obj1.get(), obj2.get(), arm_self_collision_request, arm_self_collision_result);

                    in_con(index++) = -arm_self_collision_result.min_distance + 0.0;
                }
            }

            // for (size_t i = 2; i < geomModel.geometryObjects.size(); ++i)
            // {

            //     const pinocchio::GeometryObject &go1 = geomModel.geometryObjects[i];
            //     hpp::fcl::Transform3f arm_transform;
            //     arm_transform.setTranslation(hpp::fcl::Vec3f(geomData.oMg[i].translation()));
            //     arm_transform.setQuatRotation(hpp::fcl::Quaternion3f(geomData.oMg[i].rotation()));

            //     hpp::fcl::Transform3f world_base_transform;
            //     world_base_transform.setTranslation(hpp::fcl::Vec3f(x_base[0], x_base[1], 0.5));
            //     world_base_transform.setQuatRotation(
            //         hpp::fcl::Quaternion3f(Eigen::AngleAxisd(x_base[2], Eigen::Vector3d::UnitZ())));

            //     hpp::fcl::Transform3f world_transform = world_base_transform * arm_transform;

            //     std::shared_ptr<hpp::fcl::CollisionObject> obj1(
            //         new hpp::fcl::CollisionObject(go1.geometry, world_transform));

            //     hpp::fcl::DistanceRequest arm_mobile_request;
            //     hpp::fcl::DistanceResult arm_mobile_result;

            //     hpp::fcl::distance(obj1.get(), mobile_collision.get(), arm_mobile_request, arm_mobile_result);

            //     in_con(index++) = -arm_mobile_result.min_distance + 0.1;
            //     std::cout << "distance between " << go1.name << " and mobile: " << arm_mobile_result.min_distance
            //               << std::endl;
            // }
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
