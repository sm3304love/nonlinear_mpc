#pragma once

#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/geometry.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/geometry.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <Eigen/Geometry>
#include <chrono>
#include <functional>
#include <mpc/NLMPC.hpp>
#include <nonlinear_mpc/def.hpp>
#include <nonlinear_mpc/nonlinear_mpc_interface.hpp>
#include <ros/package.h>
#include <std_msgs/Float64MultiArray.h>

namespace nonlinear_mpc
{

class NonlinearMPC : public NonlinearMPCInterface
{
  public:
    NonlinearMPC();
    ~NonlinearMPC();

    /* inherit from interface*/
    bool initialize(ros::NodeHandle &, double);
    mpc::cvec<num_inputs> computeCommand(mpc::cvec<num_states> x);
    void set_obj();
    void set_constraints();

    void setEePosRef(const Eigen::VectorXd &pos) override;
    Eigen::VectorXd getEePosRef() const override;
    void setEeOriRef(const Eigen::Quaterniond &ori) override;
    Eigen::Quaterniond getEeOriRef() const override;

    void setEePosObs(const Eigen::VectorXd &pos) override;
    Eigen::VectorXd getEePosObs() const override;
    void setEeOriObs(const Eigen::Quaterniond &ori) override;
    Eigen::Quaterniond getEeOriObs() const override;

    mpc::mat<num_states, num_states> A;
    mpc::mat<num_states, num_inputs> B;

    double ts;
    bool useHardConst = true;

    mpc::mat<3, 3> Q_trans;
    mpc::mat<3, 3> Q_ori;
    mpc::mat<3, 3> Qf_ori;
    mpc::mat<3, 3> Qf_trans;

    mpc::mat<num_inputs, num_inputs> R;
    mpc::mat<9, 9> Qf_vel;

    mpc::cvec<num_states> X_state;

    mpc::NLMPC<num_states, num_inputs, num_outputs, pred_hor, ctrl_hor, ineq_c, eq_c> mpc_solver;

    Eigen::VectorXd ee_pos_ref = Eigen::VectorXd::Zero(3);
    Eigen::Quaterniond ee_ori_ref = Eigen::Quaterniond::Identity();

    Eigen::VectorXd obs_pos = Eigen::VectorXd::Zero(3);
    Eigen::Quaterniond obs_ori = Eigen::Quaterniond::Identity();

    mpc::Result<num_inputs> r_before;
    // robotics
    pinocchio::Model model;
    pinocchio::Data data;
    pinocchio::GeometryModel geomModel;
    pinocchio::GeometryData geomData;
    std::string urdf_filename;
    int frame_id;

    std::string package_path = ros::package::getPath("ur20_description");

    Eigen::VectorXd x_lb = Eigen::VectorXd::Zero(num_states);
    Eigen::VectorXd x_Ub = Eigen::VectorXd::Zero(num_states);

    Eigen::VectorXd V_arm_lb = Eigen::VectorXd::Zero(dof);
    Eigen::VectorXd V_arm_Ub = Eigen::VectorXd::Zero(dof);

    Eigen::VectorXd V_base_lb = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd V_base_Ub = Eigen::VectorXd::Zero(3);

    std::shared_ptr<hpp::fcl::CollisionObject> obs;
    hpp::fcl::Transform3f obs_transform;

    std::shared_ptr<hpp::fcl::CollisionObject> mobile_collision;

    Eigen::Affine3d T_w_b;
    Eigen::Affine3d T_b_arm;

  protected:
    void set_dynamics(const mpc::cvec<num_states> &);
};
} // namespace nonlinear_mpc