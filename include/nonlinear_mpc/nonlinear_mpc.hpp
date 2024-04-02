#pragma once

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/multibody/data.hpp>
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

    mpc::mat<num_states, num_states> A;
    mpc::mat<num_states, num_inputs> B;

    double ts;
    bool useHardConst = true;

    mpc::mat<3, 3> Q_trans;
    mpc::mat<3, 3> Q_ori;
    mpc::mat<3, 3> Qf_ori;
    mpc::mat<3, 3> Qf_trans;

    mpc::mat<num_inputs, num_inputs> R;
    mpc::mat<dof, dof> Q_vel;

    mpc::cvec<num_states> X_state;

    mpc::NLMPC<num_states, num_inputs, num_outputs, pred_hor, ctrl_hor, ineq_c, eq_c> mpc_solver;

    Eigen::VectorXd ee_pos_ref = Eigen::VectorXd::Zero(3);
    Eigen::Quaterniond ee_ori_ref = Eigen::Quaterniond::Identity();

    mpc::Result<num_inputs> r_before;
    // robotics
    pinocchio::Model model;
    pinocchio::Data data;
    std::string urdf_filename;
    int frame_id;
    std::string package_path = ros::package::getPath("ur20_description");

    Eigen::VectorXd x_lb = Eigen::VectorXd::Zero(12);
    Eigen::VectorXd x_Ub = Eigen::VectorXd::Zero(12);

  protected:
    void set_dynamics(const mpc::cvec<num_states> &);
};
} // namespace nonlinear_mpc