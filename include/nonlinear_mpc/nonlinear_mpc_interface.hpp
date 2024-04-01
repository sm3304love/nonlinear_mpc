#pragma once
#include <mpc/Types.hpp>
#include <nonlinear_mpc/def.hpp>
#include <ros/ros.h>

namespace nonlinear_mpc
{

class NonlinearMPCInterface
{
  public:
    virtual bool initialize(ros::NodeHandle &, double) = 0;
    virtual mpc::cvec<num_inputs> computeCommand(mpc::cvec<num_states> x) = 0;
    virtual void set_obj() = 0;
    virtual void set_constraints() = 0;

    mpc::cvec<num_inputs> last_cmd;
    bool isInContact = false;

    virtual void setEePosRef(const Eigen::VectorXd &pos) = 0;
    virtual Eigen::VectorXd getEePosRef() const = 0;
    virtual void setEeOriRef(const Eigen::Quaterniond &ori) = 0;
    virtual Eigen::Quaterniond getEeOriRef() const = 0;

  protected:
    NonlinearMPCInterface()
    {
    }
};
} // namespace nonlinear_mpc