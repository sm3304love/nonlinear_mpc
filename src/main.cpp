
#include <nonlinear_mpc/nonlinear_mpc_interface.hpp>
#include <pluginlib/class_loader.h>

#include <chrono>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Pose.h>
#include <iostream>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>

class MotionPlanner
{
  public:
    MotionPlanner()
    {
        joint_vel_command_pub = nh_.advertise<std_msgs::Float64MultiArray>("/ur20/ur20_joint_controller/command", 10);
        joint_states_sub = nh_.subscribe("/ur20/joint_states", 10, &MotionPlanner::jointstatesCallback, this);
        target_state_sub = nh_.subscribe("/gazebo/model_states", 10, &MotionPlanner::target_states_callback, this);
    }

    void jointstatesCallback(const sensor_msgs::JointState::ConstPtr &JointState)
    {

        joint_state_vec[0] = JointState->position[2];
        joint_state_vec[1] = JointState->position[1];
        joint_state_vec[2] = JointState->position[0];
        joint_state_vec[3] = JointState->position[3];
        joint_state_vec[4] = JointState->position[4];
        joint_state_vec[5] = JointState->position[5];
        joint_state_vec[6] = JointState->velocity[2];
        joint_state_vec[7] = JointState->velocity[1];
        joint_state_vec[8] = JointState->velocity[0];
        joint_state_vec[9] = JointState->velocity[3];
        joint_state_vec[10] = JointState->velocity[4];
        joint_state_vec[11] = JointState->velocity[5];
    }

    void target_states_callback(const gazebo_msgs::ModelStates::ConstPtr &ModelState)
    {
        for (int i = 0; i < ModelState->name.size(); i++)
        {

            if (ModelState->name[i] == "target")
            {

                target_pose.position.x = ModelState->pose[i].position.x;
                target_pose.position.y = ModelState->pose[i].position.y;
                target_pose.position.z = ModelState->pose[i].position.z;
                target_pose.orientation = ModelState->pose[i].orientation;
            }
        }
    }

    void run()
    {
        pluginlib::ClassLoader<nonlinear_mpc::NonlinearMPCInterface> loader("nonlinear_mpc",
                                                                            "nonlinear_mpc::NonlinearMPCInterface");
        boost::shared_ptr<nonlinear_mpc::NonlinearMPCInterface> mpc =
            loader.createInstance("nonlinear_mpc::NonlinearMPC");

        float dt = 1 / hz;
        mpc->initialize(nh_, dt);

        int count = 0;

        ros::Rate loop_rate(hz);

        while (ros::ok())
        {
            if (count >= 10)
            {

                Eigen::VectorXd pos_ref(3);
                pos_ref << target_pose.position.x, target_pose.position.y, target_pose.position.z;
                Eigen::Quaterniond ori_ref(target_pose.orientation.w, target_pose.orientation.x,
                                           target_pose.orientation.y, target_pose.orientation.z);

                mpc->setEePosRef(pos_ref);
                mpc->setEeOriRef(ori_ref);
                mpc::cvec<num_states> x = joint_state_vec;

                u = mpc->computeCommand(x);
                cmd = u * dt;

                std_msgs::Float64MultiArray cmd_msg;

                for (int i = 0; i < 6; i++)
                {
                    cmd_msg.data.push_back(cmd[i]);
                }

                joint_vel_command_pub.publish(cmd_msg);

                std::cout << "u: " << u.transpose() << std::endl;
            }
            count++;
            ros::spinOnce();
            loop_rate.sleep();
        }
    }

  private:
    ros::NodeHandle nh_;
    ros::Subscriber joint_states_sub;
    ros::Subscriber target_state_sub;
    ros::Publisher joint_vel_command_pub;
    mpc::cvec<num_states> joint_state_vec;

    Eigen::VectorXd u = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd cmd = Eigen::VectorXd::Zero(6);

    geometry_msgs::Pose target_pose;
    float hz = 100;
    boost::shared_ptr<nonlinear_mpc::NonlinearMPCInterface> mpc;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "MotionPlanner");
    ROS_INFO_STREAM("Initializing controller");

    MotionPlanner NMPCPlanner;

    ros::AsyncSpinner spinner(8);
    spinner.start();

    NMPCPlanner.run();

    spinner.stop();

    return 0;
}
