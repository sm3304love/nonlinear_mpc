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

#include <nonlinear_mpc/nonlinear_mpc_interface.hpp>
#include <pluginlib/class_loader.h>

#include <chrono>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>

class MotionPlanner
{
  public:
    MotionPlanner()
    {
        joint_vel_command_pub = nh_.advertise<std_msgs::Float64MultiArray>("/ur20/ur20_joint_controller/command", 10);
        mobile_command_pub = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
        joint_states_sub = nh_.subscribe("/ur20/joint_states", 10, &MotionPlanner::jointstatesCallback, this);
        target_state_sub = nh_.subscribe("/gazebo/model_states", 10, &MotionPlanner::target_states_callback, this);
        odom_sub = nh_.subscribe("/odom", 10, &MotionPlanner::odomCallback, this);
        current_ee_pose_pub = nh_.advertise<geometry_msgs::Pose>("/current_ee_pose", 10);
        states_pub = nh_.advertise<std_msgs::Float64MultiArray>("/arm_pose_sim", 10);
        odom_sim_pub = nh_.advertise<std_msgs::Float64MultiArray>("/odom_sim", 10);

        urdf_filename = package_path + "/urdf/ur20.urdf";

        pinocchio::urdf::buildModel(urdf_filename, model);
        data = pinocchio::Data(model);

        frame_id = model.getFrameId("tool0");

        T_w_b = Eigen::Affine3d::Identity();
        T_b_arm = Eigen::Affine3d::Identity();
        T_b_arm.translation() << 0.0, 0.0, 0.5;
        state_vec_sim << 0.0, -1.0, 1.0, -1.5708, -1.5708, 0.0, 0.0, 0.0, 0.0;
    }

    void jointstatesCallback(const sensor_msgs::JointState::ConstPtr &JointState)
    {

        state_vec[0] = JointState->position[2];
        state_vec[1] = JointState->position[1];
        state_vec[2] = JointState->position[0];
        state_vec[3] = JointState->position[3];
        state_vec[4] = JointState->position[4];
        state_vec[5] = JointState->position[5];
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
            if (ModelState->name[i] == "obstacle")
            {

                obstacle_pose.position.x = ModelState->pose[i].position.x;
                obstacle_pose.position.y = ModelState->pose[i].position.y;
                obstacle_pose.position.z = ModelState->pose[i].position.z;
                obstacle_pose.orientation = ModelState->pose[i].orientation;
            }
        }
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr &odom)
    {
        state_vec[6] = odom->pose.pose.position.x;
        state_vec[7] = odom->pose.pose.position.y;
        Eigen::Quaterniond q(odom->pose.pose.orientation.w, odom->pose.pose.orientation.x,
                             odom->pose.pose.orientation.y, odom->pose.pose.orientation.z);
        double yaw = atan2(2.0 * (q.w() * q.z() + q.x() * q.y()), 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));

        state_vec[8] = yaw;
    }

    Eigen::VectorXd dynamics_sim(const Eigen::VectorXd x, const Eigen::VectorXd u)
    {
        Eigen::VectorXd dx = A * x + B * u;
        return dx;
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

                Eigen::VectorXd pos_obs(3);
                pos_obs << obstacle_pose.position.x, obstacle_pose.position.y, obstacle_pose.position.z;
                Eigen::Quaterniond ori_obs(obstacle_pose.orientation.w, obstacle_pose.orientation.x,
                                           obstacle_pose.orientation.y, obstacle_pose.orientation.z);

                // pos_obs << -1.1951199769973755, 0.47176215052604675, 1.301445484161377;
                // ori_obs.w() = 1.0;
                // ori_obs.x() = 0.0;
                // ori_obs.y() = 0.0;
                // ori_obs.z() = 0.0;

                mpc->setEePosObs(pos_obs);
                mpc->setEeOriObs(ori_obs);

                // mpc::cvec<num_states> x = state_vec;
                mpc::cvec<num_states> x = state_vec_sim;

                u = mpc->computeCommand(x);

                // std::cout << "u: " << u.transpose() << std::endl;

                Eigen::VectorXd dx = dynamics_sim(state_vec_sim, u);

                state_vec_sim += dx * dt;

                // std::cout << "state : " << state_vec_sim.transpose() << std::endl;

                Eigen::VectorXd q = state_vec_sim.head(6);
                Eigen::VectorXd q_b = state_vec_sim.tail(3);

                T_w_b.linear() = Eigen::AngleAxisd(q_b(2), Eigen::Vector3d::UnitZ()).toRotationMatrix();
                T_w_b.translation() << q_b(0), q_b(1), 0.0;

                pinocchio::framesForwardKinematics(model, data, q);
                const pinocchio::SE3 &effector_tf = data.oMf[frame_id];
                Eigen::Affine3d T_w_ee;
                T_w_ee.matrix() = T_w_b.matrix() * T_b_arm.matrix() * effector_tf.toHomogeneousMatrix();

                Eigen::Quaterniond ee_ori(T_w_ee.rotation());
                Eigen::Vector3d ee_pos = T_w_ee.translation();

                geometry_msgs::Pose current_ee_pose;
                current_ee_pose.position.x = ee_pos(0);
                current_ee_pose.position.y = ee_pos(1);
                current_ee_pose.position.z = ee_pos(2);
                current_ee_pose.orientation.w = ee_ori.w();
                current_ee_pose.orientation.x = ee_ori.x();
                current_ee_pose.orientation.y = ee_ori.y();
                current_ee_pose.orientation.z = ee_ori.z();

                std_msgs::Float64MultiArray cmd_msg;
                geometry_msgs::Twist cmd_msg_mobile;
                std_msgs::Float64MultiArray state_msg;
                std_msgs::Float64MultiArray odom_sim;

                for (int i = 0; i < 6; i++)
                {
                    cmd_msg.data.push_back(u[i]);
                }

                for (int i = 0; i < dof; i++)
                {
                    state_msg.data.push_back(state_vec_sim[i]);
                }

                cmd_msg_mobile.linear.x = u[6];
                cmd_msg_mobile.linear.y = u[7];
                cmd_msg_mobile.angular.z = u[8];

                for (int i = dof; i < 9; i++)
                {
                    odom_sim.data.push_back(state_vec_sim[i]);
                }

                // joint_vel_command_pub.publish(cmd_msg);
                // mobile_command_pub.publish(cmd_msg_mobile);
                current_ee_pose_pub.publish(current_ee_pose);
                states_pub.publish(state_msg);
                odom_sim_pub.publish(odom_sim);

                // Calculate the position difference
                Eigen::Vector3d pos_diff = pos_ref - ee_pos;

                // Calculate the orientation difference
                Eigen::Quaterniond ori_diff = ori_ref * ee_ori.inverse();

                // // 위치 차이 출력
                // std::cout << "Position Difference:" << std::endl;
                // std::cout << "x: " << pos_diff(0) << std::endl;
                // std::cout << "y: " << pos_diff(1) << std::endl;
                // std::cout << "z: " << pos_diff(2) << std::endl;

                // // 쿼터니언 차이 출력
                // std::cout << "Orientation Difference:" << std::endl;
                // std::cout << "w: " << ori_diff.w() << std::endl;
                // std::cout << "x: " << ori_diff.x() << std::endl;
                // std::cout << "y: " << ori_diff.y() << std::endl;
                // std::cout << "z: " << ori_diff.z() << std::endl;
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
    ros::Subscriber odom_sub;
    ros::Publisher joint_vel_command_pub;
    ros::Publisher mobile_command_pub;
    ros::Publisher current_ee_pose_pub;
    ros::Publisher odom_sim_pub;
    ros::Publisher states_pub;

    mpc::cvec<num_states> state_vec;
    mpc::cvec<num_states> state_vec_sim;

    Eigen::VectorXd u = Eigen::VectorXd::Zero(num_inputs);
    Eigen::VectorXd cmd = Eigen::VectorXd::Zero(num_inputs);

    geometry_msgs::Pose target_pose;
    geometry_msgs::Pose obstacle_pose;
    float hz = 100;
    boost::shared_ptr<nonlinear_mpc::NonlinearMPCInterface> mpc;

    pinocchio::Model model;
    pinocchio::Data data;
    std::string package_path = ros::package::getPath("mobile_ur20_description");
    std::string urdf_filename;
    int frame_id;
    Eigen::Affine3d T_w_b;
    Eigen::Affine3d T_b_arm;

    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(num_states, num_states);

    Eigen::MatrixXd B = Eigen::MatrixXd::Identity(num_states, num_inputs);
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "MotionPlanner");
    ROS_INFO_STREAM("Initializing controller");

    MotionPlanner NMPCPlanner;

    ros::AsyncSpinner spinner(12);
    spinner.start();

    NMPCPlanner.run();

    spinner.stop();

    return 0;
}