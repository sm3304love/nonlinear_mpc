## Notice
Please refer to the ```franka``` branch for a version that adds obstacle avoidance.

## Nonlinear Model Predictive Control based Motion Planning 
This algorithm plans the motion of franka panda based on Nonlinear Model Predictive Control (NMPC).

## Dependancies
* [Franka panda gazebo simulation environment](https://github.com/sm3304love/franka_panda_description.git)
* ROS Noetic
* [libmpc](https://github.com/nicolapiccinelli/libmpc)
* Pinocchio
* HPP-FCL

## HOW TO USE
### Get simulation evironment
```
git clone https://github.com/sm3304love/franka_panda_description.git
```
### Launch simulation
```
roslaunch franka_panda_description velocity_sim.launch
```
### Run NMPC node
```
rosrun nonlinear_mpc main
```

### Visual


https://github.com/sm3304love/nonlinear_mpc/assets/57741032/24cc350f-eebf-4f5e-8f2b-de86288a64cb



### TO DO
- [ ] Self collision
- [x] Obstacle avoidance

