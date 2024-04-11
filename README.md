# Nonlinear Model Predictive Control based Motion Planning 
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




### TO DO
- [ ] Self collision
- [x] Obstacle avoidance

