# Nonlinear Model Predictive Control based Mobile Manipulator Motion Planning
This algorithm plans the motion of 9 DOF mobile manipulator(omni-directional mobile robot + ur20) based on Nonlinear Model Predictive Control (NMPC).

## Dependancies
* [Mobile manipulator gazebo simulation environment](https://github.com/sm3304love/mobile_ur20_description)
* ROS Noetic
* [libmpc](https://github.com/nicolapiccinelli/libmpc)
* Pinocchio
* HPP-FCL

## HOW TO USE
### Get simulation evironment
```
git clone https://github.com/sm3304love/mobile_ur20_description.git
```
### Launch simulation
```
roslaunch mobile_ur20_description mobile_ur20.launch
```
### Run NMPC node
```
rosrun nonlinear_mpc main
```

### Visual


https://github.com/sm3304love/nonlinear_mpc/assets/57741032/777be840-0623-4018-afed-14951cde0549



### TO DO
- [ ] Self collision
- [x] Obstacle avoidance

