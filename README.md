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

![Peek 2024-07-03 18-43](https://github.com/sm3304love/nonlinear_mpc/assets/57741032/320cef34-66af-4084-9921-c38d1c497da9)


### TO DO
- [ ] Self collision
- [x] Obstacle avoidance

