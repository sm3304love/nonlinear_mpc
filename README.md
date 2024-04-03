# Nonlinear Model Predictive Control based Motion Planning 
This algorithm plans the motion of UR20 based on Nonlinear Model Predictive Control (NMPC).

## Dependancies
* [UR20 gazebo simulation environment](https://github.com/sm3304love/ur20_description)
* ROS Noetic
* [libmpc](https://github.com/nicolapiccinelli/libmpc)

## HOW TO USE
### Get simulation evironment
```
git clone https://github.com/sm3304love/ur20_description.git
```
### Launch simulation
```
roslaunch ur20_description velocity_sim.launch
```
### Run NMPC node
```
rosrun nonlinear_mpc main
```

### Visual

https://github.com/sm3304love/nonlinear_mpc/assets/57741032/b84f1e86-e72a-4287-82df-489cb880a1b1

### TO DO
- [ ] Self collision
- [ ] Obstacle avoidance

