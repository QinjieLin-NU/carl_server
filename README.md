# rl-collision-avoidance

This is a Pytorch implementation of the paper [Towards Optimally Decentralized Multi-Robot Collision Avoidance via Deep Reinforcement Learning](https://arxiv.org/abs/1709.10082)

![](./doc/stage2.gif)  |  ![](./doc/circle_test.gif)
:-------------------------:|:-------------------------:

## Requirement

- python2.7
- [ROS Kinetic](http://wiki.ros.org/kinetic)
- [mpi4py](https://mpi4py.readthedocs.io/en/stable/)
- [Stage](http://rtv.github.io/Stage/)
- [PyTorch](http://pytorch.org/)


## How to train
Please use the `stage_ros-add_pose_and_crash` package instead of the default package provided by ROS.
```
mkdir -p catkin_ws/src
cp stage_ros-add_pose_and_crash catkin_ws/src
cd catkin_ws
catkin_make
source devel/setup.bash
```

To train Stage1, modify the hyper-parameters in `ppo_stage1.py` as you like, and running the following command:
```
rosrun stage_ros_add_pose_and_crash stageros -g worlds/stage1.world
mpiexec -np 24 python ppo_stage1.py
```
To train Stage2, modify the hyper-parameters in `ppo_stage2.py` as you like, and running the following command:
```
rosrun stage_ros_add_pose_and_crash stageros -g worlds/stage2.world
mpiexec -np 44 python ppo_stage2.py
```
## How to test

```
rosrun stage_ros_add_pose_and_crash stageros worlds/circle.world
mpiexec -np 50 python circle_test.py
```   

## installing problem  
export LD_LIBRARY_PATH=path to stage

## How to run multi-robot and multi-env training
### rl_ws  
```  
roscore -p 11312   
rosrun stage_ros_add_pose_and_crash stageros ../../../home/long_ws/rl-collision-avoidance/worlds/stage1.world   
roscore -p 11313   
rosrun stage_ros_add_pose_and_crash stageros ../../../home/long_ws/rl-clision-avoidance/worlds/stage_map2.world    
roscore -p 11314     
rosrun stage_ros_add_pose_and_crash stageros ../../../home/long_ws/rl-clision-avoidance/worlds/stage_map3.world     
```  
### carl_server   
```    
mpiexec --allow-run-as-root -np 32 python ppo_stage1.py      
```   
### run ad and ag  (chase scenario training ad, not training ag)
```     
roscore -p 11312    
rosrun stage_ros_add_pose_and_crash stageros ../../../home/long_ws/rl-clision-avoidance/worlds/stage_adMap1.world    
mpiexec --allow-run-as-root -np 4 python ppo_ad1.py   
python ppo_ag1.py  
```   
### run ad and ag  (chase scenario, training ad, training ag)  
```     
roscore -p 11312     
export ROS_MASTER_URI=http://localhost:11312    
rosrun stage_ros_add_pose_and_crash stageros ../../../home/long_ws/rl-clision-avoidance/worlds/stage_adMap2.world      
mpiexec --allow-run-as-root -np 4 python ppo_ad_chase.py   
mpiexec --allow-run-as-root -np 1 python ppo_ag_chase.py  
```    
### run ad and ag  (push scenario, training ad, training ag)
```    
roscore -p 11323   
export ROS_MASTER_URI=http://localhost:11323  
rosrun stage_ros_add_pose_and_crash stageros ../../../home/long_ws/rl-clision-avoidance/worlds/stage_adMap3.world   
mpiexec --allow-run-as-root -np 1 python ppo_ag_push.py 
mpiexec --allow-run-as-root -np 1 python ppo_ad_push.py   
```   
### run ad and ag  (training agent in three scenatio)
``` 
roscore -p 11323   
export ROS_MASTER_URI=http://localhost:11323    
rosrun stage_ros_add_pose_and_crash stageros ../../../home/long_ws/rl-clision-avoidance/worlds/stage_adMap2.world   
roscore -p 11324  
export ROS_MASTER_URI=http://localhost:11324    
rosrun stage_ros_add_pose_and_crash stageros ../../../home/long_ws/rl-clision-avoidance/worlds/stage_adMap3.world 
roscore -p 11325   
export ROS_MASTER_URI=http://localhost:11325    
rosrun stage_ros_add_pose_and_crash stageros ../../../home/long_ws/rl-clision-avoidance/worlds/stage_adMap4.world 
mpiexec --allow-run-as-root -np 3 python ppo_ag_general.py   
mpiexec --allow-run-as-root -np 4 python ppo_ad_chase.py   
mpiexec --allow-run-as-root -np 4 python ppo_ad_push.py   
mpiexec --allow-run-as-root -np 4 python ppo_ad_notPass.py    
```   
