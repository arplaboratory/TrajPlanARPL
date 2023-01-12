
Perching with Quadratic Programming
==================================================================================

Overview
------------------------
Package that contains library for generating smooth trajectories fast with polynomial splines. ros_traj_gen_arpl_interface acts as a ROS interface. traj_gen- library files to generate trjaectories. Ros_traj_gen_utils - visualization library files along with listener and publisher libraries, includes the files for the QP_Trackers/Berstein Tracker which interface witih our control pipeline to plan trajectory.

For VISUAL PERCHING switch to fix/noetic_2204 branch. This is in the traj_gen_arpl_interface folder which has a more in depth visual perching inside although not as cleanly integrated with ARPL_quadrotor_control

**Developer: Jeffery Mao<br />
Affiliation: [NYU ARPL](https://wp.nyu.edu/arpl/)<br />
Maintainer: <br />
Jeffery Mao, jm7752@nyu.edu<br />
Guanrui Li, lguanrui@nyu.edu<br />**

## License
Please be aware that this code was originally implemented for research purposes and may be subject to changes and any fitness for a particular purpose is disclaimed. To inquire about commercial licenses, please contact Prof. Giuseppe Loianno (loiannog@nyu.edu).
```
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
    
```
## Citation
If you publish a paper with this work, please cite our paper: 
```
@article{mao_robust_perch,
  url = {https://arxiv.org/abs/2204.02458},
  Year = {2023},
  Booktitle = {IEEE Transactions on Robotics (T-RO)}
  title={Robust Active Visual Perching with Quadrotors on Inclined Surfaces},
  author={Mao, Jeffrey and Nogar, Stephen and Kroninger, Christopher and Loianno, Giuseppe}}
  
@inproceedings{mao2021aggressive,
  title={Aggressive visual perching with quadrotors on inclined surfaces},
  author={Mao, Jeffrey and Li, Guanrui and Nogar, Stephen and Kroninger, Christopher and Loianno, Giuseppe},
  booktitle={2021 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  pages={5242--5248},
  year={2021},
  organization={IEEE}
}
 
 ```
 
 
Installation Pre-requisites 
-------------------------
```
git clone https://github.com/arplaboratory/ARPL_Trajectory_Planning
cd ARPL_Trajectory_Planning
sh install_trajgen_depend.sh
```


Usage
------------------------

ON YOUR LAPTOP, clone and build the repository Waypoint Navigation Plug-in (usefull for GUI):

```
git clone https://github.com/arplaboratory/waypoint_navigation_plugin
```

Run Demo
------------------------
If you are from ARPL, please follow these steps on your computer:
```
roslaunch waypoint_navigation_plugin rviz.launch
```


Else if you are NOT from ARPL, do the following steps:
```
roslaunch ros_traj_gen_utils traj_plan.launch
```
Change Robot name to vehicle_name/waypoints <- standard trajectory 
Drag and drop waypoints (Next to publish Point top bar)
The plugin will publish a nav_msgs/path
Output visualized 3D path. 2d plots of time versions various. topic published on vehicle_name/position_cmd


Launch File
------------------------
The launch file includes the max/min velocity. Also for Bernstein trajectory only. We can declare a box that will limit the trajectory planning. 

Library Features
------------------------
Traj_gen includes
Folder include contains the .h files it is divided into 3 folders. 
  *  ooqp_interface - contains the .h files for interacting with OOQP - OoqpEigenInterface.hpp is the main one ot look at
  *  trajectory - contains the various trajectory formulations

Trajectories 
  *  QPpolyTraj.h -> Contains 2 solvers for a QP formulation of the function of a polynomial based trajectories
        *  Fast Solve -> Solves 2x ->10x faster than Normal Solve but creates a slightly less optimal path
        *  Solve -> Basic Solve that finds the optimal path that minimizes snap for a polynomial

ros_traj_gen_utils
Contains ROS interfaces.


COMMON INSTALLATION PORBLEMS
------------------------
99.99% of install problems especially if you get errors like involve gfortran directory being wrong
ma27d.f:(.text+0x59cc): undefined reference to `_gfortran_st_write'

Go to the /traj_gen/cmake/Findooqp.cmake file and edit line 105 down and set the GFORTRAN_DIR to the correct place you installed gfortran.so
You can also use dpkg -L gfortran command to find it


