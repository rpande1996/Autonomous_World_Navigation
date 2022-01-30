## Autonomous_World_Navigation
[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](https://opensource.org/licenses/MIT)
---

## Contributors

1) [Rajan Pande](https://github.com/rpande1996)
Graduate Student of M.Eng Robotics at University of Maryland. 
2) [Douglas Summerlin](https://github.com/dsumm1001)
Graduate Student of M.Eng Robotics at University of Maryland.

## Overview

The project is inspired by the the challenge of autonomous robotics for
Urban Search and Rescue (US&R). In US&R after a disaster occurs, a robot is
used to explore an unknown environment, such as a partially collapsed building,
and locates trapped or unconscious human victims of the disaster. The robot builds
a map of the collapsed building as it explores, and places markers in the map of where 
the victims are located. This map is then given to trained First Responders who use it
to go into the building and rescue the victims.

## Softwares

* Recommended IDE: Visual Studio Code 1.63.2
* Required OS: Linux Ubuntu 18.06

## Libraries

* ROS Melodic

## Programming Languages

* C++ 17

## License 

```
MIT License

Copyright (c) 2021 Rajan Pande, Douglas Summerlin

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE 
SOFTWARE.
```

## Demo

World 1:

- ![RViz: ](https://youtu.be/0AvuQx_wBzE)

![ezgif com-gif-maker](https://github.com/rpande1996/Video_Correction/blob/master/media/gif/world1_rviz.gif)

- [Gazebo: ](https://youtu.be/Br2HN8fG9wk)

![ezgif com-gif-maker](https://github.com/rpande1996/Video_Correction/blob/master/media/gif/world1_gazebo.gif)

World 2:

- ![RViz: ](https://youtu.be/S6kaEBs8Dn4)

![ezgif com-gif-maker](https://github.com/rpande1996/Video_Correction/blob/master/media/gif/world2_rviz.gif)

- [Gazebo: ](https://youtu.be/zu4gU5dqbfQ)

![ezgif com-gif-maker](https://github.com/rpande1996/Video_Correction/blob/master/media/gif/world2_gazebo.gif)

## Build

```
cd catkin_ws/src
git clone https://github.com/rpande1996/Autonomous_World_Navigation
mv Autnomous_World_Navigation final_project
cd final_project/script
chmod +x install.bash
./install.bash
cd ~
catkin build
source ~/catkin_ws/devel/setup.bash
roslaunch final_project multiple_robots.launch
```
Open new terminal
```
roslaunch final_project final_project.launch
```