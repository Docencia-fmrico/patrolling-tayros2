# Patrolling Behavior

<div align="center">
<img width=500px src="https://github.com/Docencia-fmrico/patrolling-tayros2/blob/Readme/resources/figures/map_points_no_charge.png" alt="explode"></a>
</div>

<h3 align="center"> Patrolling </h3>

<div align="center">
<img width=100px src="https://img.shields.io/badge/status-finished-brightgreen" alt="explode"></a>
<img width=100px src="https://img.shields.io/badge/license-Apache-orange" alt="explode"></a>
<img width=90px src="https://img.shields.io/badge/team-TayRos2-yellow" alt="explode"></a>
</div>

[![main](https://github.com/Docencia-fmrico/patrolling-tayros2/actions/workflows/main.yml/badge.svg)](https://github.com/Docencia-fmrico/patrolling-tayros2/actions/workflows/main.yml)



## Table of Contents
- [Table of Contents](#table-of-contents)
- [Project Goal](#project-goal)
- [Continous Integration](#continous-integration)
- [Logic and functionality](#logic-and-functionality)
- [Behavior Tree diagram](#behavior-tree-diagram)
- [Launcher](#launcher)
- [Team](#team)
- [Licencia](#licencia)

## Project Goal

The aim of this project is to create a ROS2 application in order to make able a robot to patrol around predefined waypoints.

- The robot must use Behavior Trees y Nav2.
- It is necessary to map the environment we will use.
- The robot must perform some action when it reaches a waypoint (display message, turn on itself,...).

This behaviour must work in simulator using the Tiago robot. In addition, the repository must contain a package with all the nodes, following the recommended indications and organization of repositories.


## Continous Integration

-----------------------------------------------------------------------
Snippet:
``` yaml

```
-----------------------------------------------------------------------

## Mapping
One of the principal steps in order to this project was to make the mapping. We decided to choose the house map due to performance reasons (we can see different rooms on this map where we can set the waypoints) and map size (house map is neither too small nor too big, making a fluid and expressive behaviour in the robot).

In the following video we can see the complete process of the house mapping: [Alternative Link (Youtube)](https://youtu.be/q4s_rV_GiNg)

https://user-images.githubusercontent.com/72991245/220436491-d7893a00-80ec-4b44-bfcc-f3c645751efb.mp4


Once we had the house mapped, we use the rviz2 and the Publish Point tool to set the position of the waypoints we want the robot to follow. This clicked points in the rviz are published in the /clicked_point topic:

<div align="center">
<img width=500px src="https://github.com/Docencia-fmrico/patrolling-tayros2/blob/Readme/resources/figures/rviz_visual_map.png" alt="explode"></a>
</div>

### Navigation 2 params:

Once we have configured the waypoints coordinates in the code, it is important to take in consider some parameters of the br2_navigation package. In our case, we have to slightly increase the values of the xy_goal_tolerance of the general_goal_checker parameters in tiago_nav_params.yaml. 

The reason is the following: when Tiago is too close of reaching a waypoint (e.g ~0.5 error from (x,y) waypoint coordinates, and the tolerance value is < 0.5), his velocity will be too small. That means that it will take some minutes in order to reduce the error to the waypoint coordinates. That is why we need to take a properly value for that params:

-----------------------------------------------------------------------
tiago_nav_params.yaml:
``` yaml
    general_goal_checker:
      stateful: True
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: 0.75
```
-----------------------------------------------------------------------

Notice that some of the other parameters in the yaml file are also important. Take in consider if modifying this project repository.

## Logic and functionality
The logic of this program is based on the one we saw as an example in class, which can be found in the [following repository](https://github.com/fmrico/book_ros2/tree/main/br2_bt_patrolling).


## Behavior Tree Diagram 

You can see the Behaviour Tree diagram made in **Groot**:

<div align="center">
<img width=500px src="/resources/figures/beh-tree.png" alt="explode"></a>
</div>

## Launcher

-----------------------------------------------------------------------
patrolling.launch.py:
``` python
    def generate_launch_description():

    patrolling_cmd = Node(
        package='tyros2_patrolling',
        executable='patrolling_main',
        parameters=[{
          'use_sim_time': True
        }],
        remappings=[
          ('input_scan', '/scan_raw'),
          ('output_vel', '/nav_vel')
        ],
        output='screen'
    )

    ld = LaunchDescription()

    ld.add_action(patrolling_cmd)

    return ld
```
-----------------------------------------------------------------------

## Rviz
As we know that it is very tedious to configure the rviz parameters in order to recive the maps and costmap
correcty, we provide you a correct configuration for it.

## Tests
### Patrol test
We attempted to create a test that would check the velocities of the robot. Unfortunately, we encountered an issue with a CV dependency that we were unable to resolve. As a result, we had to comment out the test.

### GetWay Point
We verify the correctness of each obtained waypoint by comparing them one by one.

### Move
The robot goes to a arbitrary point in a fake server, it should reach the point and return success.


## Team

<div align="center">
<img width=200px src="https://github.com/Docencia-fmrico/bump-and-stop-tayros2/blob/readme/resources/figures/logo.png" alt="explode"></a>
</div>

- [Adrian Cobo](https://github.com/AdrianCobo)
- [Adrian Madinabeitia](https://github.com/madport)
- [Ivan Porras](https://github.com/porrasp8)
- [Saul Navajas](https://github.com/SaulN99)

## Licencia 
<a rel="license" href="https://www.apache.org/licenses/LICENSE-2.0"><img alt="Apache License" style="border-width:0" src="https://www.apache.org/img/asf-estd-1999-logo.jpg" /></a><br/>(TayROS2) </a><br/>This work is licensed under a <a rel="license" href="https://www.apache.org/licenses/LICENSE-2.0">Apache license 2.0
