# fgm_plugin
Implementation of Follow the Gap as a local planner plugin for the ROS navigation stack move_base

## Notable Reconfigurable Parameters
`go_to_goal` toggle go-to-goal behavior
`score` toggle gap selection behavior - size vs score
`alpha` tuning parameter of planner, see follow the gap original paper

## Examples
A few launch files are prepared for a demo, fgm_world.launch launches a Gazebo world for the package. nav.launch launches the planner and view_navigation.launch provides the GUI for playing with navigation.

## Dynamic World
http://gazebosim.org/tutorials?tut=actor
