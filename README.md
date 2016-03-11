Footstep planner
==================

This project is about:

- finding planes in the point cloud (with constraints on normals etc.)
- put footprints on proper planes to make a footstep plan for robot locomotion

Compile as a standard catkin_make project in your catkin workspace.

To use different robots you should put the packages containing their models in the catkin workspace, we are currently supporting two robots:

- walkman: package *bigman_urdf* in repo **https://gitlab.robotology.eu/walkman-drc/iit-bigman-ros-pkg**
- coman: package *coman_urdf* in repo **https://github.com/EnricoMingo/iit-coman-ros-pkg**

Startup of required software
--------------------------------------------
`roscore`

Either specify a point cloud scene:

`roslaunch footstep_planner fake_primesense.launch filename:=YOURTESTSCENE.pcd`

or just load the default one:

`roslaunch footstep_planner fake_primesense.launch`

Start a static tf publisher in order to put a world frame into the point cloud, change the launch accordingly to the loaded scene.pcd. As an example, if you used stairs_2.pcd and you want to use the Walk-Man robot to plan you should launch:

`roslaunch footstep_planner stairs_walkman.launch`

Finally start the footstep planner:

`roslaunch footstep_planner footstep_planner.launch`

you can specify another robot with respect to Walk-Man using the *robot* parameter.

In order to send commands, just do a 

`rostopic pub footstep_planner/command_i std_msgs/Header TAB`

And fill the automatic empty message that ros will put in the shell with any command below.

List of available commands
==========================

1. (cap\_plan)
-----------------------

Reads a point cloud from a topic, recognizes planes, plans one step

2. (cap\_save)
-----------------------

Reads a point cloud from a topic, recognizes planes, saves planes into a file

3. (load\_plan)
------------------------

Reads set of planes from a file, plans one step

4. (plan\_all)
-----------------------

Plans as much step as possible until no steps are found

5. (draw\_path)
------------------------

Draws the planned sequence of steps up to now

6. TODO (direction x y z)
------------------------

Sets the preferred walking direction (world frame?)

7. (exit)
-------------------------

Exit the footstep planner in a CLEAN way

Parameters of the point cloud vision algorithm:
------------------------
- the grid size in meters for downsampling
- the search radius in meters for the normal estimation
- the curvature threshold for extract the planar areas in the downsampled cloud
- the minimum cluster size for euclidean clustering
- the cluster tolerance to accept points within a cluster in euclidean measure

