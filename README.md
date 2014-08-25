Footstep planner
==================

This project is about:

- finding planes in the point cloud (with constraints on normals etc.)
- put footprints on proper planes to make a footstep plan for robot locomotion

Startup of required software
--------------------------------------------
`roscore`

`yarpserver --write`

Either specify a point cloud scene:

`roslaunch footstep_planner fake_primesense.launch filename:=YOURTESTSCENE.pcd`

or just load the default one:

`roslaunch footstep_planner fake_primesense.launch`

Start a static tf publisher in order to put a world frame into the point cloud (values should be changed accordingly to the loaded scene.pcd)

`roslaunch footstep_planner static.launch`

Finally start the footstep planner:

`roslaunch footstep_planner footstep_planner.launch`

Connect a persistent /commands port to the foostep planner command input port

`yarp connect --persist /commands /footstep_planner/command:i`

Open /commands port to write commands

`yarp write /commands`

List of available commands
==========================
Note: seq_num is a required sequence number for checking command correctness, at the moment you can use 0

1. (cap\_plan) seq\_num
-----------------------

Reads a point cloud from a topic, recognizes planes, plans one step

2. (cap\_save) seq\_num
-----------------------

Reads a point cloud from a topic, recognizes planes, saves planes into a file

3. (load\_plan) seq\_num
------------------------

Reads set of planes from a file, plans one step

4. (plan\_all) seq\_num
-----------------------

Plans as much step as possible until no steps are found

5. (draw\_path) seq\_num
------------------------

Draws the planned sequence of steps up to now

6. (direction x y z) seq\_num
------------------------

Sets the preferred walking direction (world frame?)

7. (exit) seq\_num
-------------------------

Exit the footstep planner in a CLEAN way

Parameters of the point cloud vision algorithm:
------------------------
- the grid size in meters for downsampling
- the search radius in meters for the normal estimation
- the curvature threshold for extract the planar areas in the downsampled cloud
- the minimum cluster size for euclidean clustering
- the cluster tolerance to accept points within a cluster in euclidean measure

