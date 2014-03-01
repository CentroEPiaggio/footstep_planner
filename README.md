plane_segmentation
==================

This project is about:

- finding planes in the point cloud (with constraints on normals etc.)
- put footprints on proper planes to make a footstep plan for robot locomotion


Example of plane detection with mesh markers
--------------------------------------------

Term1:

`roslaunch plane_segmentation plane_detection.launch.xml`

Term2:

`rosservice call /plane_detection_srv`
