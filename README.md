plane_segmentation
==================

This project is about:

- finding planes in the point cloud (with constraints on normals etc.)
- put footprints on proper planes to make a footstep plan for robot locomotion


Example of plane detection with mesh markers
--------------------------------------------

Term1:

`roslaunch plane_segmentation plane_detection.launch`

Term2:

`rosservice call /plane_detection_srv`


Example of planar points based on the curvature estimation
----------------------------------------------------------

Term1:

`roslaunch plane_segmentation curvature_filter.launch`

Term2:

`rosservice call /filter_by_curvature`

Parameters:

- the grid size in meters for downsampling
- the search radius in meters for the normal estimation
- the curvature threshold for extract the planar areas in the downsampled cloud
- the minimum cluster size for euclidean clustering

Example of footstep placing
----------------------------------------------------------

Term1:

`roslaunch plane_segmentation curvature_filter.launch`

Term2:

`rosservice call /filter_by_curvature`

Term2:

`rosservice call /footstep_place`

The last service for now just extracts the convex hulls from clusters, and compute the centroids
