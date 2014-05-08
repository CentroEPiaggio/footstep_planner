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

Example of convex hull estimation
----------------------------------------------------------

Term1:

`roslaunch plane_segmentation curvature_filter.launch`

Term2:

`rosservice call /filter_by_curvature`

Term2:

`rosservice call /convex_hull`

The last service extracts the convex hulls from clusters, and compute the centroids


Example of borders estimation
----------------------------------------------------------

Term1:

`roslaunch plane_segmentation curvature_filter.launch`

Term2:

`rosservice call /filter_by_curvature`

Term2:

`rosservice call /border_extraction`

The last service extracts borders from clusters and generate related polygons


Example of footstep placing
----------------------------------------------------------

Term1:

`roslaunch plane_segmentation curvature_filter.launch`

Term2:

`rosservice call /filter_by_curvature`

Term2:

`rosservice call /border_extraction`

Term2:

`rosservice call /footstep_placer`

The last service for now just place the feet in the centroid of the polygons (safe if polygons are convex)
