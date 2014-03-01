#ifndef _MARKER_GENERATOR_H_
#define _MARKER_GENERATOR_H_

#include <vector>

#include <boost/ptr_container/ptr_vector.hpp>

#include <visualization_msgs/Marker.h>
#include <shape_msgs/Mesh.h>
#include <geometry_msgs/Pose.h>

namespace plane_segmentation {

//! A convenience class for generating markers based on various clustering and fitting data
class MarkerGenerator {
 public:
  //! Create a line strip marker that goes around a detected table
  static visualization_msgs::Marker getPlaneMarker(float xmin, float xmax, float ymin, float ymax);
  //! A marker showing where a convex hull plane is
  static visualization_msgs::Marker getConvexHullPlaneMarker(const shape_msgs::Mesh &mesh);
  //! Create a generic Marker
  static visualization_msgs::Marker createMarker(std::string frame_id, double duration, double xdim, double ydim, double zdim,
					  double r, double g, double b, int type, int id, std::string ns, geometry_msgs::Pose pose);
};

}//namespace

#endif
