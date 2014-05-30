#include "curvature_filter_node.h"

using namespace plane_segmentation;

bool CurvatureFilter::step_is_stable(Eigen::Matrix< double, 4, 1 > centroid)
{
  return true;
}


bool CurvatureFilter::centroid_is_reachable(Eigen::Matrix<double,4,1> centroid)
{
  return true;
}

std::vector< pcl::PointCloud<pcl::PointXYZ> > CurvatureFilter::compute_polygon_grid(pcl::PointCloud<pcl::PointXYZ> polygon)
{
  std::vector< pcl::PointCloud<pcl::PointXYZ> > polygon_grid;
  
  polygon_grid.push_back(polygon);
  
  return polygon_grid;
}


bool CurvatureFilter::footstep_placer(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
  if(polygons.size()==0) {std::cout<<"No polygons to process, you should call the [/filter_by_curvature] and [/border_extraction] services first"<<std::endl; return false;}
  std::cout<<"> Number of polygons: "<<polygons.size()<<std::endl;
  
  visualization_msgs::Marker marker;
  marker.header.frame_id="/camera_link";
  marker.ns = "feet";
  marker.type = visualization_msgs::Marker::CUBE;
  marker.scale.x=0.1;
  marker.scale.y=0.05;
  marker.scale.z=0.02;
  
  marker.color.a=1;
  marker.color.b=0;

  bool left=true;
  
  Eigen::Matrix<double,4,1> centroid;

  for(unsigned int j=0;j<polygons.size();j++)
  {
      std::cout<<"> Polygon number of points: "<<polygons.at(j).size()<<std::endl;
      
      
      std::vector< pcl::PointCloud<pcl::PointXYZ> > polygon_grid =  compute_polygon_grid(polygons.at(j)); //divide the polygon in smaller ones
      
      
      for(unsigned int i=0;i<polygon_grid.size();i++)
      {
	  //for now we place the foot in the centroid of the polygon (safe if convex)
	  if(pcl::compute3DCentroid(polygon_grid.at(i), centroid ))
	  {
	    
	      if(centroid_is_reachable(centroid)) //check if the centroid id inside the reachable area
	      {
		
		
		  if(step_is_stable(centroid))//check if the centroid can be used to perform a stable step
		  {
		      
		      // for now we just alternate a left step and a right one, we should use a proper way to do it
		      
		      if(left){ marker.color.r=1; marker.color.g=0; left=false;}
		      else { marker.color.r=0; marker.color.g=1; left=true;}
		      
		      marker.pose.position.x=centroid[0];
		      marker.pose.position.y=centroid[1];
		      marker.pose.position.z=centroid[2];
			
		      marker.pose.orientation.w=0.707; //TODO: orientation (x,y) as the plane where is placed, (z) as we want
		      marker.pose.orientation.x=0;
		      marker.pose.orientation.y=0;
		      marker.pose.orientation.z=0.707;
		      
		      marker.id = j*100+i; //to have a unique id
			
		      std::cout<<"> Foot Placed in the centroid of cell "<<i<<" of the polygon "<<j<<std::endl;
		      
		      footsteps.push_back(marker.pose);
		      
		      pub_footstep.publish(marker);
		      ros::Duration half_sec(0.1);
		      half_sec.sleep();
		  } else std::cout<<"!! Step not stable"<<std::endl;
	      } else std::cout<<"!! Step not reachable"<<std::endl;
	  } else std::cout<<"!! ERR: Error in computing the centroid !!"<<std::endl;
      }
  }
	
  return true;
}