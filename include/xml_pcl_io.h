#ifndef XML_PCL_IO_H
#define XML_PCL_IO_H

#include <data_types.h>
#include <list>

class xml_pcl_io
{
public:
    xml_pcl_io();
    bool write_to_file(std::string filename, const std::list< planner::pcl_polygon_with_normals >& clusters );
    bool read_from_file(std::string filename, std::list<planner::pcl_polygon_with_normals>& clusters );

};

#endif // XML_PCL_IO_H
