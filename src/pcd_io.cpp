/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id$
 *
 */

#ifndef PCL_IO_PCD_STREAM_IMPL_H_
#define PCL_IO_PCD_STREAM_IMPL_H_

#include <fstream>
#include <fcntl.h>
#include <string>
#include <stdlib.h>
#include <pcl/io/boost.h>
#include <pcl/console/print.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/time.h>

# include <sys/mman.h>
# define pcl_open                    open
# define pcl_close(fd)               close(fd)
# define pcl_lseek(fd,offset,origin) lseek(fd,offset,origin)

#include <pcl/io/lzf.h>
#include <pcd_stream_io.hpp>


int
pcl::StreamReader::readHeader (std::istream& fs, pcl::PCLPointCloud2 &cloud,
                            Eigen::Vector4f &origin, Eigen::Quaternionf &orientation,
                            int &pcd_version, int &data_type, unsigned int &data_idx, const int offset)
{
  // Default values
  data_idx = 0;
  data_type = 0;
  pcd_version = PCD_V6;
  origin      = Eigen::Vector4f::Zero ();
  orientation = Eigen::Quaternionf::Identity ();
  cloud.width = cloud.height = cloud.point_step = cloud.row_step = 0;
  cloud.data.clear ();

  // By default, assume that there are _no_ invalid (e.g., NaN) points
  //cloud.is_dense = true;

  int nr_points = 0;
  //std::ifstream fs;
  std::string line;

  int specified_channel_count = 0;

  // Open file in binary mode to avoid problem of
  // std::getline() corrupting the result of ifstream::tellg()

  // field_sizes represents the size of one element in a field (e.g., float = 4, char = 1)
  // field_counts represents the number of elements in a field (e.g., x = 1, normal_x = 1, fpfh = 33)
  std::vector<int> field_sizes, field_counts;
  // field_types represents the type of data in a field (e.g., F = float, U = unsigned)
  std::vector<char> field_types;
  std::vector<std::string> st;

  // Read the header and fill it in with wonderful values
  try
  {
    while (!fs.eof ())
    {
      getline (fs, line);
      // Ignore empty lines
      if (line == "")
        continue;

      // Tokenize the line
      boost::trim (line);
      boost::split (st, line, boost::is_any_of ("\t\r "), boost::token_compress_on);

      std::stringstream sstream (line);
      sstream.imbue (std::locale::classic ());

      std::string line_type;
      sstream >> line_type;

      // Ignore comments
      if (line_type.substr (0, 1) == "#")
        continue;

      // Version numbers are not needed for now, but we are checking to see if they're there
      if (line_type.substr (0, 7) == "VERSION")
        continue;

      // Get the field indices (check for COLUMNS too for backwards compatibility)
      if ( (line_type.substr (0, 6) == "FIELDS") || (line_type.substr (0, 7) == "COLUMNS") )
      {
        specified_channel_count = static_cast<int> (st.size () - 1);

        // Allocate enough memory to accommodate all fields
        cloud.fields.resize (specified_channel_count);
        for (int i = 0; i < specified_channel_count; ++i)
        {
          std::string col_type = st.at (i + 1);
          cloud.fields[i].name = col_type;
        }

        // Default the sizes and the types of each field to float32 to avoid crashes while using older PCD files
        int offset = 0;
        for (int i = 0; i < specified_channel_count; ++i, offset += 4)
        {
          cloud.fields[i].offset   = offset;
          cloud.fields[i].datatype = pcl::PCLPointField::FLOAT32;
          cloud.fields[i].count    = 1;
        }
        cloud.point_step = offset;
        continue;
      }

      // Get the field sizes
      if (line_type.substr (0, 4) == "SIZE")
      {
        specified_channel_count = static_cast<int> (st.size () - 1);

        // Allocate enough memory to accommodate all fields
        if (specified_channel_count != static_cast<int> (cloud.fields.size ()))
          throw "The number of elements in <SIZE> differs than the number of elements in <FIELDS>!";

        // Resize to accommodate the number of values
        field_sizes.resize (specified_channel_count);

        int offset = 0;
        for (int i = 0; i < specified_channel_count; ++i)
        {
          int col_type ;
          sstream >> col_type;
          cloud.fields[i].offset = offset;                // estimate and save the data offsets
          offset += col_type;
          field_sizes[i] = col_type;                      // save a temporary copy
        }
        cloud.point_step = offset;
        //if (cloud.width != 0)
          //cloud.row_step   = cloud.point_step * cloud.width;
        continue;
      }

      // Get the field types
      if (line_type.substr (0, 4) == "TYPE")
      {
        if (field_sizes.empty ())
          throw "TYPE of FIELDS specified before SIZE in header!";

        specified_channel_count = static_cast<int> (st.size () - 1);

        // Allocate enough memory to accommodate all fields
        if (specified_channel_count != static_cast<int> (cloud.fields.size ()))
          throw "The number of elements in <TYPE> differs than the number of elements in <FIELDS>!";

        // Resize to accommodate the number of values
        field_types.resize (specified_channel_count);

        for (int i = 0; i < specified_channel_count; ++i)
        {
          field_types[i] = st.at (i + 1).c_str ()[0];
          cloud.fields[i].datatype = static_cast<uint8_t> (getFieldType (field_sizes[i], field_types[i]));
        }
        continue;
      }

      // Get the field counts
      if (line_type.substr (0, 5) == "COUNT")
      {
        if (field_sizes.empty () || field_types.empty ())
          throw "COUNT of FIELDS specified before SIZE or TYPE in header!";

        specified_channel_count = static_cast<int> (st.size () - 1);

        // Allocate enough memory to accommodate all fields
        if (specified_channel_count != static_cast<int> (cloud.fields.size ()))
          throw "The number of elements in <COUNT> differs than the number of elements in <FIELDS>!";

        field_counts.resize (specified_channel_count);

        int offset = 0;
        for (int i = 0; i < specified_channel_count; ++i)
        {
          cloud.fields[i].offset = offset;
          int col_count;
          sstream >> col_count;
          cloud.fields[i].count = col_count;
          offset += col_count * field_sizes[i];
        }
        // Adjust the offset for count (number of elements)
        cloud.point_step = offset;
        continue;
      }

      // Get the width of the data (organized point cloud dataset)
      if (line_type.substr (0, 5) == "WIDTH")
      {
        sstream >> cloud.width;
        if (cloud.point_step != 0)
          cloud.row_step = cloud.point_step * cloud.width;      // row_step only makes sense for organized datasets
        continue;
      }

      // Get the height of the data (organized point cloud dataset)
      if (line_type.substr (0, 6) == "HEIGHT")
      {
        sstream >> cloud.height;
        continue;
      }

      // Get the acquisition viewpoint
      if (line_type.substr (0, 9) == "VIEWPOINT")
      {
        pcd_version = PCD_V7;
        if (st.size () < 8)
          throw "Not enough number of elements in <VIEWPOINT>! Need 7 values (tx ty tz qw qx qy qz).";

        float x, y, z, w;
        sstream >> x >> y >> z ;
        origin      = Eigen::Vector4f (x, y, z, 0.0f);
        sstream >> w >> x >> y >> z;
        orientation = Eigen::Quaternionf (w, x, y, z);
        continue;
      }

      // Get the number of points
      if (line_type.substr (0, 6) == "POINTS")
      {
        sstream >> nr_points;
        // Need to allocate: N * point_step
        cloud.data.resize (nr_points * cloud.point_step);
        continue;
      }

      // Read the header + comments line by line until we get to <DATA>
      if (line_type.substr (0, 4) == "DATA")
      {
        //data_idx = static_cast<int> (fs.tellg ());
        if (st.at (1).substr (0, 17) == "binary_compressed")
         data_type = 2;
        else
          if (st.at (1).substr (0, 6) == "binary")
            data_type = 1;
        break;
      }
      break;
    }
  }
  catch (const char *exception)
  {
    PCL_ERROR ("[pcl::PCDReader::readHeader] %s\n", exception);
    return (-1);
  }

  // Exit early: if no points have been given, there's no sense to read or check anything anymore
  if (nr_points == 0)
  {
    PCL_ERROR ("[pcl::PCDReader::readHeader] No points to read\n");
    return (-1);
  }

  // Compatibility with older PCD file versions
  if (cloud.width == 0 && cloud.height == 0)
  {
    cloud.width  = nr_points;
    cloud.height = 1;
    cloud.row_step = cloud.point_step * cloud.width;      // row_step only makes sense for organized datasets
  }
  //assert (cloud.row_step != 0);       // If row_step = 0, either point_step was not set or width is 0

  // if both height/width are not given, assume an unorganized dataset
  if (cloud.height == 0)
  {
    cloud.height = 1;
    PCL_WARN ("[pcl::PCDReader::readHeader] no HEIGHT given, setting to 1 (unorganized).\n");
    if (cloud.width == 0)
      cloud.width  = nr_points;
  }
  else
  {
    if (cloud.width == 0 && nr_points != 0)
    {
      PCL_ERROR ("[pcl::PCDReader::readHeader] HEIGHT given (%d) but no WIDTH!\n", cloud.height);
      return (-1);
    }
  }

  if (int (cloud.width * cloud.height) != nr_points)
  {
    PCL_ERROR ("[pcl::PCDReader::readHeader] HEIGHT (%d) x WIDTH (%d) != number of points (%d)\n", cloud.height, cloud.width, nr_points);
    return (-1);
  }
  return (0);
}

/*
///////////////////////////////////////////////////////////////////////////////////////////
/// \brief pcl::StreamReader::readHeader
/// \param file_name
/// \param cloud
/// \param offset
/// \return
///
int
pcl::StreamReader::readHeader (const std::string &file_name, pcl::PCLPointCloud2 &cloud, const int offset)
{
  // Default values
  cloud.width = cloud.height = cloud.point_step = cloud.row_step = 0;
  cloud.data.clear ();

  // By default, assume that there are _no_ invalid (e.g., NaN) points
  //cloud.is_dense = true;

  int nr_points = 0;
  std::ifstream fs;
  std::string line;

  int specified_channel_count = 0;

  if (file_name == "" || !boost::filesystem::exists (file_name))
  {
    PCL_ERROR ("[pcl::PCDReader::readHeader] Could not find file '%s'.\n", file_name.c_str ());
    return (-1);
  }

  // Open file in binary mode to avoid problem of
  // std::getline() corrupting the result of ifstream::tellg()
  fs.open (file_name.c_str (), std::ios::binary);
  if (!fs.is_open () || fs.fail ())
  {
    PCL_ERROR ("[pcl::PCDReader::readHeader] Could not open file '%s'! Error : %s\n", file_name.c_str (), strerror(errno));
    fs.close ();
    return (-1);
  }

  // Seek at the given offset
  fs.seekg (offset, std::ios::beg);

  // field_sizes represents the size of one element in a field (e.g., float = 4, char = 1)
  // field_counts represents the number of elements in a field (e.g., x = 1, normal_x = 1, fpfh = 33)
  std::vector<int> field_sizes, field_counts;
  // field_types represents the type of data in a field (e.g., F = float, U = unsigned)
  std::vector<char> field_types;
  std::vector<std::string> st;

  // Read the header and fill it in with wonderful values
  try
  {
    while (!fs.eof ())
    {
      getline (fs, line);
      // Ignore empty lines
      if (line == "")
        continue;

      // Tokenize the line
      boost::trim (line);
      boost::split (st, line, boost::is_any_of ("\t\r "), boost::token_compress_on);

      std::stringstream sstream (line);
      sstream.imbue (std::locale::classic ());

      std::string line_type;
      sstream >> line_type;

      // Ignore comments
      if (line_type.substr (0, 1) == "#")
        continue;

      // Version numbers are not needed for now, but we are checking to see if they're there
      if (line_type.substr (0, 7) == "VERSION")
        continue;

      // Get the field indices (check for COLUMNS too for backwards compatibility)
      if ( (line_type.substr (0, 6) == "FIELDS") || (line_type.substr (0, 7) == "COLUMNS") )
      {
        specified_channel_count = static_cast<int> (st.size () - 1);

        // Allocate enough memory to accommodate all fields
        cloud.fields.resize (specified_channel_count);
        for (int i = 0; i < specified_channel_count; ++i)
        {
          std::string col_type = st.at (i + 1);
          cloud.fields[i].name = col_type;
        }

        // Default the sizes and the types of each field to float32 to avoid crashes while using older PCD files
        int offset = 0;
        for (int i = 0; i < specified_channel_count; ++i, offset += 4)
        {
          cloud.fields[i].offset   = offset;
          cloud.fields[i].datatype = pcl::PCLPointField::FLOAT32;
          cloud.fields[i].count    = 1;
        }
        cloud.point_step = offset;
        continue;
      }

      // Get the field sizes
      if (line_type.substr (0, 4) == "SIZE")
      {
        specified_channel_count = static_cast<int> (st.size () - 1);

        // Allocate enough memory to accommodate all fields
        if (specified_channel_count != static_cast<int> (cloud.fields.size ()))
          throw "The number of elements in <SIZE> differs than the number of elements in <FIELDS>!";

        // Resize to accommodate the number of values
        field_sizes.resize (specified_channel_count);

        int offset = 0;
        for (int i = 0; i < specified_channel_count; ++i)
        {
          int col_type ;
          sstream >> col_type;
          cloud.fields[i].offset = offset;                // estimate and save the data offsets
          offset += col_type;
          field_sizes[i] = col_type;                      // save a temporary copy
        }
        cloud.point_step = offset;
        //if (cloud.width != 0)
          //cloud.row_step   = cloud.point_step * cloud.width;
        continue;
      }

      // Get the field types
      if (line_type.substr (0, 4) == "TYPE")
      {
        if (field_sizes.empty ())
          throw "TYPE of FIELDS specified before SIZE in header!";

        specified_channel_count = static_cast<int> (st.size () - 1);

        // Allocate enough memory to accommodate all fields
        if (specified_channel_count != static_cast<int> (cloud.fields.size ()))
          throw "The number of elements in <TYPE> differs than the number of elements in <FIELDS>!";

        // Resize to accommodate the number of values
        field_types.resize (specified_channel_count);

        for (int i = 0; i < specified_channel_count; ++i)
        {
          field_types[i] = st.at (i + 1).c_str ()[0];
          cloud.fields[i].datatype = static_cast<uint8_t> (getFieldType (field_sizes[i], field_types[i]));
        }
        continue;
      }

      // Get the field counts
      if (line_type.substr (0, 5) == "COUNT")
      {
        if (field_sizes.empty () || field_types.empty ())
          throw "COUNT of FIELDS specified before SIZE or TYPE in header!";

        specified_channel_count = static_cast<int> (st.size () - 1);

        // Allocate enough memory to accommodate all fields
        if (specified_channel_count != static_cast<int> (cloud.fields.size ()))
          throw "The number of elements in <COUNT> differs than the number of elements in <FIELDS>!";

        field_counts.resize (specified_channel_count);

        int offset = 0;
        for (int i = 0; i < specified_channel_count; ++i)
        {
          cloud.fields[i].offset = offset;
          int col_count;
          sstream >> col_count;
          cloud.fields[i].count = col_count;
          offset += col_count * field_sizes[i];
        }
        // Adjust the offset for count (number of elements)
        cloud.point_step = offset;
        continue;
      }

      // Get the width of the data (organized point cloud dataset)
      if (line_type.substr (0, 5) == "WIDTH")
      {
        sstream >> cloud.width;
        if (cloud.point_step != 0)
          cloud.row_step = cloud.point_step * cloud.width;      // row_step only makes sense for organized datasets
        continue;
      }

      // Get the height of the data (organized point cloud dataset)
      if (line_type.substr (0, 6) == "HEIGHT")
      {
        sstream >> cloud.height;
        continue;
      }

      // Check the format of the acquisition viewpoint
      if (line_type.substr (0, 9) == "VIEWPOINT")
      {
        if (st.size () < 8)
          throw "Not enough number of elements in <VIEWPOINT>! Need 7 values (tx ty tz qw qx qy qz).";
        continue;
      }

      // Get the number of points
      if (line_type.substr (0, 6) == "POINTS")
      {
        sstream >> nr_points;
        // Need to allocate: N * point_step
        cloud.data.resize (nr_points * cloud.point_step);
        continue;
      }
      break;
    }
  }
  catch (const char *exception)
  {
    PCL_ERROR ("[pcl::PCDReader::readHeader] %s\n", exception);
    fs.close ();
    return (-1);
  }

  // Exit early: if no points have been given, there's no sense to read or check anything anymore
  if (nr_points == 0)
  {
    PCL_ERROR ("[pcl::PCDReader::readHeader] No points to read\n");
    fs.close ();
    return (-1);
  }

  // Compatibility with older PCD file versions
  if (cloud.width == 0 && cloud.height == 0)
  {
    cloud.width  = nr_points;
    cloud.height = 1;
    cloud.row_step = cloud.point_step * cloud.width;      // row_step only makes sense for organized datasets
  }
  //assert (cloud.row_step != 0);       // If row_step = 0, either point_step was not set or width is 0

  // if both height/width are not given, assume an unorganized dataset
  if (cloud.height == 0)
  {
    cloud.height = 1;
    PCL_WARN ("[pcl::PCDReader::readHeader] no HEIGHT given, setting to 1 (unorganized).\n");
    if (cloud.width == 0)
      cloud.width  = nr_points;
  }
  else
  {
    if (cloud.width == 0 && nr_points != 0)
    {
      PCL_ERROR ("[pcl::PCDReader::readHeader] HEIGHT given (%d) but no WIDTH!\n", cloud.height);
      fs.close ();
      return (-1);
    }
  }

  if (int (cloud.width * cloud.height) != nr_points)
  {
    PCL_ERROR ("[pcl::PCDReader::readHeader] HEIGHT (%d) x WIDTH (%d) != number of points (%d)\n", cloud.height, cloud.width, nr_points);
    fs.close ();
    return (-1);
  }

  // Close file
  fs.close ();

  return (0);
}

*/
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int
pcl::StreamReader::read (std::istream& fs, pcl::PCLPointCloud2 &cloud,
                      Eigen::Vector4f &origin, Eigen::Quaternionf &orientation, int &pcd_version,
                      const int offset)
{
  pcl::console::TicToc tt;
  tt.tic ();

  int data_type;
  unsigned int data_idx;

  int res = readHeader (fs, cloud, origin, orientation, pcd_version, data_type, data_idx, offset);

  if (res < 0)
    return (res);

  unsigned int idx = 0;

  // Get the number of points the cloud should have
  unsigned int nr_points = cloud.width * cloud.height;

  // Setting the is_dense property to true by default
  cloud.is_dense = true;

  // if ascii
  if (data_type == 0)
  {
    std::string line;
    std::vector<std::string> st;

    // Read the rest of the file
    try
    {
      while (idx < nr_points && !fs.eof ())
      {
        getline (fs, line);
        // Ignore empty lines
        if (line == "")
          continue;

        // Tokenize the line
        boost::trim (line);
        boost::split (st, line, boost::is_any_of ("\t\r "), boost::token_compress_on);

        if (idx >= nr_points)
        {
          PCL_WARN ("[pcl::PCDReader::read] input file has more points (%d) than advertised (%d)!\n", idx, nr_points);
          break;
        }

        size_t total = 0;
        // Copy data
        for (unsigned int d = 0; d < static_cast<unsigned int> (cloud.fields.size ()); ++d)
        {
          // Ignore invalid padded dimensions that are inherited from binary data
          if (cloud.fields[d].name == "_")
          {
            total += cloud.fields[d].count; // jump over this many elements in the string token
            continue;
          }
          for (unsigned int c = 0; c < cloud.fields[d].count; ++c)
          {
            switch (cloud.fields[d].datatype)
            {
              case pcl::PCLPointField::INT8:
              {
                copyStringValue<pcl::traits::asType<pcl::PCLPointField::INT8>::type> (
                    st.at (total + c), cloud, idx, d, c);
                break;
              }
              case pcl::PCLPointField::UINT8:
              {
                copyStringValue<pcl::traits::asType<pcl::PCLPointField::UINT8>::type> (
                    st.at (total + c), cloud, idx, d, c);
                break;
              }
              case pcl::PCLPointField::INT16:
              {
                copyStringValue<pcl::traits::asType<pcl::PCLPointField::INT16>::type> (
                    st.at (total + c), cloud, idx, d, c);
                break;
              }
              case pcl::PCLPointField::UINT16:
              {
                copyStringValue<pcl::traits::asType<pcl::PCLPointField::UINT16>::type> (
                    st.at (total + c), cloud, idx, d, c);
                break;
              }
              case pcl::PCLPointField::INT32:
              {
                copyStringValue<pcl::traits::asType<pcl::PCLPointField::INT32>::type> (
                    st.at (total + c), cloud, idx, d, c);
                break;
              }
              case pcl::PCLPointField::UINT32:
              {
                copyStringValue<pcl::traits::asType<pcl::PCLPointField::UINT32>::type> (
                    st.at (total + c), cloud, idx, d, c);
                break;
              }
              case pcl::PCLPointField::FLOAT32:
              {
                copyStringValue<pcl::traits::asType<pcl::PCLPointField::FLOAT32>::type> (
                    st.at (total + c), cloud, idx, d, c);
                break;
              }
              case pcl::PCLPointField::FLOAT64:
              {
                copyStringValue<pcl::traits::asType<pcl::PCLPointField::FLOAT64>::type> (
                    st.at (total + c), cloud, idx, d, c);
                break;
              }
              default:
                PCL_WARN ("[pcl::PCDReader::read] Incorrect field data type specified (%d)!\n",cloud.fields[d].datatype);
                break;
            }
          }
          total += cloud.fields[d].count; // jump over this many elements in the string token
        }
        idx++;
      }
    }
    catch (const char *exception)
    {
      PCL_ERROR ("[pcl::PCDReader::read] %s\n", exception);
      return (-1);
    }

  }
  else
  /// ---[ Binary mode only
  {
    fs.read((char*)&cloud.data[0],cloud.data.size());
  }

  if ((idx != nr_points) && (data_type == 0))
  {
    PCL_ERROR ("[pcl::PCDReader::read] Number of points read (%d) is different than expected (%d)\n", idx, nr_points);
    return (-1);
  }

  // No need to do any extra checks if the data type is ASCII
  if (data_type != 0)
  {
    int point_size = static_cast<int> (cloud.data.size () / (cloud.height * cloud.width));
    // Once copied, we need to go over each field and check if it has NaN/Inf values and assign cloud.is_dense to true or false
    for (uint32_t i = 0; i < cloud.width * cloud.height; ++i)
    {
      for (unsigned int d = 0; d < static_cast<unsigned int> (cloud.fields.size ()); ++d)
      {
        for (uint32_t c = 0; c < cloud.fields[d].count; ++c)
        {
          switch (cloud.fields[d].datatype)
          {
            case pcl::PCLPointField::INT8:
            {
              if (!isValueFinite<pcl::traits::asType<pcl::PCLPointField::INT8>::type>(cloud, i, point_size, d, c))
                cloud.is_dense = false;
              break;
            }
            case pcl::PCLPointField::UINT8:
            {
              if (!isValueFinite<pcl::traits::asType<pcl::PCLPointField::UINT8>::type>(cloud, i, point_size, d, c))
                cloud.is_dense = false;
              break;
            }
            case pcl::PCLPointField::INT16:
            {
              if (!isValueFinite<pcl::traits::asType<pcl::PCLPointField::INT16>::type>(cloud, i, point_size, d, c))
                cloud.is_dense = false;
              break;
            }
            case pcl::PCLPointField::UINT16:
            {
              if (!isValueFinite<pcl::traits::asType<pcl::PCLPointField::UINT16>::type>(cloud, i, point_size, d, c))
                cloud.is_dense = false;
              break;
            }
            case pcl::PCLPointField::INT32:
            {
              if (!isValueFinite<pcl::traits::asType<pcl::PCLPointField::INT32>::type>(cloud, i, point_size, d, c))
                cloud.is_dense = false;
              break;
            }
            case pcl::PCLPointField::UINT32:
            {
              if (!isValueFinite<pcl::traits::asType<pcl::PCLPointField::UINT32>::type>(cloud, i, point_size, d, c))
                cloud.is_dense = false;
              break;
            }
            case pcl::PCLPointField::FLOAT32:
            {
              if (!isValueFinite<pcl::traits::asType<pcl::PCLPointField::FLOAT32>::type>(cloud, i, point_size, d, c))
                cloud.is_dense = false;
              break;
            }
            case pcl::PCLPointField::FLOAT64:
            {
              if (!isValueFinite<pcl::traits::asType<pcl::PCLPointField::FLOAT64>::type>(cloud, i, point_size, d, c))
                cloud.is_dense = false;
              break;
            }
          }
        }
      }
    }
  }
  double total_time = tt.toc ();
  printf ("[pcl::PCDReader::read] Loaded as a %s cloud in %g ms with %d points. Available dimensions: %s.\n",
             cloud.is_dense ? "dense" : "non-dense", total_time,
             cloud.width * cloud.height, pcl::getFieldsList (cloud).c_str ());
  return (0);
}

/*
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// \brief pcl::StreamReader::read
/// \param file_name
/// \param cloud
/// \param offset
/// \return
///
int
pcl::StreamReader::read (const std::string &file_name, pcl::PCLPointCloud2 &cloud, const int offset)
{
  int pcd_version;
  Eigen::Vector4f origin;
  Eigen::Quaternionf orientation;
  // Load the data
  int res = read (file_name, cloud, origin, orientation, pcd_version, offset);

  if (res < 0)
    return (res);

  return (0);
}
*/

#endif  //#ifndef PCL_IO_PCD_STREAM_H_
