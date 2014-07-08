#ifndef PCD_STREAM_IO_HPP
#define PCD_STREAM_IO_HPP

#include <fstream>
#include <fcntl.h>
#include <string>
#include <stdlib.h>
#include <pcl/io/boost.h>
#include <pcl/console/print.h>
#include <pcl/io/pcd_io.h>

# include <sys/mman.h>
# define pcl_open                    open
# define pcl_close(fd)               close(fd)
# define pcl_lseek(fd,offset,origin) lseek(fd,offset,origin)

#include <pcl/io/lzf.h>


namespace pcl
{

class StreamWriter
{
  public:
    StreamWriter(){}
    ~StreamWriter() {}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> std::string
generateHeader (const pcl::PointCloud<PointT> &cloud, const int nr_points = std::numeric_limits<int>::max ())
{
  std::ostringstream oss;
  oss.imbue (std::locale::classic ());

  oss << "# .PCD v0.7 - Point Cloud Data file format"
         "\nVERSION 0.7"
         "\nFIELDS";

  std::vector<pcl::PCLPointField> fields;
  pcl::getFields (cloud, fields);

  std::stringstream field_names, field_types, field_sizes, field_counts;
  for (size_t i = 0; i < fields.size (); ++i)
  {
    if (fields[i].name == "_")
      continue;
    // Add the regular dimension
    field_names << " " << fields[i].name;
    field_sizes << " " << pcl::getFieldSize (fields[i].datatype);
    field_types << " " << pcl::getFieldType (fields[i].datatype);
    int count = abs (static_cast<int> (fields[i].count));
    if (count == 0) count = 1;  // check for 0 counts (coming from older converter code)
    field_counts << " " << count;
  }
  oss << field_names.str ();
  oss << "\nSIZE" << field_sizes.str ()
      << "\nTYPE" << field_types.str ()
      << "\nCOUNT" << field_counts.str ();
  // If the user passes in a number of points value, use that instead
  if (nr_points != std::numeric_limits<int>::max ())
    oss << "\nWIDTH " << nr_points << "\nHEIGHT " << 1 << "\n";
  else
    oss << "\nWIDTH " << cloud.width << "\nHEIGHT " << cloud.height << "\n";

  oss << "VIEWPOINT " << cloud.sensor_origin_[0] << " " << cloud.sensor_origin_[1] << " " << cloud.sensor_origin_[2] << " " <<
                         cloud.sensor_orientation_.w () << " " <<
                         cloud.sensor_orientation_.x () << " " <<
                         cloud.sensor_orientation_.y () << " " <<
                         cloud.sensor_orientation_.z () << "\n";

  // If the user passes in a number of points value, use that instead
  if (nr_points != std::numeric_limits<int>::max ())
    oss << "POINTS " << nr_points << "\n";
  else
    oss << "POINTS " << cloud.points.size () << "\n";

  return (oss.str ());
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> int
writeBinary (std::ostream& oss, const pcl::PointCloud<PointT> &cloud)
{
  if (cloud.empty ())
  {
    throw pcl::IOException ("[pcl::PCDWriter::writeBinary] Input point cloud has no data!");
    return (-1);
  }
  int data_idx = 0;
  //std::ostringstream oss;
  oss << generateHeader<PointT> (cloud) << "DATA binary\n";
  oss.flush ();
  data_idx = static_cast<int> (oss.tellp ());

  std::vector<pcl::PCLPointField> fields;
  std::vector<int> fields_sizes;
  size_t fsize = 0;
  size_t data_size = 0;
  size_t nri = 0;
  pcl::getFields (cloud, fields);
  // Compute the total size of the fields
  for (size_t i = 0; i < fields.size (); ++i)
  {
    if (fields[i].name == "_")
      continue;

    int fs = fields[i].count * getFieldSize (fields[i].datatype);
    fsize += fs;
    fields_sizes.push_back (fs);
    fields[nri++] = fields[i];
  }
  fields.resize (nri);

  data_size = cloud.points.size () * fsize;

  // Copy the data
  for (size_t i = 0; i < cloud.points.size (); ++i)
  {
    int nrj = 0;
    for (size_t j = 0; j < fields.size (); ++j)
    {
      //memcpy (out, reinterpret_cast<const char*> (&cloud.points[i]) + fields[j].offset, fields_sizes[nrj]);
      //out += fields_sizes[nrj++];
      oss.write(reinterpret_cast<const char*> (&cloud.points[i]) + fields[j].offset, fields_sizes[nrj]);
      nrj++;

    }
  }

  return (0);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> int
writeASCII (std::ostream& fs, const pcl::PointCloud<PointT> &cloud,
                            const int precision=8)
{
  if (cloud.empty ())
  {
    throw pcl::IOException ("[pcl::PCDWriter::writeASCII] Input point cloud has no data!");
    return (-1);
  }

  if (cloud.width * cloud.height != cloud.points.size ())
  {
    throw pcl::IOException ("[pcl::PCDWriter::writeASCII] Number of points different than width * height!");
    return (-1);
  }

//  std::ofstream fs;

  fs.precision (precision);
  fs.imbue (std::locale::classic ());

  std::vector<pcl::PCLPointField> fields;
  pcl::getFields (cloud, fields);

  // Write the header information
  fs << generateHeader<PointT> (cloud) << "DATA ascii\n";

  std::ostringstream stream;
  stream.precision (precision);
  stream.imbue (std::locale::classic ());
  // Iterate through the points
  for (size_t i = 0; i < cloud.points.size (); ++i)
  {
    for (size_t d = 0; d < fields.size (); ++d)
    {
      // Ignore invalid padded dimensions that are inherited from binary data
      if (fields[d].name == "_")
        continue;

      int count = fields[d].count;
      if (count == 0)
        count = 1;          // we simply cannot tolerate 0 counts (coming from older converter code)

      for (int c = 0; c < count; ++c)
      {
        switch (fields[d].datatype)
        {
          case pcl::PCLPointField::INT8:
          {
            int8_t value;
            memcpy (&value, reinterpret_cast<const char*> (&cloud.points[i]) + fields[d].offset + c * sizeof (int8_t), sizeof (int8_t));
            if (pcl_isnan (value))
              stream << "nan";
            else
              stream << boost::numeric_cast<uint32_t>(value);
            break;
          }
          case pcl::PCLPointField::UINT8:
          {
            uint8_t value;
            memcpy (&value, reinterpret_cast<const char*> (&cloud.points[i]) + fields[d].offset + c * sizeof (uint8_t), sizeof (uint8_t));
            if (pcl_isnan (value))
              stream << "nan";
            else
              stream << boost::numeric_cast<uint32_t>(value);
            break;
          }
          case pcl::PCLPointField::INT16:
          {
            int16_t value;
            memcpy (&value, reinterpret_cast<const char*> (&cloud.points[i]) + fields[d].offset + c * sizeof (int16_t), sizeof (int16_t));
            if (pcl_isnan (value))
              stream << "nan";
            else
              stream << boost::numeric_cast<int16_t>(value);
            break;
          }
          case pcl::PCLPointField::UINT16:
          {
            uint16_t value;
            memcpy (&value, reinterpret_cast<const char*> (&cloud.points[i]) + fields[d].offset + c * sizeof (uint16_t), sizeof (uint16_t));
            if (pcl_isnan (value))
              stream << "nan";
            else
              stream << boost::numeric_cast<uint16_t>(value);
            break;
          }
          case pcl::PCLPointField::INT32:
          {
            int32_t value;
            memcpy (&value, reinterpret_cast<const char*> (&cloud.points[i]) + fields[d].offset + c * sizeof (int32_t), sizeof (int32_t));
            if (pcl_isnan (value))
              stream << "nan";
            else
              stream << boost::numeric_cast<int32_t>(value);
            break;
          }
          case pcl::PCLPointField::UINT32:
          {
            uint32_t value;
            memcpy (&value, reinterpret_cast<const char*> (&cloud.points[i]) + fields[d].offset + c * sizeof (uint32_t), sizeof (uint32_t));
            if (pcl_isnan (value))
              stream << "nan";
            else
              stream << boost::numeric_cast<uint32_t>(value);
            break;
          }
          case pcl::PCLPointField::FLOAT32:
          {
            float value;
            memcpy (&value, reinterpret_cast<const char*> (&cloud.points[i]) + fields[d].offset + c * sizeof (float), sizeof (float));
            if (pcl_isnan (value))
              stream << "nan";
            else
              stream << boost::numeric_cast<float>(value);
            break;
          }
          case pcl::PCLPointField::FLOAT64:
          {
            double value;
            memcpy (&value, reinterpret_cast<const char*> (&cloud.points[i]) + fields[d].offset + c * sizeof (double), sizeof (double));
            if (pcl_isnan (value))
              stream << "nan";
            else
              stream << boost::numeric_cast<double>(value);
            break;
          }
          default:
            PCL_WARN ("[pcl::PCDWriter::writeASCII] Incorrect field data type specified (%d)!\n", fields[d].datatype);
            break;
        }

        if (d < fields.size () - 1 || c < static_cast<int> (fields[d].count - 1))
          stream << " ";
      }
    }
    // Copy the stream, trim it, and write it to disk
    std::string result = stream.str ();
    boost::trim (result);
    stream.str ("");
    fs << result << "\n";
  }
  return (0);
}


};



class StreamReader
{
  public:
    /** Empty constructor */
    StreamReader () {}
    /** Empty destructor */
    ~StreamReader () {}

    enum
    {
      PCD_V6 = 0,
      PCD_V7 = 1
    };

    int
    readHeader (std::istream &fs, pcl::PCLPointCloud2 &cloud,
                Eigen::Vector4f &origin, Eigen::Quaternionf &orientation, int &pcd_version,
                int &data_type, unsigned int &data_idx, const int offset = 0);

    //int readHeader (const std::string &file_name, pcl::PCLPointCloud2 &cloud, const int offset = 0);

    int
    read (std::istream &fs, pcl::PCLPointCloud2 &cloud,
          Eigen::Vector4f &origin, Eigen::Quaternionf &orientation, int &pcd_version, const int offset = 0);

    template<typename PointT> int
    read (std::istream &fs, pcl::PointCloud<PointT> &cloud, const int offset = 0)
    {
      pcl::PCLPointCloud2 blob;
      int pcd_version;
      int res = read (fs, blob, cloud.sensor_origin_, cloud.sensor_orientation_,
                      pcd_version, offset);

      // If no error, convert the data
      if (res == 0)
        pcl::fromPCLPointCloud2 (blob, cloud);
      return (res);
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


}
#endif // PCD_STREAM_IO_HPP
