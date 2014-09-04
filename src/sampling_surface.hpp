/*
 * Software License Agreement (BSD License)
 * 
 * Point Cloud Library (PCL) - www.pointclouds.org
 * Copyright (c) 2009-2011, Willow Garage, Inc.
 * 
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met: 
 * 
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of the copyright holder(s) nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef PCL_FILTERS_IMPL_SAMPLING_SURFACE_H_
#define PCL_FILTERS_IMPL_SAMPLING_SURFACE_H_

#include <iostream>
#include <vector>
#include <pcl/common/eigen.h>
#include <sampling_surface.h>

///////////////////////////////////////////////////////////////////////////////
template<typename PointT> void
pcl::SamplingSurface<PointT>::applyFilter (PointCloud &output)
{
  std::vector <int> indices;
  size_t npts = input_->points.size ();
  for (unsigned int i = 0; i < npts; i++)
    indices.push_back (i);

  Vector max_vec (3, 1);
  Vector min_vec (3, 1);
  findXYZMaxMin (*input_, max_vec, min_vec);
  PointCloud data = *input_;
  partition (data, 0, npts, min_vec, max_vec, indices, output);
  output.width = 1;
  output.height = uint32_t (output.points.size ());
  std::cout<<"size:"<<final_indices.size()<<std::endl;
}

///////////////////////////////////////////////////////////////////////////////
template<typename PointT> void 
pcl::SamplingSurface<PointT>::findXYZMaxMin (const PointCloud& cloud, Vector& max_vec, Vector& min_vec)
{
  float maxval = cloud.points[0].x;
  float minval = cloud.points[0].x;

  for (unsigned int i = 0; i < cloud.points.size (); i++)
  {
    if (cloud.points[i].x > maxval)
    {
      maxval = cloud.points[i].x;
    }
    if (cloud.points[i].x < minval)
    {
      minval = cloud.points[i].x;
    }
  }
  max_vec (0) = maxval;
  min_vec (0) = minval;

  maxval = cloud.points[0].y;
  minval = cloud.points[0].y;

  for (unsigned int i = 0; i < cloud.points.size (); i++)
  {
    if (cloud.points[i].y > maxval)
    {
      maxval = cloud.points[i].y;
    }
    if (cloud.points[i].y < minval)
    {
      minval = cloud.points[i].y;
    }
  }
  max_vec (1) = maxval;
  min_vec (1) = minval;

  maxval = cloud.points[0].z;
  minval = cloud.points[0].z;

  for (unsigned int i = 0; i < cloud.points.size (); i++)
  {
    if (cloud.points[i].z > maxval)
    {
      maxval = cloud.points[i].z;
    }
    if (cloud.points[i].z < minval)
    {
      minval = cloud.points[i].z;
    }
  }
  max_vec (2) = maxval;
  min_vec (2) = minval;
}

///////////////////////////////////////////////////////////////////////////////
template<typename PointT> void 
pcl::SamplingSurface<PointT>::partition (
    const PointCloud& cloud, const int first, const int last,
    const Vector min_values, const Vector max_values, 
    std::vector<int>& indices, PointCloud&  output)
{
	const int count (last - first);
  if (count <= static_cast<int> (sample_))
  {
    samplePartition (cloud, first, last, indices, output);
    return;
  }
	int cutDim = 0;
  (max_values - min_values).maxCoeff (&cutDim);

	const int rightCount (count / 2);
	const int leftCount (count - rightCount);
	assert (last - rightCount == first + leftCount);
	
	// sort, hack std::nth_element
	std::nth_element (indices.begin () + first, indices.begin () + first + leftCount,
                    indices.begin () + last, CompareDim (cutDim, cloud));

	const int cutIndex (indices[first+leftCount]);
	const float cutVal = findCutVal (cloud, cutDim, cutIndex);
	
	// update bounds for left
	Vector leftMaxValues (max_values);
	leftMaxValues[cutDim] = cutVal;
	// update bounds for right
	Vector rightMinValues (min_values);
	rightMinValues[cutDim] = cutVal;
	
	// recurse
	partition (cloud, first, first + leftCount, min_values, leftMaxValues, indices, output);
	partition (cloud, first + leftCount, last, rightMinValues, max_values, indices, output);
}

///////////////////////////////////////////////////////////////////////////////
template<typename PointT> void 
pcl::SamplingSurface<PointT>::samplePartition (
    const PointCloud& data, const int first, const int last,
    std::vector <int>& indices, PointCloud& output)
{
  for (int i = first; i < last; i++)
  {
    // TODO: change to Boost random number generators!
    const float r = float (std::rand ()) / float (RAND_MAX);

    if (r < ratio_)
    {
      output.points.push_back (data.points[indices[i]]);
      final_indices.push_back(indices[i]);
 
    }
    
  }
  return;
}


///////////////////////////////////////////////////////////////////////////////
template <typename PointT> float
pcl::SamplingSurface<PointT>::findCutVal (
    const PointCloud& cloud, const int cut_dim, const int cut_index)
{
  if (cut_dim == 0)
    return (cloud.points[cut_index].x);
  else if (cut_dim == 1)
    return (cloud.points[cut_index].y);
  else if (cut_dim == 2)
    return (cloud.points[cut_index].z);

  return (0.0f);
}


#define PCL_INSTANTIATE_SamplingSurface(T) template class PCL_EXPORTS pcl::SamplingSurface<T>;

#endif    // PCL_FILTERS_IMPL_NORMAL_SPACE_SAMPLE_H_

