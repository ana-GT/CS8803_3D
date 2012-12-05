/**
 * @file builder.h
 */

#ifndef _BUILDER_H_
#define _BUILDER_H_

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>
#include <stdio.h>
#include <iostream>
#include <vector>
#include <string>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>

#include "PCL_Tools/PCL_Tools.h"

// General stuff to avoid typing
using pcl::visualization::PointCloudColorHandlerGenericField;
using pcl::visualization::PointCloudColorHandlerCustom;

//convenient typedefs
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

/**
 * @structure PCD
 * @brief convenient structure to handle our pointclouds
 */
struct PCD {

  PointCloud::Ptr cloud;
  std::string f_name;
  
PCD() : cloud (new PointCloud) {};
};


/**
 * @class MyPointRepresentation 
 * @brief  Define a new point representation for < x, y, z, curvature >
 */
class MyPointRepresentation : public pcl::PointRepresentation <PointNormalT>
{
  using pcl::PointRepresentation<PointNormalT>::nr_dimensions_;
 public:
  MyPointRepresentation () {
    // Define the number of dimensions
    nr_dimensions_ = 4;
  }
  
  // Override the copyToFloatArray method to define our feature vector
  virtual void copyToFloatArray (const PointNormalT &p, float * out) const {
    // < x, y, z, curvature >
    out[0] = p.x;
    out[1] = p.y;
    out[2] = p.z;
    out[3] = p.curvature;
  }
};


/**
 * @class Builder
 */
class Builder {
 public:

  Builder();
  ~Builder();

  bool addPCD( pcl::PointCloud<pcl::PointXYZ>::Ptr _pcd, Eigen::Matrix4d _worldTransform );
  void show( pcl::PointCloud<pcl::PointXYZ>::Ptr _pcd );

  void getMesh( pcl::PointCloud<pcl::PointXYZ>::Ptr _cloud );
  void showMesh( );

  void bundle1();
  void showFullPCD();
  void loadData ( std::vector<PCD, Eigen::aligned_allocator<PCD> > &models );
  void pairAlign( const PointCloud::Ptr cloud_src, 
		  const PointCloud::Ptr cloud_tgt, 
		  PointCloud::Ptr output, 
		  Eigen::Matrix4f &final_transform, 
		  bool downsample );
  void stitchTransformedData();
  
  void bundle2();
  void pairAlign2( int _index, 
		   const pcl::PointCloud<pcl::PointXYZ>::Ptr &_target, 
		   pcl::PointCloud<pcl::PointXYZ>::Ptr _output, 
		   Eigen::Matrix4d &_final_transform );
    
  // Variables
  boost::shared_ptr<pcl::visualization::PCLVisualizer> mViewer;
  int mViewport1;
  int mViewport2;

  std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr > mPCData;
  std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr > mTransformedPCData;
  std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr > mSmallTransformedPCData;
  pcl::PointCloud<pcl::PointXYZ>::Ptr mBundledTransformedPointClouds;
  pcl::PointCloud<pcl::PointXYZ>::Ptr mSmallBundledTransformedPointClouds;
  pcl::PolygonMesh mTriangleMesh;
  Eigen::Matrix4f mGlobalTransform;
  std::vector<Eigen::MatrixXd> mEndEffectorTransforms;

  // Utils
  bool mSavePCDFlag;
  bool mGotDataFlag;
};


#endif /** _BUILDER_H_ */
