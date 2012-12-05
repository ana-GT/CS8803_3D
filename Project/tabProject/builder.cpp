/**
 * @file builder.cpp
 */
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>

#include <pcl/features/normal_3d.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>

#include "builder.h"
#include "PCL_Tools/PCL_Tools.h"
#include <boost/thread/thread.hpp>

#include <Eigen/Dense>

/**
 * @function Builder
 */
Builder::Builder() {

  mSavePCDFlag = false;
}

/**
 * @function ~Builder
 */
Builder::~Builder() {

}


/**
 * @function addPCD
 */
bool Builder::addPCD( pcl::PointCloud<pcl::PointXYZ>::Ptr _pcd,
		      Eigen::Matrix4d _transform ) {

  mPCData.push_back( _pcd );
  printf("Added Pointcloud. Current number of PCD: %d \n", mPCData.size() );
  mEndEffectorTransforms.push_back( _transform );
  // Save the transformation
  std::cout << "Transformation saved:\n " << _transform << std::endl;
  return true;
}

/**
 * @function show
 */
void Builder::show( pcl::PointCloud<pcl::PointXYZ>::Ptr _pcd ) {

  mViewer = createViewer( 0, 0, 0);
  viewPCD( _pcd, mViewer, 200, 200, 200 );

  while( !mViewer->wasStopped() ) {
    mViewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::seconds (1));
  }

  mViewer.reset();

  printf( "Stopped show Viewer \n" );

}

/**
 * @function showFullPCD
 */
void Builder::showFullPCD() {
  
  mViewer = createViewer( 0, 0, 0);
  //mViewer->createViewPort (0.0, 0, 0.5, 1.0, mViewport1 );
  //mViewer->createViewPort (0.5, 0, 1.0, 1.0, mViewport2 );
  for( int i = 0; i < mTransformedPCData.size(); ++i ) {
    viewPCD( mTransformedPCData[i], mViewer, 200, 200, 200 );
  }
  viewBall(0,0,0, 0.2, mViewer, 250,0,0);
  while( !mViewer->wasStopped() ) {
    mViewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::seconds (1));
  }

  mViewer.reset();

  printf( "Stopped show Viewer \n" );
}

/**
 * @function showMesh
 */
void Builder::showMesh( ) {

  mViewer = createViewer( 0, 0, 0);
  viewMesh( &mTriangleMesh, mViewer, 200, 200, 200 );

  while( !mViewer->wasStopped() ) {
    mViewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::seconds (1));
  }

  mViewer.reset();

  printf( "Stopped show Full PCD Viewer \n" );

}


/**
 * @function getMesh
 */
void Builder::getMesh(  pcl::PointCloud<pcl::PointXYZ>::Ptr _cloud ) {

  printf("Getting mesh \n");
  // Normal estimation*
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);

  tree->setInputCloud ( _cloud );

  n.setInputCloud (_cloud);
  n.setSearchMethod (tree);
  n.setKSearch (20);
  n.compute (*normals);
  //* normals should not contain the point normals + surface curvatures

  // Concatenate the XYZ and normal fields*
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals ( new pcl::PointCloud<pcl::PointNormal> );
  pcl::concatenateFields ( *_cloud, *normals, *cloud_with_normals );
  //* cloud_with_normals = cloud + normals

  // Create search tree*
  pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
  tree2->setInputCloud (cloud_with_normals);

  // Initialize objects
  pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;

  // Set the maximum distance between connected points (maximum edge length)
  gp3.setSearchRadius (0.1);

  // Set typical values for the parameters
  gp3.setMu (2.5);
  gp3.setMaximumNearestNeighbors (100); // originally 100
  gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
  gp3.setMinimumAngle(M_PI/18); // 10 degrees
  gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
  gp3.setNormalConsistency(false);

  // Get result
  gp3.setInputCloud ( cloud_with_normals );
  gp3.setSearchMethod ( tree2 );
  gp3.reconstruct ( mTriangleMesh );

  // Additional vertex information
  //std::vector<int> parts = gp3.getPartIDs();
  //std::vector<int> states = gp3.getPointStates();

  printf("End of mesh \n");
}

/**
 * @function bundle1
 */
void Builder::bundle1() {

  // Load data
  std::vector< PCD, Eigen::aligned_allocator<PCD> > data;
  Eigen::Matrix4f pairTransform;
  PointCloud::Ptr result (new PointCloud), source, target;

  mTransformedPCData.resize(0);
  mSmallTransformedPCData.resize(0);

  // Pre-process data
  loadData ( data );

  PCL_INFO ("* Loaded %d datasets.", (int)data.size ());
  /**  
  // Create a PCLVisualizer object
  p = new pcl::visualization::PCLVisualizer (argc, argv, "Pairwise Incremental Registration example");
  p->createViewPort (0.0, 0, 0.5, 1.0, vp_1);
  p->createViewPort (0.5, 0, 1.0, 1.0, vp_2);
  */

  // Initialize global transform
  mGlobalTransform = Eigen::Matrix4f::Identity ();

  // Add first one by default
  mTransformedPCData.push_back( mPCData[0] );
  // And its small version
  pcl::VoxelGrid<PointT> grid0;
  PointCloud::Ptr smaller0 (new PointCloud);       
  grid0.setLeafSize (0.5, 0.5, 0.5);
  grid0.setInputCloud (mPCData[0]);
  grid0.filter (*smaller0);
  mSmallTransformedPCData.push_back( smaller0 );


  // For all pairs
  for (size_t i = 1; i < data.size (); ++i)
  {
    source = data[i-1].cloud;
    target = data[i].cloud;

    /*
    // Add visualization data
    showCloudsLeft(source, target);
    */
    PointCloud::Ptr temp (new PointCloud);
    PCL_INFO ("Aligning (%d) with (%d).\n", i-1, i );
    pairAlign (source, target, temp, pairTransform, true);

    //transform current pair into the global transform
    pcl::transformPointCloud (*temp, *result, mGlobalTransform);

    //update the global transform
    mGlobalTransform = pairTransform * mGlobalTransform;

    //save aligned pair, transformed into the first cloud's frame
    std::stringstream ss;
    ss << i << ".pcd";
    pcl::io::savePCDFile (ss.str (), *result, true);  
    mTransformedPCData.push_back( result );

    // Also save downsampled version
    pcl::VoxelGrid<PointT> grid;
    PointCloud::Ptr smaller (new PointCloud);       
    grid.setLeafSize (0.5, 0.5, 0.5);
    grid.setInputCloud (result);
    grid.filter (*smaller);
    mSmallTransformedPCData.push_back( smaller );

  }


  // Stitch them together
  stitchTransformedData();
}
  
  /**
 * @function loadData
 */
void Builder::loadData ( std::vector<PCD, Eigen::aligned_allocator<PCD> > &models ) {

  printf("Load data \n");
  // Suppose the first argument is the actual test model
  for ( int i = 0; i < mPCData.size(); i++ )  {
    // Load the cloud and saves it into the global list of models
    PCD m;

    char name[50]; sprintf( name, "cloud%.3d", i );
    m.f_name = name;
    m.cloud = mPCData[i];
    //remove NAN points from the cloud
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud( *m.cloud, *m.cloud, indices );
    
    models.push_back (m);
  }
  printf("End load data \n");
}

/**
 * @function stitchTransformedData
 */
void Builder::stitchTransformedData() {

  // Save each PCD 
  int numPoints = 0;

  pcl::PointCloud<pcl::PointXYZ>::Ptr temp( new pcl::PointCloud<pcl::PointXYZ> );
  for( int i = 0; i < mTransformedPCData.size(); ++i ) {
    numPoints += mTransformedPCData[i]->points.size();
  }
  temp->width = 1;
  temp->height = numPoints;
  temp->points.resize( temp->width*temp->height );


  // Bundle them together in a big PCD file
  int index = 0;
  for( unsigned int i = 0; i < mTransformedPCData.size(); ++i ) {
    for( unsigned int j = 0; j < mTransformedPCData[i]->points.size(); ++j ) {
      temp->points[index].x = mTransformedPCData[i]->at(j).x;
      temp->points[index].y = mTransformedPCData[i]->at(j).y;
      temp->points[index].z = mTransformedPCData[i]->at(j).z;
      index++;
    }
  }    
  
  mBundledTransformedPointClouds = temp;

  // Save it in a file
  std::stringstream stream;
  stream << "BundledGlobalFrames.pcd";
  std::string filename = stream.str();
  
  if( pcl::io::savePCDFile( filename, *mBundledTransformedPointClouds, true ) == 0 ) {
    std::cout << "Saved " << filename << std::endl;
  }

  /// Small version
  // Save each PCD 
  /*
  numPoints = 0;

  pcl::PointCloud<pcl::PointXYZ>::Ptr tempS( new pcl::PointCloud<pcl::PointXYZ> );
  for( unsigned int i = 0; i < mSmallTransformedPCData.size(); ++i ) {
    numPoints += mSmallTransformedPCData[i]->points.size();
  }
  tempS->width = 1;
  tempS->height = numPoints;
  tempS->points.resize( tempS->width*tempS->height );


  // Bundle them together in a big PCD file
  index = 0;
  for( int i = 0; i < mSmallTransformedPCData.size(); ++i ) {
    for( int j = 0; j < mSmallTransformedPCData[i]->points.size(); ++j ) {
      tempS->points[index].x = mSmallTransformedPCData[i]->at(j).x;
      tempS->points[index].y = mSmallTransformedPCData[i]->at(j).y;
      tempS->points[index].z = mSmallTransformedPCData[i]->at(j).z;
      index++;
    }
  }    
  
  mSmallBundledTransformedPointClouds = tempS;

  // Save it in a file
  std::stringstream streamS;
  streamS << "SmallBundledGlobalFrames.pcd";
  std::string filenameS = stream.str();
  
  if( pcl::io::savePCDFile( filenameS, *mSmallBundledTransformedPointClouds, true ) == 0 ) {
    std::cout << "Saved " << filename << std::endl;
  }
  */
}

/**
 * @function pairAlign
 */
void Builder::pairAlign( const PointCloud::Ptr cloud_src, 
			 const PointCloud::Ptr cloud_tgt, 
			 PointCloud::Ptr output, 
			 Eigen::Matrix4f &final_transform, 
			 bool downsample ) {

  // Downsample for consistency and speed
  // \note enable this for large datasets
  PointCloud::Ptr src (new PointCloud);
  PointCloud::Ptr tgt (new PointCloud);
  pcl::VoxelGrid<PointT> grid;
  if (downsample)
  {
    grid.setLeafSize (0.05, 0.05, 0.05);
    grid.setInputCloud (cloud_src);
    grid.filter (*src);

    grid.setInputCloud (cloud_tgt);
    grid.filter (*tgt);
  }
  else
  {
    src = cloud_src;
    tgt = cloud_tgt;
  }


  // Compute surface normals and curvature
  PointCloudWithNormals::Ptr points_with_normals_src ( new PointCloudWithNormals );
  PointCloudWithNormals::Ptr points_with_normals_tgt ( new PointCloudWithNormals );

  pcl::NormalEstimation<PointT, PointNormalT> norm_est;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  norm_est.setSearchMethod (tree);
  norm_est.setKSearch (30);
  
  norm_est.setInputCloud (src);
  norm_est.compute (*points_with_normals_src);
  pcl::copyPointCloud (*src, *points_with_normals_src);

  norm_est.setInputCloud (tgt);
  norm_est.compute (*points_with_normals_tgt);
  pcl::copyPointCloud (*tgt, *points_with_normals_tgt);

  //
  // Instantiate our custom point representation (defined above) ...
  MyPointRepresentation point_representation;
  // ... and weight the 'curvature' dimension so that it is balanced against x, y, and z
  float alpha[4] = {1.0, 1.0, 1.0, 1.0};
  point_representation.setRescaleValues (alpha);

  //
  // Align
  pcl::IterativeClosestPointNonLinear<PointNormalT, PointNormalT> reg;
  reg.setTransformationEpsilon (1e-6);
  // Set the maximum distance between two correspondences (src<->tgt) to 10cm
  // Note: adjust this based on the size of your datasets
  reg.setMaxCorrespondenceDistance (0.1);  
  // Set the point representation
  reg.setPointRepresentation (boost::make_shared<const MyPointRepresentation> (point_representation));

  reg.setInputCloud (points_with_normals_src);
  reg.setInputTarget (points_with_normals_tgt);


  // Run the same optimization in a loop and visualize the results
  Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity (), prev, targetToSource;
  PointCloudWithNormals::Ptr reg_result = points_with_normals_src;
  reg.setMaximumIterations (2);
  for (int i = 0; i < 30; ++i)
  {
    PCL_INFO ("Iteration Nr. %d.\n", i);

    // save cloud for visualization purpose
    points_with_normals_src = reg_result;

    // Estimate
    reg.setInputCloud (points_with_normals_src);
    reg.align (*reg_result);
    
    //accumulate transformation between each Iteration
    Ti = reg.getFinalTransformation () * Ti;
    
    //if the difference between this transformation and the previous one
    //is smaller than the threshold, refine the process by reducing
    //the maximal correspondence distance
    if (fabs ((reg.getLastIncrementalTransformation () - prev).sum ()) < reg.getTransformationEpsilon ())
      reg.setMaxCorrespondenceDistance (reg.getMaxCorrespondenceDistance () - 0.001);
    
    prev = reg.getLastIncrementalTransformation ();
    
    // visualize current state
    //showCloudsRight(points_with_normals_tgt, points_with_normals_src);
  }
  
  // Get the transformation from target to source
  targetToSource = Ti.inverse();
  
  // Transform target back in source frame
  pcl::transformPointCloud (*cloud_tgt, *output, targetToSource);
  /*
  p->removePointCloud ("source");
  p->removePointCloud ("target");
  
  PointCloudColorHandlerCustom<PointT> cloud_tgt_h (output, 0, 255, 0);
  PointCloudColorHandlerCustom<PointT> cloud_src_h (cloud_src, 255, 0, 0);
  p->addPointCloud (output, cloud_tgt_h, "target", vp_2);
  p->addPointCloud (cloud_src, cloud_src_h, "source", vp_2);
  
  PCL_INFO ("Press q to continue the registration.\n");
  p->spin ();
  
  p->removePointCloud ("source"); 
  p->removePointCloud ("target");
  */
  //add the source to the transformed target

  //*output += *cloud_src; // Only output
  printf("Size source: %d size target: %d size output: %d \n", src->points.size(), tgt->points.size(), output->points.size() );
  final_transform = targetToSource;

}

/**
 * @function bundle2
 */
void Builder::bundle2() {


  Eigen::Matrix4d pairTransform;

  mTransformedPCData.resize(0);
  mSmallTransformedPCData.resize(0);

  // Add first one by default
  //mTransformedPCData.push_back( mPCData[0] );

  // For all pairs
  for (size_t i = 0; i < mPCData.size (); ++i)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr result(new pcl::PointCloud<pcl::PointXYZ> );
    PCL_INFO ("Aligning (%d) with (%d).\n", i, 0 );
    pairAlign2 ( i, mPCData[i], result, pairTransform );

    //save aligned pair, transformed into the first cloud's frame
    std::stringstream ss;
    ss << i << ".pcd";
    pcl::io::savePCDFile (ss.str (), *result, true);  
    mTransformedPCData.push_back( result );
  }

  // Stitch them together
  stitchTransformedData();
}

/**
 * @function pairAlign2
 */
void Builder::pairAlign2( int _index, 
			  const pcl::PointCloud<pcl::PointXYZ>::Ptr &_target, 
			  pcl::PointCloud<pcl::PointXYZ>::Ptr _output, 
			  Eigen::Matrix4d &_final_transform ) {
  
  
  // Find the transformation
  _final_transform = Eigen::Matrix4d::Identity();
  //_final_transform = ( mEndEffectorTransforms[_index].inverse() )*( mEndEffectorTransforms[0] );
  // _final_transform = ( mEndEffectorTransforms[_index] )*( mEndEffectorTransforms[0].inverse() );
  //_final_transform = ( mEndEffectorTransforms[0] )*( mEndEffectorTransforms[_index].inverse() );
  Eigen::FullPivLU<Eigen::MatrixXd> lu(  mEndEffectorTransforms[_index] );
  _final_transform = mEndEffectorTransforms[_index];

  Eigen::MatrixXd temp = mEndEffectorTransforms[_index];

  std::cout << "Final transform for "<< _index << ": \n"<< _final_transform << std::endl;
  std::cout << "With normal version: \n "<< temp << std::endl;
  
  // Transform target back in source frame
  Eigen::Matrix4f ma = Eigen::MatrixXf::Identity(4,4);
  
  ma(0,0) = (float) _final_transform(0,0);
  ma(0,1) = (float) _final_transform(0,1);
  ma(0,2) = (float) _final_transform(0,2);
  ma(0,3) = (float) _final_transform(0,3);
  
  ma(1,0) = (float) _final_transform(1,0);
  ma(1,1) = (float) _final_transform(1,1);
  ma(1,2) = (float) _final_transform(1,2);
  ma(1,3) = (float) _final_transform(1,3);


  ma(2,0) = (float) _final_transform(2,0);
  ma(2,1) = (float) _final_transform(2,1);
  ma(2,2) = (float) _final_transform(2,2);
  ma(2,3) = (float) _final_transform(2,3);
  
  ma(3,0) = (float) _final_transform(3,0);
  ma(3,1) = (float) _final_transform(3,1);
  ma(3,2) = (float) _final_transform(3,2);
  ma(3,3) = (float) _final_transform(3,3); 
  
  
  std::cout << "Final transform in float for "<< _index << ": \n"<< ma << std::endl;

    //  pcl::transformPointCloud (*_target, *_output, _final_transform );
  pcl::transformPointCloud (*_target, *_output, ma );
  //*output += *cloud_src; // Only output
  printf("Size target: %d size output: %d \n", _target->points.size(), _output->points.size() );

}
