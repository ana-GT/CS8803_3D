/**
 * @file PCL_Tools
 * @brief
 * @author A. Huaman
 * @date 2012-08-15
 */
#include "PCL_Tools.h"

int PCL_TOOLS_COUNTER_PCD;
int PCL_TOOLS_COUNTER_PATH;
int PCL_TOOLS_COUNTER_BALL;
int PCL_TOOLS_COUNTER_MESH;

/**
 * @function reset_PCL_Tools_counters
 */
void reset_PCL_Tools_counters() {
  
  PCL_TOOLS_COUNTER_PCD = 0;
  PCL_TOOLS_COUNTER_PATH = 0;
  PCL_TOOLS_COUNTER_BALL = 0;
  PCL_TOOLS_COUNTER_MESH = 0;
}

/**
 * @function createViewer
 */
boost::shared_ptr<pcl::visualization::PCLVisualizer> createViewer( int _r, int _g, int _b ) {

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer( new pcl::visualization::PCLVisualizer("3D viewer") );
  viewer->setBackgroundColor( _r/255.0, _g/255.0, _b/255.0 );
  return viewer;
}

/**
 * @function viewPCD
 **/
void viewPCD( pcl::PointCloud<pcl::PointXYZ>::Ptr _cloud,
	      boost::shared_ptr<pcl::visualization::PCLVisualizer> _viewer,
	      int _r, int _g, int _b ) {

  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> colorCloud( _cloud, _r, _g, _b );

  char linename[15];
  sprintf( linename, "PCD-%d", PCL_TOOLS_COUNTER_PCD );
  std::string id( linename );
  PCL_TOOLS_COUNTER_PCD++;

  _viewer->addPointCloud<pcl::PointXYZ> ( _cloud, colorCloud, id );
  _viewer->setPointCloudRenderingProperties( pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, id );

  _viewer->addCoordinateSystem(1.0);
  _viewer->initCameraParameters();
}

/**
 * @function viewPoints
 */
void viewPoints( std::vector<std::vector<double> > _points,
		 boost::shared_ptr<pcl::visualization::PCLVisualizer> _viewer,
		 int _r, int _g, int _b ) {

  // Create PCD
  pcl::PointCloud<pcl::PointXYZ>::Ptr pointsCloud;
  pointsCloud = writePCD( _points );

  viewPCD( pointsCloud, _viewer, _r, _g, _b );
}

/**
 * @function viewPath
 */
void viewPath( std::vector<Eigen::Vector3d> _path, 
	       boost::shared_ptr<pcl::visualization::PCLVisualizer> _viewer,
	       int _r, int _g, int _b ) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
  cloud = writePCD( _path );

  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> colorCloud( cloud, _r, _g, _b );

  char linename[15];
  sprintf( linename, "Path-%d", PCL_TOOLS_COUNTER_PATH );
  std::string id( linename );
  PCL_TOOLS_COUNTER_PATH++;

  _viewer->addPointCloud<pcl::PointXYZ> ( cloud, colorCloud, id );
  _viewer->setPointCloudRenderingProperties( pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, id );

  _viewer->addCoordinateSystem(1.0);
  _viewer->initCameraParameters();
  
}

/**
 * @function viewPath
 */
void viewPath( std::vector< std::vector<double> > _path,
	       boost::shared_ptr<pcl::visualization::PCLVisualizer> _viewer,
	       int _r, int _g, int _b ) {
  
  // Create PCD
  pcl::PointCloud<pcl::PointXYZ>::Ptr pathCloud;
  pathCloud = writePCD( _path );
  
  // Color
  double r; double g; double b;

  r = ( _r % 256 )/255.0;
  g = ( _g % 256 )/255.0;
  b = ( _b % 256 )/255.0;

  for( int j = 0; j < pathCloud->points.size() - 1; ++j ) {
    char linename[15];
    sprintf( linename, "Path-%d-%d", PCL_TOOLS_COUNTER_PATH, j );
    std::string id(linename);
    _viewer->addLine<pcl::PointXYZ>( pathCloud->points[j], pathCloud->points[j + 1], r, g, b, id );
    _viewer->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 3, id );
  }
  
  PCL_TOOLS_COUNTER_PATH++;
}

/**
 * @function viewBall
 */
void viewBall( double _x, double _y, double _z,
	       double _radius,
	       boost::shared_ptr<pcl::visualization::PCLVisualizer> _viewer,
	       int _r, int _g, int _b ) {
  
  pcl::PointXYZ pos;
  pos.x = _x;
  pos.y = _y;
  pos.z = _z;
  
  char linename[15];
  sprintf( linename, "Ball-%d", PCL_TOOLS_COUNTER_BALL );
  std::string id( linename );
  PCL_TOOLS_COUNTER_BALL++;
  
  _viewer->addSphere ( pos, _radius, _r/255.0, _g/255.0, _b/255.0, id );
  _viewer->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 5, id );
  
}

/**
 * @function viewMesh
 **/
void viewMesh( pcl::PolygonMesh *_triangles,
	       boost::shared_ptr<pcl::visualization::PCLVisualizer> _viewer,
	       int _r, int _g, int _b ) {

  _viewer->addPolygonMesh( *_triangles );
} 


/**
 * @function readPCDFile
 */
void readPCDFile( char* _filename, 
		  pcl::PointCloud<pcl::PointXYZ>::Ptr _cloud ) {
  
  if( pcl::io::loadPCDFile<pcl::PointXYZ>( _filename, *_cloud ) == -1 ) {
    printf("[readPCDFile] Could not read the file. Exiting! \n");
    return;
  }
  printf("[readPCDFile] Correctly loaded! \n");
  return;
}

/**
 * @function writePCD
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr writePCD( std::vector<Eigen::Vector3d> _points ) {
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud( new pcl::PointCloud<pcl::PointXYZ> );
  // Fill cloud
  cloud->width = _points.size();
  cloud->height = 1;
  cloud->is_dense = false;
  cloud->points.resize( cloud->width*cloud->height );

  for( size_t i = 0; i < cloud->points.size(); ++i ) {
    cloud->points[i].x = _points[i](0);
    cloud->points[i].y = _points[i](1);
    cloud->points[i].z = _points[i](2);
  }

  return cloud;
}

/**
 * @function writePCD
 */ 
pcl::PointCloud<pcl::PointXYZ>::Ptr writePCD( std::vector<std::vector< double > > _points ) {
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud( new pcl::PointCloud<pcl::PointXYZ> );
  
  // Fill cloud
  cloud->width = _points.size();
  cloud->height = 1;
  cloud->is_dense = false;
  cloud->points.resize( cloud->width*cloud->height );
  
  for( size_t i = 0; i < cloud->points.size(); ++i ) {
    cloud->points[i].x = _points[i][0];
    cloud->points[i].y = _points[i][1];
    cloud->points[i].z = _points[i][2];
  }
  
  return cloud;
} 


/**
 * @function getMesh
 **/
void getMesh( pcl::PointCloud<pcl::PointXYZ>::Ptr _cloud, 
	      pcl::PolygonMesh &_triangles,
	      int _numNeighbors,
	      float _searchRadius ) {

  // Normal estimation *
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
  pcl::PointCloud<pcl::Normal>::Ptr normals( new pcl::PointCloud<pcl::Normal> );
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree( new pcl::search::KdTree<pcl::PointXYZ> );
  tree->setInputCloud( _cloud );
  n.setInputCloud( _cloud );
  n.setSearchMethod( tree );
  n.setKSearch( _numNeighbors );
  n.compute( *normals );
  //* normals should not contain the point normals  + surface curvatures
  
  // Concatenate the XYZ and normal fields*
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals( new pcl::PointCloud<pcl::PointNormal> );
  pcl::concatenateFields( *_cloud, *normals, *cloud_with_normals );
  //* cloud_with_normals = cloud + normals

  // Create search tree*
  pcl::search::KdTree<pcl::PointNormal>::Ptr tree2( new pcl::search::KdTree<pcl::PointNormal> );
  tree2->setInputCloud( cloud_with_normals );
  
  // Initialize objects
  pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;

  // Set the maximum distance between connected points
  gp3.setSearchRadius( _searchRadius );

  // Set typical values for the parameters 
  gp3.setMu(2.5);
  gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
  gp3.setMinimumAngle(M_PI/18); // 10 degrees
  gp3.setMaximumAngle(2*M_PI/3);  
  gp3.setNormalConsistency(false);


  // Get result
  gp3.setInputCloud ( cloud_with_normals );
  gp3.setSearchMethod( tree2 );
  gp3.reconstruct( _triangles );

}

/**
 * @function convertToRGB
 */
pcl::PointCloud<pcl::PointXYZRGB>::Ptr convertToRGB( pcl::PointCloud<pcl::PointXYZ>::Ptr _cloud,
						     int _r, int _g, int _b ) {
  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_RGB( new pcl::PointCloud<pcl::PointXYZRGB> );
  
  // Fill cloud
  cloud_RGB->width = _cloud->width;
  cloud_RGB->height = _cloud->height;
  cloud_RGB->is_dense = false;
  cloud_RGB->points.resize( cloud_RGB->width*cloud_RGB->height );
  
  for( size_t i = 0; i < cloud_RGB->points.size(); ++i ) {
    cloud_RGB->points[i].x = _cloud->points[i].x;
    cloud_RGB->points[i].y = _cloud->points[i].y;
    cloud_RGB->points[i].z = _cloud->points[i].z;
    cloud_RGB->points[i].r = _r;
    cloud_RGB->points[i].g = _g;
    cloud_RGB->points[i].b = _b;
  }
  
  return cloud_RGB;

}


