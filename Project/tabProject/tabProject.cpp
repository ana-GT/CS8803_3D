/*
 * Copyright (c) 2010, Georgia Tech Research Corporation
 * 
 * Humanoid Robotics Lab      Georgia Institute of Technology
 * Director: Mike Stilman     http://www.golems.org
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *     * Redistributions of source code must retain the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials
 *       provided with the distribution.
 *     * Neither the name of the Georgia Tech Research Corporation nor
 *       the names of its contributors may be used to endorse or
 *       promote products derived from this software without specific
 *       prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY GEORGIA TECH RESEARCH CORPORATION ''AS
 * IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL GEORGIA
 * TECH RESEARCH CORPORATION BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */


#include "tabProject.h"

#include <wx/wx.h>
#include <GUI/Viewer.h>
#include <GUI/Camera.h>
#include <GUI/GUI.h>
#include <GUI/GRIPSlider.h>
#include <GUI/GRIPFrame.h>

#include <robotics/World.h>
#include <robotics/Object.h>
#include <robotics/Robot.h>

#include <fstream>

#include "builder.h"

using namespace std;
using namespace Eigen;


/// Builder object
Builder gB;

/**
 * @function onButton
 */
void ProjectTab::onButton(wxCommandEvent &evt) {

  // Check if a world exists
  if(mWorld == NULL) {
    printf("%c[%d;%dmCamera: Cannot handle event because a world is not loaded.%c[%dm\n",27,1,33,27,0);
    return;
  }
  
  // Traverse each robot in the world to check for a camera
  bool noCamera = true;
  for( unsigned int i = 0; i < mWorld->getNumRobots(); i++ ) { 
    robotics::Robot* robot = mWorld->getRobot(i);
    kinematics::BodyNode* cameraNode = robot->getNode("Camera");
    if(cameraNode != NULL) {
      noCamera = false;
      break;
    }
  }
  
  if(noCamera) {
    printf("%c[%d;%dmCamera: Cannot handle event because the world does not contain a camera.%c[%dm\n",27,1,33,27,0);
    return;
  }
  
  // Get the button and switch on the set symbols
  int button_num = evt.GetId();
  switch (button_num) {
  case button_attention:
    attention();
    break;
  case button_startSearch:
    startSearch();
    break;
  case button_grabCloud:
    grabCloud();
    break;
  case button_showCloud:
    showCloud();
    break;
  case button_depthMap:
    depthMap();
    break;	
  case button_addPCD:
    gB.addPCD( mGrabbedPCD );
    break;
  case button_getMesh:
    gB.getMesh(  gB.mBundledTransformedPointClouds );
    break;
  case button_showMesh:
    gB.showMesh();
    break;
  case button_runICP:
    gB.bundle1();
    break;
  case button_showFullPCD:
    gB.showFullPCD();
    break;
  }
}

/**
 * @function attention
 */
void ProjectTab::attention () {

  // Set the predetermined configuration
  VectorXd conf (7);
  conf << 1.57, -0.2618, 0, 0, 0, 2.09, 0; // 90, -15, 120

  mWorld->getRobot(0)->setQuickDofs(conf);
  mWorld->getRobot(0)->update();
}

/**
 * @function startSearch
 */
void ProjectTab::startSearch () {
  
  // Read in the trajectory from the file
  ifstream in("../projects/tabVision/trajectory.txt");
  if(!in.is_open()) {
    printf("%c[%d;%dmCTabVision: Could not find the trajectory file.%c[%dm\n",27,1,33,27,0);
    return;
  }
  vector <VectorXd> trajectory;
  while(true) {
    VectorXd conf (7);
    for (size_t x = 0; x < 7; x++) 
      in >> conf(x);
    if(in.fail() || in.eof() || in.bad()) break; 
    trajectory.push_back(conf);
  }
  in.close();
  
  // Draw the trajectory
  size_t fps = 30;
  for(size_t i = 0; i < trajectory.size(); i++) {
    
    // Set the polar bear location
    double x, y, z;
    robotics::Object* bear = mWorld->getObject(4);
    bear->getPositionXYZ(x,y,z);
    if(i < 50) {}
    else if(i < 60)
      bear->setPositionXYZ(x,y-0.1,z);
    else if(i < 65)
      bear->setPositionXYZ(x-0.1,y-0.1,z);
    
    bear->update();
    
    // Set the robot location
    mWorld->getRobot(0)->setQuickDofs(trajectory[i]);
    mWorld->getRobot(0)->update();
    
    // Update the view
    usleep(1000000/fps);
    wxPaintEvent ev; 
    viewer->render(ev);
    camera->render(ev);
  }
}

/**
 * @function getDisparities
 */
void ProjectTab::getDisparities (vector <Vector3d>& disparities, double& focalLength) {

  // Set the constants
  size_t kWidth = 640, kHeight = 480;
  float kBaseline = 0.10;
  float kDisparityNoise = 0.1;
  
  // Assert the baseline is non-negative
  assert((kBaseline > 0.0) && "Non-positive baseline.");
  
  printf("Starting to save... "); fflush(stdout);
  
  // Get the color data
  unsigned char* im = new unsigned char [3 * kWidth * kHeight];
  glReadPixels(0,0, kWidth, kHeight, GL_RGB, GL_UNSIGNED_BYTE, im);
  
  // Get the depths of the objects
  float* depths = new float [kWidth * kHeight];
  glReadPixels(0,0, kWidth, kHeight, GL_DEPTH_COMPONENT, GL_FLOAT, depths);
  
  // Get the view options
  glLoadIdentity();
  GLdouble modelMatrix[16];
  glGetDoublev(GL_MODELVIEW_MATRIX,modelMatrix);
  GLdouble projMatrix[16];
  glGetDoublev(GL_PROJECTION_MATRIX,projMatrix);
  int viewport[4];
  glGetIntegerv(GL_VIEWPORT,viewport);
  
  // Get focal length using some pixel
  double x_, y_, z_;
  gluUnProject(300, 200, depths[200 * kWidth + 300], modelMatrix, projMatrix, viewport, &x_, &y_, &z_);
  focalLength = (300 - 320) * (-z_ / x_);
  
  // Create the pixel data
  disparities.clear();
  for(size_t v = 0; v < kHeight; v++) {
    for(size_t u = 0; u < kWidth; u++) {
      
      // Skip the pixel if it is in the background
      Vector2d pix (u,v); //= (*pix_it);
      size_t k = (pix(1) * kWidth + pix(0)) * 3;
      if((im[k] == 0) && (im[k+1] == 0) && (im[k+2] == 0)) continue;
      
      // Get the position data - if the 'z' is the far clip, skip
      size_t k2 = (pix(1) * kWidth + pix(0));
      double x_, y_, z_;
      gluUnProject(u, v, depths[k2], modelMatrix, projMatrix, viewport, &x_, &y_, &z_);
      Vector3d loc (x_, y_, z_);
      if(-loc(2) > 15.0) continue;		// TODO: Replace 15.0 with camera.zFar
      
      // Compute the disparity and add noise to it
      float disparity = (kBaseline * focalLength) / -loc(2);
      float r1 = ((float) rand()) / RAND_MAX, r2 = ((float) rand()) / RAND_MAX;
      float noiseEffect = sqrt(-2.0 * log(r1)) * cos(2 * M_PI * r2) * kDisparityNoise; 
      disparity += noiseEffect;
      
      // Save the pixel location and the disparity
      size_t color = (im[k] << 16) | (im[k+1] << 8) | (im[k+2]);
      disparities.push_back(Vector3d(k2, disparity, color));
    }
  }
  
  printf("Data acquired.\n"); fflush(stdout);
}

/**
 * @function printPCDHeader
 * @brief Prints the PCD file header
 */
void ProjectTab::printPCDHeader (FILE* file, size_t numPoints) {
  
  fprintf(file, "VERSION 0.7\n");
  fprintf(file, "FIELDS x y z rgb\n");
  fprintf(file, "SIZE 4 4 4 4\n");
  fprintf(file, "TYPE F F F U\n");
  fprintf(file, "COUNT 1 1 1 1\n");
  fprintf(file, "WIDTH 1\n");
  fprintf(file, "HEIGHT %lu\n", numPoints);
  fprintf(file, "VIEWPOINT 0 0 0 1 0 0 0\n");
  fprintf(file, "POINTS %lu\n", numPoints);
  fprintf(file, "DATA ascii\n");
}

/**
 * @function  grabCloud
 */
void ProjectTab::grabCloud () {

  // Set the constants; TODO: Move these constants to class definition
  size_t kWidth = 640, kHeight = 480;
  float kBaseline = 0.10;
  float kDisparityNoise = 0.1;
  
  // Get the disparities
  double focalLength;
  vector <Vector3d> disparities;
  getDisparities(disparities, focalLength);
  
  // Create the 3D points and PCD
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  cloud->width = 1;
  cloud->height = disparities.size();
  cloud->points.resize( cloud->width*cloud->height );

  vector <Vector3d> points;
  for(size_t i = 0; i < disparities.size(); i++) {
    
    // Get the pixel
    size_t v = (size_t) disparities[i](0) / kWidth;
    size_t u = (size_t) disparities[i](0) % kWidth;
    
    // Change the location based on the new disparity
    double disparity = disparities[i](1);
    float distZ = -(kBaseline * focalLength) / disparity;
    float distX = ((int) u - (int) kWidth/2) * distZ / focalLength;
    float distY = ((int) v - (int) kHeight/2) * distZ / focalLength;
    
    // Save the pixel
    points.push_back(Vector3d(-distX, distY, -distZ));
    // Save the pointcloud
    cloud->points[i].x = -distX;
    cloud->points[i].y = distY;
    cloud->points[i].z = -distZ;
  }
  
  mGrabbedPCD = cloud;
  printf(" [i] Grabbed cloud with %d points \n", mGrabbedPCD->height );
}

/**
 * @functino showCloud
 */
void  ProjectTab::showCloud() {
  gB.show( mGrabbedPCD );
}

/**
 * @function depthMap
 */
void ProjectTab::depthMap () {
  
  vector <Vector2d> disparities;
  //getDisparities(disparities);
  
  
}

/**
 * @function ProjectTab
 */
ProjectTab::ProjectTab(wxWindow *parent, const wxWindowID id,
		       const wxPoint& pos, const wxSize& size, long style) : GRIPTab(parent, id, pos, size, style) {
  
  // ===========================================================
  // 1. Create the left side for the vision demonstration
  
  // Create StaticBox container for the two buttons: "Attention!" and "Start Search!"
  wxStaticBox* leftBox = new wxStaticBox(this, -1, wxT("Demonstration"));
  wxStaticBoxSizer* leftBoxSizer = new wxStaticBoxSizer(leftBox, wxVERTICAL);
  
  // Add the "Attention!" button
  leftBoxSizer->Add(new wxButton(this, button_attention, wxT("Attention!")), 0, wxALL, 10);
  
  // Add the "Start Search!" button
  leftBoxSizer->Add(new wxButton(this, button_startSearch, wxT("Find Bear!")), 0, wxALL, 10);
  
  // ===========================================================
  // 2. Create the right side for 3D data acquisition
  
  // Create StaticBox container for the two buttons: "Show Cloud" and "Show Depth Map"
  wxStaticBox* rightBox = new wxStaticBox(this, -1, wxT("3D Data"));
  wxStaticBoxSizer* rightBoxSizer = new wxStaticBoxSizer(rightBox, wxVERTICAL);
  
  // Add the "grab Cloud" button
  rightBoxSizer->Add(new wxButton(this, button_grabCloud, wxT("Grab Cloud")), 0, wxALL, 10);

  // Add the "show Cloud" button
  rightBoxSizer->Add(new wxButton(this, button_showCloud, wxT("Show Cloud")), 0, wxALL, 10);
  
  // Add the "depth Map" button
  rightBoxSizer->Add(new wxButton(this, button_depthMap, wxT("Show Depth Map")), 0, wxALL, 10);
  
  // ===========================================================
  // 3. Create Registration container
  wxStaticBox* registrationBox = new wxStaticBox(this, -1, wxT("Registration"));
  wxStaticBoxSizer* registrationBoxSizer = new wxStaticBoxSizer(registrationBox, wxVERTICAL);

  // Add the "Add PCD" Button
  registrationBoxSizer->Add(new wxButton(this, button_addPCD, wxT("Add PCD")), 0, wxALL, 10);
  
  // Add the "ICP" button
  registrationBoxSizer->Add(new wxButton(this, button_runICP, wxT("Run ICP")), 0, wxALL, 10);

  // Add the "ICP" button
  registrationBoxSizer->Add(new wxButton(this, button_showFullPCD, wxT("Show Full PCD")), 0, wxALL, 10);

  // ===========================================================
  // 4. Create Surface container
  wxStaticBox* surfaceBox = new wxStaticBox(this, -1, wxT("Surface"));
  wxStaticBoxSizer* surfaceBoxSizer = new wxStaticBoxSizer(surfaceBox, wxVERTICAL);

  // Add the "Get Mesh" Button
  surfaceBoxSizer->Add(new wxButton(this, button_getMesh, wxT("Get Mesh")), 0, wxALL, 10);


  // Add the "View Mesh" Button
  surfaceBoxSizer->Add(new wxButton(this, button_showMesh, wxT("View Mesh")), 0, wxALL, 10);
  
  // ===========================================================
  // 5. Add both sizers to the full sizer
  
  // Create the sizer that controls the tab panel
  wxBoxSizer* sizerFull = new wxBoxSizer (wxHORIZONTAL);
  
  // Add the sides
  sizerFull->Add( leftBoxSizer, 2, wxALL | wxEXPAND, 10 );
  sizerFull->Add( rightBoxSizer, 2, wxALL | wxEXPAND, 10 );
  sizerFull->Add( registrationBoxSizer, 2, wxALL | wxEXPAND, 10 );
  sizerFull->Add( surfaceBoxSizer, 2, wxALL | wxEXPAND, 10 );

  // Set the full sizer as the sizer of this tab
  SetSizer(sizerFull);
}

//Add a handlers for UI changes
BEGIN_EVENT_TABLE(ProjectTab, wxPanel)
EVT_COMMAND (wxID_ANY, wxEVT_COMMAND_BUTTON_CLICKED, ProjectTab::onButton)
END_EVENT_TABLE ()

// Class constructor for the tab
IMPLEMENT_DYNAMIC_CLASS(ProjectTab, GRIPTab)
