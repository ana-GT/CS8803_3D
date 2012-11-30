/**
 * @file tabProjectApp.cpp
 * @author Can Erdogan - A. Huaman
 * @date Nov 27, 2012
 * @brief The main application interface for the tab which contains: "IMPLEMENT_APP".
 */

#include "GRIPApp.h"
#include "tabProject.h"
#include <GUI/GUI.h>
#include <GUI/GRIPFrame.h>

extern wxNotebook* tabView;

/**
 * @class ProjectTabApp
 */
class ProjectTabApp : public GRIPApp {

  /// Add the vision tab
  virtual void AddTabs() {
    tabView->AddPage(new ProjectTab(tabView), wxT("3D Project"));
  }
  
  /// Initialize the frame and then load a scene
  virtual bool OnInit() {
    
    // Create the app
    bool success = GRIPApp::OnInit();
    
    // Load the frame
    frame->DoLoad("../scenes/DesktopArmCamera.urdf");
    return true;
  }
};

IMPLEMENT_APP(ProjectTabApp)

