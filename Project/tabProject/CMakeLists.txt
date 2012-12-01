cmake_minimum_required (VERSION 2.6)
project (tabProject)

message (STATUS "Creating Makefile for the tabProject")

if(WIN32)
set (CMAKE_INSTALL_PREFIX "C:/Golems" CACHE PATH "Install prefix" FORCE)
    message (STATUS "Setup Visual Studio Specific Flags")
set (CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /MDd /Zi /GL /Gy /W1 /EHsc")
set (CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /MD /Zi /GL /Gy /W1 /EHsc")
set (CMAKE_DEBUG_POSTFIX "d")
set (CMAKE_EXE_LINKER_FLAGS "/LTCG")
set (CMAKE_MODULE_LINKER_FLAGS "/LTCG")
else ()
   set (CMAKE_INSTALL_PREFIX /usr/local )
   set (CMAKE_CXX_FLAGS "-g -O0 -Wall -Wextra -Wpointer-arith -fno-default-inline -fno-implicit-inline-templates -DwxUSE_THREADS -std=c++0x -fpermissive")
endif()

include (${CMAKE_INSTALL_PREFIX}/share/dart/DARTConfig.cmake)
include (${CMAKE_INSTALL_PREFIX}/share/grip/GRIP_config.cmake)

find_package( PCL 1.6 REQUIRED )
LINK_DIRECTORIES( ${DART_LIBRARY_DIRS} /usr/lib )
INCLUDE_DIRECTORIES( ${DART_INCLUDE_DIRS}  ${PCL_INCLUDE_DIRS} )

# Search all header and source files
set ( srcs_files tabProject.cpp tabProjectApp.cpp builder.cpp PCL_Tools/PCL_Tools.cpp )
set ( hdrs_files tabProject.h builder.h PCL_Tools/PCL_Tools.h )

# Set some extra libraries
set( myPCL_Libs 
/usr/lib/libboost_system-mt.so
/usr/lib/libboost_thread-mt.so
/usr/lib/libboost_date_time-mt.so
/usr/lib/libboost_iostreams-mt.so
pcl_common
pcl_io
pcl_features
pcl_filters
pcl_geometry
pcl_kdtree
pcl_visualization
pcl_search
pcl_surface
vtkHybrid
vtkIO
vtkViews
vtkCommon
vtkRendering
vtkVolumeRendering
pcl_io_ply
OpenNI
pcl_apps )


if(WIN32)
  set (wxWidgets_EXCLUDE_COMMON_LIBRARIES TRUE)
  set (wxWidgets_USE_LIBS base core gl png tiff jpeg zlib)
else(WIN32)
  set (wxWidgets_USE_LIBS base core gl)
endif(WIN32)
find_package (wxWidgets)
if (wxWidgets_FOUND)

  include (${wxWidgets_USE_FILE})
  include_directories(${DART_INCLUDE_DIRS} ${GRIP_HEADERS} ${WXWIDGETS_SHARED_INCLUDE_DIRS})
  link_directories(${CMAKE_SOURCE_DIR}/lib ${GRIP_LIBDIR} ${WXWIDGETS_SHARED_LIBRARY_DIRS})

 if(MSVC)
  add_executable (tabProject WIN32 ${hdrs_files} ${srcs_files} )
set_target_properties (tabProject PROPERTIES LINK_FLAGS "/SUBSYSTEM:WINDOWS")
   else()
  add_executable (tabProject ${hdrs_files} ${srcs_files} )
endif()
  target_link_libraries (tabProject ${GRIP_LIBRARIES} ggui ${DART_LIBRARIES} ${wxWidgets_LIBRARIES} ${OPENGL_LIBRARIES} ${GLUT_glut_LIBRARY} ${GLU_LIBRARIES} ${myPCL_Libs})

else (wxWidgets_FOUND)
  message ("wxWidgets not found!")
endif (wxWidgets_FOUND)
