cmake_minimum_required(VERSION 2.8.3)
project(rose_gui_application)

set(CMAKE_CXX_FLAGS "-std=c++0x -g")

# Find all packages
find_package(catkin REQUIRED COMPONENTS 
	actionlib
	actionlib_msgs
	camera_info_manager
	cv_bridge
	image_transport
	qt_build
	roscomm
	roscpp
	rose_common
	rose_joystick
	rose_ui_item_selector
	rose_ui_map_display
	rose_ui_script_selector
	rose_shared_variables
) 

find_package(roscomm REQUIRED)
find_package(LibXml2 REQUIRED)
find_package(image_transport)
find_package(Qt4 REQUIRED 
	COMPONENTS QtCore QtGui)

catkin_package(
    INCLUDE_DIRS 
   		include
	CATKIN_DEPENDS 
		actionlib
		actionlib_msgs
		camera_info_manager
		cv_bridge
		image_transport
		roscomm
		roscpp
		rose_common
		rose_joystick
		rose_ui_item_selector
		rose_ui_map_display
		rose_ui_script_selector
		rose_shared_variables
	DEPENDS 
		qt4
		qt_build
		tinyxml
	)

#################
## Usage of Qt ##
#################
rosbuild_prepare_qt4(QtCore QtGui)

include(${QT_USE_FILE})
include_directories(${CMAKE_CURRENT_BINARY_DIR})
include_directories(${catkin_INCLUDE_DIRS})

file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} forms/*.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} forms/*.qrc)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS 
	forms/*.h 
	include/*.h 
	include/*.hpp
)

QT4_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
QT4_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT4_WRAP_CPP(QT_MOC_HPP ${QT_MOC})

file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS *.cpp)

add_executable(rose_gui_application_node ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})

add_dependencies( rose_gui_application_node ${catkin_EXPORTED_TARGETS})

target_link_libraries(rose_gui_application_node 
	${QT_LIBRARIES} 
	${ROS_LIBRARIES} 
	${catkin_LIBRARIES}
)

target_link_libraries(rose_gui_application_node 
	sqlite3 
	xml2 
	asound
	avcodec 
	swscale
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
	include
	forms 
	forms/blocks 
	extra
)

include_directories(
	${LIBXML2_INCLUDE_DIR}
	${Boost_INCLUDE_DIRS}
	${usb_cam_INCLUDE_DIRS}
)

#############
## Install ##
#############

install(TARGETS rose_gui_application_node
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
