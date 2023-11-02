cmake_minimum_required(VERSION 3.5)
project(ros2_bullet3_vis)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
include(FindPkgConfig)
include(modules/CMakeFunctions.cmake)

#set(CMAKE_INSTALL_PREFIX ${PROJECT_SOURCE_DIR})
set(PROJ_INSTALL_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/bin/inc)
set(PROJ_INSTALL_BIN_DIR ${PROJECT_SOURCE_DIR}/bin)
set(PROJ_INSTALL_LIB_DIR ${PROJECT_SOURCE_DIR}/lib)

set(CMAKE_DEBUG_POSTFIX "d")
set(CMAKE_MODULE_PATH  ${CMAKE_CURRENT_SOURCE_DIR}/modules)
set(BUILD_SHARED_LIBS TRUE)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#===============================================================================
# Build type settings
#===============================================================================
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build, options are: Debug | Release | RelWithDebInfo | MinSizeRel" FORCE)
endif()
string(TOUPPER ${CMAKE_BUILD_TYPE} CMAKE_BUILD_TYPE_UPPERCASE)

set(BUILD_TYPE_DEBUG TRUE)
set(BUILD_TYPE_RELEASE TRUE)
set(BUILD_TYPE_RELWITHDEBINFO FALSE)
set(BUILD_TYPE_MINSIZEREL FALSE)

if("${CMAKE_BUILD_TYPE_UPPERCASE}" STREQUAL "DEBUG")
    set(BUILD_TYPE_DEBUG TRUE)
elseif("${CMAKE_BUILD_TYPE_UPPERCASE}" STREQUAL "RELEASE")
    set(BUILD_TYPE_RELEASE TRUE)
elseif("${CMAKE_BUILD_TYPE_UPPERCASE}" STREQUAL "RELWITHDEBINFO")
    set(BUILD_TYPE_RELWITHDEBINFO TRUE)
elseif("${CMAKE_BUILD_TYPE_UPPERCASE}" STREQUAL "MINSIZEREL")
    set(BUILD_TYPE_MINSIZEREL TRUE)
else()
    message(WARNING "CMAKE_BUILD_TYPE ${CMAKE_BUILD_TYPE} unknown. Valid options are: Debug | Release | RelWithDebInfo | MinSizeRel")
endif()

#===============================================================================
# Compiler flags
#===============================================================================
include(CheckCXXCompilerFlag)
check_cxx_compiler_flag(-std=c++17 CXX17_SUPPORT)
if(CXX17_SUPPORT)
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
    set(CMAKE_CXX_EXTENSIONS OFF)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
else()
    check_cxx_compiler_flag(-std=c++14 CXX14_SUPPORT)
    set(CMAKE_CXX_STANDARD 14)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
    set(CMAKE_CXX_EXTENSIONS OFF)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14" )
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -fPIC" )
execute_process(
        COMMAND ${CMAKE_CXX_COMPILER} -dumpfullversion -dumpversion OUTPUT_VARIABLE GCC_VERSION)
if(GCC_VERSION VERSION_LESS 5.3.1)
    message(FATAL_ERROR "The installed g++ version is ${GCC_VERSION}. ${PROJECT_NAME} requires g++ 5.3.1 or greater.")
endif()

set(CMAKE_CXX_FLAGS_RELEASE "-O2 -DNDEBUG")
set(CMAKE_CXX_FLAGS_DEBUG "-g -fno-omit-frame-pointer -fno-inline-functions -fno-inline-functions-called-once -fno-optimize-sibling-calls")
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELEASE} ${CMAKE_CXX_FLAGS_DEBUG}")
set(CMAKE_CXX_FLAGS_PROFILE "${CMAKE_CXX_FLAGS_DEBUG} -pg")
set(CMAKE_SHARED_LINKER_FLAGS "-Wl,--no-undefined")




#QT
find_package(Qt5 COMPONENTS Widgets Core Quick REQUIRED)


set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(QTWIDGET_HEADERS_DIR ${PROJECT_SOURCE_DIR}/include/QtWidgets)
set(DARKSTYLE_HEADERS_DIR ${PROJECT_SOURCE_DIR}/include/DarkStyleWindow)
set(QCUSTOMPLOT_HEADERS_DIR ${PROJECT_SOURCE_DIR}/include/QCustomPlot_library/lib)
set(SUBDIRECTORIES
	#"include/QtWidgets"
	#"include/DarkStyleWindow"
	# "include/QCustomPlot_library"
)
# for문을 사용하여 각 하위 디렉토리를 추가
foreach(SUBDIR ${SUBDIRECTORIES})
    add_subdirectory(${SUBDIR})
endforeach()
set(QT_LIBRARIES
	sub::QtWidgets
	sub::DarkStyleWindow
	Qt5::Widgets 
	Qt5::Core
	Qt5::Gui
	Qt5::Network
	Qt5::Quick
	# qcustomplot
)
SET(QT_SRCS src/mainwindow.cpp src/mainwindow.h src/mainwindow.ui)

INCLUDE_DIRECTORIES( 
		#${QTWIDGET_HEADERS_DIR}
		#${DARKSTYLE_HEADERS_DIR}		
    # ${QCUSTOMPLOT_HEADERS_DIR}
  )

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Threads REQUIRED)

find_package(robot_state_publisher REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rmw_fastrtps_cpp REQUIRED)
find_package(hyu_robot_states REQUIRED)
find_package(hyu_robot_control REQUIRED)
find_package(realtime_tools)
find_package(jsoncpp)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(hyu_visual_info REQUIRED)


set(ament_target_dependencies_list 
    "robot_state_publisher"
    "sensor_msgs"
    "std_msgs"
    "geometry_msgs"
    "nav_msgs"
    "rmw_fastrtps_cpp"
    "hyu_robot_states"
    "OpenCV"
    "cv_bridge"
    "image_transport"
    "realtime_tools"
    "hyu_robot_control"
    "hyu_visual_info"
)
SET(LIEGROUP_ROBOTICS_HEADERS_DIR ${PROJECT_SOURCE_DIR}/include/LieGroupRobotics)
SET(SIM_ROBOT_HEADERS_DIR ${PROJECT_SOURCE_DIR}/include/SimRobot)
set(SUBDIRECTORIES
    "include/SimRobot"
    "include/LieGroupRobotics"
)
# for문을 사용하여 각 하위 디렉토리를 추가
foreach(SUBDIR ${SUBDIRECTORIES})
    add_subdirectory(${SUBDIR})
endforeach()
set(COMMON_LIBRARIES
    sub::SimRobot
	sub::LR
	Threads::Threads
	rt    
)
set(BULLET_PHYSICS_SOURCE_DIR  "/opt/bullet3")
set(BULLET_PHYSICS_LIB  "/opt/bullet3/build_cmake/local_install/lib")

set(BULLET_LIBRARIES 
    BulletRobotics
    BulletFileLoader
    BulletWorldImporter
    BulletSoftBody 
    BulletDynamics 
    BulletCollision 
    BulletInverseDynamicsUtils 
    BulletInverseDynamics 
    LinearMath 
    Bullet3Common
)

SET(RobotSimulator_SRCS_GUI
	${BULLET_PHYSICS_SOURCE_DIR}/examples/RobotSimulator/b3RobotSimulatorClientAPI.cpp
	${BULLET_PHYSICS_SOURCE_DIR}/examples/RobotSimulator/b3RobotSimulatorClientAPI.h	
	${BULLET_PHYSICS_SOURCE_DIR}/examples/RobotSimulator/MinitaurSetup.cpp
	${BULLET_PHYSICS_SOURCE_DIR}/examples/RobotSimulator/MinitaurSetup.h
	${BULLET_PHYSICS_SOURCE_DIR}/examples/RobotSimulator/../../examples/ExampleBrowser/InProcessExampleBrowser.cpp
	${BULLET_PHYSICS_SOURCE_DIR}/examples/RobotSimulator/../SharedMemory/GraphicsServerExample.cpp
	${BULLET_PHYSICS_SOURCE_DIR}/examples/RobotSimulator/../SharedMemory/GraphicsClientExample.cpp
	${BULLET_PHYSICS_SOURCE_DIR}/examples/RobotSimulator/../SharedMemory/RemoteGUIHelper.cpp
#	${BULLET_PHYSICS_SOURCE_DIR}/examples/RobotSimulator/../SharedMemory/RemoteGUIHelperTCP.cpp
	${BULLET_PHYSICS_SOURCE_DIR}/examples/RobotSimulator/../SharedMemory/GraphicsServerExample.h
	${BULLET_PHYSICS_SOURCE_DIR}/examples/RobotSimulator/../SharedMemory/GraphicsClientExample.h
	${BULLET_PHYSICS_SOURCE_DIR}/examples/RobotSimulator/../SharedMemory/RemoteGUIHelper.h
	${BULLET_PHYSICS_SOURCE_DIR}/examples/RobotSimulator/../SharedMemory/GraphicsSharedMemoryCommands.h
	${BULLET_PHYSICS_SOURCE_DIR}/examples/RobotSimulator/../SharedMemory/GraphicsSharedMemoryPublic.h
	${BULLET_PHYSICS_SOURCE_DIR}/examples/RobotSimulator/../../examples/SharedMemory/PhysicsServerExample.cpp
	${BULLET_PHYSICS_SOURCE_DIR}/examples/RobotSimulator/../../examples/SharedMemory/PhysicsServerExampleBullet2.cpp
	${BULLET_PHYSICS_SOURCE_DIR}/examples/RobotSimulator/../../examples/SharedMemory/SharedMemoryInProcessPhysicsC_API.cpp		
)	
add_executable(ros2_bullet3_vis src/ros2_bullet3_vis.cpp ${RobotSimulator_SRCS_GUI})
target_include_directories(ros2_bullet3_vis PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PROJ_INSTALL_INCLUDE_DIR}
  ${KERNELHEADERS_DIR}
  ${BULLET_PHYSICS_SOURCE_DIR}/src
  ${BULLET_PHYSICS_SOURCE_DIR}/examples
  ${BULLET_PHYSICS_SOURCE_DIR}/examples/RobotSimulator		
  ${BULLET_PHYSICS_SOURCE_DIR}/examples/SharedMemory
  ${BULLET_PHYSICS_SOURCE_DIR}/examples/ThirdPartyLibs
  ${BULLET_PHYSICS_SOURCE_DIR}/examples/ThirdPartyLibs/enet/include
  ${BULLET_PHYSICS_SOURCE_DIR}/examples/ThirdPartyLibs/clsocket/src
  ${SIM_ROBOT_HEADERS_DIR}
  ${LIEGROUP_ROBOTICS_HEADERS_DIR}    
  ${QTWIDGET_HEADERS_DIR}
  ${DARKSTYLE_HEADERS_DIR}		
    # ${QCUSTOMPLOT_HEADERS_DIR}  
    ${OpenCV_INCLUDE_DIRS}
  )
  target_link_libraries(${PROJECT_NAME} jsoncpp_lib)
  target_link_libraries(${PROJECT_NAME}
    ${BULLET_LIBRARIES} ${COMMON_LIBRARIES}   ${OpenCV_LIBRARIES}
  )  
  target_compile_definitions(${PROJECT_NAME} PUBLIC -DBT_USE_DOUBLE_PRECISION)
  # target_compile_definitions(${PROJECT_NAME} PRIVATE QCUSTOMPLOT_USE_LIBRARY)

install(TARGETS ros2_bullet3_vis  DESTINATION lib/${PROJECT_NAME})
# install(TARGETS qcustomplot
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin
# )
ament_target_dependencies(${PROJECT_NAME} ${ament_target_dependencies_list})
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()
# xenomai
# find_package(Xenomai 3.1.1 REQUIRED NATIVE)
# message(STATUS "Xenomai found with theses skins: ${Xenomai_SKINS}")
# if(Xenomai_FOUND)
#     target_include_directories(${PROJECT_NAME} PUBLIC ${KERNELHEADERS_DIR})
#     foreach(comp_lists ${Xenomai_FOUND_COMP})
#         target_include_directories(${PROJECT_NAME} PUBLIC ${Xenomai_${comp_lists}_INCLUDE_DIRS})
#         target_link_libraries(${PROJECT_NAME} ${Xenomai_${comp_lists}_DL_FLAGS})
#         set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS ${Xenomai_${comp_lists}_DEFINITIONS})
#     endforeach(comp_lists ${Xenomai_SKINS})
# endif(Xenomai_FOUND)


ament_package()
