# Install script for directory: /home/xusj/Documents/0LAB/23ChangliuLiu/Hierarchical-LTL/hierarchial_simulation/src/stmotion_controller

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/xusj/Documents/0LAB/23ChangliuLiu/Hierarchical-LTL/hierarchial_simulation/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
        file(MAKE_DIRECTORY "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
      endif()
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin")
        file(WRITE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin" "")
      endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/xusj/Documents/0LAB/23ChangliuLiu/Hierarchical-LTL/hierarchial_simulation/install/_setup_util.py")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/xusj/Documents/0LAB/23ChangliuLiu/Hierarchical-LTL/hierarchial_simulation/install" TYPE PROGRAM FILES "/home/xusj/Documents/0LAB/23ChangliuLiu/Hierarchical-LTL/hierarchial_simulation/build/stmotion_controller/catkin_generated/installspace/_setup_util.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/xusj/Documents/0LAB/23ChangliuLiu/Hierarchical-LTL/hierarchial_simulation/install/env.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/xusj/Documents/0LAB/23ChangliuLiu/Hierarchical-LTL/hierarchial_simulation/install" TYPE PROGRAM FILES "/home/xusj/Documents/0LAB/23ChangliuLiu/Hierarchical-LTL/hierarchial_simulation/build/stmotion_controller/catkin_generated/installspace/env.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/xusj/Documents/0LAB/23ChangliuLiu/Hierarchical-LTL/hierarchial_simulation/install/setup.bash;/home/xusj/Documents/0LAB/23ChangliuLiu/Hierarchical-LTL/hierarchial_simulation/install/local_setup.bash")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/xusj/Documents/0LAB/23ChangliuLiu/Hierarchical-LTL/hierarchial_simulation/install" TYPE FILE FILES
    "/home/xusj/Documents/0LAB/23ChangliuLiu/Hierarchical-LTL/hierarchial_simulation/build/stmotion_controller/catkin_generated/installspace/setup.bash"
    "/home/xusj/Documents/0LAB/23ChangliuLiu/Hierarchical-LTL/hierarchial_simulation/build/stmotion_controller/catkin_generated/installspace/local_setup.bash"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/xusj/Documents/0LAB/23ChangliuLiu/Hierarchical-LTL/hierarchial_simulation/install/setup.sh;/home/xusj/Documents/0LAB/23ChangliuLiu/Hierarchical-LTL/hierarchial_simulation/install/local_setup.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/xusj/Documents/0LAB/23ChangliuLiu/Hierarchical-LTL/hierarchial_simulation/install" TYPE FILE FILES
    "/home/xusj/Documents/0LAB/23ChangliuLiu/Hierarchical-LTL/hierarchial_simulation/build/stmotion_controller/catkin_generated/installspace/setup.sh"
    "/home/xusj/Documents/0LAB/23ChangliuLiu/Hierarchical-LTL/hierarchial_simulation/build/stmotion_controller/catkin_generated/installspace/local_setup.sh"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/xusj/Documents/0LAB/23ChangliuLiu/Hierarchical-LTL/hierarchial_simulation/install/setup.zsh;/home/xusj/Documents/0LAB/23ChangliuLiu/Hierarchical-LTL/hierarchial_simulation/install/local_setup.zsh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/xusj/Documents/0LAB/23ChangliuLiu/Hierarchical-LTL/hierarchial_simulation/install" TYPE FILE FILES
    "/home/xusj/Documents/0LAB/23ChangliuLiu/Hierarchical-LTL/hierarchial_simulation/build/stmotion_controller/catkin_generated/installspace/setup.zsh"
    "/home/xusj/Documents/0LAB/23ChangliuLiu/Hierarchical-LTL/hierarchial_simulation/build/stmotion_controller/catkin_generated/installspace/local_setup.zsh"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/xusj/Documents/0LAB/23ChangliuLiu/Hierarchical-LTL/hierarchial_simulation/install/.rosinstall")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/xusj/Documents/0LAB/23ChangliuLiu/Hierarchical-LTL/hierarchial_simulation/install" TYPE FILE FILES "/home/xusj/Documents/0LAB/23ChangliuLiu/Hierarchical-LTL/hierarchial_simulation/build/stmotion_controller/catkin_generated/installspace/.rosinstall")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/stmotion_controller/srv" TYPE FILE FILES
    "/home/xusj/Documents/0LAB/23ChangliuLiu/Hierarchical-LTL/hierarchial_simulation/src/stmotion_controller/srv/robot_action.srv"
    "/home/xusj/Documents/0LAB/23ChangliuLiu/Hierarchical-LTL/hierarchial_simulation/src/stmotion_controller/srv/lego_pickup.srv"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/stmotion_controller/cmake" TYPE FILE FILES "/home/xusj/Documents/0LAB/23ChangliuLiu/Hierarchical-LTL/hierarchial_simulation/build/stmotion_controller/catkin_generated/installspace/stmotion_controller-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/xusj/Documents/0LAB/23ChangliuLiu/Hierarchical-LTL/hierarchial_simulation/devel/.private/stmotion_controller/include/stmotion_controller")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/xusj/Documents/0LAB/23ChangliuLiu/Hierarchical-LTL/hierarchial_simulation/devel/.private/stmotion_controller/share/roseus/ros/stmotion_controller")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/xusj/Documents/0LAB/23ChangliuLiu/Hierarchical-LTL/hierarchial_simulation/devel/.private/stmotion_controller/share/common-lisp/ros/stmotion_controller")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/xusj/Documents/0LAB/23ChangliuLiu/Hierarchical-LTL/hierarchial_simulation/devel/.private/stmotion_controller/share/gennodejs/ros/stmotion_controller")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/local/miniconda3/bin/python3" -m compileall "/home/xusj/Documents/0LAB/23ChangliuLiu/Hierarchical-LTL/hierarchial_simulation/devel/.private/stmotion_controller/lib/python3/dist-packages/stmotion_controller")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/home/xusj/Documents/0LAB/23ChangliuLiu/Hierarchical-LTL/hierarchial_simulation/devel/.private/stmotion_controller/lib/python3/dist-packages/stmotion_controller")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/xusj/Documents/0LAB/23ChangliuLiu/Hierarchical-LTL/hierarchial_simulation/build/stmotion_controller/catkin_generated/installspace/stmotion_controller.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/stmotion_controller/cmake" TYPE FILE FILES "/home/xusj/Documents/0LAB/23ChangliuLiu/Hierarchical-LTL/hierarchial_simulation/build/stmotion_controller/catkin_generated/installspace/stmotion_controller-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/stmotion_controller/cmake" TYPE FILE FILES
    "/home/xusj/Documents/0LAB/23ChangliuLiu/Hierarchical-LTL/hierarchial_simulation/build/stmotion_controller/catkin_generated/installspace/stmotion_controllerConfig.cmake"
    "/home/xusj/Documents/0LAB/23ChangliuLiu/Hierarchical-LTL/hierarchial_simulation/build/stmotion_controller/catkin_generated/installspace/stmotion_controllerConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/stmotion_controller" TYPE FILE FILES "/home/xusj/Documents/0LAB/23ChangliuLiu/Hierarchical-LTL/hierarchial_simulation/src/stmotion_controller/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/stmotion_controller" TYPE PROGRAM FILES "/home/xusj/Documents/0LAB/23ChangliuLiu/Hierarchical-LTL/hierarchial_simulation/build/stmotion_controller/catkin_generated/installspace/lego_state_refresher.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/stmotion_controller" TYPE DIRECTORY FILES "/home/xusj/Documents/0LAB/23ChangliuLiu/Hierarchical-LTL/hierarchial_simulation/src/stmotion_controller/launch")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/xusj/Documents/0LAB/23ChangliuLiu/Hierarchical-LTL/hierarchial_simulation/build/stmotion_controller/gtest/cmake_install.cmake")
  include("/home/xusj/Documents/0LAB/23ChangliuLiu/Hierarchical-LTL/hierarchial_simulation/build/stmotion_controller/3rd-party/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/xusj/Documents/0LAB/23ChangliuLiu/Hierarchical-LTL/hierarchial_simulation/build/stmotion_controller/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
