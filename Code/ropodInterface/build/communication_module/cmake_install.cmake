# Install script for directory: /home/pritesh/Desktop/sem2/sdp/sdp_ss2019_P6_IntentionComm/Code/ropodInterface/src/communication_module

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/pritesh/Desktop/sem2/sdp/sdp_ss2019_P6_IntentionComm/Code/ropodInterface/install")
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

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/pritesh/Desktop/sem2/sdp/sdp_ss2019_P6_IntentionComm/Code/ropodInterface/build/communication_module/catkin_generated/installspace/communication_module.pc")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/communication_module/cmake" TYPE FILE FILES
    "/home/pritesh/Desktop/sem2/sdp/sdp_ss2019_P6_IntentionComm/Code/ropodInterface/build/communication_module/catkin_generated/installspace/communication_moduleConfig.cmake"
    "/home/pritesh/Desktop/sem2/sdp/sdp_ss2019_P6_IntentionComm/Code/ropodInterface/build/communication_module/catkin_generated/installspace/communication_moduleConfig-version.cmake"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/communication_module" TYPE FILE FILES "/home/pritesh/Desktop/sem2/sdp/sdp_ss2019_P6_IntentionComm/Code/ropodInterface/src/communication_module/package.xml")
endif()

