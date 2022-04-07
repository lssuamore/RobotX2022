# Install script for directory: /home/taylor/RobotX2022/src/sensors/ublox/ublox_msgs

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/taylor/RobotX2022/install")
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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ublox_msgs/msg" TYPE FILE FILES
    "/home/taylor/RobotX2022/src/sensors/ublox/ublox_msgs/msg/Ack.msg"
    "/home/taylor/RobotX2022/src/sensors/ublox/ublox_msgs/msg/AidALM.msg"
    "/home/taylor/RobotX2022/src/sensors/ublox/ublox_msgs/msg/AidEPH.msg"
    "/home/taylor/RobotX2022/src/sensors/ublox/ublox_msgs/msg/AidHUI.msg"
    "/home/taylor/RobotX2022/src/sensors/ublox/ublox_msgs/msg/CfgANT.msg"
    "/home/taylor/RobotX2022/src/sensors/ublox/ublox_msgs/msg/CfgCFG.msg"
    "/home/taylor/RobotX2022/src/sensors/ublox/ublox_msgs/msg/CfgDAT.msg"
    "/home/taylor/RobotX2022/src/sensors/ublox/ublox_msgs/msg/CfgDGNSS.msg"
    "/home/taylor/RobotX2022/src/sensors/ublox/ublox_msgs/msg/CfgGNSS.msg"
    "/home/taylor/RobotX2022/src/sensors/ublox/ublox_msgs/msg/CfgGNSS_Block.msg"
    "/home/taylor/RobotX2022/src/sensors/ublox/ublox_msgs/msg/CfgHNR.msg"
    "/home/taylor/RobotX2022/src/sensors/ublox/ublox_msgs/msg/CfgINF.msg"
    "/home/taylor/RobotX2022/src/sensors/ublox/ublox_msgs/msg/CfgINF_Block.msg"
    "/home/taylor/RobotX2022/src/sensors/ublox/ublox_msgs/msg/CfgMSG.msg"
    "/home/taylor/RobotX2022/src/sensors/ublox/ublox_msgs/msg/CfgNAV5.msg"
    "/home/taylor/RobotX2022/src/sensors/ublox/ublox_msgs/msg/CfgNAVX5.msg"
    "/home/taylor/RobotX2022/src/sensors/ublox/ublox_msgs/msg/CfgNMEA.msg"
    "/home/taylor/RobotX2022/src/sensors/ublox/ublox_msgs/msg/CfgNMEA6.msg"
    "/home/taylor/RobotX2022/src/sensors/ublox/ublox_msgs/msg/CfgNMEA7.msg"
    "/home/taylor/RobotX2022/src/sensors/ublox/ublox_msgs/msg/CfgPRT.msg"
    "/home/taylor/RobotX2022/src/sensors/ublox/ublox_msgs/msg/CfgRATE.msg"
    "/home/taylor/RobotX2022/src/sensors/ublox/ublox_msgs/msg/CfgRST.msg"
    "/home/taylor/RobotX2022/src/sensors/ublox/ublox_msgs/msg/CfgSBAS.msg"
    "/home/taylor/RobotX2022/src/sensors/ublox/ublox_msgs/msg/CfgTMODE3.msg"
    "/home/taylor/RobotX2022/src/sensors/ublox/ublox_msgs/msg/CfgUSB.msg"
    "/home/taylor/RobotX2022/src/sensors/ublox/ublox_msgs/msg/EsfINS.msg"
    "/home/taylor/RobotX2022/src/sensors/ublox/ublox_msgs/msg/EsfMEAS.msg"
    "/home/taylor/RobotX2022/src/sensors/ublox/ublox_msgs/msg/EsfRAW.msg"
    "/home/taylor/RobotX2022/src/sensors/ublox/ublox_msgs/msg/EsfRAW_Block.msg"
    "/home/taylor/RobotX2022/src/sensors/ublox/ublox_msgs/msg/EsfSTATUS.msg"
    "/home/taylor/RobotX2022/src/sensors/ublox/ublox_msgs/msg/EsfSTATUS_Sens.msg"
    "/home/taylor/RobotX2022/src/sensors/ublox/ublox_msgs/msg/HnrPVT.msg"
    "/home/taylor/RobotX2022/src/sensors/ublox/ublox_msgs/msg/Inf.msg"
    "/home/taylor/RobotX2022/src/sensors/ublox/ublox_msgs/msg/MgaGAL.msg"
    "/home/taylor/RobotX2022/src/sensors/ublox/ublox_msgs/msg/MonGNSS.msg"
    "/home/taylor/RobotX2022/src/sensors/ublox/ublox_msgs/msg/MonHW.msg"
    "/home/taylor/RobotX2022/src/sensors/ublox/ublox_msgs/msg/MonHW6.msg"
    "/home/taylor/RobotX2022/src/sensors/ublox/ublox_msgs/msg/MonVER.msg"
    "/home/taylor/RobotX2022/src/sensors/ublox/ublox_msgs/msg/MonVER_Extension.msg"
    "/home/taylor/RobotX2022/src/sensors/ublox/ublox_msgs/msg/NavATT.msg"
    "/home/taylor/RobotX2022/src/sensors/ublox/ublox_msgs/msg/NavCLOCK.msg"
    "/home/taylor/RobotX2022/src/sensors/ublox/ublox_msgs/msg/NavDGPS.msg"
    "/home/taylor/RobotX2022/src/sensors/ublox/ublox_msgs/msg/NavDGPS_SV.msg"
    "/home/taylor/RobotX2022/src/sensors/ublox/ublox_msgs/msg/NavDOP.msg"
    "/home/taylor/RobotX2022/src/sensors/ublox/ublox_msgs/msg/NavHPPOSECEF.msg"
    "/home/taylor/RobotX2022/src/sensors/ublox/ublox_msgs/msg/NavHPPOSLLH.msg"
    "/home/taylor/RobotX2022/src/sensors/ublox/ublox_msgs/msg/NavPOSECEF.msg"
    "/home/taylor/RobotX2022/src/sensors/ublox/ublox_msgs/msg/NavPOSLLH.msg"
    "/home/taylor/RobotX2022/src/sensors/ublox/ublox_msgs/msg/NavPVT.msg"
    "/home/taylor/RobotX2022/src/sensors/ublox/ublox_msgs/msg/NavPVT7.msg"
    "/home/taylor/RobotX2022/src/sensors/ublox/ublox_msgs/msg/NavRELPOSNED.msg"
    "/home/taylor/RobotX2022/src/sensors/ublox/ublox_msgs/msg/NavRELPOSNED9.msg"
    "/home/taylor/RobotX2022/src/sensors/ublox/ublox_msgs/msg/NavSAT.msg"
    "/home/taylor/RobotX2022/src/sensors/ublox/ublox_msgs/msg/NavSAT_SV.msg"
    "/home/taylor/RobotX2022/src/sensors/ublox/ublox_msgs/msg/NavSBAS.msg"
    "/home/taylor/RobotX2022/src/sensors/ublox/ublox_msgs/msg/NavSBAS_SV.msg"
    "/home/taylor/RobotX2022/src/sensors/ublox/ublox_msgs/msg/NavSOL.msg"
    "/home/taylor/RobotX2022/src/sensors/ublox/ublox_msgs/msg/NavSTATUS.msg"
    "/home/taylor/RobotX2022/src/sensors/ublox/ublox_msgs/msg/NavSVIN.msg"
    "/home/taylor/RobotX2022/src/sensors/ublox/ublox_msgs/msg/NavSVINFO.msg"
    "/home/taylor/RobotX2022/src/sensors/ublox/ublox_msgs/msg/NavSVINFO_SV.msg"
    "/home/taylor/RobotX2022/src/sensors/ublox/ublox_msgs/msg/NavTIMEGPS.msg"
    "/home/taylor/RobotX2022/src/sensors/ublox/ublox_msgs/msg/NavTIMEUTC.msg"
    "/home/taylor/RobotX2022/src/sensors/ublox/ublox_msgs/msg/NavVELECEF.msg"
    "/home/taylor/RobotX2022/src/sensors/ublox/ublox_msgs/msg/NavVELNED.msg"
    "/home/taylor/RobotX2022/src/sensors/ublox/ublox_msgs/msg/RxmALM.msg"
    "/home/taylor/RobotX2022/src/sensors/ublox/ublox_msgs/msg/RxmEPH.msg"
    "/home/taylor/RobotX2022/src/sensors/ublox/ublox_msgs/msg/RxmRAW.msg"
    "/home/taylor/RobotX2022/src/sensors/ublox/ublox_msgs/msg/RxmRAWX.msg"
    "/home/taylor/RobotX2022/src/sensors/ublox/ublox_msgs/msg/RxmRAWX_Meas.msg"
    "/home/taylor/RobotX2022/src/sensors/ublox/ublox_msgs/msg/RxmRAW_SV.msg"
    "/home/taylor/RobotX2022/src/sensors/ublox/ublox_msgs/msg/RxmRTCM.msg"
    "/home/taylor/RobotX2022/src/sensors/ublox/ublox_msgs/msg/RxmSFRB.msg"
    "/home/taylor/RobotX2022/src/sensors/ublox/ublox_msgs/msg/RxmSFRBX.msg"
    "/home/taylor/RobotX2022/src/sensors/ublox/ublox_msgs/msg/RxmSVSI.msg"
    "/home/taylor/RobotX2022/src/sensors/ublox/ublox_msgs/msg/RxmSVSI_SV.msg"
    "/home/taylor/RobotX2022/src/sensors/ublox/ublox_msgs/msg/TimTM2.msg"
    "/home/taylor/RobotX2022/src/sensors/ublox/ublox_msgs/msg/UpdSOS.msg"
    "/home/taylor/RobotX2022/src/sensors/ublox/ublox_msgs/msg/UpdSOS_Ack.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ublox_msgs/cmake" TYPE FILE FILES "/home/taylor/RobotX2022/build/sensors/ublox/ublox_msgs/catkin_generated/installspace/ublox_msgs-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/taylor/RobotX2022/devel/include/ublox_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/taylor/RobotX2022/devel/share/roseus/ros/ublox_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/taylor/RobotX2022/devel/share/common-lisp/ros/ublox_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/taylor/RobotX2022/devel/share/gennodejs/ros/ublox_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python3" -m compileall "/home/taylor/RobotX2022/devel/lib/python3/dist-packages/ublox_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/home/taylor/RobotX2022/devel/lib/python3/dist-packages/ublox_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/taylor/RobotX2022/build/sensors/ublox/ublox_msgs/catkin_generated/installspace/ublox_msgs.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ublox_msgs/cmake" TYPE FILE FILES "/home/taylor/RobotX2022/build/sensors/ublox/ublox_msgs/catkin_generated/installspace/ublox_msgs-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ublox_msgs/cmake" TYPE FILE FILES
    "/home/taylor/RobotX2022/build/sensors/ublox/ublox_msgs/catkin_generated/installspace/ublox_msgsConfig.cmake"
    "/home/taylor/RobotX2022/build/sensors/ublox/ublox_msgs/catkin_generated/installspace/ublox_msgsConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ublox_msgs" TYPE FILE FILES "/home/taylor/RobotX2022/src/sensors/ublox/ublox_msgs/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libublox_msgs.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libublox_msgs.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libublox_msgs.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/taylor/RobotX2022/devel/lib/libublox_msgs.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libublox_msgs.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libublox_msgs.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libublox_msgs.so"
         OLD_RPATH "/opt/ros/noetic/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libublox_msgs.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/taylor/RobotX2022/src/sensors/ublox/ublox_msgs/include/" REGEX "/\\.svn$" EXCLUDE)
endif()

