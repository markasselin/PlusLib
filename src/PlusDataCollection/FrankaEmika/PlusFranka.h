/*=Plus=header=begin======================================================
Program: Plus
Copyright (c) Laboratory for Percutaneous Surgery. All rights reserved.
See License.txt for details.
=========================================================Plus=header=end*/

#ifndef __PlusFranka_h
#define __PlusFranka_h

// Franka includes
#include <franka/gripper_state.h>
#include <franka/robot.h>
#include <franka/robot_state.h>

// STL includes
#include <string>

//-----------------------------------------------------------------------------
// ROBOT
std::string RobotStateToString(franka::RobotState rs);

//-----------------------------------------------------------------------------
// GRIPPER
std::string GripperStateToString(franka::GripperState gs);



#endif
