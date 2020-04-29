/*=Plus=header=begin======================================================
Program: Plus
Copyright (c) Laboratory for Percutaneous Surgery. All rights reserved.
See License.txt for details.
=========================================================Plus=header=end*/

#ifndef __vtkPlusFrankaRobot_h
#define __vtkPlusFrankaRobot_h

// Local includes
#include "vtkPlusDataCollectionExport.h"
#include "vtkPlusDevice.h"

// STL includes
#include <string>

/*!
\class vtkPlusFrankaRobot
\brief Interface to the Franka Panda robot
This class talks with a Frank Panda Robot using PlusFranka wrapper
around libfranka.
Requires PLUS_USE_FRANKA_PANDA option in CMake.
\ingroup PlusLibDataCollection
*/
class vtkPlusDataCollectionExport vtkPlusFrankaRobot : public vtkPlusDevice
{
public:
  static vtkPlusFrankaRobot *New();
  vtkTypeMacro(vtkPlusFrankaRobot, vtkPlusDevice);
  void PrintSelf(ostream& os, vtkIndent indent);

  /* Device is a hardware tracker. */
  virtual bool IsTracker() const { return true; }
  virtual bool IsVirtual() const { return false; }

  /*! Read configuration from xml data */
  virtual PlusStatus ReadConfiguration(vtkXMLDataElement* config);

  /*! Write configuration to xml data */
  virtual PlusStatus WriteConfiguration(vtkXMLDataElement* config);

  /*! Connect to device */
  PlusStatus InternalConnect();

  /*! Disconnect from device */
  virtual PlusStatus InternalDisconnect();

  /*! Probe to see if the tracking system is present. */
  PlusStatus Probe();

  /*!  */
  PlusStatus InternalUpdate();

public:
  // COMMANDS
  // robot
  static const char* FRANKA_ROBOT_READ_STATE;
  static const char* FRANKA_ROBOT_ERROR_RECOVERY;
  static const char* FRANKA_SET_VIRTUAL_WALL;
  static const char* FRANKA_MOVE_CARTESIAN_POSE;
  static const char* FRANKA_MOVE_CARTESIAN_VELOCITIES;
  // gripper
  static const char* FRANKA_GRIPPER_READ_STATE;
  static const char* FRANKA_GRIPPER_HOME;
  static const char* FRANKA_GRIPPER_GRASP;
  static const char* FRANKA_GRIPPER_MOVE;
  static const char* FRANKA_GRIPPER_STOP;

  // COMMAND METHODS
  /*! */
  PlusStatus ReadRobotState();

  /*! */
  PlusStatus SetVirtualWall();

  /*! */
  PlusStatus MoveCartesianPose();

  /*! */
  PlusStatus MoveCartesianVelocities();

  /*! */
  PlusStatus ReadGripperState();

  /*! */
  PlusStatus GripperHome();

  /*! */
  PlusStatus GripperGrasp();

  /*! */
  PlusStatus GripperMove();

  /*! */
  PlusStatus GripperStop();

protected:
  vtkPlusFrankaRobot();
  ~vtkPlusFrankaRobot();

private: // Functions
  vtkPlusFrankaRobot(const vtkPlusFrankaRobot&);
  void operator=(const vtkPlusFrankaRobot&);

  /*! Start the tracking system. */
  PlusStatus InternalStartRecording();

  /*! Stop the tracking system and bring it back to its initial state. */
  PlusStatus InternalStopRecording();

  std::vector<std::string> DisabledToolIds;

  class vtkInternal;
  vtkInternal* Internal;
};

#endif
