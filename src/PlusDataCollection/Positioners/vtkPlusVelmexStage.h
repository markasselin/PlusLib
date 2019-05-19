/*=Plus=header=begin======================================================
Program: Plus
Copyright (c) Laboratory for Percutaneous Surgery. All rights reserved.
See License.txt for details.
=========================================================Plus=header=end*/

#ifndef __vtkPlusVelmexStage_h
#define __vtkPlusVelmexStage_h

#include "vtkPlusDataCollectionExport.h"
#include "vtkPlusPositioningDevice.h"

/*!
\class vtkPlusVelmexStage
\brief Interface to a Velmex position stage
with SENC-150 encoders
\ingroup PlusLibDataCollection
*/
class vtkPlusDataCollectionExport vtkPlusVelmexStage : public vtkPlusPositioningDevice
{
public:
  static vtkPlusVelmexStage* New();
  vtkTypeMacro(vtkPlusVelmexStage, vtkPlusPositioningDevice);
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

  /*! Start the tracking system. */
  PlusStatus InternalStartRecording();

  /*! Stop the tracking system and bring it back to its initial state. */
  PlusStatus InternalStopRecording();

  /*! Probe to see if the tracking system is present. */
  PlusStatus Probe();

  /*! Perform update */
  PlusStatus InternalUpdate();

protected:
  vtkPlusVelmexStage();
  ~vtkPlusVelmexStage();

  // movement methods
  virtual PlusStatus HomeAllAxes();
  virtual PlusStatus PauseMovement();
  virtual PlusStatus ResumeMovement();
  virtual PlusStatus StopMovement();
  virtual PlusStatus MoveToAbsolute(const vtkMatrix4x4& position);
  virtual PlusStatus MoveByRelative(const vtkMatrix4x4& position);

  // position methods
  virtual PlusStatus GetCurrentPosition(vtkMatrix4x4* position);

private:
  vtkPlusVelmexStage(const vtkPlusVelmexStage&); // Not implemented
  void operator=(const vtkPlusVelmexStage&); // Not implemented

  class vtkInternal;
  vtkInternal* Internal;

};

#endif
