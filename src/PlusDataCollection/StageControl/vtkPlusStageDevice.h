/*=Plus=header=begin======================================================
Program: Plus
Copyright (c) Laboratory for Percutaneous Surgery. All rights reserved.
See License.txt for details.
=========================================================Plus=header=end*/

#ifndef __vtkPlusStageDevice_h
#define __vtkPlusStageDevice_h

#include "vtkPlusDataCollectionExport.h"
#include "vtkPlusDevice.h"

/*!
\class vtkPlusStageDevice
\brief Abstract class for handling positioning stage devices
\ingroup PlusLibDataCollection
*/
class vtkPlusDataCollectionExport vtkPlusStageDevice : public vtkPlusDevice
{
public:
  static vtkPlusStageDevice *New();
  vtkTypeMacro(vtkPlusStageDevice, vtkPlusDevice);
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
  vtkPlusStageDevice();
  ~vtkPlusStageDevice();

  // movement methods
  virtual PlusStatus HomeAllAxes();
  virtual PlusStatus PauseMovement();
  virtual PlusStatus ResumeMovement();
  virtual PlusStatus StopMovement();
  virtual PlusStatus MoveToAbsolute(const vtkMatrix4x4& position);
  virtual PlusStatus MoveByRelative(const vtkMatrix4x4& position);

  // position methods
  virtual PlusStatus GetCurrentPosition(vtkMatrix4x4* position);

  // getters and setters for boundaries
  PlusStatus GetHardBoundary(vtkMatrix4x4* boundaryMin, vtkMatrix4x4* boundaryMax);
  PlusStatus SetHardBoundary(const vtkMatrix4x4& boundaryMin, const vtkMatrix4x4& boundaryMax);
  PlusStatus GetSoftBoundary(vtkMatrix4x4* boundaryMin, vtkMatrix4x4* boundaryMax);
  PlusStatus SetSoftBoundary(const vtkMatrix4x4& boundaryMin, const vtkMatrix4x4& boundaryMax);
  PlusStatus PostBoundaryUpdate(); // update x, y and z to be in increasing order after setting a new boundary

  // getter and setter for the offset
  PlusStatus GetOffset(vtkMatrix4x4* offset);
  PlusStatus SetOffset(const vtkMatrix4x4& newOffset);

  // methods to check vaidity of a movement
  bool CheckWithinBoundaries(vtkMatrix4x4* position);
  bool CheckWithinHardBoundary(vtkMatrix4x4* position);
  bool CheckWithinSoftBoundary(vtkMatrix4x4* position);

private:
  vtkPlusStageDevice(const vtkPlusStageDevice&); // Not implemented
  void operator=(const vtkPlusStageDevice&); // Not implemented

  // offset allows the stage to be 'zeroed', enabling sending of transforms relative to a user defined
  // coordinate frame (does not need to be within stage hard boundary, but probably should be)
  vtkSmartPointer<vtkMatrix4x4> Offset;

  // hard boundary is set in config file, it contains the physical limitations of the stage
  vtkSmartPointer<vtkMatrix4x4> HardBoundaryMin;
  vtkSmartPointer<vtkMatrix4x4> HardBoundaryMax;

  // soft boundary encodes a user defined subset of the stage volume to allow the stage to travel within
  // this is to prevent collisions between a probe and the stage deck, etc.
  vtkSmartPointer<vtkMatrix4x4> SoftBoundaryMin;
  vtkSmartPointer<vtkMatrix4x4> SoftBoundaryMax;

  // private boundary check helper
  bool PerformBoundaryCheck(vtkMatrix4x4* position, vtkMatrix4x4* boundaryMin, vtkMatrix4x4* boundaryMax);
};

#endif
