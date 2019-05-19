/*=Plus=header=begin======================================================
Program: Plus
Copyright (c) Laboratory for Percutaneous Surgery. All rights reserved.
See License.txt for details.
=========================================================Plus=header=end*/

#ifndef __vtkPlusPositioningDevice_h
#define __vtkPlusPositioningDevice_h

#include "vtkPlusDataCollectionExport.h"
#include "vtkPlusDevice.h"
#include <string>

/*!
\class vtkPlusPositioningDevice
\brief Abstract class for handling positioning stage devices
\ingroup PlusLibDataCollection
*/
class vtkPlusDataCollectionExport vtkPlusPositioningDevice : public vtkPlusDevice
{
public:
  static vtkPlusPositioningDevice *New();
  vtkTypeMacro(vtkPlusPositioningDevice, vtkPlusDevice);
  void PrintSelf(ostream& os, vtkIndent indent);

  /* Device is a hardware tracker. */
  virtual bool IsTracker() const { return true; }
  virtual bool IsVirtual() const { return false; }

  /*! Read configuration from xml data */
  virtual PlusStatus ReadConfiguration(vtkXMLDataElement* config);

  /*! Write configuration to xml data */
  virtual PlusStatus WriteConfiguration(vtkXMLDataElement* config);

protected:
  vtkPlusPositioningDevice();
  ~vtkPlusPositioningDevice();

  // movement methods
  virtual PlusStatus HomeAllAxes();
  virtual PlusStatus PauseMovement();
  virtual PlusStatus ResumeMovement();
  virtual PlusStatus StopMovement();
  virtual PlusStatus MoveToAbsolute(const vtkMatrix4x4& position);
  virtual PlusStatus MoveByRelative(const vtkMatrix4x4& position);

  // getters and setters for boundaries
  PlusStatus GetHardBoundary(vtkMatrix4x4* boundaryMin, vtkMatrix4x4* boundaryMax);
  PlusStatus SetHardBoundary(float x_min, float y_min, float z_min, float x_max, float y_max, float z_max);
  PlusStatus GetSoftBoundary(vtkMatrix4x4* boundaryMin, vtkMatrix4x4* boundaryMax);
  PlusStatus SetSoftBoundary(float x_min, float y_min, float z_min, float x_max, float y_max, float z_max);
  PlusStatus PostBoundaryUpdate(); // update x, y and z to be in increasing order after setting a new boundary, bring stage within boundary

  // getter and setter for the offset
  PlusStatus GetOffset(vtkMatrix4x4* offset);
  PlusStatus SetOffset(float x_offset, float y_offset, float z_offset);

  // methods to check vaidity of a movement
  bool CheckWithinBoundaries(vtkMatrix4x4* position);
  bool CheckWithinHardBoundary(vtkMatrix4x4* position);
  bool CheckWithinSoftBoundary(vtkMatrix4x4* position);
  
  // get igsioTransformName of tool position
  std::string GetPositionTransformName();
  PlusStatus SetPositionTransformName(std::string aFrom, std::string aTo);

private:
  vtkPlusPositioningDevice(const vtkPlusPositioningDevice&); // Not implemented
  void operator=(const vtkPlusPositioningDevice&); // Not implemented

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
  bool SoftBoundariesEnabled;

  // private boundary check helper
  bool PerformBoundaryCheck(vtkMatrix4x4* position, vtkMatrix4x4* boundaryMin, vtkMatrix4x4* boundaryMax);

  // name of the tool position transform
  igsioTransformName PositionTransformName;
};

#endif
