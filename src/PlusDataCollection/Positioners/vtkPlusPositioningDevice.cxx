/*=Plus=header=begin======================================================
Program: Plus
Copyright (c) Laboratory for Percutaneous Surgery. All rights reserved.
See License.txt for details.
=========================================================Plus=header=end*/

#include "PlusConfigure.h"

// Local includes
#include "vtkPlusPositioningDevice.h"
#include "vtkIGSIOAccurateTimer.h"

// VTK includes
#include <vtkMatrix4x4.h>
#include <vtkSmartPointer.h>

// System includes


//----------------------------------------------------------------------------
// vtkPlusPositioningDevice
//----------------------------------------------------------------------------

vtkStandardNewMacro(vtkPlusPositioningDevice);

//----------------------------------------------------------------------------
vtkPlusPositioningDevice::vtkPlusPositioningDevice()
  : vtkPlusDevice(),
  SoftBoundariesEnabled(false)
{
  LOG_TRACE("vtkPlusPositioningDevice::vtkPlusPositioningDevice()");
  this->HardBoundaryMin = vtkSmartPointer<vtkMatrix4x4>::New();
  this->HardBoundaryMax = vtkSmartPointer<vtkMatrix4x4>::New();
  this->SoftBoundaryMin = vtkSmartPointer<vtkMatrix4x4>::New();
  this->SoftBoundaryMax = vtkSmartPointer<vtkMatrix4x4>::New();
  this->Offset = vtkSmartPointer<vtkMatrix4x4>::New();
}

//----------------------------------------------------------------------------
vtkPlusPositioningDevice::~vtkPlusPositioningDevice()
{
  LOG_TRACE("vtkPlusPositioningDevice::~vtkPlusPositioningDevice()");
}

//----------------------------------------------------------------------------
void vtkPlusPositioningDevice::PrintSelf(ostream& os, vtkIndent indent)
{
  LOG_TRACE("vtkPlusPositioningDevice::PrintSelf(ostream& os, vtkIndent indent)");
  Superclass::PrintSelf(os, indent);
}

//----------------------------------------------------------------------------
PlusStatus vtkPlusPositioningDevice::ReadConfiguration(vtkXMLDataElement* rootConfigElement)
{
  LOG_TRACE("vtkPlusPositioningDevice::ReadConfiguration");

  vtkXMLDataElement* deviceConfig = this->FindThisDeviceElement(rootConfigElement);

  // Read hard boundary
  float hbMin[3] = { 0, 0, 0 }, hbMax[3] = { 0, 0, 0 };
  XML_READ_VECTOR_ATTRIBUTE_NONMEMBER_REQUIRED(float, 3, HardBoundaryMin, hbMin, deviceConfig);
  XML_READ_VECTOR_ATTRIBUTE_NONMEMBER_REQUIRED(float, 3, HardBoundaryMax, hbMax, deviceConfig);
  this->SetHardBoundary(hbMin[0], hbMin[1], hbMin[2], hbMax[0], hbMax[1], hbMax[2]);
  
  // Read soft boundary
  if (deviceConfig->GetAttribute("SoftBoundaryMin") != nullptr &&
    deviceConfig->GetAttribute("SoftBoundaryMax") != nullptr)
  {
    float sbMin[3] = { 0, 0, 0 }, sbMax[3] = { 0, 0, 0 };
    XML_READ_VECTOR_ATTRIBUTE_NONMEMBER_OPTIONAL(float, 3, SoftBoundaryMin, sbMin, deviceConfig);
    XML_READ_VECTOR_ATTRIBUTE_NONMEMBER_OPTIONAL(float, 3, SoftBoundaryMax, sbMax, deviceConfig);
    this->SetSoftBoundary(sbMin[0], sbMin[1], sbMin[2], sbMax[0], sbMax[1], sbMax[2]);
  }

  // Read offset
  float offset[3] = { 0, 0, 0 };
  XML_READ_VECTOR_ATTRIBUTE_NONMEMBER_REQUIRED(float, 3, Offset, offset, deviceConfig);
  this->SetOffset(offset[0], offset[1], offset[2]);

  return Superclass::ReadConfiguration(rootConfigElement);
}

//----------------------------------------------------------------------------
PlusStatus vtkPlusPositioningDevice::WriteConfiguration(vtkXMLDataElement* rootConfigElement)
{
  LOG_TRACE("vtkPlusPositioningDevice::WriteConfiguration");
  XML_FIND_DEVICE_ELEMENT_REQUIRED_FOR_WRITING(deviceConfig, rootConfigElement);

  return PLUS_FAIL;
}

//----------------------------------------------------------------------------
// STAGE METHODS
//----------------------------------------------------------------------------

PlusStatus vtkPlusPositioningDevice::GetHardBoundary(vtkMatrix4x4* boundaryMin, vtkMatrix4x4* boundaryMax)
{
  LOG_TRACE("vtkPlusPositioningDevice::GetHardBoundary(vtkMatrix4x4* boundaryMin, vtkMatrix4x4* boundaryMax)");
  boundaryMin->DeepCopy(this->HardBoundaryMin);
  boundaryMax->DeepCopy(this->HardBoundaryMax);
  return PLUS_FAIL;
}

//----------------------------------------------------------------------------
PlusStatus vtkPlusPositioningDevice::SetHardBoundary(float x_min, float y_min, float z_min, float x_max, float y_max, float z_max)
{
  LOG_TRACE("vtkPlusPositioningDevice::SetHardBoundary(float x_min, float y_min, float z_min, float x_max, float y_max, float z_max)");

  this->HardBoundaryMin->Identity();
  this->HardBoundaryMin->SetElement(0, 3, x_min);
  this->HardBoundaryMin->SetElement(1, 3, y_min);
  this->HardBoundaryMin->SetElement(2, 3, z_min);

  this->HardBoundaryMax->Identity();
  this->HardBoundaryMax->SetElement(0, 3, x_max);
  this->HardBoundaryMax->SetElement(1, 3, y_max);
  this->HardBoundaryMax->SetElement(2, 3, z_max);

  return PLUS_SUCCESS;
}

//----------------------------------------------------------------------------
PlusStatus vtkPlusPositioningDevice::GetSoftBoundary(vtkMatrix4x4* boundaryMin, vtkMatrix4x4* boundaryMax)
{
  LOG_TRACE("vtkPlusPositioningDevice::GetSoftBoundary(vtkMatrix4x4* boundaryMin, vtkMatrix4x4* boundaryMax)");
  boundaryMin->DeepCopy(this->SoftBoundaryMin);
  boundaryMax->DeepCopy(this->SoftBoundaryMax);
  return PLUS_SUCCESS;
}

//----------------------------------------------------------------------------
PlusStatus vtkPlusPositioningDevice::SetSoftBoundary(float x_min, float y_min, float z_min, float x_max, float y_max, float z_max)
{
  LOG_TRACE("vtkPlusPositioningDevice::SetSoftBoundary(float x_min, float y_min, float z_min, float x_max, float y_max, float z_max)");

  this->SoftBoundaryMin->Identity();
  this->SoftBoundaryMin->SetElement(0, 3, x_min);
  this->SoftBoundaryMin->SetElement(1, 3, y_min);
  this->SoftBoundaryMin->SetElement(2, 3, z_min);

  this->SoftBoundaryMax->Identity();
  this->SoftBoundaryMax->SetElement(0, 3, x_max);
  this->SoftBoundaryMax->SetElement(1, 3, y_max);
  this->SoftBoundaryMax->SetElement(2, 3, z_max);

  this->SoftBoundariesEnabled = true;

  return PLUS_SUCCESS;
}

//----------------------------------------------------------------------------
PlusStatus vtkPlusPositioningDevice::PostBoundaryUpdate()
{
  // ensure the x, y, z components are in ascending order, and the current position is
  // within the bounding box
  LOG_TRACE("vtkPlusPositioningDevice::PostBoundaryUpdate()");
  return PLUS_FAIL;
}

//----------------------------------------------------------------------------
PlusStatus vtkPlusPositioningDevice::GetOffset(vtkMatrix4x4* offset)
{
  LOG_TRACE("vtkPlusPositioningDevice::GetOffset(vtkMatrix4x4* offset)");

  offset->DeepCopy(this->Offset);

  return PLUS_FAIL;
}

//----------------------------------------------------------------------------
PlusStatus vtkPlusPositioningDevice::SetOffset(float x_offset, float y_offset, float z_offset)
{
  LOG_TRACE("vtkPlusPositioningDevice::GetOffset(vtkMatrix4x4* offset)");

  this->Offset->Identity();
  this->Offset->SetElement(0, 3, x_offset);
  this->Offset->SetElement(1, 3, y_offset);
  this->Offset->SetElement(2, 3, z_offset);

  return PLUS_FAIL;
}

//----------------------------------------------------------------------------
std::string vtkPlusPositioningDevice::GetPositionTransformName()
{
  return this->PositionTransformName.GetTransformName();
}

//----------------------------------------------------------------------------
PlusStatus vtkPlusPositioningDevice::SetPositionTransformName(std::string aFrom, std::string aTo)
{
  return this->PositionTransformName.SetTransformName(aFrom, aTo);
}

//----------------------------------------------------------------------------
bool vtkPlusPositioningDevice::CheckWithinBoundaries(vtkMatrix4x4* position)
{
  LOG_TRACE("vtkPlusPositioningDevice::CheckWithinBoundaries(vtkMatrix4x4* position)");
  return this->CheckWithinHardBoundary(position) && this->CheckWithinSoftBoundary(position);
}

//----------------------------------------------------------------------------
bool vtkPlusPositioningDevice::CheckWithinHardBoundary(vtkMatrix4x4* position)
{
  LOG_TRACE("vtkPlusPositioningDevice::CheckWithinHardBoundary(vtkMatrix4x4* position)");
  return this->PerformBoundaryCheck(position, this->HardBoundaryMin, this->HardBoundaryMax);
}
//----------------------------------------------------------------------------
bool vtkPlusPositioningDevice::CheckWithinSoftBoundary(vtkMatrix4x4* position)
{
  LOG_TRACE("");
  return this->PerformBoundaryCheck(position, this->SoftBoundaryMin, this->SoftBoundaryMax);
}
//----------------------------------------------------------------------------
bool vtkPlusPositioningDevice::PerformBoundaryCheck(vtkMatrix4x4* position, vtkMatrix4x4* boundaryMin, vtkMatrix4x4* boundaryMax)
{
  LOG_TRACE("vtkPlusPositioningDevice::CheckWithinSoftBoundary(vtkMatrix4x4* position)");
  return false;
}

//----------------------------------------------------------------------------
PlusStatus vtkPlusPositioningDevice::HomeAllAxes()
{
  LOG_TRACE("vtkPlusPositioningDevice::HomeAllAxes()");
  LOG_INFO("This method must be implemented in vtkPlusPositioningDevice sub-classes.");
  return PLUS_FAIL;
}

//----------------------------------------------------------------------------
PlusStatus vtkPlusPositioningDevice::PauseMovement()
{
  LOG_TRACE("vtkPlusPositioningDevice::PauseMovement()");
  LOG_INFO("This method must be implemented in vtkPlusPositioningDevice sub-classes.");
  return PLUS_FAIL;
}

//----------------------------------------------------------------------------
PlusStatus vtkPlusPositioningDevice::ResumeMovement()
{
  LOG_TRACE("vtkPlusPositioningDevice::ResumeMovement()");
  LOG_INFO("This method must be implemented in vtkPlusPositioningDevice sub-classes.");
  return PLUS_FAIL;
}

//----------------------------------------------------------------------------
PlusStatus vtkPlusPositioningDevice::StopMovement()
{
  LOG_TRACE("vtkPlusPositioningDevice::StopMovement()");
  LOG_INFO("This method must be implemented in vtkPlusPositioningDevice sub-classes.");
  return PLUS_FAIL;
}

//----------------------------------------------------------------------------
PlusStatus vtkPlusPositioningDevice::MoveToAbsolute(const vtkMatrix4x4& position)
{
  LOG_TRACE("vtkPlusPositioningDevice::MoveToAbsolute(const vtkMatrix4x4& position)");
  LOG_INFO("This method must be implemented in vtkPlusPositioningDevice sub-classes.");
  return PLUS_FAIL;
}

//----------------------------------------------------------------------------
PlusStatus vtkPlusPositioningDevice::MoveByRelative(const vtkMatrix4x4& position)
{
  LOG_TRACE("vtkPlusPositioningDevice::MoveByRelative(const vtkMatrix4x4& position)");
  LOG_INFO("This method must be implemented in vtkPlusPositioningDevice sub-classes.");
  return PLUS_FAIL;
}