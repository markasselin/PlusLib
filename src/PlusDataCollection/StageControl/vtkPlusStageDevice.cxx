/*=Plus=header=begin======================================================
Program: Plus
Copyright (c) Laboratory for Percutaneous Surgery. All rights reserved.
See License.txt for details.
=========================================================Plus=header=end*/

#include "PlusConfigure.h"

// Local includes
#include "vtkPlusStageDevice.h"
#include "vtkIGSIOAccurateTimer.h"

// VTK includes
#include <vtkMatrix4x4.h>
#include <vtkSmartPointer.h>

// System includes


//----------------------------------------------------------------------------
// vtkPlusStageDevice
//----------------------------------------------------------------------------

vtkStandardNewMacro(vtkPlusStageDevice);

//----------------------------------------------------------------------------
vtkPlusStageDevice::vtkPlusStageDevice()
  : vtkPlusDevice()
{
  LOG_TRACE("vtkPlusStageDevice::vtkPlusStageDevice()");
}

//----------------------------------------------------------------------------
vtkPlusStageDevice::~vtkPlusStageDevice()
{
  LOG_TRACE("vtkPlusStageDevice::~vtkPlusStageDevice()");
}

//----------------------------------------------------------------------------
void vtkPlusStageDevice::PrintSelf(ostream& os, vtkIndent indent)
{
  LOG_TRACE("vtkPlusStageDevice::PrintSelf(ostream& os, vtkIndent indent)");
  Superclass::PrintSelf(os, indent);
}

//----------------------------------------------------------------------------
PlusStatus vtkPlusStageDevice::ReadConfiguration(vtkXMLDataElement* rootConfigElement)
{
  LOG_TRACE("vtkPlusStageDevice::ReadConfiguration");

  XML_FIND_DEVICE_ELEMENT_REQUIRED_FOR_READING(deviceConfig, rootConfigElement);

  XML_FIND_NESTED_ELEMENT_REQUIRED(dataSourcesElement, deviceConfig, "DataSources");
  for (int nestedElementIndex = 0; nestedElementIndex < dataSourcesElement->GetNumberOfNestedElements(); nestedElementIndex++)
  {
    vtkXMLDataElement* toolDataElement = dataSourcesElement->GetNestedElement(nestedElementIndex);
    if (STRCASECMP(toolDataElement->GetName(), "DataSource") != 0)
    {
      // if this is not a data source element, skip it
      continue;
    }
    if (toolDataElement->GetAttribute("Type") != NULL && STRCASECMP(toolDataElement->GetAttribute("Type"), "Tool") != 0)
    {
      // if this is not a Tool element, skip it
      continue;
    }
    std::string toolId(toolDataElement->GetAttribute("Id"));
    if (toolId.empty())
    {
      // tool doesn't have ID needed to generate transform
      LOG_ERROR("Failed to initialize Atracsys tool: DataSource Id is missing.");
      continue;
    }

  }

  return PLUS_FAIL;
}

//----------------------------------------------------------------------------
PlusStatus vtkPlusStageDevice::WriteConfiguration(vtkXMLDataElement* rootConfigElement)
{
  LOG_TRACE("vtkPlusStageDevice::WriteConfiguration");
  XML_FIND_DEVICE_ELEMENT_REQUIRED_FOR_WRITING(deviceConfig, rootConfigElement);
  return PLUS_FAIL;
}

//----------------------------------------------------------------------------
PlusStatus vtkPlusStageDevice::InternalConnect()
{
  LOG_TRACE("vtkPlusStageDevice::InternalConnect");
  return PLUS_FAIL;
}

//----------------------------------------------------------------------------
PlusStatus vtkPlusStageDevice::InternalDisconnect()
{
  LOG_TRACE("vtkPlusStageDevice::InternalDisconnect");
  return PLUS_FAIL;
}

//----------------------------------------------------------------------------
PlusStatus vtkPlusStageDevice::InternalStartRecording()
{
  LOG_TRACE("vtkPlusStageDevice::InternalStartRecording");
  return PLUS_FAIL;
}

//----------------------------------------------------------------------------
PlusStatus vtkPlusStageDevice::InternalStopRecording()
{
  LOG_TRACE("vtkPlusStageDevice::InternalStopRecording");
  return PLUS_FAIL;
}

//----------------------------------------------------------------------------
PlusStatus vtkPlusStageDevice::Probe()
{
  LOG_TRACE("vtkPlusStageDevice::Probe");
  return PLUS_FAIL;
}

//----------------------------------------------------------------------------
PlusStatus vtkPlusStageDevice::InternalUpdate()
{
  LOG_TRACE("vtkPlusStageDevice::InternalUpdate");
  return PLUS_FAIL;
}

//----------------------------------------------------------------------------
// STAGE METHODS
//----------------------------------------------------------------------------

PlusStatus vtkPlusStageDevice::GetHardBoundary(vtkMatrix4x4* boundaryMin, vtkMatrix4x4* boundaryMax)
{
  LOG_TRACE("vtkPlusStageDevice::GetHardBoundary(vtkMatrix4x4* boundaryMin, vtkMatrix4x4* boundaryMax)");
  boundaryMin->DeepCopy(this->HardBoundaryMin);
  boundaryMax->DeepCopy(this->HardBoundaryMax);
  return PLUS_FAIL;
}

//----------------------------------------------------------------------------
PlusStatus vtkPlusStageDevice::SetHardBoundary(const vtkMatrix4x4& boundaryMin, const vtkMatrix4x4& boundaryMax)
{
  LOG_TRACE("vtkPlusStageDevice::SetHardBoundary(const vtkMatrix4x4& boundaryMin, const vtkMatrix4x4& boundaryMax)");
  return PLUS_FAIL;
}

//----------------------------------------------------------------------------
PlusStatus vtkPlusStageDevice::GetSoftBoundary(vtkMatrix4x4* boundaryMin, vtkMatrix4x4* boundaryMax)
{
  LOG_TRACE("");
  return PLUS_FAIL;
}

//----------------------------------------------------------------------------
PlusStatus vtkPlusStageDevice::SetSoftBoundary(const vtkMatrix4x4& boundaryMin, const vtkMatrix4x4& boundaryMax)
{
  LOG_TRACE("vtkPlusStageDevice::GetSoftBoundary(vtkMatrix4x4* boundaryMin, vtkMatrix4x4* boundaryMax)");
  return PLUS_FAIL;
}

//----------------------------------------------------------------------------
PlusStatus vtkPlusStageDevice::PostBoundaryUpdate()
{
  LOG_TRACE("vtkPlusStageDevice::PostBoundaryUpdate()");
  return PLUS_FAIL;
}

//----------------------------------------------------------------------------
PlusStatus vtkPlusStageDevice::GetOffset(vtkMatrix4x4* offset)
{
  LOG_TRACE("");
  return PLUS_FAIL;
}

//----------------------------------------------------------------------------
PlusStatus vtkPlusStageDevice::SetOffset(const vtkMatrix4x4& newOffset)
{
  LOG_TRACE("vtkPlusStageDevice::GetOffset(vtkMatrix4x4* offset)");
  return PLUS_FAIL;
}

//----------------------------------------------------------------------------
bool vtkPlusStageDevice::CheckWithinBoundaries(vtkMatrix4x4* position)
{
  LOG_TRACE("vtkPlusStageDevice::CheckWithinBoundaries(vtkMatrix4x4* position)");
  return this->CheckWithinHardBoundary(position) && this->CheckWithinSoftBoundary(position);
}

//----------------------------------------------------------------------------
bool vtkPlusStageDevice::CheckWithinHardBoundary(vtkMatrix4x4* position)
{
  LOG_TRACE("vtkPlusStageDevice::CheckWithinHardBoundary(vtkMatrix4x4* position)");
  return this->PerformBoundaryCheck(position, this->HardBoundaryMin, this->HardBoundaryMax);
}
//----------------------------------------------------------------------------
bool vtkPlusStageDevice::CheckWithinSoftBoundary(vtkMatrix4x4* position)
{
  LOG_TRACE("");
  return this->PerformBoundaryCheck(position, this->SoftBoundaryMin, this->SoftBoundaryMax);
}
//----------------------------------------------------------------------------
bool vtkPlusStageDevice::PerformBoundaryCheck(vtkMatrix4x4* position, vtkMatrix4x4* boundaryMin, vtkMatrix4x4* boundaryMax)
{
  LOG_TRACE("vtkPlusStageDevice::CheckWithinSoftBoundary(vtkMatrix4x4* position)");
  return false;
}

//----------------------------------------------------------------------------
PlusStatus vtkPlusStageDevice::HomeAllAxes()
{
  LOG_TRACE("vtkPlusStageDevice::HomeAllAxes()");
  LOG_INFO("This method must be implemented in vtkPlusStageDevice sub-classes.");
  return PLUS_FAIL;
}

//----------------------------------------------------------------------------
PlusStatus vtkPlusStageDevice::PauseMovement()
{
  LOG_TRACE("vtkPlusStageDevice::PauseMovement()");
  LOG_INFO("This method must be implemented in vtkPlusStageDevice sub-classes.");
  return PLUS_FAIL;
}

//----------------------------------------------------------------------------
PlusStatus vtkPlusStageDevice::ResumeMovement()
{
  LOG_TRACE("vtkPlusStageDevice::ResumeMovement()");
  LOG_INFO("This method must be implemented in vtkPlusStageDevice sub-classes.");
  return PLUS_FAIL;
}

//----------------------------------------------------------------------------
PlusStatus vtkPlusStageDevice::StopMovement()
{
  LOG_TRACE("vtkPlusStageDevice::StopMovement()");
  LOG_INFO("This method must be implemented in vtkPlusStageDevice sub-classes.");
  return PLUS_FAIL;
}

//----------------------------------------------------------------------------
PlusStatus vtkPlusStageDevice::MoveToAbsolute(const vtkMatrix4x4& position)
{
  LOG_TRACE("vtkPlusStageDevice::MoveToAbsolute(const vtkMatrix4x4& position)");
  LOG_INFO("This method must be implemented in vtkPlusStageDevice sub-classes.");
  return PLUS_FAIL;
}

//----------------------------------------------------------------------------
PlusStatus vtkPlusStageDevice::MoveByRelative(const vtkMatrix4x4& position)
{
  LOG_TRACE("vtkPlusStageDevice::MoveByRelative(const vtkMatrix4x4& position)");
  LOG_INFO("This method must be implemented in vtkPlusStageDevice sub-classes.");
  return PLUS_FAIL;
}

//----------------------------------------------------------------------------
PlusStatus vtkPlusStageDevice::GetCurrentPosition(vtkMatrix4x4* position)
{
  LOG_TRACE("vtkPlusStageDevice::GetCurrentPosition(vtkMatrix4x4* position)");
  LOG_INFO("This method must be implemented in vtkPlusStageDevice sub-classes.");
  return PLUS_FAIL;
}