/*=Plus=header=begin======================================================
Program: Plus
Copyright (c) Laboratory for Percutaneous Surgery. All rights reserved.
See License.txt for details.
=========================================================Plus=header=end*/

#include "PlusConfigure.h"

// Local includes
#include "vtkPlusVelmexStage.h"
#include "vtkPlusStageDevice.h"
#include "vtkIGSIOAccurateTimer.h"

// VTK includes
#include <vtkMatrix4x4.h>
#include <vtkSmartPointer.h>

// System includes


//----------------------------------------------------------------------------
// vtkPlusVelmexStage::vtkInternal
//----------------------------------------------------------------------------

class vtkPlusVelmexStage::vtkInternal
{
public:
  vtkPlusVelmexStage* External;

  vtkInternal(vtkPlusVelmexStage* external)
    : External(external)
  {
  }

  virtual ~vtkInternal()
  {
  }

};


//----------------------------------------------------------------------------
// vtkPlusVelmexStage
//----------------------------------------------------------------------------

vtkStandardNewMacro(vtkPlusVelmexStage);

//----------------------------------------------------------------------------
vtkPlusVelmexStage::vtkPlusVelmexStage()
  : vtkPlusStageDevice()
  , Internal(new vtkInternal(this))
{
  LOG_TRACE("vtkPlusVelmexStage::vtkPlusVelmexStage()");

  this->FrameNumber = 0;
  this->StartThreadForInternalUpdates = true;
  this->InternalUpdateRate = 30;
}

//----------------------------------------------------------------------------
vtkPlusVelmexStage::~vtkPlusVelmexStage()
{
  LOG_TRACE("vtkPlusVelmexStage::~vtkPlusVelmexStage()");

  delete Internal;
  Internal = nullptr;
}

//----------------------------------------------------------------------------
void vtkPlusVelmexStage::PrintSelf(ostream& os, vtkIndent indent)
{
  LOG_TRACE("vtkPlusVelmexStage::PrintSelf(ostream& os, vtkIndent indent)");
  Superclass::PrintSelf(os, indent);
}

//----------------------------------------------------------------------------
PlusStatus vtkPlusVelmexStage::ReadConfiguration(vtkXMLDataElement* rootConfigElement)
{
  LOG_TRACE("vtkPlusVelmexStage::ReadConfiguration");

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
PlusStatus vtkPlusVelmexStage::WriteConfiguration(vtkXMLDataElement* rootConfigElement)
{
  LOG_TRACE("vtkPlusVelmexStage::WriteConfiguration");
  XML_FIND_DEVICE_ELEMENT_REQUIRED_FOR_WRITING(deviceConfig, rootConfigElement);
  return PLUS_FAIL;
}

//----------------------------------------------------------------------------
PlusStatus vtkPlusVelmexStage::InternalConnect()
{
  LOG_TRACE("vtkPlusVelmexStage::InternalConnect");

  return PLUS_FAIL;
}

//----------------------------------------------------------------------------
PlusStatus vtkPlusVelmexStage::InternalDisconnect()
{
  LOG_TRACE("vtkPlusVelmexStage::InternalDisconnect");

  return PLUS_FAIL;
}

//----------------------------------------------------------------------------
PlusStatus vtkPlusVelmexStage::InternalStartRecording()
{
  LOG_TRACE("vtkPlusVelmexStage::InternalStartRecording");
  return PLUS_FAIL;
}

//----------------------------------------------------------------------------
PlusStatus vtkPlusVelmexStage::InternalStopRecording()
{
  LOG_TRACE("vtkPlusVelmexStage::InternalStopRecording");
  return PLUS_FAIL;
}

//----------------------------------------------------------------------------
PlusStatus vtkPlusVelmexStage::Probe()
{
  LOG_TRACE("vtkPlusVelmexStage::Probe");
  return PLUS_FAIL;
}

//----------------------------------------------------------------------------
PlusStatus vtkPlusVelmexStage::InternalUpdate()
{
  LOG_TRACE("vtkPlusVelmexStage::InternalUpdate");

  return PLUS_FAIL;
}

//----------------------------------------------------------------------------
// STAGE METHODS
//----------------------------------------------------------------------------

PlusStatus vtkPlusVelmexStage::HomeAllAxes()
{
  LOG_TRACE("vtkPlusVelmexStage::HomeAllAxes()");
  return PLUS_FAIL;
}

PlusStatus vtkPlusVelmexStage::PauseMovement()
{
  LOG_TRACE("vtkPlusVelmexStage::PauseMovement()");
  return PLUS_FAIL;
}

PlusStatus vtkPlusVelmexStage::ResumeMovement()
{
  LOG_TRACE("vtkPlusVelmexStage::ResumeMovement()");
  return PLUS_FAIL;
}

PlusStatus vtkPlusVelmexStage::StopMovement()
{
  LOG_TRACE("vtkPlusVelmexStage::StopMovement()");
  return PLUS_FAIL;
}

PlusStatus vtkPlusVelmexStage::MoveToAbsolute(const vtkMatrix4x4& position)
{
  LOG_TRACE("vtkPlusVelmexStage::MoveToAbsolute(const vtkMatrix4x4& position)");
  return PLUS_FAIL;
}

PlusStatus vtkPlusVelmexStage::MoveByRelative(const vtkMatrix4x4& position)
{
  LOG_TRACE("vtkPlusVelmexStage::MoveByRelative(const vtkMatrix4x4& position)");
  return PLUS_FAIL;
}

PlusStatus vtkPlusVelmexStage::GetCurrentPosition(vtkMatrix4x4* position)
{
  LOG_TRACE("vtkPlusVelmexStage::GetCurrentPosition(vtkMatrix4x4* position)");
  return PLUS_FAIL;
}