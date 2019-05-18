/*=Plus=header=begin======================================================
Program: Plus
Copyright (c) Laboratory for Percutaneous Surgery. All rights reserved.
See License.txt for details.
=========================================================Plus=header=end*/

#include "PlusConfigure.h"

// Local includes
#include "vtkPlusMonopriceStage.h"
#include "vtkPlusStageDevice.h"
#include "vtkIGSIOAccurateTimer.h"

// VTK includes
#include <vtkMatrix4x4.h>
#include <vtkSmartPointer.h>

// System includes


//----------------------------------------------------------------------------
// vtkPlusMonopriceStage::vtkInternal
//----------------------------------------------------------------------------

class vtkPlusMonopriceStage::vtkInternal
{
public:
  vtkPlusMonopriceStage* External;

  vtkInternal(vtkPlusMonopriceStage* external)
    : External(external)
  {
  }

  virtual ~vtkInternal()
  {
  }

};


//----------------------------------------------------------------------------
// vtkPlusMonopriceStage
//----------------------------------------------------------------------------

vtkStandardNewMacro(vtkPlusMonopriceStage);

//----------------------------------------------------------------------------
vtkPlusMonopriceStage::vtkPlusMonopriceStage()
  : vtkPlusStageDevice()
  , Internal(new vtkInternal(this))
{
  LOG_TRACE("vtkPlusMonopriceStage::vtkPlusMonopriceStage()");

  this->FrameNumber = 0;
  this->StartThreadForInternalUpdates = true;
  this->InternalUpdateRate = 30;
}

//----------------------------------------------------------------------------
vtkPlusMonopriceStage::~vtkPlusMonopriceStage()
{
  LOG_TRACE("vtkPlusMonopriceStage::~vtkPlusMonopriceStage()");

  delete Internal;
  Internal = nullptr;
}

//----------------------------------------------------------------------------
void vtkPlusMonopriceStage::PrintSelf(ostream& os, vtkIndent indent)
{
  LOG_TRACE("vtkPlusMonopriceStage::PrintSelf(ostream& os, vtkIndent indent)");
  Superclass::PrintSelf(os, indent);
}

//----------------------------------------------------------------------------
PlusStatus vtkPlusMonopriceStage::ReadConfiguration(vtkXMLDataElement* rootConfigElement)
{
  LOG_TRACE("vtkPlusMonopriceStage::ReadConfiguration");

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
PlusStatus vtkPlusMonopriceStage::WriteConfiguration(vtkXMLDataElement* rootConfigElement)
{
  LOG_TRACE("vtkPlusMonopriceStage::WriteConfiguration");
  XML_FIND_DEVICE_ELEMENT_REQUIRED_FOR_WRITING(deviceConfig, rootConfigElement);
  return PLUS_FAIL;
}

//----------------------------------------------------------------------------
PlusStatus vtkPlusMonopriceStage::InternalConnect()
{
  LOG_TRACE("vtkPlusMonopriceStage::InternalConnect");

  return PLUS_FAIL;
}

//----------------------------------------------------------------------------
PlusStatus vtkPlusMonopriceStage::InternalDisconnect()
{
  LOG_TRACE("vtkPlusMonopriceStage::InternalDisconnect");

  return PLUS_FAIL;
}

//----------------------------------------------------------------------------
PlusStatus vtkPlusMonopriceStage::InternalStartRecording()
{
  LOG_TRACE("vtkPlusMonopriceStage::InternalStartRecording");
  return PLUS_FAIL;
}

//----------------------------------------------------------------------------
PlusStatus vtkPlusMonopriceStage::InternalStopRecording()
{
  LOG_TRACE("vtkPlusMonopriceStage::InternalStopRecording");
  return PLUS_FAIL;
}

//----------------------------------------------------------------------------
PlusStatus vtkPlusMonopriceStage::Probe()
{
  LOG_TRACE("vtkPlusMonopriceStage::Probe");
  return PLUS_FAIL;
}

//----------------------------------------------------------------------------
PlusStatus vtkPlusMonopriceStage::InternalUpdate()
{
  LOG_TRACE("vtkPlusMonopriceStage::InternalUpdate");

  return PLUS_FAIL;
}

//----------------------------------------------------------------------------
// STAGE METHODS
//----------------------------------------------------------------------------

PlusStatus vtkPlusMonopriceStage::HomeAllAxes()
{
  LOG_TRACE("vtkPlusMonopriceStage::HomeAllAxes()");
  // home all axes (G28)
  return PLUS_FAIL;
}

PlusStatus vtkPlusMonopriceStage::PauseMovement()
{
  LOG_TRACE("vtkPlusMonopriceStage::PauseMovement()");
  return PLUS_FAIL;
}

PlusStatus vtkPlusMonopriceStage::ResumeMovement()
{
  LOG_TRACE("vtkPlusMonopriceStage::ResumeMovement()");
  return PLUS_FAIL;
}

PlusStatus vtkPlusMonopriceStage::StopMovement()
{
  LOG_TRACE("vtkPlusMonopriceStage::StopMovement()");
  // emergency stop (M112)
  return PLUS_FAIL;
}

PlusStatus vtkPlusMonopriceStage::MoveToAbsolute(const vtkMatrix4x4& position)
{
  LOG_TRACE("vtkPlusMonopriceStage::MoveToAbsolute(const vtkMatrix4x4& position)");
  // move to position rapid (G0)
  // move to position precise (G1)
  return PLUS_FAIL;
}

PlusStatus vtkPlusMonopriceStage::MoveByRelative(const vtkMatrix4x4& position)
{
  LOG_TRACE("vtkPlusMonopriceStage::MoveByRelative(const vtkMatrix4x4& position)");
  // move to position rapid (G0)
  // move to position precise (G1)
  return PLUS_FAIL;
}

PlusStatus vtkPlusMonopriceStage::GetCurrentPosition(vtkMatrix4x4* position)
{
  LOG_TRACE("vtkPlusMonopriceStage::GetCurrentPosition(vtkMatrix4x4* position)");
  // get current position (M114)
  return PLUS_FAIL;
}