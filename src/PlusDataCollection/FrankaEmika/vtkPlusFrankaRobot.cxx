/*=Plus=header=begin======================================================
Program: Plus
Copyright (c) Laboratory for Percutaneous Surgery. All rights reserved.
See License.txt for details.
=========================================================Plus=header=end*/

#include "PlusConfigure.h"

// Local includes
#include "PlusFranka.h"
#include "vtkIGSIOAccurateTimer.h"
#include "vtkPlusDataSource.h"
#include "vtkPlusFrankaRobot.h"

// VTK includes
#include <vtkMatrix4x4.h>
#include <vtkSmartPointer.h>

// System includes


vtkStandardNewMacro(vtkPlusFrankaRobot);

//----------------------------------------------------------------------------
class vtkPlusFrankaRobot::vtkInternal
{
public:
  vtkPlusFrankaRobot* External;

  vtkInternal(vtkPlusFrankaRobot* external)
    : External(external)
  {
  }

  virtual ~vtkInternal()
  {
  }


};

//----------------------------------------------------------------------------
vtkPlusFrankaRobot::vtkPlusFrankaRobot()
  : vtkPlusDevice()
  , Internal(new vtkInternal(this))
{
  LOG_TRACE("vtkPlusFrankaRobot::vtkPlusFrankaRobot()");

  this->FrameNumber = 0;
  this->StartThreadForInternalUpdates = true;
  this->InternalUpdateRate = 80;
}

//----------------------------------------------------------------------------
vtkPlusFrankaRobot::~vtkPlusFrankaRobot()
{
  LOG_TRACE("vtkPlusFrankaRobot::~vtkPlusFrankaRobot()");

  delete Internal;
  Internal = nullptr;
}

//----------------------------------------------------------------------------
void vtkPlusFrankaRobot::PrintSelf(ostream& os, vtkIndent indent)
{
  Superclass::PrintSelf(os, indent);
}

//----------------------------------------------------------------------------
PlusStatus vtkPlusFrankaRobot::ReadConfiguration(vtkXMLDataElement* rootConfigElement)
{
  LOG_TRACE("vtkPlusFrankaRobot::ReadConfiguration");

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
    continue;
    }
  }

  return PLUS_FAIL;
}

//----------------------------------------------------------------------------
PlusStatus vtkPlusFrankaRobot::WriteConfiguration(vtkXMLDataElement* rootConfigElement)
{
  LOG_TRACE("vtkPlusFrankaRobot::WriteConfiguration");
  XML_FIND_DEVICE_ELEMENT_REQUIRED_FOR_WRITING(deviceConfig, rootConfigElement);
  return PLUS_FAIL;
}

//----------------------------------------------------------------------------
PlusStatus vtkPlusFrankaRobot::Probe()
{
  LOG_TRACE("vtkPlusFrankaRobot::Probe");
  return PLUS_FAIL;
}

//----------------------------------------------------------------------------
PlusStatus vtkPlusFrankaRobot::InternalConnect()
{
  LOG_TRACE("vtkPlusFrankaRobot::InternalConnect");
  return PLUS_FAIL;
}

//----------------------------------------------------------------------------
PlusStatus vtkPlusFrankaRobot::InternalDisconnect()
{
  LOG_TRACE("vtkPlusFrankaRobot::InternalDisconnect");
  return PLUS_FAIL;
}

//----------------------------------------------------------------------------
PlusStatus vtkPlusFrankaRobot::InternalStartRecording()
{
  LOG_TRACE("vtkPlusFrankaRobot::InternalStartRecording");
  return PLUS_FAIL;
}

//----------------------------------------------------------------------------
PlusStatus vtkPlusFrankaRobot::InternalStopRecording()
{
  LOG_TRACE("vtkPlusFrankaRobot::InternalStopRecording");
  return PLUS_FAIL;
}

//----------------------------------------------------------------------------
PlusStatus vtkPlusFrankaRobot::InternalUpdate()
{
  LOG_TRACE("vtkPlusFrankaRobot::InternalUpdate");
  this->FrameNumber++;
  return PLUS_FAIL;
}

//----------------------------------------------------------------------------
// Command methods
//----------------------------------------------------------------------------
