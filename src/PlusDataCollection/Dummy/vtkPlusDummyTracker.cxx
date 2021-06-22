/*=Plus=header=begin======================================================
Program: Plus
Copyright (c) Laboratory for Percutaneous Surgery. All rights reserved.
See License.txt for details.
=========================================================Plus=header=end*/

// Local includes
#include "PlusConfigure.h"
#include "vtkPlusDummyTracker.h"
#include "vtkPlusDataSource.h"

// VTK includes
#include <vtkMatrix4x4.h>
#include <vtkSmartPointer.h>

// System includes
#include <fstream>
#include <iostream>
#include <map>
#include <string>

vtkStandardNewMacro(vtkPlusDummyTracker);

//----------------------------------------------------------------------------
vtkPlusDummyTracker::vtkPlusDummyTracker()
  : vtkPlusDevice()
{
  LOG_TRACE("vtkPlusDummyTracker::vtkPlusDummyTracker");

  this->FrameNumber = 0;
  this->StartThreadForInternalUpdates = true;
  this->InternalUpdateRate = 30;
}

//----------------------------------------------------------------------------
vtkPlusDummyTracker::~vtkPlusDummyTracker()
{
  LOG_TRACE("vtkPlusDummyTracker::~vtkPlusDummyTracker");
}

//----------------------------------------------------------------------------
void vtkPlusDummyTracker::PrintSelf(ostream& os, vtkIndent indent)
{
  Superclass::PrintSelf(os, indent);
}

//----------------------------------------------------------------------------
PlusStatus vtkPlusDummyTracker::ReadConfiguration(vtkXMLDataElement* rootConfigElement)
{
  LOG_TRACE("vtkPlusDummyTracker::ReadConfiguration");

  XML_FIND_DEVICE_ELEMENT_REQUIRED_FOR_READING(deviceConfig, rootConfigElement);

  // no device specific attributes to read

  return PLUS_SUCCESS;
}

//----------------------------------------------------------------------------
PlusStatus vtkPlusDummyTracker::WriteConfiguration(vtkXMLDataElement* rootConfigElement)
{
  LOG_TRACE("vtkPlusDummyTracker::WriteConfiguration");
  XML_FIND_DEVICE_ELEMENT_REQUIRED_FOR_WRITING(deviceConfig, rootConfigElement);

  // no device specific attributes to write
  return PLUS_SUCCESS;
}

//----------------------------------------------------------------------------
PlusStatus vtkPlusDummyTracker::Probe()
{
  LOG_TRACE("vtkPlusDummyTracker::Probe");
  return PLUS_SUCCESS;
}

//----------------------------------------------------------------------------
PlusStatus vtkPlusDummyTracker::InternalConnect()
{
  LOG_TRACE("vtkPlusDummyTracker::InternalConnect");
  
  // nothing to do to connect

  return PLUS_SUCCESS;
}

//----------------------------------------------------------------------------
PlusStatus vtkPlusDummyTracker::InternalDisconnect()
{
  LOG_TRACE("vtkPlusDummyTracker::InternalDisconnect");

  // nothing to do to disconnect

  return PLUS_SUCCESS;
}

//----------------------------------------------------------------------------
PlusStatus vtkPlusDummyTracker::InternalStartRecording()
{
  LOG_TRACE("vtkPlusDummyTracker::InternalStartRecording");

  // nothing to do to start recording

  return PLUS_SUCCESS;
}

//----------------------------------------------------------------------------
PlusStatus vtkPlusDummyTracker::InternalStopRecording()
{
  LOG_TRACE("vtkPlusDummyTracker::InternalStopRecording");

  // nothing to do to stop recording

  return PLUS_SUCCESS;
}

//----------------------------------------------------------------------------
PlusStatus vtkPlusDummyTracker::InternalUpdate()
{
  LOG_TRACE("vtkPlusDummyTracker::InternalUpdate");
  const double unfilteredTimestamp = vtkIGSIOAccurateTimer::GetSystemTime();

  // create dummy transform
  vtkSmartPointer<vtkMatrix4x4> transform = vtkSmartPointer<vtkMatrix4x4>::New();
  transform->Identity();
  transform->SetElement(0, 3, 1);
  transform->SetElement(1, 3, 2);
  transform->SetElement(2, 3, 3);

  // create metadata
  igsioFieldMapType customFields;
  customFields["SampleMeta"] = std::make_pair(igsioFrameFieldFlags::FRAMEFIELD_FORCE_SERVER_SEND, "Example meta value");

  // add to PLUS buffers
  this->ToolTimeStampedUpdateWithoutFiltering("DummyToTracker", transform, TOOL_OK, this->FrameNumber, unfilteredTimestamp, &customFields);

  this->FrameNumber++;
  return PLUS_SUCCESS;
}
