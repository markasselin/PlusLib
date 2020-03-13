/*=Plus=header=begin======================================================
Program: Plus
Copyright (c) Laboratory for Percutaneous Surgery. All rights reserved.
See License.txt for details.
=========================================================Plus=header=end*/

#include "PlusConfigure.h"
#include "vtkPlusFrankaCommand.h"
#include "vtkPlusFrankaRobot.h"

#include "vtkPlusDataCollector.h"
#include "vtkObjectFactory.h"
#include "vtkPlusChannel.h"
#include "vtkPlusDevice.h"

#include <vtkVariant.h>
#include <vtkSmartPointer.h>

#include <iterator>
#include <limits>
#include <map>
#include <string>

namespace
{
  static const std::string FRANKA_CMD_NAME = "FrankaCommand";
}

vtkStandardNewMacro(vtkPlusFrankaCommand);

//----------------------------------------------------------------------------
vtkPlusFrankaCommand::vtkPlusFrankaCommand()
{
  this->FrankaDeviceId = "";
}

//----------------------------------------------------------------------------
vtkPlusFrankaCommand::~vtkPlusFrankaCommand()
{
}

//----------------------------------------------------------------------------
void vtkPlusFrankaCommand::SetNameToSetUsParameter()
{
  SetName(FRANKA_CMD_NAME);
}

//----------------------------------------------------------------------------
void vtkPlusFrankaCommand::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
}

//----------------------------------------------------------------------------
void vtkPlusFrankaCommand::GetCommandNames(std::list<std::string>& cmdNames)
{
  cmdNames.clear();
  cmdNames.push_back(FRANKA_CMD_NAME);
}

//----------------------------------------------------------------------------
std::string vtkPlusFrankaCommand::GetDescription(const std::string& commandName)
{
  std::string desc;
  if (commandName.empty() || igsioCommon::IsEqualInsensitive(commandName, FRANKA_CMD_NAME))
  {
    desc += FRANKA_CMD_NAME;
    desc += ": Send command to Franka device.";
  }

  return desc;
}

//----------------------------------------------------------------------------
PlusStatus vtkPlusFrankaCommand::ReadConfiguration(vtkXMLDataElement* aConfig)
{
  if (vtkPlusCommand::ReadConfiguration(aConfig) != PLUS_SUCCESS)
  {
    return PLUS_FAIL;
  }

  this->SetFrankaDeviceId(aConfig->GetAttribute("DeviceId"));

  // Parse nested elements and store requested parameter changes
  for (int elemIndex = 0; elemIndex < aConfig->GetNumberOfNestedElements(); ++elemIndex)
  {
    vtkXMLDataElement* currentElem = aConfig->GetNestedElement(elemIndex);
    // if (igsioCommon::IsEqualInsensitive(currentElem->GetName(), vtkPlusAtracsysTracker::ATRACSYS_COMMAND_SET_FLAG))
    // {
    //   const char* parameterName = currentElem->GetAttribute("Name");
    //   const char* parameterValue = currentElem->GetAttribute("Value");

    //   if (!parameterName || !parameterValue)
    //   {
    //     LOG_ERROR("Unable to find required Name or Value attribute in " << (currentElem->GetName() ? currentElem->GetName() : "(undefined)") << " element in Atracsys SetFlag command");
    //     continue;
    //   }

    //   this->CommandList[parameterName] = parameterValue;
    // }
    // else
    // {
    //   // invalid command
    //   LOG_ERROR("Invalid command name (" << currentElem->GetName() << ") provided to AtracsysCommand.")
    // }
  }

  return PLUS_SUCCESS;
}

//----------------------------------------------------------------------------
PlusStatus vtkPlusFrankaCommand::WriteConfiguration(vtkXMLDataElement* aConfig)
{
  if (vtkPlusCommand::WriteConfiguration(aConfig) != PLUS_SUCCESS)
  {
    return PLUS_FAIL;
  }

  XML_WRITE_STRING_ATTRIBUTE_IF_NOT_EMPTY(FrankaDeviceId, aConfig);

  // Write parameters as nested elements
  std::map<std::string, std::string>::iterator commandIt;
  for (commandIt = this->CommandList.begin(); commandIt != this->CommandList.end(); commandIt++)
  {
    // if (igsioCommon::IsEqualInsensitive(commandIt->first, vtkPlusAtracsysTracker::ATRACSYS_COMMAND_SET_LED_RGBF))
    // {
    //   // write SetLED element
    //   vtkSmartPointer<vtkXMLDataElement> ledElem = vtkSmartPointer<vtkXMLDataElement>::New();
    //   ledElem->SetName(vtkPlusAtracsysTracker::ATRACSYS_COMMAND_SET_LED_RGBF);
    //   ledElem->SetAttribute("Red", std::to_string(this->LedR).c_str());
    //   ledElem->SetAttribute("Green", std::to_string(this->LedG).c_str());
    //   ledElem->SetAttribute("Blue", std::to_string(this->LedB).c_str());
    //   ledElem->SetAttribute("Frequency", std::to_string(this->LedFreq).c_str());
    //   aConfig->AddNestedElement(ledElem);
    // }
    // else
    // {
    //   // write SetFlag element
    //   vtkSmartPointer<vtkXMLDataElement> flagElem = vtkSmartPointer<vtkXMLDataElement>::New();
    //   flagElem->SetName(vtkPlusAtracsysTracker::ATRACSYS_COMMAND_SET_FLAG);
    //   flagElem->SetAttribute("Name", commandIt->first.c_str());
    //   flagElem->SetAttribute("Value", commandIt->second.c_str());
    //   aConfig->AddNestedElement(flagElem);
    // }
  }

  return PLUS_SUCCESS;
}

PlusStatus vtkPlusFrankaCommand::StringToBool(std::string strVal, bool& boolVal)
{
  if (igsioCommon::IsEqualInsensitive(strVal, "TRUE"))
  {
    boolVal = true;
    return PLUS_SUCCESS;
  }
  else if (igsioCommon::IsEqualInsensitive(strVal, "FALSE"))
  {
    boolVal = false;
    return PLUS_SUCCESS;
  }
  return PLUS_FAIL;
}

//----------------------------------------------------------------------------
PlusStatus vtkPlusFrankaCommand::Execute()
{
  // LOG_DEBUG("vtkPlusFrankaCommand::Execute: " << (!this->Name.empty() ? this->Name : "(undefined)")
  //           << ", device: " << (this->AtracsysDeviceId.empty() ? "(undefined)" : this->AtracsysDeviceId));

  // if (this->Name.empty())
  // {
  //   this->QueueCommandResponse(PLUS_FAIL, "Command failed. See error message.", "No command name specified.");
  //   return PLUS_FAIL;
  // }
  // else if (!igsioCommon::IsEqualInsensitive(this->Name, ATRACSYS_CMD_NAME))
  // {
  //   this->QueueCommandResponse(PLUS_FAIL, "Command failed. See error message.", "Unknown command name: " + this->Name + ".");
  //   return PLUS_FAIL;
  // }

  // vtkPlusAtracsysTracker* atracsysDevice = GetAtracsysDevice();
  // if (atracsysDevice == NULL)
  // {
  //   this->QueueCommandResponse(PLUS_FAIL, "Command failed. See error message.", std::string("Device ")
  //                              + (this->AtracsysDeviceId.empty() ? "(undefined)" : this->AtracsysDeviceId) + std::string(" is not found."));
  //   return PLUS_FAIL;
  // }

  // std::string atracsysDeviceId = (atracsysDevice->GetDeviceId().empty() ? "(unknown)" : atracsysDevice->GetDeviceId());
  // std::string resultString = "<CommandReply>";
  // std::string error = "";
  // std::map <std::string, std::pair<IANA_ENCODING_TYPE, std::string> > metaData;
  // PlusStatus status = PLUS_SUCCESS;

  // std::map<std::string, std::string>::iterator commandIt;
  // for (commandIt = this->CommandList.begin(); commandIt != this->CommandList.end(); commandIt++)
  // {
  //   std::string commandName = commandIt->first;
  //   std::string value = commandIt->second;
  //   resultString += "<Parameter Name=\"" + commandName + "\"";

  //   PlusStatus conversion = PLUS_SUCCESS;
  //   PlusStatus setInDevice = PLUS_SUCCESS;

  //   if (commandName == vtkPlusAtracsysTracker::ATRACSYS_COMMAND_LED_ENABLED)
  //   {
  //     bool enabled;
  //     conversion = this->StringToBool(value, enabled);
  //     setInDevice = atracsysDevice->SetLedEnabled(enabled);
  //   }
  //   else if (commandName == vtkPlusAtracsysTracker::ATRACSYS_COMMAND_LASER_ENABLED)
  //   {
  //     bool enabled;
  //     conversion = this->StringToBool(value, enabled);
  //     setInDevice = atracsysDevice->SetLaserEnabled(enabled);
  //   }
  //   else if (commandName == vtkPlusAtracsysTracker::ATRACSYS_COMMAND_VIDEO_ENABLED)
  //   {
  //     bool enabled;
  //     conversion = this->StringToBool(value, enabled);
  //     setInDevice = atracsysDevice->SetVideoEnabled(enabled);
  //   }
  //   else if (commandName == vtkPlusAtracsysTracker::ATRACSYS_COMMAND_SET_LED_RGBF)
  //   {
  //     // value is a placeholder, we can ignore it
  //     setInDevice = atracsysDevice->SetUserLEDState(this->LedR, this->LedG, this->LedB, this->LedFreq, true);
  //   }
  //   else if (commandName == vtkPlusAtracsysTracker::ATRACSYS_COMMAND_ENABLE_TOOL)
  //   {
  //     // value contains ToolId to enable / disable
  //     std::string strEnabled = this->EnableDisableTools.find(value)->second;
  //     bool boolEnabled;
  //     conversion = this->StringToBool(strEnabled, boolEnabled);
  //     setInDevice = atracsysDevice->SetToolEnabled(value, boolEnabled);
  //   }
  //   else if (commandName == vtkPlusAtracsysTracker::ATRACSYS_COMMAND_ADD_TOOL)
  //   {
  //     // value contains ToolId of geometry to add
  //     std::string geometry = this->Markers.find(value)->second;
  //     setInDevice = atracsysDevice->AddToolGeometry(value, geometry);
  //   }
  //   else
  //   {
  //     LOG_WARNING("Unrecognized AtracsysCommand recieved with name: " << commandName
  //       << ". Please see the documentation for a list of available commands.");
  //     return PLUS_FAIL;
  //   }

  //   if (conversion == PLUS_FAIL || setInDevice == PLUS_FAIL)
  //   {
  //     status = PLUS_FAIL;
  //   }

  //   resultString += " Success=\"true\"/>";
  //   metaData[commandName] = std::make_pair(IANA_TYPE_US_ASCII, "SUCCESS");
  // }
  // resultString += "</CommandReply>";
  
  // vtkSmartPointer<vtkPlusCommandRTSCommandResponse> commandResponse = vtkSmartPointer<vtkPlusCommandRTSCommandResponse>::New();
  // commandResponse->UseDefaultFormatOff();
  // commandResponse->SetClientId(this->ClientId);
  // commandResponse->SetOriginalId(this->Id);
  // commandResponse->SetDeviceName(this->DeviceName);
  // commandResponse->SetCommandName(this->GetName());
  // commandResponse->SetStatus(status);
  // commandResponse->SetRespondWithCommandMessage(this->RespondWithCommandMessage);
  // commandResponse->SetErrorString(error);
  // commandResponse->SetResultString(resultString);
  // commandResponse->SetParameters(metaData);
  // this->CommandResponseQueue.push_back(commandResponse);

  //return status;
  return PLUS_FAIL;
}

//----------------------------------------------------------------------------
vtkPlusFrankaRobot* vtkPlusFrankaCommand::GetFrankaDevice()
{
  // vtkPlusDataCollector* dataCollector = GetDataCollector();
  // if (dataCollector == NULL)
  // {
  //   LOG_ERROR("Data collector is invalid");
  //   return NULL;
  // }
  // if (!this->AtracsysDeviceId.empty())
  // {
  //   // Reconstructor device ID is specified
  //   vtkPlusDevice* device = NULL;
  //   if (dataCollector->GetDevice(device, this->AtracsysDeviceId) != PLUS_SUCCESS)
  //   {
  //     LOG_ERROR("No Atracsys device has been found by the name " << this->AtracsysDeviceId);
  //     return NULL;
  //   }
  //   // device found
  //   vtkPlusAtracsysTracker* atracsysDevice = vtkPlusAtracsysTracker::SafeDownCast(device);
  //   if (atracsysDevice == NULL)
  //   {
  //     // wrong type
  //     LOG_ERROR("The specified device " << this->AtracsysDeviceId << " is not an Atracsys device");
  //     return NULL;
  //   }
  //   return atracsysDevice;
  // }
  // else
  // {
  //   // No Atracsys device id is specified, auto-detect the first one and use that
  //   for (DeviceCollectionConstIterator it = dataCollector->GetDeviceConstIteratorBegin(); it != dataCollector->GetDeviceConstIteratorEnd(); ++it)
  //   {
  //     vtkPlusAtracsysTracker* atracsysDevice = vtkPlusAtracsysTracker::SafeDownCast(*it);
  //     if (atracsysDevice != NULL)
  //     {
  //       // found an Atracsys device
  //       this->SetAtracsysDeviceId(atracsysDevice->GetDeviceId());
  //       return atracsysDevice;
  //     }
  //   }
  //   LOG_ERROR("No Atracsys device has been found");
  //   return NULL;
  // }
  return NULL;
}
