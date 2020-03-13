/*=Plus=header=begin======================================================
  Program: Plus
  Copyright (c) Laboratory for Percutaneous Surgery. All rights reserved.
  See License.txt for details.
=========================================================Plus=header=end*/

#ifndef __vtkPlusFrankaCommand_h
#define __vtkPlusFrankaCommand_h

#include "vtkPlusServerExport.h"
#include "vtkPlusCommand.h"

class vtkPlusFrankaRobot;

/*!
  \class vtkPlusFrankaCommand
  \brief This command allows OpenIGTLink commands to operate Franka Emika
  robots. See the documentation for command specifics.
  \ingroup PlusLibPlusServer
 */
class vtkPlusServerExport vtkPlusFrankaCommand : public vtkPlusCommand
{
public:

  static vtkPlusFrankaCommand* New();
  vtkTypeMacro(vtkPlusFrankaCommand, vtkPlusCommand);
  virtual void PrintSelf(ostream& os, vtkIndent indent);
  virtual vtkPlusCommand* Clone() { return New(); }

  /*! Executes the command  */
  virtual PlusStatus Execute();

  /*! Read command parameters from XML */
  virtual PlusStatus ReadConfiguration(vtkXMLDataElement* aConfig);

  /*! Write command parameters to XML */
  virtual PlusStatus WriteConfiguration(vtkXMLDataElement* aConfig);

  /*! Get all the command names that this class can execute */
  virtual void GetCommandNames(std::list<std::string>& cmdNames);

  /*! Gets the description for the specified command name. */
  virtual std::string GetDescription(const std::string& commandName);

  /*! Id of the ultrasound device to change the parameters of at the next Execute */
  vtkGetStdStringMacro(FrankaDeviceId);
  vtkSetStdStringMacro(FrankaDeviceId);

  void SetNameToSetUsParameter();

protected:
  vtkPlusFrankaRobot* GetFrankaDevice();

  vtkPlusFrankaCommand();
  virtual ~vtkPlusFrankaCommand();

protected:
  std::string FrankaDeviceId;

  // list of commands to execute
  std::map<std::string, std::string> CommandList;

  // helper to convert string to boolean
  PlusStatus StringToBool(std::string strVal, bool& boolVal);

  vtkPlusFrankaCommand(const vtkPlusFrankaCommand&);
  void operator=(const vtkPlusFrankaCommand&);
};

#endif
