/*=Plus=header=begin======================================================
Program: Plus
Copyright (c) Laboratory for Percutaneous Surgery. All rights reserved.
See License.txt for details.
=========================================================Plus=header=end*/

#ifndef __vtkPlusDummyTracker_h
#define __vtkPlusDummyTracker_h

#include "vtkPlusDataCollectionExport.h"
#include "vtkPlusDevice.h"

/*!
\class vtkPlusDummyTracker
\brief Simulated PLUS tracker for testing
sending transform metadata. Requires 
PLUS_USE_DUMMY_TRACKER option in CMake.
\ingroup PlusLibDataCollection
*/
class vtkPlusDataCollectionExport vtkPlusDummyTracker : public vtkPlusDevice
{
public:
  static vtkPlusDummyTracker *New();
  vtkTypeMacro(vtkPlusDummyTracker, vtkPlusDevice);
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

  /*! Probe to see if the tracking system is present. */
  PlusStatus Probe();

  /*! Start the tracking system. */
  PlusStatus InternalStartRecording();

  /*! Stop the tracking system and bring it back to its initial state. */
  PlusStatus InternalStopRecording();

  /*!  */
  PlusStatus InternalUpdate();

protected:
  vtkPlusDummyTracker();
  ~vtkPlusDummyTracker();

private:
  vtkPlusDummyTracker(const vtkPlusDummyTracker&);
  void operator=(const vtkPlusDummyTracker&);

};

#endif
