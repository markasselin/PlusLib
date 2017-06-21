/*=Plus=header=begin======================================================
  Progra  : Plus
  Copyright (c) Laboratory for Percutaneous Surgery. All rights reserved.
  See License.txt for details.
=========================================================Plus=header=end*/

#ifndef __vtkPlusIntelRealSenseVideoSource_h
#define __vtkPlusIntelRealSenseVideoSource_h

#include "vtkPlusDataCollectionExport.h"
#include "vtkPlusDevice.h"



/*!
  \class vtkPlusIntelRealSenseVideoSource
  \brief Interface class to Intel RealSense cameras
  \ingroup PlusLibDataCollection
*/
class vtkPlusDataCollectionExport vtkPlusIntelRealSenseVideoSource : public vtkPlusDevice
{
public:
  /*! Defines whether or not depth stream is used. */
  enum OUTPUT_VIDEO_TYPE
  {
    OPTICAL,
    OPTICAL_AND_DEPTH
  };

  static vtkPlusIntelRealSenseVideoSource *New();
  vtkTypeMacro(vtkPlusIntelRealSenseVideoSource,vtkPlusDevice);

  /*! Hardware device SDK version. */
  virtual std::string GetSdkVersion(); 
 
  virtual bool IsTracker() const { return true; }

  /*!
    Probe to see if the tracking system is present.
  */
  PlusStatus Probe();

  /*!
    Get an update from the tracking system and push the new transforms
    to the tools.  This should only be used within vtkTracker.cxx.
  */
  PlusStatus InternalUpdate(); 
 
  /*! Read IntelRealSenseVideoSource configuration and update the tracker settings accordingly */
  virtual PlusStatus ReadConfiguration( vtkXMLDataElement* config );

  /*! Write current IntelRealSenseVideoSource configuration settings to XML */
  virtual PlusStatus WriteConfiguration(vtkXMLDataElement* rootConfigElement);

  /*! Connect to the tracker hardware */
  PlusStatus InternalConnect();
  /*! Disconnect from the tracker hardware */
  PlusStatus InternalDisconnect();

  //vtkSetMacro(CameraCalibrationFile, std::string);
  //vtkGetMacro(CameraCalibrationFile, std::string);

  //vtkSetMacro(DeviceName, std::string);
  //vtkGetMacro(DeviceName, std::string);

protected:
  vtkPlusIntelRealSenseVideoSource();
  ~vtkPlusIntelRealSenseVideoSource();

  class vtkInternal;
  vtkInternal* Internal;

  /*!
    Start the tracking system.  The tracking system is brought from
    its ground state into full tracking mode.  The POLARIS will
    only be reset if communication cannot be established without
    a reset.
  */
  PlusStatus InternalStartRecording();

  /*! Stop the tracking system and bring it back to its initial state. */
  PlusStatus InternalStopRecording();

  void GeneratePolyData(vtkSmartPointer<vtkPolyData>, unsigned char * image);

  /*! Non-zero if the tracker has been initialized */
  int IsTrackingInitialized;

  
  long FrameNumber;
  //std::string CameraCalibrationFile;
  //std::string DeviceName;

  vtkPlusDataSource *output;

private:
  vtkPlusIntelRealSenseVideoSource(const vtkPlusIntelRealSenseVideoSource&);
  void operator=(const vtkPlusIntelRealSenseVideoSource&);
  
};

#endif
