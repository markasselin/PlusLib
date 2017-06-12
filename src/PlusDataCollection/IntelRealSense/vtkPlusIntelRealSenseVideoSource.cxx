/*=Plus=header=begin======================================================
Program: Plus
Copyright (c) Laboratory for Percutaneous Surgery. All rights reserved.
See License.txt for details.
=========================================================Plus=header=end*/
//Marker Location
//C:\devel\PlusExp-bin\PlusLibData\ConfigFiles\IntelRealSenseToolDefinitions
#include "PlusConfigure.h"
#include "vtkPlusIntelRealSenseVideoSource.h"

#include "PlusVideoFrame.h"
#include "vtkImageData.h"
#include "vtkImageImport.h"
#include "vtkMath.h"
#include "vtkMatrix4x4.h"
#include "vtkObjectFactory.h"
#include "vtkPlusDataSource.h"

#include <fstream>
#include <iostream>
#include <set>

#include "pxcsession.h"
#include "pxcsensemanager.h"
#include "pxcprojection.h"

//TODO: What is this?
// From FF_ObjectTracking sample
#define ID_DEVICEX   21000

vtkStandardNewMacro(vtkPlusIntelRealSenseVideoSource);

void StringToWString(std::wstring &ws, const std::string &s)
{
  std::wstring wsTmp(s.begin(), s.end());
  ws = wsTmp;
}

class vtkPlusIntelRealSenseVideoSource::vtkInternal
{
public:
  vtkPlusIntelRealSenseVideoSource *External;

  PXCSession* Session;

  PXCSenseManager* SenseMgr;
  PXCCaptureManager* CaptureMgr;
  PXCProjection* Projection;

  pxcCHAR File[1024];
  pxcCHAR CalibrationFile[1024];

  bool GetDeviceInfo(int deviceIndex, PXCCapture::DeviceInfo& dinfo)
  {
    PXCSession::ImplDesc desc;
    memset(&desc, 0, sizeof(desc));
    desc.group = PXCSession::IMPL_GROUP_SENSOR;
    desc.subgroup = PXCSession::IMPL_SUBGROUP_VIDEO_CAPTURE;
    for (int i = 0, k = ID_DEVICEX;; i++)
    {
      PXCSession::ImplDesc desc1;
      if (this->Session->QueryImpl(&desc, i, &desc1) < PXC_STATUS_NO_ERROR)
      {
        break;
      }
      PXCCapture *capture;
      if (this->Session->CreateImpl<PXCCapture>(&desc1, &capture) < PXC_STATUS_NO_ERROR)
      {
        continue;
      }
      for (int j = 0;; j++)
      {
        if (capture->QueryDeviceInfo(j, &dinfo) < PXC_STATUS_NO_ERROR) break;
        if (dinfo.orientation == PXCCapture::DEVICE_ORIENTATION_REAR_FACING) break;
        if (j == deviceIndex)
        {
          return true;
        }
      }
    }
    return false;
  }


  vtkInternal(vtkPlusIntelRealSenseVideoSource* external)
    : External(external)
    , Session(NULL)
    , SenseMgr(NULL)
    , CaptureMgr(NULL)
    , Projection(NULL)
  {
  }

  virtual ~vtkInternal()
  {
  }
};

//----------------------------------------------------------------------------
vtkPlusIntelRealSenseVideoSource::vtkPlusIntelRealSenseVideoSource()
  : Internal(new vtkInternal(this))
{
#ifdef USE_INTELREALSENSE_TIMESTAMPS
  this->TrackerTimeToSystemTimeSec = 0;
  this->TrackerTimeToSystemTimeComputed = false;
#endif

  // for accurate timing
  this->FrameNumber = 0;
  
  // No callback function provided by the device, so the data capture thread will be used to poll the hardware and add new items to the buffer
  this->StartThreadForInternalUpdates=true;
  this->AcquisitionRate = 20;
  
  this->DeviceName = "Intel(R) RealSense(TM) 3D Camera SR300";
}

//----------------------------------------------------------------------------
vtkPlusIntelRealSenseVideoSource::~vtkPlusIntelRealSenseVideoSource()
{

}

//----------------------------------------------------------------------------
std::string vtkPlusIntelRealSenseVideoSource::GetSdkVersion()
{
	return "";
}

//----------------------------------------------------------------------------
PlusStatus vtkPlusIntelRealSenseVideoSource::Probe()
{  
  //TODO: run calibration from file here
  return PLUS_SUCCESS;
} 

//----------------------------------------------------------------------------
PlusStatus vtkPlusIntelRealSenseVideoSource::InternalStartRecording()
{
  return PLUS_SUCCESS;
}

//----------------------------------------------------------------------------
PlusStatus vtkPlusIntelRealSenseVideoSource::InternalStopRecording()
{
  // No need to do anything here, as the IntelRealSenseVideoSource only performs grabbing on request
  return PLUS_SUCCESS;
}

//----------------------------------------------------------------------------
PlusStatus vtkPlusIntelRealSenseVideoSource::InternalUpdate()
{
  // Generate a frame number, as the tool does not provide a frame number.
  // FrameNumber will be used in ToolTimeStampedUpdate for timestamp filtering
  ++this->FrameNumber;

  // Setting the timestamp
  const double unfilteredTimestamp = vtkPlusAccurateTimer::GetSystemTime();

#ifdef USE_INTELREALSENSE_TIMESTAMPS
  if (!this->TrackerTimeToSystemTimeComputed)
  {
    const double timeSystemSec = unfilteredTimestamp;
    const double timeTrackerSec = this->MT->mtGetLatestFrameTime();
    this->TrackerTimeToSystemTimeSec = timeSystemSec-timeTrackerSec;
    this->TrackerTimeToSystemTimeComputed = true;
  }
  const double timeTrackerSec = this->MT->mtGetLatestFrameTime();
  const double timeSystemSec = timeTrackerSec + this->TrackerTimeToSystemTimeSec;        
#endif

  if (this->Internal->SenseMgr->AcquireFrame(true) < PXC_STATUS_NO_ERROR)
  {
    LOG_ERROR("AcquireFrame failed");
    return PLUS_FAIL;
  }


  /* Display Results */
  const PXCCapture::Sample *sample = this->Internal->SenseMgr->QueryTrackerSample();
  if (sample)
  {
    LOG_DEBUG("Sample found!")
  }

  this->Internal->SenseMgr->ReleaseFrame();

  return PLUS_SUCCESS;
}

//----------------------------------------------------------------------------
//TODO: Why is there a mutex lock here?
PlusStatus vtkPlusIntelRealSenseVideoSource::GetImage(vtkImageData* leftImage, vtkImageData* rightImage)
{
  PlusLockGuard<vtkPlusRecursiveCriticalSection> updateMutexGuardedLock(this->UpdateMutex);
  return PLUS_SUCCESS;
}

//----------------------------------------------------------------------------
PlusStatus vtkPlusIntelRealSenseVideoSource::ReadConfiguration( vtkXMLDataElement* rootConfigElement )
{
  XML_FIND_DEVICE_ELEMENT_REQUIRED_FOR_READING(deviceConfig, rootConfigElement);
  XML_READ_CSTRING_ATTRIBUTE_OPTIONAL(CameraCalibrationFile, deviceConfig);
  XML_READ_CSTRING_ATTRIBUTE_OPTIONAL(DeviceName, deviceConfig);

  return PLUS_SUCCESS;
}

//----------------------------------------------------------------------------
PlusStatus vtkPlusIntelRealSenseVideoSource::WriteConfiguration(vtkXMLDataElement* rootConfigElement)
{
  XML_FIND_DEVICE_ELEMENT_REQUIRED_FOR_WRITING(trackerConfig, rootConfigElement);

  return PLUS_SUCCESS;
} 

//----------------------------------------------------------------------------
PlusStatus vtkPlusIntelRealSenseVideoSource::InternalConnect()
{ 
  this->Internal->Session = PXCSession::CreateInstance();
  if (!this->Internal->Session)
  {
    LOG_ERROR("Failed to create a RealSense SDK Session");
    return PLUS_FAIL;
  }
  
  this->Internal->SenseMgr = this->Internal->Session->CreateSenseManager();
  if (!this->Internal->SenseMgr)
  {
    LOG_ERROR("Failed to create a RealSense SDK SenseManager");
    return PLUS_FAIL;
  }
  
  PXCSession *session = this->Internal->SenseMgr->QuerySession();

  this->Internal->CaptureMgr = this->Internal->SenseMgr->QueryCaptureManager(); //no need to Release it is released with senseMgr
  // Live streaming
  PXCCapture::DeviceInfo dinfo;
  this->Internal->GetDeviceInfo(0, dinfo);
  pxcCHAR* device = dinfo.name;
  this->Internal->CaptureMgr->FilterByDeviceInfo(device, 0, 0);
  bool stsFlag = true;



  //if (!this->CameraCalibrationFile.empty())
  //{
  //  std::string cameraCalibrationFilePath = vtkPlusConfig::GetInstance()->GetDeviceSetConfigurationPath(this->CameraCalibrationFile);
  //  LOG_DEBUG("Use IntelRealSenseVideoSource ini file: " << cameraCalibrationFilePath);
  //  if (!vtksys::SystemTools::FileExists(cameraCalibrationFilePath.c_str(), true))
  //  {
  //    LOG_WARNING("Unable to find IntelRealSenseVideoSource camera calibration file at: " << cameraCalibrationFilePath);
  //  }
  //  std::wstring cameraCalibrationFileW;
  //  StringToWString(cameraCalibrationFileW, cameraCalibrationFilePath);
  //  // camera calib load was here
  //}
  LOG_ERROR("PXC INIT: " << this->Internal->SenseMgr->Init());
  if (this->Internal->SenseMgr->Init() < PXC_STATUS_NO_ERROR)
  {
    LOG_ERROR("senseMgr->Init failed");
    return PLUS_FAIL;
  }

  return PLUS_SUCCESS;
}

//----------------------------------------------------------------------------
PlusStatus vtkPlusIntelRealSenseVideoSource::InternalDisconnect()
{ 
  this->Internal->SenseMgr->Close();
  this->Internal->SenseMgr->Release();
  this->IsTrackingInitialized=false;
  return PLUS_SUCCESS;
}
