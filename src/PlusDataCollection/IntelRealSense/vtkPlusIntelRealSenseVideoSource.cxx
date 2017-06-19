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
#include "PixelCodec.h"

#include <fstream>
#include <iostream>
#include <set>
#include <typeinfo>
#include "RealSense\Session.h"
#include "RealSense\SenseManager.h"
#include "RealSense\SampleReader.h"
#include "RealSense\Projection.h"

#define DEFAULT_FRAME_RATE            30
#define SR300_MAX_FRAME_RATE          30    // for optical && depth
#define SR300_MAX_OPTICAL_WIDTH       1920
#define SR300_MAX_OPTICAL_HEIGHT      1080
#define SR300_DEFAULT_OPTICAL_WIDTH   640
#define SR300_DEFAULT_OPTICAL_HEIGHT  480
#define SR300_MAX_DEPTH_WIDTH         640
#define SR300_MAX_DEPTH_HEIGHT        480
#define SR300_DEFAULT_DEPTH_WIDTH     640
#define SR300_DEFAULT_DEPTH_HEIGHT    480

namespace RS = Intel::RealSense;

vtkStandardNewMacro(vtkPlusIntelRealSenseVideoSource);

class vtkPlusIntelRealSenseVideoSource::vtkInternal
{
public:
  vtkPlusIntelRealSenseVideoSource *External;

  struct RSVideoFormat
  {
    // width, height, depth
    int FrameSizePx[3] = { 0, 0, 1 };
    RS::Image::PixelFormat RSPixelFormat;
  };

  struct RSVideoFrame
  {
    RS::ImageData RSImageData;
    PlusVideoFrame PlusFrame;
    vtkPlusDataSource* aSource;
  };

  OUTPUT_VIDEO_TYPE OutputVideoType;
  
  RSVideoFormat ColorFormat;
  RSVideoFrame ColorFrame;

  RSVideoFormat DepthFormat;
  RSVideoFrame DepthFrame;

  // global Intel RealSense objects
  RS::Session* Session;
  RS::SenseManager* SenseManager;
  RS::Projection* Projection;

  vtkInternal(vtkPlusIntelRealSenseVideoSource* external)
    : External(external)
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
  this->RequireImageOrientationInConfiguration = true;

  // for accurate timing
  this->FrameNumber = 0;
  
  // No callback function provided by the device, so the data capture thread will be used to poll the hardware and add new items to the buffer
  this->StartThreadForInternalUpdates=true;
  this->AcquisitionRate = 20;
  
  this->DeviceName = "Intel RealSense 3D Camera SR300";
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
  //TODO: Use projection to map optical->depth (or vice versa)
  // wait until both optical and depth frames are ready
  if (this->Internal->SenseManager->AcquireFrame(true) < RS::Status::STATUS_NO_ERROR)
  {
    return PLUS_SUCCESS;
  }

  // get frame from RSSDK
  RS::Capture::Sample *sample = this->Internal->SenseManager->QuerySample();

  // get updated system time 
  const double unfilteredTimestamp = vtkPlusAccurateTimer::GetSystemTime();

  // Update optical frame and add to buffer
  //TODO: Re-implement PlusStatus functionality on the buffer frame additions so that update fails if image cannot be added
  if (this->Internal->OutputVideoType == OPTICAL || this->Internal->OutputVideoType == OPTICAL_AND_DEPTH)
  {
    sample->color->AcquireAccess(RS::Image::ACCESS_READ,
      RS::Image::PixelFormat::PIXEL_FORMAT_RGB24,
      &this->Internal->ColorFrame.RSImageData);

    PixelCodec::ConvertToBmp24(PixelCodec::ComponentOrder_RGB,
      PixelCodec::PixelEncoding_BGR24,
      this->Internal->ColorFormat.FrameSizePx[0],
      this->Internal->ColorFormat.FrameSizePx[1],
      this->Internal->ColorFrame.RSImageData.planes[0],
      (unsigned char*)this->Internal->ColorFrame.PlusFrame.GetScalarPointer());

    this->Internal->ColorFrame.aSource->AddItem(&this->Internal->ColorFrame.PlusFrame,
      (long)this->FrameNumber, unfilteredTimestamp);
  }

  // update depth frame and add to buffer
  if (this->Internal->OutputVideoType == OPTICAL_AND_DEPTH)
  {
  }


  this->Modified();
  this->FrameNumber++;
  LOG_INFO("FRAME: " << this->FrameNumber);

  this->Internal->SenseManager->ReleaseFrame();

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

  XML_FIND_NESTED_ELEMENT_REQUIRED(dataSourcesElement, deviceConfig, "DataSources");

  // set OutputVideoType based on number of DataSource elements
  if ((int)dataSourcesElement->GetNumberOfNestedElements() == 1)
  {
    this->Internal->OutputVideoType = OPTICAL;
  }
  else if ((int)dataSourcesElement->GetNumberOfNestedElements() == 2)
  {
    this->Internal->OutputVideoType = OPTICAL_AND_DEPTH;
  }
  else
  {
    LOG_ERROR("IntelRealSenseVideo must have 1 (Id=OPTICAL) or 2 (Id=OPTICAL and Id=DEPTH) datasource elements only.");
    return PLUS_FAIL;
  }

  for (int nestedElementIndex = 0; nestedElementIndex < dataSourcesElement->GetNumberOfNestedElements(); nestedElementIndex++)
  {
    vtkXMLDataElement* videoDataElement = dataSourcesElement->GetNestedElement(nestedElementIndex);
    if (STRCASECMP(videoDataElement->GetName(), "DataSource") != 0)
    {
      // if this is not a data source element, skip it
      continue;
    }
    if (videoDataElement->GetAttribute("Type") != NULL && STRCASECMP(videoDataElement->GetAttribute("Type"), "Video") != 0)
    {
      // if this is not a Video element, skip it
      continue;
    }

    const char* videoId = videoDataElement->GetAttribute("Id");
    if (videoId == NULL)
    {
      // tool doesn't have ID needed to generate transform
      LOG_ERROR("Failed to initialize OpticalMarkerTracking tool: DataSource Id is missing");
      continue;
    }

    int frameSizePxTemp[2];
    if (!videoDataElement->GetVectorAttribute("FrameSize", 2, frameSizePxTemp))
    {
      LOG_ERROR("Frame size must be provided");
      continue;
    }
    if (STRCASECMP(videoId, "Optical")==0)
    {
      // this is the optical stream
      // check and set width/height parameters
      if (frameSizePxTemp[0] > 0 && frameSizePxTemp[0] <= SR300_MAX_OPTICAL_WIDTH)
      {
        this->Internal->ColorFormat.FrameSizePx[0] = frameSizePxTemp[0];
      }
      else
      {
        LOG_WARNING("Optical frame width exceeds camera maximum of " << SR300_MAX_OPTICAL_WIDTH
          << ". Using default of "
          << SR300_DEFAULT_OPTICAL_WIDTH)
        this->Internal->ColorFormat.FrameSizePx[0] = SR300_DEFAULT_OPTICAL_WIDTH;
      }
      if (frameSizePxTemp[1] > 0 && frameSizePxTemp[1] <= SR300_MAX_OPTICAL_HEIGHT)
      {
        this->Internal->ColorFormat.FrameSizePx[1] = frameSizePxTemp[1];
      }
      else
      {
        LOG_WARNING("Optical frame height exceeds camera maximum of " << SR300_MAX_OPTICAL_HEIGHT
          << ". Using default of "
          << SR300_DEFAULT_OPTICAL_HEIGHT)
        this->Internal->ColorFormat.FrameSizePx[1] = SR300_DEFAULT_OPTICAL_HEIGHT;
      }
      // set RS PixelFormat
      this->Internal->ColorFormat.RSPixelFormat = RS::PixelFormat::PIXEL_FORMAT_RGB24;
      // log to user
      LOG_INFO("Using Optical Frame Size: " << this->Internal->ColorFormat.FrameSizePx[0]
        << "x"
        << this->Internal->ColorFormat.FrameSizePx[1]);
    }
    else if (STRCASECMP(videoId, "Depth")==0)
    {
      // this is the depth stream
      // check and set width/height parameters
      if (frameSizePxTemp[0] > 0 && frameSizePxTemp[0] <= SR300_MAX_DEPTH_WIDTH)
      {
        this->Internal->DepthFormat.FrameSizePx[0] = frameSizePxTemp[0];
      }
      else
      {
        LOG_WARNING("Depth frame width exceeds camera maximum of " << SR300_MAX_DEPTH_WIDTH
          << ". Using default of "
          << SR300_DEFAULT_DEPTH_WIDTH)
          this->Internal->DepthFormat.FrameSizePx[0] = SR300_DEFAULT_DEPTH_WIDTH;
      }
      if (frameSizePxTemp[1] > 0 && frameSizePxTemp[1] <= SR300_MAX_DEPTH_HEIGHT)
      {
        this->Internal->DepthFormat.FrameSizePx[1] = frameSizePxTemp[1];
      }
      else
      {
        LOG_WARNING("Depth frame width exceeds camera maximum of " << SR300_MAX_DEPTH_HEIGHT
          << ". Using default of "
          << SR300_DEFAULT_DEPTH_HEIGHT)
          this->Internal->DepthFormat.FrameSizePx[1] = SR300_DEFAULT_DEPTH_HEIGHT;
      }
      // set RS PixelFormat
      this->Internal->DepthFormat.RSPixelFormat = RS::PixelFormat::PIXEL_FORMAT_DEPTH;
      // log to user
      LOG_INFO("Using Depth Frame Size: " << this->Internal->DepthFormat.FrameSizePx[0]
        << "x"
        << this->Internal->DepthFormat.FrameSizePx[1]);
    }
    else
    {
      LOG_ERROR("Unrecognized video data source ID, must be 'Optical' or 'Depth'");
      continue;
    }


    
  }

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
  this->Internal->Session = RS::Session::CreateInstance();
  
  if (!this->Internal->Session)
  {
    LOG_ERROR("Failed to create a RealSense SDK Session");
    return PLUS_FAIL;
  }
  
  this->Internal->SenseManager = this->Internal->Session->CreateSenseManager();
  if (!this->Internal->SenseManager)
  {
    LOG_ERROR("Failed to create a RealSense SDK SenseManager");
    return PLUS_FAIL;
  }
  
  // enable streams based on OPTICAL / OPTICAL_AND_DEPTH

  RS::SampleReader* SampleRdr;
  SampleRdr = RS::SampleReader::Activate(this->Internal->SenseManager);

  // configure RS video streams
  if (this->Internal->OutputVideoType == OPTICAL || this->Internal->OutputVideoType == OPTICAL_AND_DEPTH)
  {
    // configure RS optical stream
    SampleRdr->EnableStream(RS::StreamType::STREAM_TYPE_COLOR,
      this->Internal->ColorFormat.FrameSizePx[0],
      this->Internal->ColorFormat.FrameSizePx[1],
      DEFAULT_FRAME_RATE);

    // configure optical buffer
    if (this->GetVideoSource("Optical", this->Internal->ColorFrame.aSource) != PLUS_SUCCESS)
    {
      LOG_ERROR("Unable to retrieve the OPTICAL video source in the Intel RealSense Video device.");
      return PLUS_FAIL;
    }
    this->Internal->ColorFrame.aSource->SetPixelType(VTK_UNSIGNED_CHAR);
    this->Internal->ColorFrame.aSource->SetImageType(US_IMG_RGB_COLOR);
    this->Internal->ColorFrame.aSource->SetNumberOfScalarComponents(3);
    this->Internal->ColorFrame.aSource->SetInputFrameSize(this->Internal->ColorFormat.FrameSizePx);

    // configure optical frame
    this->Internal->ColorFrame.PlusFrame.SetImageType(this->Internal->ColorFrame.aSource->GetImageType());
    this->Internal->ColorFrame.PlusFrame.SetImageOrientation(this->Internal->ColorFrame.aSource->GetInputImageOrientation());
    this->Internal->ColorFrame.PlusFrame.SetImageType(US_IMAGE_TYPE::US_IMG_RGB_COLOR);
    this->Internal->ColorFrame.PlusFrame.AllocateFrame(this->Internal->ColorFormat.FrameSizePx, VTK_UNSIGNED_CHAR, 3); 
  }

  if (this->Internal->OutputVideoType == OPTICAL_AND_DEPTH)
  {
    SampleRdr->EnableStream(RS::StreamType::STREAM_TYPE_DEPTH,
      this->Internal->DepthFormat.FrameSizePx[0],
      this->Internal->DepthFormat.FrameSizePx[1],
      DEFAULT_FRAME_RATE);

    // configure depth stream buffer

    //configure depth frame
  }
  
  this->Internal->SenseManager->Init();

  return PLUS_SUCCESS;
}

//----------------------------------------------------------------------------
PlusStatus vtkPlusIntelRealSenseVideoSource::InternalDisconnect()
{ 
  this->Internal->SenseManager->Close();
  this->Internal->SenseManager->Release();
  this->IsTrackingInitialized=false;
  return PLUS_SUCCESS;
}
