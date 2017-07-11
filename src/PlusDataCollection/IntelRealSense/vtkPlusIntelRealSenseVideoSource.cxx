/*=Plus=header=begin======================================================
Program: Plus
Copyright (c) Laboratory for Percutaneous Surgery. All rights reserved.
See License.txt for details.
=========================================================Plus=header=end*/
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

#include "vtkXMLPolyDataWriter.h"
#include "vtkPolyDataMapper.h"
#include "vtkActor.h"
#include "vtkRenderWindowInteractor.h"
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkProperty.h>


#include "opencv2/highgui.hpp"

#define DEFAULT_FRAME_RATE            60
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

  struct RSVideo
  {
    int FrameSizePx[3] = { 0, 0, 1 };
    RS::Image::PixelFormat RSPixelFormat;
    PlusVideoFrame PlusFrame;
    vtkPlusDataSource* PlusSource;
  };
  
  RSVideo* ColorStream = new RSVideo;
  RSVideo* DepthStream = new RSVideo;

  // global Intel RealSense objects
  RS::Session* Session;
  RS::SenseManager* SenseManager;
  RS::CaptureManager* CaptureManager;
  RS::Device* Device;
  //RS::Capture* Capture;
  RS::Projection* Projection;
  RS::Image* ColorImage;
  RS::Image* DepthImage;
  RS::Point3DF32 *depthMap;

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
  // wait until both optical and depth frames are ready
  if (this->Internal->SenseManager->AcquireFrame(true) < RS::Status::STATUS_NO_ERROR)
  {
    return PLUS_SUCCESS;
  }

  // get frame from RealSense
  RS::Capture::Sample *sample = this->Internal->SenseManager->QuerySample();

  // get updated system time 
  const double unfilteredTimestamp = vtkPlusAccurateTimer::GetSystemTime();

  
  // Update optical frame and add to buffer
  //TODO: Re-implement PlusStatus functionality on the buffer frame additions so that update fails if image cannot be added
  if (this->OutputType == OPTICAL || this->OutputType == OPTICAL_AND_DEPTH)
  {
    //RS::Image* imageColor = sample->color;
    this->Internal->ColorImage = sample->color;
    RS::ImageData dataColor;
    this->Internal->ColorImage->AcquireAccess(RS::ImageAccess::ACCESS_READ, RS::PixelFormat::PIXEL_FORMAT_RGB, &dataColor);
    
    PixelCodec::ConvertToBmp24(PixelCodec::ComponentOrder_RGB,
      PixelCodec::PixelEncoding_RGB24,
      this->Internal->ColorStream->FrameSizePx[0],
      this->Internal->ColorStream->FrameSizePx[1],
      dataColor.planes[0],
      (unsigned char*)this->Internal->ColorStream->PlusFrame.GetScalarPointer());
      
    

    this->Internal->ColorImage->ReleaseAccess(&dataColor);
  }
  
  vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();

  // If wanted, update depth frame and add to buffer
  if (this->OutputType == OPTICAL_AND_DEPTH)
  {
    this->Internal->DepthImage = sample->depth;
    RS::ImageData dataDepth;
    this->Internal->DepthImage->AcquireAccess(RS::ImageAccess::ACCESS_READ, RS::PixelFormat::PIXEL_FORMAT_DEPTH, &dataDepth);

    this->Internal->Projection = this->Internal->Device->CreateProjection();
    
    RS::Image* depthImageMappedToColor = this->Internal->Projection->CreateDepthImageMappedToColor(
      this->Internal->DepthImage,
      this->Internal->ColorImage);

    this->Internal->Projection->QueryVertices(depthImageMappedToColor, this->Internal->depthMap);

    int numDepthPixels = this->Internal->DepthStream->FrameSizePx[0] * this->Internal->DepthStream->FrameSizePx[1];
    vtkIdType numPoints = numDepthPixels;
    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
    points->SetNumberOfPoints(numPoints);
    vtkSmartPointer<vtkCellArray> vertices = vtkSmartPointer<vtkCellArray>::New();
    vtkIdType pid[1];
    for (int i = 0; i < numDepthPixels; i++)
    {
      if (this->Internal->depthMap[i].z > 0)
      {
        pid[0] = i;
        points->SetPoint(pid[0], this->Internal->depthMap[i].x, this->Internal->depthMap[i].y, this->Internal->depthMap[i].z);
        vertices->InsertNextCell(1, pid); // is there a faster way to do this?
      }
    }


    polydata->SetPoints(points);
    polydata->SetVerts(vertices);
    
    

    depthImageMappedToColor->Release();
    this->Internal->Projection->Release();
    this->Internal->DepthImage->ReleaseAccess(&dataDepth);
  }
  
  // Write data to buffers
  if (this->OutputType == OPTICAL || this->OutputType == OPTICAL_AND_DEPTH)
  {
    this->Internal->ColorStream->PlusSource->AddItem(&this->Internal->ColorStream->PlusFrame,
      this->FrameNumber, unfilteredTimestamp);
  }
  
  if (this->OutputType == OPTICAL_AND_DEPTH)
  {
    this->Internal->DepthStream->PlusSource->AddItem(polydata, this->FrameNumber, unfilteredTimestamp);
  }
  
  


  this->Modified();
  this->FrameNumber++;
  this->Internal->SenseManager->ReleaseFrame();
  return PLUS_SUCCESS;
}

//----------------------------------------------------------------------------
PlusStatus vtkPlusIntelRealSenseVideoSource::ReadConfiguration( vtkXMLDataElement* rootConfigElement )
{
  XML_FIND_DEVICE_ELEMENT_REQUIRED_FOR_READING(deviceConfig, rootConfigElement);
  XML_FIND_NESTED_ELEMENT_REQUIRED(dataSourcesElement, deviceConfig, "DataSources");

  // set OutputVideoType based on number of DataSource elements
  if ((int)dataSourcesElement->GetNumberOfNestedElements() == 1)
  {
    this->OutputType = OPTICAL;
  }
  else if ((int)dataSourcesElement->GetNumberOfNestedElements() == 2)
  {
    this->OutputType = OPTICAL_AND_DEPTH;
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
    if (STRCASECMP(videoId, "Optical") == 0)
    {
      // this is the optical stream
      // check and set width/height parameters
      if (frameSizePxTemp[0] > 0 && frameSizePxTemp[0] <= SR300_MAX_OPTICAL_WIDTH)
      {
        this->Internal->ColorStream->FrameSizePx[0] = frameSizePxTemp[0];
      }
      else
      {
        LOG_WARNING("Optical frame width exceeds camera maximum of " << SR300_MAX_OPTICAL_WIDTH
          << ". Using default of "
          << SR300_DEFAULT_OPTICAL_WIDTH)
          this->Internal->ColorStream->FrameSizePx[0] = SR300_DEFAULT_OPTICAL_WIDTH;
      }
      if (frameSizePxTemp[1] > 0 && frameSizePxTemp[1] <= SR300_MAX_OPTICAL_HEIGHT)
      {
        this->Internal->ColorStream->FrameSizePx[1] = frameSizePxTemp[1];
      }
      else
      {
        LOG_WARNING("Optical frame height exceeds camera maximum of " << SR300_MAX_OPTICAL_HEIGHT
          << ". Using default of "
          << SR300_DEFAULT_OPTICAL_HEIGHT)
          this->Internal->ColorStream->FrameSizePx[1] = SR300_DEFAULT_OPTICAL_HEIGHT;
      }
      // set RS PixelFormat
      this->Internal->ColorStream->RSPixelFormat = RS::PixelFormat::PIXEL_FORMAT_RGB24;
      // log to user
      LOG_INFO("Using Optical Frame Size: " << this->Internal->ColorStream->FrameSizePx[0]
        << "x"
        << this->Internal->ColorStream->FrameSizePx[1]);
    }
    else if (STRCASECMP(videoId, "Depth") == 0)
    {
      // this is the depth stream
      // check and set width/height parameters
      if (frameSizePxTemp[0] > 0 && frameSizePxTemp[0] <= SR300_MAX_DEPTH_WIDTH)
      {
        this->Internal->DepthStream->FrameSizePx[0] = frameSizePxTemp[0];
      }
      else
      {
        LOG_WARNING("Depth frame width exceeds camera maximum of " << SR300_MAX_DEPTH_WIDTH
          << ". Using default of "
          << SR300_DEFAULT_DEPTH_WIDTH)
          this->Internal->DepthStream->FrameSizePx[0] = SR300_DEFAULT_DEPTH_WIDTH;
      }
      if (frameSizePxTemp[1] > 0 && frameSizePxTemp[1] <= SR300_MAX_DEPTH_HEIGHT)
      {
        this->Internal->DepthStream->FrameSizePx[1] = frameSizePxTemp[1];
      }
      else
      {
        LOG_WARNING("Depth frame width exceeds camera maximum of " << SR300_MAX_DEPTH_HEIGHT
          << ". Using default of "
          << SR300_DEFAULT_DEPTH_HEIGHT)
          this->Internal->DepthStream->FrameSizePx[1] = SR300_DEFAULT_DEPTH_HEIGHT;
      }
      // set RS PixelFormat
      this->Internal->DepthStream->RSPixelFormat = RS::PixelFormat::PIXEL_FORMAT_DEPTH_F32;
      // log to user
      LOG_INFO("Using Depth Frame Size: " << this->Internal->DepthStream->FrameSizePx[0]
        << "x"
        << this->Internal->DepthStream->FrameSizePx[1]);
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

  // TODO: Populate this

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
  
  //this->Internal->Session->SetCoordinateSystem(RS::CoordinateSystem::COORDINATE_SYSTEM_REAR_OPENCV);

  this->Internal->SenseManager = this->Internal->Session->CreateSenseManager();
  if (!this->Internal->SenseManager)
  {
    LOG_ERROR("Failed to create a RealSense SDK SenseManager");
    return PLUS_FAIL;
  }
  
  // enable streams based on OPTICAL / OPTICAL_AND_DEPTH
  if (this->OutputType == OPTICAL || this->OutputType == OPTICAL_AND_DEPTH)
  {
    // configure RS optical stream
    this->Internal->SenseManager->EnableStream( RS::StreamType::STREAM_TYPE_COLOR,
      this->Internal->ColorStream->FrameSizePx[0],
      this->Internal->ColorStream->FrameSizePx[1],
      DEFAULT_FRAME_RATE);

    // configure optical buffer
    if (this->GetVideoSource("Optical", this->Internal->ColorStream->PlusSource) != PLUS_SUCCESS)
    {
      LOG_ERROR("Unable to retrieve the OPTICAL video source in the Intel RealSense Video device.");
      return PLUS_FAIL;
    }
    this->Internal->ColorStream->PlusSource->SetPixelType(VTK_UNSIGNED_CHAR);
    this->Internal->ColorStream->PlusSource->SetImageType(US_IMG_RGB_COLOR);
    this->Internal->ColorStream->PlusSource->SetNumberOfScalarComponents(3);
    this->Internal->ColorStream->PlusSource->SetInputFrameSize(this->Internal->ColorStream->FrameSizePx);

    // configure optical frame
    this->Internal->ColorStream->PlusFrame.SetImageType(this->Internal->ColorStream->PlusSource->GetImageType());
    this->Internal->ColorStream->PlusFrame.SetImageOrientation(this->Internal->ColorStream->PlusSource->GetInputImageOrientation());
    this->Internal->ColorStream->PlusFrame.SetImageType(US_IMAGE_TYPE::US_IMG_RGB_COLOR);
    this->Internal->ColorStream->PlusFrame.AllocateFrame(this->Internal->ColorStream->FrameSizePx, VTK_UNSIGNED_CHAR, 3);
  }

  if (this->OutputType == OPTICAL_AND_DEPTH)
  {
    this->Internal->SenseManager->EnableStream(RS::StreamType::STREAM_TYPE_DEPTH,
      this->Internal->DepthStream->FrameSizePx[0],
      this->Internal->DepthStream->FrameSizePx[1],
      DEFAULT_FRAME_RATE);

    // configure depth stream buffer (vtkPolyData)
    if (this->GetVideoSource("Depth", this->Internal->DepthStream->PlusSource) != PLUS_SUCCESS)
    {
      LOG_ERROR("Unable to retrieve the DEPTH video source in the Intel RealSense Video device.");
      return PLUS_FAIL;
    }
    //configure depth frame (Intel RealSense params)

  }
  
  this->Internal->SenseManager->Init();
  
  this->Internal->CaptureManager = this->Internal->SenseManager->QueryCaptureManager();
  this->Internal->Device = this->Internal->CaptureManager->QueryDevice();
  int numDepthPixels = this->Internal->DepthStream->FrameSizePx[0] * this->Internal->DepthStream->FrameSizePx[1];
  this->Internal->depthMap = new RS::Point3DF32[numDepthPixels];

  return PLUS_SUCCESS;
}

//----------------------------------------------------------------------------
PlusStatus vtkPlusIntelRealSenseVideoSource::InternalDisconnect()
{ 
  this->Internal->SenseManager->Close();
  this->Internal->SenseManager->Release();
  cout << "disconnected" << endl;
  return PLUS_SUCCESS;
}
