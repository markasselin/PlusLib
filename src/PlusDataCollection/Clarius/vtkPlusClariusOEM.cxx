/*=Plus=header=begin======================================================
    Program: Plus
    Copyright (c) UBC Biomedical Signal and Image Computing Laboratory. All rights reserved.
    See License.txt for details.
=========================================================Plus=header=end*/

// Local includes
#include "PlusConfigure.h"
#include "vtkPlusChannel.h"
#include "vtkPlusClariusOEM.h"
#include "PixelCodec.h"
#include "vtkPlusDataSource.h"
#include "vtkPlusUsImagingParameters.h"

// IGSIO includes
#include <vtkIGSIOAccurateTimer.h>
#include <igsioMath.h>

// VTK includes
#include <vtk_zlib.h>
#include <vtkImageData.h>
#include <vtkImageImport.h>
#include <vtkMatrix4x4.h>
#include <vtkObjectFactory.h>
#include <vtkSmartPointer.h>
#include <vtkTransform.h>
#include <vtkXMLUtilities.h>

// OpenCV includes
#include <opencv2/imgproc.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/opencv.hpp>

// vtkxio includes
#include "MadgwickAhrsAlgo.h"
#include "MahonyAhrsAlgo.h"

// Clarius Includes
#include "oem.h"
#include "oem_def.h"


namespace
{
  static const void* BLOCKINGCALL = nullptr;
  static const int DEFAULT_FRAME_WIDTH = 640;
  static const int DEFAULT_FRAME_HEIGHT = 480;
  static const int BUFFER_SIZE = 200;
  static const std::string DEFAULT_PATH_TO_SEC_KEY = "/tmp/";
}

//----------------------------------------------------------------------------

vtkPlusClariusOEM* vtkPlusClariusOEM::instance;

//----------------------------------------------------------------------------
// vtkInternal
//----------------------------------------------------------------------------

class vtkPlusClariusOEM::vtkInternal
{
public:
  vtkPlusClariusOEM* External;

  vtkInternal(vtkPlusClariusOEM* ext);

  virtual ~vtkInternal()
  {
  }

protected:

  friend class vtkPlusClariusOEM;

  // Clarius callbacks
  static void ListFn(const char* list, int sz);

  static void ConnectFn(int ret, int port, const char* status);

  static void CertFn(int daysValid);

  static void PowerDownFn(int ret, int tm);

  static void SwUpdateFn(int ret);

  static void RawImageCallback(const void* newImage, const ClariusRawImageInfo* nfo, int npos, const ClariusPosInfo* pos);

  static void ProcessedImageCallback(const void* newImage, const ClariusProcessedImageInfo* nfo, int npos, const ClariusPosInfo* pos);

  static void ImagingFn(int ready, int imaging);

  static void ButtonFn(int btn, int clicks);

  static void ProgressFn(int progress);

  static void ErrorFn(const char* msg);

  //TODO: This should be removed once poses are available over OpenIGTLink
  PlusStatus WritePosesToCsv(const ClariusProcessedImageInfo* nfo, int npos, const ClariusPosInfo* pos, int frameNum, double systemTime, double convertedTime);

  // helper methods

  // member variables
  unsigned int TcpPort;
  int UdpPort;
  std::string IpAddress;
  std::string PathToSecKey; // path to security key, required by the clarius api
  std::ofstream RawImuDataStream;
  std::string ImuOutputFileName;
  double SystemStartTimestamp;
  double ClariusStartTimestamp;
  double ClariusLastTimestamp;
  bool ImuEnabled;
  bool WriteImagesToDisk;

  /*!
  Compress raw data using gzip if enabled
  */
  bool CompressRawData;

  bool IsReceivingRawData;
  int RawDataSize;
  void* RawDataPointer;

  /*!
  Output filename of the raw Clarius data
  If empty, data will be written to the Plus output directory
  */
  std::string RawDataOutputFilename;

  // TODO: should think about whether or not these are all necessary to store
  vtkPlusDataSource* AccelerometerTool;
  vtkPlusDataSource* GyroscopeTool;
  vtkPlusDataSource* MagnetometerTool;
  vtkPlusDataSource* TiltSensorTool;
  vtkPlusDataSource* FilteredTiltSensorTool;
  vtkPlusDataSource* OrientationSensorTool;
  vtkNew<vtkMatrix4x4> LastAccelerometerToTrackerTransform;
  vtkNew<vtkMatrix4x4> LastGyroscopeToTrackerTransform;
  vtkNew<vtkMatrix4x4> LastMagnetometerToTrackerTransform;
  vtkNew<vtkMatrix4x4> LastTiltSensorToTrackerTransform;
  vtkNew<vtkMatrix4x4> LastFilteredTiltSensorToTrackerTransform;
  vtkNew<vtkMatrix4x4> LastOrientationSensorToTrackerTransform;

  enum AHRS_METHOD
  {
    AHRS_MADGWICK,
    AHRS_MAHONY
  };

  AhrsAlgo* FilteredTiltSensorAhrsAlgo;

  AhrsAlgo* AhrsAlgo;

  /*!
    If AhrsUseMagnetometer enabled (a ..._MARG algorithm is chosen) then heading will be estimated using magnetometer data.
    Otherwise (when a ..._IMU algorithm is chosen) only the gyroscope data will be used for getting the heading information.
    IMU may be more noisy, but not sensitive to magnetic field distortions.
  */
  bool AhrsUseMagnetometer;

  /*!
    Gain values used by the AHRS algorithm (Mahony: first parameter is proportional, second is integral gain; Madgwick: only the first parameter is used)
    Higher gain gives higher reliability to accelerometer&magnetometer data.
  */
  double AhrsAlgorithmGain[2];
  double FilteredTiltSensorAhrsAlgorithmGain[2];

  /*! last AHRS update time (in system time) */
  double AhrsLastUpdateTime;
  double FilteredTiltSensorAhrsLastUpdateTime;

  /*!
    In tilt sensor mode we don't use the magnetometer, so we have to provide a direction reference.
    The orientation is specified by specifying an axis that will always point to the "West" direction.
    Recommended values:
    If sensor axis 0 points down (the sensor plane is about vertical) => TiltSensorDownAxisIndex = 2.
    If sensor axis 1 points down (the sensor plane is about vertical) => TiltSensorDownAxisIndex = 0.
    If sensor axis 2 points down (the sensor plane is about horizontal) => TiltSensorDownAxisIndex = 1.
  */
  int TiltSensorWestAxisIndex;
  int FilteredTiltSensorWestAxisIndex;
};

//----------------------------------------------------------------------------
vtkPlusClariusOEM::vtkInternal::vtkInternal(vtkPlusClariusOEM* ext)
: External(ext)
, TcpPort(-1)
, UdpPort(-1)
, IpAddress("192.168.1.1")
, PathToSecKey(DEFAULT_PATH_TO_SEC_KEY)
, ImuOutputFileName("ClariusImuData.csv")
, ImuEnabled(false)
, SystemStartTimestamp(0)
, ClariusStartTimestamp(0)
, ClariusLastTimestamp(0)
, WriteImagesToDisk(false)
, CompressRawData(false)
, IsReceivingRawData(false)
, RawDataPointer(nullptr)
{
  // Set up the AHRS algorithm used by the orientation sensor tool
  this->AhrsAlgo = new MadgwickAhrsAlgo;
  this->AhrsUseMagnetometer = true;
  this->AhrsAlgorithmGain[0] = 1.5; // proportional
  this->AhrsAlgorithmGain[1] = 0.0; // integral
  this->AhrsLastUpdateTime = -1;

  // set up the AHRS algorithm used by the FilteredTiltSensor sensor tool
  this->FilteredTiltSensorAhrsAlgo = new MadgwickAhrsAlgo;
  this->FilteredTiltSensorAhrsAlgorithmGain[0] = 1.5; // proportional
  this->FilteredTiltSensorAhrsAlgorithmGain[1] = 0.0; // integral
  this->FilteredTiltSensorAhrsLastUpdateTime = -1;
  this->FilteredTiltSensorWestAxisIndex = 1;
  this->TiltSensorWestAxisIndex = 1; // the sensor plane is horizontal (axis 2 points down, axis 1 points West)
}

//----------------------------------------------------------------------------
void vtkPlusClariusOEM::vtkInternal::ListFn(const char* list, int sz)
{

}

//----------------------------------------------------------------------------
void vtkPlusClariusOEM::vtkInternal::ConnectFn(int ret, int port, const char* status)
{

}

//----------------------------------------------------------------------------
void vtkPlusClariusOEM::vtkInternal::CertFn(int daysValid)
{

}

//----------------------------------------------------------------------------
void vtkPlusClariusOEM::vtkInternal::PowerDownFn(int ret, int tm)
{

}

//----------------------------------------------------------------------------
void vtkPlusClariusOEM::vtkInternal::SwUpdateFn(int ret)
{

}

//----------------------------------------------------------------------------
void vtkPlusClariusOEM::vtkInternal::RawImageCallback(const void* newImage, const ClariusRawImageInfo* nfo, int npos, const ClariusPosInfo* pos)
{
  LOG_TRACE("vtkPlusClariusOEM::RawImageCallback");
  vtkPlusClariusOEM* device = vtkPlusClariusOEM::GetInstance();
  if (device == NULL)
  {
    LOG_ERROR("Clarius instance is NULL");
    return;
  }

  LOG_TRACE("New raw image (" << newImage << "): " << nfo->lines << " lines using " << nfo->samples << " samples, @ " << nfo->bitsPerSample << " bits."
    << nfo->axialSize << " axial microns per sample, " << nfo->lateralSize << " lateral microns per line.");

  // Check if still connected
  if (device->Connected == 0)
  {
    LOG_ERROR("Trouble connecting to Clarius Device. IpAddress = " << device->Internal->IpAddress
      << " port = " << device->Internal->TcpPort);
    return;
  }

  if (newImage == NULL)
  {
    LOG_ERROR("No frame received by the device");
    return;
  }

  vtkPlusDataSource* rfModeSource = nullptr;
  std::vector<vtkPlusDataSource*> rfModeSources;
  device->GetVideoSourcesByPortName(vtkPlusDevice::RFMODE_PORT_NAME, rfModeSources);
  if (!rfModeSources.empty())
  {
    rfModeSource = rfModeSources[0];
  }
  else
  {
    LOG_WARNING("Raw image was received, however no output RF video source was found.");
    return;
  }

  // Set Image Properties
  int pixelType = VTK_UNSIGNED_CHAR;
  int frameBufferBytesPerSample = (nfo->bitsPerSample / 8);
  switch (frameBufferBytesPerSample)
  {
  case VTK_SIZEOF_LONG_LONG:
    pixelType = VTK_LONG_LONG;
    break;
  case VTK_SIZEOF_INT:
    pixelType = VTK_UNSIGNED_INT;
    break;
  case VTK_SIZEOF_SHORT:
    pixelType = VTK_UNSIGNED_SHORT;
    break;
  case VTK_SIZEOF_CHAR:
  default:
    pixelType = VTK_UNSIGNED_CHAR;
    break;
  }
  rfModeSource->SetInputFrameSize(nfo->lines, nfo->samples, 1);
  rfModeSource->SetPixelType(pixelType);
  rfModeSource->SetImageType(US_IMG_RF_REAL);
  rfModeSource->SetOutputImageOrientation(US_IMG_ORIENT_MF);

  int frameSizeInBytes = nfo->lines * nfo->samples * frameBufferBytesPerSample;

  // the clarius timestamp is in nanoseconds
  device->Internal->ClariusLastTimestamp = static_cast<double>((double)nfo->tm / (double)1000000000);
  // Get system time (elapsed time since last reboot), return Internal system time in seconds
  double systemTime = vtkIGSIOAccurateTimer::GetSystemTime();
  if (device->FrameNumber == 0)
  {
    device->Internal->SystemStartTimestamp = systemTime;
    device->Internal->ClariusStartTimestamp = device->Internal->ClariusLastTimestamp;
  }

  // Need to copy newImage to new char vector vtkDataSource::AddItem() do not accept const char array
  std::vector<char> imageData;
  if (imageData.size() < static_cast<size_t>(frameSizeInBytes))
  {
    imageData.resize(static_cast<size_t>(frameSizeInBytes));
  }
  memcpy(imageData.data(), newImage, static_cast<size_t>(frameSizeInBytes));

  double convertedTimestamp = device->Internal->SystemStartTimestamp + (device->Internal->ClariusLastTimestamp - device->Internal->ClariusStartTimestamp);
  rfModeSource->AddItem(
    (void*)newImage, // pointer to char array
    rfModeSource->GetInputImageOrientation(), // refer to this url: http://perk-software.cs.queensu.ca/plus/doc/nightly/dev/UltrasoundImageOrientation.html for reference;
                                              // Set to UN to keep the orientation of the image the same as on tablet
    rfModeSource->GetInputFrameSize(),
    pixelType,
    1,
    US_IMG_RF_REAL,
    0,
    device->FrameNumber,
    convertedTimestamp,
    convertedTimestamp);
}

//----------------------------------------------------------------------------
void vtkPlusClariusOEM::vtkInternal::ProcessedImageCallback(const void* newImage, const ClariusProcessedImageInfo* nfo, int npos, const ClariusPosInfo* pos)
{
  LOG_TRACE("vtkPlusClariusOEM::ProcessedImageCallback");
  vtkPlusClariusOEM* device = vtkPlusClariusOEM::GetInstance();
  if (device == NULL)
  {
    LOG_ERROR("Clarius instance is NULL!!!");
    return;
  }


  LOG_TRACE("new image (" << newImage << "): " << nfo->width << " x " << nfo->height << " @ " << nfo->bitsPerPixel
    << "bits. @ " << nfo->micronsPerPixel << " microns per pixel. imu points: " << npos);

  // Check if still connected
  if (device->Connected == 0)
  {
    LOG_ERROR("Trouble connecting to Clarius Device. IpAddress = " << device->Internal->IpAddress
      << " port = " << device->Internal->TcpPort);
    return;
  }

  if (newImage == NULL)
  {
    LOG_ERROR("No frame received by the device");
    return;
  }

  // check if there exist active data source;
  vtkPlusDataSource* bModeSource;
  std::vector<vtkPlusDataSource*> bModeSources;
  device->GetVideoSourcesByPortName(vtkPlusDevice::BMODE_PORT_NAME, bModeSources);
  if (!bModeSources.empty())
  {
    bModeSource = bModeSources[0];
  }
  else
  {
    LOG_WARNING("Processed image was received, however no output B-Mode video source was found.");
    return;
  }

  // Set Image Properties
  bModeSource->SetInputFrameSize(nfo->width, nfo->height, 1);
  int frameBufferBytesPerPixel = (nfo->bitsPerPixel / 8);
  int frameSizeInBytes = nfo->width * nfo->height * frameBufferBytesPerPixel;
  bModeSource->SetNumberOfScalarComponents(frameBufferBytesPerPixel);

  // need to copy newImage to new char vector vtkDataSource::AddItem() do not accept const char array
  std::vector<char> _image;
  size_t img_sz = nfo->width * nfo->height * (nfo->bitsPerPixel / 8);
  if (_image.size() < img_sz)
  {
    _image.resize(img_sz);
  }
  memcpy(_image.data(), newImage, img_sz);

  // the clarius timestamp is in nanoseconds
  device->Internal->ClariusLastTimestamp = static_cast<double>((double)nfo->tm / (double)1000000000);
  // Get system time (elapsed time since last reboot), return Internal system time in seconds
  double systemTime = vtkIGSIOAccurateTimer::GetSystemTime();
  if (device->FrameNumber == 0)
  {
    device->Internal->SystemStartTimestamp = systemTime;
    device->Internal->ClariusStartTimestamp = device->Internal->ClariusLastTimestamp;
  }

  // The timestamp that each image is tagged with is
  // (system_start_time + current_clarius_time - clarius_start_time)
  double converted_timestamp = device->Internal->SystemStartTimestamp + (device->Internal->ClariusLastTimestamp - device->Internal->ClariusStartTimestamp);
  if (npos != 0)
  {
    device->Internal->WritePosesToCsv(nfo, npos, pos, device->FrameNumber, systemTime, converted_timestamp);
  }

  if (device->Internal->WriteImagesToDisk)
  {
    // create cvimg to write to disk
    cv::Mat cvimg = cv::Mat(nfo->width, nfo->height, CV_8UC4);
    cvimg.data = cvimg.data = (unsigned char*)_image.data();
    if (cv::imwrite("Clarius_Image" + std::to_string(device->Internal->ClariusLastTimestamp) + ".bmp", cvimg) == false)
    {
      LOG_ERROR("ERROR writing clarius image" + std::to_string(device->Internal->ClariusLastTimestamp) + " to disk");
    }
  }

  igsioFieldMapType customField;
  customField["micronsPerPixel"] = std::make_pair(igsioFrameFieldFlags::FRAMEFIELD_FORCE_SERVER_SEND, std::to_string(nfo->micronsPerPixel));
  bModeSource->AddItem(
    _image.data(), // pointer to char array
    bModeSource->GetInputImageOrientation(), // refer to this url: http://perk-software.cs.queensu.ca/plus/doc/nightly/dev/UltrasoundImageOrientation.html for reference;
                                         // Set to UN to keep the orientation of the image the same as on tablet
    bModeSource->GetInputFrameSize(),
    VTK_UNSIGNED_CHAR,
    frameBufferBytesPerPixel,
    US_IMG_BRIGHTNESS,
    0,
    device->FrameNumber,
    converted_timestamp,
    converted_timestamp,
    &customField
  );

  for (int i = 0; i < npos; i++)
  {
    double angularRate[3] = { pos[i].gx , pos[i].gy , pos[i].gz };
    double magneticField[3] = { pos[i].mx , pos[i].my , pos[i].mz };
    double acceleration[3] = { pos[i].ax , pos[i].ay , pos[i].az };

    if (device->Internal->AccelerometerTool != NULL)
    {
      device->Internal->LastAccelerometerToTrackerTransform->Identity();
      device->Internal->LastAccelerometerToTrackerTransform->SetElement(0, 3, acceleration[0]);
      device->Internal->LastAccelerometerToTrackerTransform->SetElement(1, 3, acceleration[1]);
      device->Internal->LastAccelerometerToTrackerTransform->SetElement(2, 3, acceleration[2]);
      device->ToolTimeStampedUpdateWithoutFiltering(device->Internal->AccelerometerTool->GetId(), device->Internal->LastAccelerometerToTrackerTransform, TOOL_OK, converted_timestamp, converted_timestamp);
    }
    if (device->Internal->GyroscopeTool != NULL)
    {
      device->Internal->LastGyroscopeToTrackerTransform->Identity();
      device->Internal->LastGyroscopeToTrackerTransform->SetElement(0, 3, angularRate[0]);
      device->Internal->LastGyroscopeToTrackerTransform->SetElement(1, 3, angularRate[1]);
      device->Internal->LastGyroscopeToTrackerTransform->SetElement(2, 3, angularRate[2]);
      device->ToolTimeStampedUpdateWithoutFiltering(device->Internal->GyroscopeTool->GetId(), device->Internal->LastGyroscopeToTrackerTransform, TOOL_OK, converted_timestamp, converted_timestamp);
    }
    if (device->Internal->MagnetometerTool != NULL)
    {
      if (magneticField[0] > 1e100)
      {
        // magnetometer data is not available, use the last transform with an invalid status to not have any missing transform
        device->ToolTimeStampedUpdateWithoutFiltering(device->Internal->MagnetometerTool->GetId(), device->Internal->LastMagnetometerToTrackerTransform, TOOL_INVALID, converted_timestamp, converted_timestamp);
      }
      else
      {
        // magnetometer data is valid
        device->Internal->LastMagnetometerToTrackerTransform->Identity();
        device->Internal->LastMagnetometerToTrackerTransform->SetElement(0, 3, magneticField[0]);
        device->Internal->LastMagnetometerToTrackerTransform->SetElement(1, 3, magneticField[1]);
        device->Internal->LastMagnetometerToTrackerTransform->SetElement(2, 3, magneticField[2]);
        device->ToolTimeStampedUpdateWithoutFiltering(device->Internal->MagnetometerTool->GetId(), device->Internal->LastMagnetometerToTrackerTransform, TOOL_OK, converted_timestamp, converted_timestamp);
      }
    }

    if (device->Internal->TiltSensorTool != NULL)
    {
      // Compose matrix that transforms the x axis to the input vector by rotations around two orthogonal axes
      vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();

      double downVector_Sensor[4] = { acceleration[0], acceleration[1], acceleration[2], 0 }; // provided by the sensor
      vtkMath::Normalize(downVector_Sensor);

      igsioMath::ConstrainRotationToTwoAxes(downVector_Sensor, device->Internal->TiltSensorWestAxisIndex, device->Internal->LastTiltSensorToTrackerTransform);

      device->ToolTimeStampedUpdateWithoutFiltering(device->Internal->TiltSensorTool->GetId(), device->Internal->LastTiltSensorToTrackerTransform, TOOL_OK, converted_timestamp, converted_timestamp);
    }

    if (device->Internal->OrientationSensorTool != NULL)
    {
      if (magneticField[0] > 1e100)
      {
        // magnetometer data is not available, use the last transform with an invalid status to not have any missing transform
        device->ToolTimeStampedUpdateWithoutFiltering(device->Internal->OrientationSensorTool->GetId(), device->Internal->LastOrientationSensorToTrackerTransform, TOOL_INVALID, converted_timestamp, converted_timestamp);
      }
      else
      {
        // magnetometer data is valid

        //LOG_TRACE("samplingTime(msec)="<<1000.0*timeSinceLastAhrsUpdateSec<<", packetCount="<<count);
        //LOG_TRACE("gyroX="<<std::fixed<<std::setprecision(2)<<std::setw(6)<<angularRate[0]<<", gyroY="<<angularRate[1]<<", gyroZ="<<angularRate[2]);
        //LOG_TRACE("magX="<<std::fixed<<std::setprecision(2)<<std::setw(6)<<magneticField[0]<<", magY="<<magneticField[1]<<", magZ="<<magneticField[2]);

        if (device->Internal->AhrsUseMagnetometer)
        {
          device->Internal->AhrsAlgo->UpdateWithTimestamp(
            vtkMath::RadiansFromDegrees(angularRate[0]), vtkMath::RadiansFromDegrees(angularRate[1]), vtkMath::RadiansFromDegrees(angularRate[2]),
            acceleration[0], acceleration[1], acceleration[2],
            magneticField[0], magneticField[1], magneticField[2], converted_timestamp);
        }
        else
        {
          device->Internal->AhrsAlgo->UpdateIMUWithTimestamp(
            vtkMath::RadiansFromDegrees(angularRate[0]), vtkMath::RadiansFromDegrees(angularRate[1]), vtkMath::RadiansFromDegrees(angularRate[2]),
            acceleration[0], acceleration[1], acceleration[2], converted_timestamp);
        }


        double rotQuat[4] = { 0 };
        device->Internal->AhrsAlgo->GetOrientation(rotQuat[0], rotQuat[1], rotQuat[2], rotQuat[3]);

        double rotMatrix[3][3] = { 0 };
        vtkMath::QuaternionToMatrix3x3(rotQuat, rotMatrix);

        for (int c = 0; c < 3; c++)
        {
          for (int r = 0; r < 3; r++)
          {
            device->Internal->LastOrientationSensorToTrackerTransform->SetElement(r, c, rotMatrix[r][c]);
          }
        }

        device->ToolTimeStampedUpdateWithoutFiltering(device->Internal->OrientationSensorTool->GetId(), device->Internal->LastOrientationSensorToTrackerTransform, TOOL_OK, converted_timestamp, converted_timestamp);
      }
    }
    if (device->Internal->FilteredTiltSensorTool != NULL)
    {
      device->Internal->FilteredTiltSensorAhrsAlgo->UpdateIMUWithTimestamp(
        vtkMath::RadiansFromDegrees(angularRate[0]), vtkMath::RadiansFromDegrees(angularRate[1]), vtkMath::RadiansFromDegrees(angularRate[2]),
        acceleration[0], acceleration[1], acceleration[2], converted_timestamp);

      double rotQuat[4] = { 0 };
      device->Internal->AhrsAlgo->GetOrientation(rotQuat[0], rotQuat[1], rotQuat[2], rotQuat[3]);

      double rotMatrix[3][3] = { 0 };
      vtkMath::QuaternionToMatrix3x3(rotQuat, rotMatrix);

      double filteredDownVector_Sensor[4] = { rotMatrix[2][0], rotMatrix[2][1], rotMatrix[2][2], 0 };
      vtkMath::Normalize(filteredDownVector_Sensor);

      igsioMath::ConstrainRotationToTwoAxes(filteredDownVector_Sensor, device->Internal->FilteredTiltSensorWestAxisIndex, device->Internal->LastFilteredTiltSensorToTrackerTransform);

      device->ToolTimeStampedUpdateWithoutFiltering(device->Internal->FilteredTiltSensorTool->GetId(), device->Internal->LastFilteredTiltSensorToTrackerTransform, TOOL_OK, converted_timestamp, converted_timestamp);

      // write back the results to the FilteredTiltSensor_AHRS algorithm
      for (int c = 0; c < 3; c++)
      {
        for (int r = 0; r < 3; r++)
        {
          rotMatrix[r][c] = device->Internal->LastFilteredTiltSensorToTrackerTransform->GetElement(r, c);
        }
      }
      double filteredTiltSensorRotQuat[4] = { 0 };
      vtkMath::Matrix3x3ToQuaternion(rotMatrix, filteredTiltSensorRotQuat);
      device->Internal->FilteredTiltSensorAhrsAlgo->SetOrientation(filteredTiltSensorRotQuat[0], filteredTiltSensorRotQuat[1], filteredTiltSensorRotQuat[2], filteredTiltSensorRotQuat[3]);
    }
  }

  device->FrameNumber++;
}

//----------------------------------------------------------------------------
void vtkPlusClariusOEM::vtkInternal::ImagingFn(int ready, int imaging)
{

}

//----------------------------------------------------------------------------
/*! callback for button clicks
 * @param[in] btn 0 = up, 1 = down
 * @param[in] clicks # of clicks performed*/
void vtkPlusClariusOEM::vtkInternal::ButtonFn(int btn, int clicks)
{
  LOG_DEBUG("button: " << btn << "clicks: " << clicks << "%");
}

//----------------------------------------------------------------------------
/*! callback for readback progress
 * @pram[in] progress the readback process*/
void vtkPlusClariusOEM::vtkInternal::ProgressFn(int progress)
{
  LOG_DEBUG("Download: " << progress << "%");
}


//----------------------------------------------------------------------------
/*! callback for error messages
 * @param[in] err the error message sent from the listener module
 * */
void vtkPlusClariusOEM::vtkInternal::ErrorFn(const char* err)
{
  LOG_ERROR("error: " << err);
}

//----------------------------------------------------------------------------
PlusStatus vtkPlusClariusOEM::vtkInternal::WritePosesToCsv(const ClariusProcessedImageInfo* nfo, int npos, const ClariusPosInfo* pos, int frameNum, double systemTime, double convertedTime)
{
  LOG_TRACE("vtkPlusClariusOEM::WritePosesToCsv");
  if (npos != 0)
  {
    LOG_TRACE("timestamp in nanoseconds ClariusPosInfo" << pos[0].tm);
    std::string posInfo = "";
    for (auto i = 0; i < npos; i++)
    {
      posInfo += (std::to_string(frameNum) + ",");
      posInfo += (std::to_string(systemTime) + ",");
      posInfo += (std::to_string(convertedTime) + ",");
      posInfo += (std::to_string(nfo->tm) + ",");
      posInfo += (std::to_string(pos[i].tm) + ",");
      posInfo += (std::to_string(pos[i].ax) + ",");
      posInfo += (std::to_string(pos[i].ay) + ",");
      posInfo += (std::to_string(pos[i].az) + ",");
      posInfo += (std::to_string(pos[i].gx) + ",");
      posInfo += (std::to_string(pos[i].gy) + ",");
      posInfo += (std::to_string(pos[i].gz) + ",");
      posInfo += (std::to_string(pos[i].mx) + ",");
      posInfo += (std::to_string(pos[i].my) + ",");
      posInfo += (std::to_string(pos[i].mz) + ",");
      posInfo += "\n";
    }

    // write the string to file
    this->RawImuDataStream.open(this->ImuOutputFileName, std::ofstream::app);
    if (this->RawImuDataStream.is_open() == false)
    {
      LOG_ERROR("Error opening file for raw imu data");
      return PLUS_FAIL;
    }

    this->RawImuDataStream << posInfo;
    this->RawImuDataStream.close();
    return PLUS_SUCCESS;
  }

  return PLUS_SUCCESS;
}

//----------------------------------------------------------------------------
/*! callback for a new image sent from the scanner
 * @param[in] newImage a pointer to the raw image bits of
 * @param[in] nfo the image properties
 * @param[in] npos the # fo positional data points embedded with the frame
 * @param[in] pos the buffer of positional data
 * */
 //void vtkPlusClariusOEM::NewImageFn(const void* newImage, const ClariusProcessedImageInfo* nfo, int npos, const ClariusPosInfo* pos)
 //{
 //  LOG_TRACE("new image (" << newImage << "): " << nfo->width << " x " << nfo->height << " @ " << nfo->bitsPerPixel
 //    << "bits. @ " << nfo->micronsPerPixel << " microns per pixel. imu points: " << npos);
 //  if (npos)
 //  {
 //    for (auto i = 0; i < npos; i++)
 //    {
 //      LOG_TRACE("imu: " << i << ", time: " << pos[i].tm);
 //      LOG_TRACE("accel: " << pos[i].ax << "," << pos[i].ay << "," << pos[i].az);
 //      LOG_TRACE("gyro: " << pos[i].gx << "," << pos[i].gy << "," << pos[i].gz);
 //      LOG_TRACE("magnet: " << pos[i].mx << "," << pos[i].my << "," << pos[i].mz);
 //    }
 //  }
 //}





//----------------------------------------------------------------------------
// vtkPlusClariusOEM definitions
//----------------------------------------------------------------------------
vtkPlusClariusOEM* vtkPlusClariusOEM::New()
{
  if (instance == NULL)
  {
    instance = new vtkPlusClariusOEM();
  }
  return instance;
}

//----------------------------------------------------------------------------
vtkPlusClariusOEM::vtkPlusClariusOEM()
  : Internal(new vtkInternal(this))
{
  LOG_TRACE("vtkPlusClariusOEM: Constructor");
  this->StartThreadForInternalUpdates = false;
  this->ImagingParameters->SetImageSize(DEFAULT_FRAME_WIDTH, DEFAULT_FRAME_HEIGHT, 1);

  this->RequirePortNameInDeviceSetConfiguration = true;

  instance = this;
}

//----------------------------------------------------------------------------
vtkPlusClariusOEM::~vtkPlusClariusOEM()
{

  if (this->Recording)
  {
    this->StopRecording();
  }

  if (this->Connected)
  {
    cusOemDisconnect();
  }

  int destroyed = cusOemDestroy();
  if (destroyed != 0)
  {
    LOG_ERROR("Error destoying the listener");
  }

  this->instance = NULL;
}

//----------------------------------------------------------------------------
vtkPlusClariusOEM* vtkPlusClariusOEM::GetInstance()
{
  LOG_TRACE("vtkPlusClariusOEM: GetInstance()");
  if (instance != NULL)
  {
    return instance;
  }

  else
  {
    LOG_ERROR("Instance is null, creating new instance");
    instance = new vtkPlusClariusOEM();
    return instance;
  }
}

//----------------------------------------------------------------------------
void vtkPlusClariusOEM::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
  os << indent << "ipAddress" << this->Internal->IpAddress << std::endl;
  os << indent << "tcpPort" << this->Internal->TcpPort << std::endl;
  os << indent << "UdpPort" << this->Internal->UdpPort << std::endl;
  os << indent << "FrameNumber" << this->FrameNumber << std::endl;
  FrameSizeType fs = this->ImagingParameters->GetImageSize();
  os << indent << "FrameWidth" << fs[0] << std::endl;
  os << indent << "FrameHeight" << fs[1] << std::endl;
}

//----------------------------------------------------------------------------
PlusStatus vtkPlusClariusOEM::ReadConfiguration(vtkXMLDataElement* rootConfigElement)
{
  LOG_TRACE("vtkPlusClariusOEM::ReadConfiguration");
  XML_FIND_DEVICE_ELEMENT_REQUIRED_FOR_READING(deviceConfig, rootConfigElement);

  XML_READ_STRING_ATTRIBUTE_NONMEMBER_REQUIRED(IpAddress, this->Internal->IpAddress, deviceConfig);
  XML_READ_SCALAR_ATTRIBUTE_NONMEMBER_REQUIRED(int, TcpPort, this->Internal->TcpPort, deviceConfig);

  // if not specified, the default value for FrameWidth is 640 and FrameHeight is 480 according to clarius;
  int frame_width, frame_height;
  XML_READ_SCALAR_ATTRIBUTE_NONMEMBER_OPTIONAL(int, FrameWidth, frame_width, deviceConfig);
  XML_READ_SCALAR_ATTRIBUTE_NONMEMBER_OPTIONAL(int, FrameHeight, frame_height, deviceConfig);
  this->ImagingParameters->SetImageSize(frame_width, frame_height, 1);

  XML_READ_BOOL_ATTRIBUTE_NONMEMBER_OPTIONAL(ImuEnabled, this->Internal->ImuEnabled, deviceConfig);
  XML_READ_BOOL_ATTRIBUTE_NONMEMBER_OPTIONAL(WriteImagesToDisk, this->Internal->WriteImagesToDisk, deviceConfig);
  if (this->Internal->ImuEnabled)
  {
    XML_READ_STRING_ATTRIBUTE_NONMEMBER_REQUIRED(ImuOutputFileName, this->Internal->ImuOutputFileName, deviceConfig);
  }

  int tiltSensorWestAxisIndex = 0;
  if (deviceConfig->GetScalarAttribute("TiltSensorWestAxisIndex", tiltSensorWestAxisIndex))
  {
    if (tiltSensorWestAxisIndex < 0 || tiltSensorWestAxisIndex > 2)
    {
      LOG_ERROR("TiltSensorWestAxisIndex is invalid. Specified value: " << tiltSensorWestAxisIndex << ". Valid values: 0, 1, 2. Keep using the default value: "
        << this->Internal->TiltSensorWestAxisIndex);
    }
    else
    {
      this->Internal->TiltSensorWestAxisIndex = tiltSensorWestAxisIndex;
    }
  }

  int FilteredTiltSensorWestAxisIndex = 0;
  if (deviceConfig->GetScalarAttribute("FilteredTiltSensorWestAxisIndex", FilteredTiltSensorWestAxisIndex))
  {
    if (FilteredTiltSensorWestAxisIndex < 0 || FilteredTiltSensorWestAxisIndex > 2)
    {
      LOG_ERROR("FilteredTiltSensorWestAxisIndex is invalid. Specified value: " << FilteredTiltSensorWestAxisIndex << ". Valid values: 0, 1, 2. Keep using the default value: "
        << this->Internal->FilteredTiltSensorWestAxisIndex);
    }
    else
    {
      this->Internal->FilteredTiltSensorWestAxisIndex = FilteredTiltSensorWestAxisIndex;
    }
  }

  XML_READ_VECTOR_ATTRIBUTE_NONMEMBER_OPTIONAL(double, 2, AhrsAlgorithmGain, this->Internal->AhrsAlgorithmGain, deviceConfig);
  XML_READ_VECTOR_ATTRIBUTE_NONMEMBER_OPTIONAL(double, 2, FilteredTiltSensorAhrsAlgorithmGain, this->Internal->FilteredTiltSensorAhrsAlgorithmGain, deviceConfig);

  const char* ahrsAlgoName = deviceConfig->GetAttribute("AhrsAlgorithm");
  if (ahrsAlgoName != NULL)
  {
    if (STRCASECMP("MADGWICK_MARG", ahrsAlgoName) == 0 || STRCASECMP("MADGWICK_IMU", ahrsAlgoName) == 0)
    {
      if (dynamic_cast<MadgwickAhrsAlgo*>(this->Internal->AhrsAlgo) == 0)
      {
        // not the requested type
        // delete the old algo and create a new one with the correct type
        delete this->Internal->AhrsAlgo;
        this->Internal->AhrsAlgo = new MadgwickAhrsAlgo;
      }
      if (STRCASECMP("MADGWICK_MARG", ahrsAlgoName) == 0)
      {
        this->Internal->AhrsUseMagnetometer = true;
      }
      else
      {
        this->Internal->AhrsUseMagnetometer = false;
      }
    }
    else if (STRCASECMP("MAHONY_MARG", ahrsAlgoName) == 0 || STRCASECMP("MAHONY_IMU", ahrsAlgoName) == 0)
    {
      if (dynamic_cast<MahonyAhrsAlgo*>(this->Internal->AhrsAlgo) == 0)
      {
        // not the requested type
        // delete the old algo and create a new one with the correct type
        delete this->Internal->AhrsAlgo;
        this->Internal->AhrsAlgo = new MahonyAhrsAlgo;
      }
      if (STRCASECMP("MAHONY_MARG", ahrsAlgoName) == 0)
      {
        this->Internal->AhrsUseMagnetometer = true;
      }
      else
      {
        this->Internal->AhrsUseMagnetometer = false;
      }
    }
    else
    {
      LOG_ERROR("Unable to recognize AHRS algorithm type: " << ahrsAlgoName << ". Supported types: MADGWICK_MARG, MAHONY_MARG, MADGWICK_IMU, MAHONY_IMU");
      return PLUS_FAIL;
    }
  }
  const char* FilteredTiltSensorAhrsAlgoName = deviceConfig->GetAttribute("FilteredTiltSensorAhrsAlgorithm");
  if (FilteredTiltSensorAhrsAlgoName != NULL)
  {
    if (STRCASECMP("MADGWICK_IMU", FilteredTiltSensorAhrsAlgoName) == 0)
    {
      if (dynamic_cast<MadgwickAhrsAlgo*>(this->Internal->FilteredTiltSensorAhrsAlgo) == 0)
      {
        // not the requested type
        // delete the old algo and create a new one with the correct type
        delete this->Internal->FilteredTiltSensorAhrsAlgo;
        this->Internal->FilteredTiltSensorAhrsAlgo = new MadgwickAhrsAlgo;
      }
    }
    else if (STRCASECMP("MAHONY_IMU", FilteredTiltSensorAhrsAlgoName) == 0)
    {
      if (dynamic_cast<MahonyAhrsAlgo*>(this->Internal->FilteredTiltSensorAhrsAlgo) == 0)
      {
        // not the requested type
        // delete the old algo and create a new one with the correct type
        delete this->Internal->FilteredTiltSensorAhrsAlgo;
        this->Internal->FilteredTiltSensorAhrsAlgo = new MahonyAhrsAlgo;
      }
    }
    else
    {
      LOG_ERROR("Unable to recognize AHRS algorithm type for Filtered Tilt: " << FilteredTiltSensorAhrsAlgoName << ". Supported types: MADGWICK_IMU, MAHONY_IMU");
      return PLUS_FAIL;
    }
  }

  return PLUS_SUCCESS;
}

//----------------------------------------------------------------------------
PlusStatus vtkPlusClariusOEM::WriteConfiguration(vtkXMLDataElement* rootConfigElement)
{
  LOG_TRACE("vtkPlusClariusOEM::WriteConfiguration");
  XML_FIND_DEVICE_ELEMENT_REQUIRED_FOR_WRITING(deviceConfig, rootConfigElement);

  deviceConfig->SetAttribute("IpAddress", this->Internal->IpAddress.c_str());
  deviceConfig->SetIntAttribute("TcpPort", this->Internal->TcpPort);
  FrameSizeType fs = this->ImagingParameters->GetImageSize();
  deviceConfig->SetIntAttribute("FrameWidth", fs[0]);
  deviceConfig->SetIntAttribute("FrameHeight", fs[1]);
  XML_WRITE_BOOL_ATTRIBUTE_NONMEMBER(ImuEnabled, this->Internal->ImuEnabled, deviceConfig);
  if (this->Internal->ImuEnabled)
  {
    deviceConfig->SetAttribute("ImuOutputFileName", this->Internal->ImuOutputFileName.c_str());
  }
  return PLUS_SUCCESS;
}

//----------------------------------------------------------------------------
PlusStatus vtkPlusClariusOEM::NotifyConfigured()
{
  LOG_TRACE("vtkPlusClariusOEM::NotifyConfigured");

  vtkPlusClariusOEM* device = vtkPlusClariusOEM::GetInstance();
  if (device->OutputChannels.size() > 1)
  {
    LOG_WARNING("vtkPlusClariusOEM is expecting one output channel and there are " <<
      this->OutputChannels.size() << " channels. First output channel will be used.");
  }

  if (device->OutputChannels.empty())
  {
    LOG_ERROR("No output channels defined for vtkPlusClariusOEM. Cannot proceed.");
    this->CorrectlyConfigured = false;
    return PLUS_FAIL;
  }

  std::vector<vtkPlusDataSource*> sources;
  sources = device->GetVideoSources();
  if (sources.size() > 1)
  {
    LOG_WARNING("More than one output video source found. First will be used");
  }
  if (sources.size() == 0)
  {
    LOG_ERROR("Video source required in configuration. Cannot proceed.");
    return PLUS_FAIL;
  }

  // Check if output channel has data source
  vtkPlusDataSource* aSource(NULL);
  if (device->OutputChannels[0]->GetVideoSource(aSource) != PLUS_SUCCESS)
  {
    LOG_ERROR("Unable to retrieve the video source in the vtkPlusClariusOEM device.");
    return PLUS_FAIL;
  }

  return PLUS_SUCCESS;
}

//----------------------------------------------------------------------------
PlusStatus vtkPlusClariusOEM::Probe()
{
  LOG_TRACE("vtkPlusClariusOEM: Probe");
  if (this->Internal->UdpPort == -1)
  {
    return PLUS_FAIL;
  }

  return PLUS_SUCCESS;
};

//----------------------------------------------------------------------------
std::string vtkPlusClariusOEM::vtkPlusClariusOEM::GetSdkVersion()
{
  std::ostringstream version;
  version << "Sdk version not available" << "\n";
  return version.str();
}

//----------------------------------------------------------------------------
PlusStatus vtkPlusClariusOEM::InternalConnect()
{
  LOG_DEBUG("vtkPlusClariusOEM: InternalConnect");

  if (this->Internal->ImuEnabled)
  {
    this->Internal->RawImuDataStream.open(this->Internal->ImuOutputFileName, std::ofstream::app);
    this->Internal->RawImuDataStream << "FrameNum,SystemTimestamp,ConvertedTimestamp,ImageTimestamp,ImuTimeStamp,ax,ay,az,gx,gy,gz,mx,my,mz,\n";

    this->Internal->RawImuDataStream.close();
  }

  this->Internal->AccelerometerTool = NULL;
  this->GetToolByPortName("Accelerometer", this->Internal->AccelerometerTool);

  this->Internal->GyroscopeTool = NULL;
  this->GetToolByPortName("Gyroscope", this->Internal->GyroscopeTool);

  this->Internal->MagnetometerTool = NULL;
  this->GetToolByPortName("Magnetometer", this->Internal->MagnetometerTool);

  this->Internal->TiltSensorTool = NULL;
  this->GetToolByPortName("TiltSensor", this->Internal->TiltSensorTool);

  this->Internal->FilteredTiltSensorTool = NULL;
  this->GetToolByPortName("FilteredTiltSensor", this->Internal->FilteredTiltSensorTool);

  this->Internal->OrientationSensorTool = NULL;
  this->GetToolByPortName("OrientationSensor", this->Internal->OrientationSensorTool);

  vtkPlusClariusOEM* device = vtkPlusClariusOEM::GetInstance();
  // Initialize Clarius Listener Before Connecting
  if (!device->Connected)
  {
    int argc = 1;
    char** argv = new char* [1];
    argv[0] = new char[4];
    strcpy(argv[0], "abc");
    const char* path = device->Internal->PathToSecKey.c_str();

    // Callbacks
    ClariusListFn listFnPtr = static_cast<ClariusListFn>(&vtkPlusClariusOEM::vtkInternal::ListFn);
    ClariusConnectFn connectFnPtr = static_cast<ClariusConnectFn>(&vtkPlusClariusOEM::vtkInternal::ConnectFn);
    ClariusCertFn certFnPtr = static_cast<ClariusCertFn>(&vtkPlusClariusOEM::vtkInternal::CertFn);
    ClariusPowerDownFn powerDownFnPtr = static_cast<ClariusPowerDownFn>(&vtkPlusClariusOEM::vtkInternal::PowerDownFn);
    ClariusSwUpdateFn swUpdateFnPtr = static_cast<ClariusSwUpdateFn>(&vtkPlusClariusOEM::vtkInternal::SwUpdateFn);
    ClariusNewRawImageFn newRawImageFnPtr = static_cast<ClariusNewRawImageFn>(&vtkPlusClariusOEM::vtkInternal::RawImageCallback);
    ClariusNewProcessedImageFn newProcessedImageFnPtr = static_cast<ClariusNewProcessedImageFn>(&vtkPlusClariusOEM::vtkInternal::ProcessedImageCallback);
    ClariusImagingFn imagingFnPtr = static_cast<ClariusImagingFn>(&vtkPlusClariusOEM::vtkInternal::ImagingFn);
    ClariusButtonFn buttonFnPtr = static_cast<ClariusButtonFn>(&vtkPlusClariusOEM::vtkInternal::ButtonFn);
    ClariusProgressFn progressFnPtr = static_cast<ClariusProgressFn>(&vtkPlusClariusOEM::vtkInternal::ProgressFn);
    ClariusErrorFn errorFnPtr = static_cast<ClariusErrorFn>(&vtkPlusClariusOEM::vtkInternal::ErrorFn);

    // No B-mode data sources. Disable B mode callback.
    std::vector<vtkPlusDataSource*> bModeSources;
    device->GetVideoSourcesByPortName(vtkPlusDevice::BMODE_PORT_NAME, bModeSources);
    if (bModeSources.empty())
    {
      newProcessedImageFnPtr = nullptr;
    }

    // No RF-mode data sources. Disable RF mode callback.
    std::vector<vtkPlusDataSource*> rfModeSources;
    device->GetVideoSourcesByPortName(vtkPlusDevice::RFMODE_PORT_NAME, rfModeSources);
    if (rfModeSources.empty())
    {
      newRawImageFnPtr = nullptr;
    }

    try
    {
      FrameSizeType fs = this->ImagingParameters->GetImageSize();

      int init_res = cusOemInit(
        argc,
        argv,
        path,
        connectFnPtr,
        certFnPtr,
        powerDownFnPtr,
        newProcessedImageFnPtr,
        newRawImageFnPtr,
        imagingFnPtr,
        buttonFnPtr,
        errorFnPtr,
        fs[0],
        fs[1]
      );

      if (init_res < 0)
      {
        return PLUS_FAIL;
      }
    }
    catch (const std::runtime_error& re)
    {
      LOG_ERROR("Runtime error: " << re.what());
      return PLUS_FAIL;
    }
    catch (const std::exception& ex)
    {
      LOG_ERROR("Error occurred: " << ex.what());
      return PLUS_FAIL;
    }
    catch (...)
    {
      LOG_ERROR("Unknown failure occured");
      return PLUS_FAIL;
    }

    // Attempt to connect;
    int isConnected = -1;
    const char* ip = device->Internal->IpAddress.c_str();
    try {
      isConnected = cusOemConnect(ip, device->Internal->TcpPort);
    }
    catch (const std::runtime_error& re)
    {
      LOG_ERROR("Runtime error: " << re.what());
      return PLUS_FAIL;
    }
    catch (const std::exception& ex)
    {
      LOG_ERROR("Error occurred: " << ex.what());
      return PLUS_FAIL;
    }
    catch (...)
    {
      LOG_ERROR("Unknown failure occured");
      return PLUS_FAIL;
    }

    if (isConnected < 0)
    {
      LOG_ERROR("Could not connect to scanner at Ip = " << ip << " port number= " << device->Internal->TcpPort << " isConnected = " << isConnected);
      return PLUS_FAIL;
    }

    /*device->UdpPort = clariusGetUdpPort();
    if (device->UdpPort != -1)
    {
      LOG_DEBUG("... Clarius device connected, streaming port: " << clariusGetUdpPort());
      if (clariusSetOutputSize(device->FrameWidth, device->FrameHeight) < 0)
      {
        LOG_DEBUG("Clarius Output size can not be set, falling back to default 640*480");
        device->FrameWidth = DEFAULT_FRAME_WIDTH;
        device->FrameHeight = DEFAULT_FRAME_HEIGHT;
      }
      return PLUS_SUCCESS;
    }
    else
    {
      LOG_ERROR("... Clarius device connected but could not get valid udp port");
      return PLUS_FAIL;
    }*/
  }
  else
  {
    LOG_DEBUG("Scanner already connected to IP address=" << device->Internal->IpAddress
      << " TCP Port Number =" << device->Internal->TcpPort << "Streaming Image at UDP Port=" << device->Internal->UdpPort);
    device->Connected = 1;
    return PLUS_SUCCESS;
  }

  return PLUS_FAIL;
};


//----------------------------------------------------------------------------
PlusStatus vtkPlusClariusOEM::InternalStartRecording()
{
  return PLUS_FAIL;
}

//----------------------------------------------------------------------------
PlusStatus vtkPlusClariusOEM::InternalStopRecording()
{
  return PLUS_FAIL;
}

//----------------------------------------------------------------------------
PlusStatus vtkPlusClariusOEM::InternalDisconnect()
{
  LOG_DEBUG("vtkPlusClariusOEM: InternalDisconnect");
  vtkPlusClariusOEM* device = vtkPlusClariusOEM::GetInstance();
  if (device->GetConnected())
  {
    if (cusOemDisconnect() < 0)
    {
      LOG_ERROR("could not disconnect from scanner");
      return PLUS_FAIL;
    }
    else
    {
      device->Connected = 0;
      LOG_DEBUG("Clarius device is now disconnected");
      return PLUS_SUCCESS;
    }
  }
  else
  {
    LOG_DEBUG("...Clarius device already disconnected");
    return PLUS_SUCCESS;
  }
};