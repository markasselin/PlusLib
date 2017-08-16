/*=Plus=header=begin======================================================
Program: Plus
Copyright (c) Laboratory for Percutaneous Surgery. All rights reserved.
See License.txt for details.
=========================================================Plus=header=end*/

#define MM_PER_M 1000

// Local includes
#include "PixelCodec.h"
#include "PlusConfigure.h"
#include "PlusVideoFrame.h"
#include "vtkPlusDataSource.h"
#include "vtkPlusOpticalMarkerTracker.h"

// VTK includes
#include <vtkExtractVOI.h>
#include <vtkImageData.h>
#include <vtkImageImport.h>
#include <vtkMath.h>
#include <vtkMatrix4x4.h>
#include <vtkObjectFactory.h>

// OS includes
#include <fstream>
#include <iostream>
#include <set>

// RANSAC includes
#include <RANSAC.h>
#include <ParametersEstimator.h>
#include <PlaneParametersEstimator.h>

// VNL includes
#include <vnl_cross.h>

// aruco includes
#include <markerdetector.h>
#include <cameraparameters.h>
#include <dictionary.h>
#include <posetracker.h>

// OpenCV includes
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>

//TODO: for testing
#include "vtkXMLPolyDataReader.h"
#include "vtkPolyDataMapper.h"
#include "vtkActor.h"
#include "vtkRenderer.h"
#include "vtkRenderWindow.h"
#include "vtkRenderWindowInteractor.h"
#include "vtkAxesActor.h"
#include "vtkOrientationMarkerWidget.h"
// are you really for testing?
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"

// TODO: clean this up... shouldn't have global vars (move them into their respective methods)
static const int CHANNEL_INDEX_VIDEO = 0;
static const int CHANNEL_INDEX_POLYDATA = 1;
static const int LEFT_BOUNDARY = false;
static const int RIGHT_BOUNDARY = true;
static const double PI = 3.14159265358979323846;

//----------------------------------------------------------------------------
vtkStandardNewMacro(vtkPlusOpticalMarkerTracker);

namespace
{
  class TrackedTool
  {
  public:
    enum TOOL_MARKER_TYPE
    {
      SINGLE_MARKER,
      MARKER_MAP
    };

    TrackedTool(int markerId, float markerSizeMm, const std::string& toolSourceId)
      : ToolMarkerType(SINGLE_MARKER)
      , MarkerId(markerId)
      , MarkerSizeMm(markerSizeMm)
      , ToolSourceId(toolSourceId)
    {
    }
    TrackedTool(const std::string& markerMapFile, const std::string& toolSourceId)
      : ToolMarkerType(MARKER_MAP)
      , MarkerMapFile(markerMapFile)
      , ToolSourceId(toolSourceId)
    {
    }

    int MarkerId;
    TOOL_MARKER_TYPE ToolMarkerType;
    float MarkerSizeMm;

    std::string MarkerMapFile;
    std::string ToolSourceId;
    std::string ToolName;
    aruco::MarkerPoseTracker MarkerPoseTracker;
    vtkSmartPointer<vtkMatrix4x4> OpticalMarkerToCamera = vtkSmartPointer<vtkMatrix4x4>::New();
    vtkSmartPointer<vtkMatrix4x4> DepthMarkerToCamera = vtkSmartPointer<vtkMatrix4x4>::New();
  };
}

//----------------------------------------------------------------------------
class vtkPlusOpticalMarkerTracker::vtkInternal : public vtkObject
{
public:
  vtkPlusOpticalMarkerTracker* External;

  vtkInternal(vtkPlusOpticalMarkerTracker* external)
    : External(external)
    , MarkerDetector(std::make_shared<aruco::MarkerDetector>())
    , CameraParameters(std::make_shared<aruco::CameraParameters>())
  {
  }

  virtual ~vtkInternal()
  {
    MarkerDetector = nullptr;
    CameraParameters = nullptr;
  }

  vtkGetMacro(TrackingMethod, TRACKING_METHOD);
  vtkGetStdStringMacro(CameraCalibrationFile);
  vtkGetStdStringMacro(MarkerDictionary);

public:
  PlusStatus BuildTransformMatrix(vtkSmartPointer<vtkMatrix4x4> transformMatrix, const cv::Mat& Rvec, const cv::Mat& Tvec);

  vtkSetMacro(TrackingMethod, TRACKING_METHOD);
  vtkSetStdStringMacro(CameraCalibrationFile);
  vtkSetStdStringMacro(MarkerDictionary);

  std::string               CameraCalibrationFile;
  TRACKING_METHOD           TrackingMethod;
  std::string               MarkerDictionary;
  std::vector<TrackedTool>  Tools;

  /*! Pointer to main aruco objects */
  std::shared_ptr<aruco::MarkerDetector>    MarkerDetector;
  std::shared_ptr<aruco::CameraParameters>  CameraParameters;
  std::vector<aruco::Marker>                Markers;
};

//----------------------------------------------------------------------------
vtkPlusOpticalMarkerTracker::vtkPlusOpticalMarkerTracker()
  : vtkPlusDevice()
  , Internal(new vtkInternal(this))
{
  this->FrameNumber = 0;
  this->StartThreadForInternalUpdates = true;
}

//----------------------------------------------------------------------------
vtkPlusOpticalMarkerTracker::~vtkPlusOpticalMarkerTracker()
{
  delete Internal;
  Internal = nullptr;
}

//----------------------------------------------------------------------------
void vtkPlusOpticalMarkerTracker::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
}


//----------------------------------------------------------------------------
PlusStatus vtkPlusOpticalMarkerTracker::ReadConfiguration(vtkXMLDataElement* rootConfigElement)
{
  // TODO: Improve error checking
  XML_FIND_DEVICE_ELEMENT_REQUIRED_FOR_READING(deviceConfig, rootConfigElement);

  XML_READ_STRING_ATTRIBUTE_NONMEMBER_REQUIRED("CameraCalibrationFile", this->Internal->CameraCalibrationFile, deviceConfig);
  XML_READ_ENUM2_ATTRIBUTE_NONMEMBER_OPTIONAL("TrackingMethod", this->Internal->TrackingMethod, deviceConfig, "OPTICAL", TRACKING_OPTICAL, "OPTICAL_AND_DEPTH", TRACKING_OPTICAL_AND_DEPTH);
  XML_READ_STRING_ATTRIBUTE_NONMEMBER_REQUIRED("MarkerDictionary", this->Internal->MarkerDictionary, deviceConfig);

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

    const char* toolId = toolDataElement->GetAttribute("Id");
    if (toolId == NULL)
    {
      // tool doesn't have ID needed to generate transform
      LOG_ERROR("Failed to initialize OpticalMarkerTracking tool: DataSource Id is missing");
      continue;
    }

    PlusTransformName toolTransformName(toolId, this->GetToolReferenceFrameName());
    std::string toolSourceId = toolTransformName.GetTransformName();

    if (toolDataElement->GetAttribute("MarkerId") != NULL && toolDataElement->GetAttribute("MarkerSizeMm") != NULL)
    {
      // this tool is tracked by a single marker
      int MarkerId;
      toolDataElement->GetScalarAttribute("MarkerId", MarkerId);
      float MarkerSizeMm;
      toolDataElement->GetScalarAttribute("MarkerSizeMm", MarkerSizeMm);
      TrackedTool newTool(MarkerId, MarkerSizeMm, toolSourceId);
      this->Internal->Tools.push_back(newTool);
    }
    else if (toolDataElement->GetAttribute("MarkerMapFile") != NULL)
    {
      // this tool is tracked by a marker map
      // TODO: Implement marker map tracking.
    }
    else
    {
      LOG_ERROR("Incorrectly formatted tool data source.");
    }
  }

  //TODO: read tracking type from number of inputs
  this->TrackingMethod = TRACKING_OPTICAL_AND_DEPTH;
  return PLUS_SUCCESS;
}

//----------------------------------------------------------------------------
PlusStatus vtkPlusOpticalMarkerTracker::WriteConfiguration(vtkXMLDataElement* rootConfigElement)
{
  XML_FIND_DEVICE_ELEMENT_REQUIRED_FOR_WRITING(deviceConfig, rootConfigElement);

  if (!this->Internal->CameraCalibrationFile.empty())
  {
    deviceConfig->SetAttribute("CameraCalibrationFile", this->Internal->CameraCalibrationFile.c_str());
  }
  if (!this->Internal->MarkerDictionary.empty())
  {
    deviceConfig->SetAttribute("MarkerDictionary", this->Internal->MarkerDictionary.c_str());
  }
  switch (this->Internal->TrackingMethod)
  {
  case TRACKING_OPTICAL:
    deviceConfig->SetAttribute("TrackingMethod", "OPTICAL");
    break;
  case TRACKING_OPTICAL_AND_DEPTH:
    deviceConfig->SetAttribute("TrackingMethod", "OPTICAL_AND_DEPTH");
    break;
  default:
    LOG_ERROR("Unknown tracking method passed to vtkPlusOpticalMarkerTracker::WriteConfiguration");
    return PLUS_FAIL;
  }

  //TODO: Write data for custom attributes

  return PLUS_SUCCESS;
}

//----------------------------------------------------------------------------
PlusStatus vtkPlusOpticalMarkerTracker::Probe()
{
  return PLUS_SUCCESS;
}

//----------------------------------------------------------------------------
PlusStatus vtkPlusOpticalMarkerTracker::InternalConnect()
{
  // get calibration file path && check file exists
  std::string calibFilePath = vtkPlusConfig::GetInstance()->GetDeviceSetConfigurationPath(this->Internal->CameraCalibrationFile);
  LOG_INFO("Use aruco camera calibration file located at: " << calibFilePath);
  if (!vtksys::SystemTools::FileExists(calibFilePath.c_str(), true))
  {
    LOG_ERROR("Unable to find aruco camera calibration file at: " << calibFilePath);
    return PLUS_FAIL;
  }

  // TODO: Need error handling for this?
  this->Internal->CameraParameters->readFromXMLFile(calibFilePath);
  this->Internal->MarkerDetector->setDictionary(this->Internal->MarkerDictionary);
  // threshold tuning numbers from aruco_test
  this->Internal->MarkerDetector->setThresholdParams(7, 7);
  this->Internal->MarkerDetector->setThresholdParamRange(2, 0);

  bool lowestRateKnown = false;
  double lowestRate = 30; // just a usual value (FPS)
  for (ChannelContainerConstIterator it = begin(this->InputChannels); it != end(this->InputChannels); ++it)
  {
    vtkPlusChannel* anInputStream = (*it);
    if (anInputStream->GetOwnerDevice()->GetAcquisitionRate() < lowestRate || !lowestRateKnown)
    {
      lowestRate = anInputStream->GetOwnerDevice()->GetAcquisitionRate();
      lowestRateKnown = true;
    }
  }
  if (lowestRateKnown)
  {
    this->AcquisitionRate = lowestRate;
  }
  else
  {
    LOG_WARNING("vtkPlusOpticalMarkerTracker acquisition rate is not known");
  }

  this->LastProcessedInputDataTimestamp = 0;
  return PLUS_SUCCESS;
}

//----------------------------------------------------------------------------
PlusStatus vtkPlusOpticalMarkerTracker::InternalDisconnect()
{
  return PLUS_SUCCESS;
}

//----------------------------------------------------------------------------
PlusStatus vtkPlusOpticalMarkerTracker::InternalStartRecording()
{
  return PLUS_SUCCESS;
}

//----------------------------------------------------------------------------
PlusStatus vtkPlusOpticalMarkerTracker::InternalStopRecording()
{
  return PLUS_SUCCESS;
}

//----------------------------------------------------------------------------
PlusStatus vtkPlusOpticalMarkerTracker::vtkInternal::BuildTransformMatrix(
  vtkSmartPointer<vtkMatrix4x4> transformMatrix,
  const cv::Mat& Rvec,
  const cv::Mat& Tvec)
{
  transformMatrix->Identity();
  cv::Mat Rmat(3, 3, CV_32FC1);
  try
  {
    cv::Rodrigues(Rvec, Rmat);
  }
  catch (...)
  {
    return PLUS_FAIL;
  }

  for (int x = 0; x <= 2; x++)
  {
    transformMatrix->SetElement(x, 3, MM_PER_M * Tvec.at<float>(x, 0));
    for (int y = 0; y <= 2; y++)
    {
      transformMatrix->SetElement(x, y, Rmat.at<float>(x, y));
    }
  }

  return PLUS_SUCCESS;
}

//----------------------------------------------------------------------------
vtkPlusOpticalMarkerTracker::MARKER_ORIENTATION vtkPlusOpticalMarkerTracker::DetermineMarkerOrientation(std::vector<cv::Point2d>& corners)
{
  double yMin = corners[0].y;
  int top = 0;
  for (int i = 1; i < 4; i++)
  {
    if (corners[i].y < yMin)
    {
      yMin = corners[i].y;
      top = i;
    }
  }

  std::vector<cv::Point2d> orderedCorners;
  // set vertices in clockwise order (top = 0, ...)
  for (int i = 0; i < 4; i++) {
    orderedCorners.push_back(corners[(top + i) % 4]);
  }

  corners = orderedCorners;

  // find index of bottom corner
  int yMax = corners[0].y;
  int bottom = 0;
  for (int i = 1; i < 4; i++)
  {
    if (corners[i].y > yMax)
    {
      yMax = corners[i].y;
      bottom = i;
    }
  }

  switch (bottom)
  {
  case 1:
    LOG_INFO("SKEW_LEFT");
    return SKEW_LEFT;
  case 2:
    LOG_INFO("ROTATED");
    return ROTATED;
  default:
    LOG_INFO("SKEW_RIGHT");
    return SKEW_RIGHT;
  }
}

//----------------------------------------------------------------------------
void vtkPlusOpticalMarkerTracker::CopyToItkData(
  vtkSmartPointer<vtkPolyData> vtkDepthData,
  std::vector<itk::Point<double, 3>> &itkData,
  int top,
  int bottom,
  int *leftBoundary,
  int *rightBoundary,
  cv::Mat image)
{
  itk::Point<double, 3> itkPoint;
  // for testing
  vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
  vtkSmartPointer<vtkCellArray> vertices = vtkSmartPointer<vtkCellArray>::New();

  for (int yPx = top; yPx <= bottom; yPx++)
  {
    for (int xPx = leftBoundary[yPx - top]; xPx <= rightBoundary[yPx - top]; xPx++)
    {
      // color in image for visualization
      cv::Vec3b color = image.at<cv::Vec3b>(cv::Point(xPx, yPx));
      color[0] = 0;
      color[1] = 0;
      color[2] = 255;
      //image.at<cv::Vec3b>(cv::Point(xPx, yPx)) = color;

      // TODO: Use non hard-coded dimension
      vtkIdType ptId = 640 * yPx + xPx;
      double vtkPoint[3];
      vtkDepthData->GetPoint(ptId, vtkPoint);

      // depth filter to select only points between 5cm and 200cm
      if (vtkPoint[2] > 50 && vtkPoint[2] < 2000)
      {
        itkPoint[0] = vtkPoint[0];
        itkPoint[1] = vtkPoint[1];
        itkPoint[2] = vtkPoint[2];
        itkData.push_back(itkPoint);

        //LOG_WARNING("x: " << xPx << " y: " << yPx);
        //TODO: use non hard-coded dimensions here
        vtkIdType pid[1];
        pid[0] = points->InsertNextPoint(vtkPoint[0], vtkPoint[1], vtkPoint[2]);
        vertices->InsertNextCell(1, pid);
      }
    }
  }

  if (false)
  {
    vtkSmartPointer<vtkPolyData> polyPlane = vtkSmartPointer<vtkPolyData>::New();
    polyPlane->SetPoints(points);
    polyPlane->SetVerts(vertices);

    // show polydata plane for testing
    vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputData(polyPlane);
    vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);
    vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
    vtkSmartPointer<vtkRenderWindow> renderWindow = vtkSmartPointer<vtkRenderWindow>::New();
    renderWindow->AddRenderer(renderer);
    vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
    renderWindowInteractor->SetRenderWindow(renderWindow);
    renderer->AddActor(actor);
    renderer->SetBackground(.2, .3, .4);
    vtkSmartPointer<vtkAxesActor> axes = vtkSmartPointer<vtkAxesActor>::New();
    vtkSmartPointer<vtkOrientationMarkerWidget> widget = vtkSmartPointer<vtkOrientationMarkerWidget>::New();
    widget->SetOutlineColor(0.93, 0.57, 0.13);
    widget->SetOrientationMarker(axes);
    widget->SetInteractor(renderWindowInteractor);
    widget->SetViewport(0, 0, 0.4, 0.4);
    widget->SetEnabled(1);
    widget->InteractiveOn();
    renderer->ResetCamera();
    renderWindow->Render();
    renderWindowInteractor->Start();
    LOG_INFO("num points: " << polyPlane->GetNumberOfPoints());
  }
}

//----------------------------------------------------------------------------
float vtkPlusOpticalMarkerTracker::DetermineSlope(cv::Point2d corner1, cv::Point2d corner2)
{
  if (corner1.y == corner2.y)
  {
    return 0.0;
  }
  else
  {
    return ((float)(corner1.x - corner2.x)) / (corner1.y - corner2.y);
  }
}

//----------------------------------------------------------------------------
void vtkPlusOpticalMarkerTracker::GenerateBoundary(int* boundary, std::vector<cv::Point2d> corners, int top, bool isRight)
{
  int numSegments = corners.size() - 1;

  for (int segIndex = 0; segIndex < numSegments; segIndex++)
  {
    int segTop = corners[segIndex].y;
    int segBottom = corners[segIndex + 1].y;
    float mPx = DetermineSlope(corners[segIndex], corners[segIndex + 1]);
    int x1Px = corners[segIndex].x;
    int y1Px = corners[segIndex].y;
    for (int yPx = segTop; yPx <= segBottom; yPx++)
    {
      boundary[yPx - top] = mPx * (yPx - y1Px) + x1Px;
    }
  }
}

//----------------------------------------------------------------------------
void vtkPlusOpticalMarkerTracker::GenerateSkewLeftItkData(
  vtkSmartPointer<vtkPolyData> vtkDepthData,
  std::vector<itk::Point<double, 3>> &itkData,
  std::vector<cv::Point2d> corners,
  /*for testing*/
  unsigned int dim[],
  cv::Mat image)
{
  const char TOP = 0, BOTTOM = 1, LOWER_LEFT = 2, UPPER_LEFT = 3;
  int top = corners[TOP].y, bottom = corners[BOTTOM].y;
  int height = bottom - top;

  //LOG_WARNING("TOP        " << corners[TOP].y);
  //LOG_WARNING("BOTTOM     " << corners[BOTTOM].y);
  //LOG_WARNING("UPPER LEFT " << corners[UPPER_LEFT].y);
  //LOG_WARNING("LOWER LEFT " << corners[LOWER_LEFT].y);

  // generate left boundary
  int* leftBoundary = new int[height];
  std::vector<cv::Point2d> leftPath;
  leftPath.push_back(corners[TOP]);
  leftPath.push_back(corners[UPPER_LEFT]);
  leftPath.push_back(corners[LOWER_LEFT]);
  leftPath.push_back(corners[BOTTOM]);
  GenerateBoundary(leftBoundary, leftPath, top, LEFT_BOUNDARY);

  // generate right boundary
  int* rightBoundary = new int[height];
  std::vector<cv::Point2d> rightPath;
  rightPath.push_back(corners[TOP]);
  rightPath.push_back(corners[BOTTOM]);
  GenerateBoundary(rightBoundary, rightPath, top, RIGHT_BOUNDARY);

  // copy vtk->itk
  CopyToItkData(vtkDepthData, itkData, top, bottom, leftBoundary, rightBoundary, image);
}

//----------------------------------------------------------------------------
void vtkPlusOpticalMarkerTracker::GenerateSkewRightItkData(
  vtkSmartPointer<vtkPolyData> vtkDepthData,
  std::vector<itk::Point<double, 3>> &itkData,
  std::vector<cv::Point2d> corners,
  /*for testing*/
  unsigned int dim[],
  cv::Mat image)
{
  const char TOP = 0, UPPER_RIGHT = 1, LOWER_RIGHT = 2, BOTTOM = 3;
  int top = corners[TOP].y;
  int bottom = corners[BOTTOM].y;
  int height = bottom - top + 1;

  //LOG_WARNING("TOP          x: " << corners[TOP].x << " y: " << corners[TOP].y);
  //LOG_WARNING("BOTTOM       X: " << corners[BOTTOM].x << " y: " << corners[BOTTOM].y);
  //LOG_WARNING("UPPER RIGHT  x: " << corners[UPPER_RIGHT].x << " y: " << corners[UPPER_RIGHT].y);
  //LOG_WARNING("LOWER RIGHT  x: " << corners[LOWER_RIGHT].x << " y: " << corners[LOWER_RIGHT].y);

  // generate left boundary
  int* leftBoundary = new int[height];
  std::vector<cv::Point2d> leftPath;
  leftPath.push_back(corners[TOP]);
  leftPath.push_back(corners[BOTTOM]);
  GenerateBoundary(leftBoundary, leftPath, top, LEFT_BOUNDARY);

  // generate right boundary
  int* rightBoundary = new int[height];
  std::vector<cv::Point2d> rightPath;
  rightPath.push_back(corners[TOP]);
  rightPath.push_back(corners[UPPER_RIGHT]);
  rightPath.push_back(corners[LOWER_RIGHT]);
  rightPath.push_back(corners[BOTTOM]);
  GenerateBoundary(rightBoundary, rightPath, top, RIGHT_BOUNDARY);

  // copy vtk->itk
  CopyToItkData(vtkDepthData, itkData, top, bottom, leftBoundary, rightBoundary, image);
}

//----------------------------------------------------------------------------
void vtkPlusOpticalMarkerTracker::GenerateRotatedItkData(
  vtkSmartPointer<vtkPolyData> vtkDepthData,
  std::vector<itk::Point<double, 3>> &itkData,
  std::vector<cv::Point2d> corners,
  /*for testing*/
  unsigned int dim[],
  cv::Mat image)
{
  const char TOP = 0, RIGHT = 1, BOTTOM = 2, LEFT = 3;
  int top = corners[TOP].y;
  int bottom = corners[BOTTOM].y;
  int height = bottom - top + 1;

  //LOG_WARNING("TOP    x:" << corners[TOP].x << " y: " << corners[TOP].y);
  //LOG_WARNING("BOTTOM x:" << corners[BOTTOM].x << " y: " << corners[BOTTOM].y);
  //LOG_WARNING("LEFT   x:" << corners[LEFT].x << " y: " << corners[LEFT].y);
  //LOG_WARNING("RIGHT  x:" << corners[RIGHT].x << " y: " << corners[RIGHT].y);
  //LOG_WARNING("height: " << height);

  // generate left boundary
  int* leftBoundary = new int[height];
  std::vector<cv::Point2d> leftPath;
  leftPath.push_back(corners[TOP]);
  leftPath.push_back(corners[LEFT]);
  leftPath.push_back(corners[BOTTOM]);
  GenerateBoundary(leftBoundary, leftPath, top, false);

  // generate right boundary
  int* rightBoundary = new int[height];
  std::vector<cv::Point2d> rightPath;
  rightPath.push_back(corners[TOP]);
  rightPath.push_back(corners[RIGHT]);
  rightPath.push_back(corners[BOTTOM]);
  GenerateBoundary(rightBoundary, rightPath, top, true);

  // copy vtk->itk
  CopyToItkData(vtkDepthData, itkData, top, bottom, leftBoundary, rightBoundary, image);
}

//----------------------------------------------------------------------------
void vtkPlusOpticalMarkerTracker::GenerateItkData(
  vtkSmartPointer<vtkPolyData> vtkDepthData,
  std::vector<itk::Point<double, 3>> &itkData,
  std::vector<cv::Point2d> corners,
  /*for testing*/
  unsigned int dim[],
  cv::Mat image)
{
  MARKER_ORIENTATION orientation = DetermineMarkerOrientation(corners);

  switch (orientation)
  {
  case SKEW_LEFT:
    GenerateSkewLeftItkData(vtkDepthData, itkData, corners, dim, image);
    return;
  case ROTATED:
    GenerateRotatedItkData(vtkDepthData, itkData, corners, dim, image);
    return;
  case SKEW_RIGHT:
    GenerateSkewRightItkData(vtkDepthData, itkData, corners, dim, image);
    return;
  }
}

//----------------------------------------------------------------------------
double* vtkPlusOpticalMarkerTracker::GetXAxis(
  vtkSmartPointer<vtkPolyData> vtkDepthData,
  std::vector<cv::Point2d> corners,
  cv::Mat image)
{
  cv::Point2d xBeginPx, xEndPx;

  xBeginPx.x = round((float)(corners[0].x + corners[3].x)/2);
  xBeginPx.y = round((float)(corners[0].y + corners[3].y)/2);
  xEndPx.x = round((float)(corners[1].x + corners[2].x)/2);
  xEndPx.y = round((float)(corners[1].y + corners[2].y)/2);

  double* x_axis = new double[3];
  x_axis[0] = 0;
  x_axis[1] = 0;
  x_axis[2] = 0;

  // compute slope of x_axis in image
  double mPx;
  if (xBeginPx.y == xEndPx.y)
  {
    mPx = 0.0;
  }
  else
  {
    mPx = ((float)(xBeginPx.y - xEndPx.y)) / (xBeginPx.x- xEndPx.x);
  }

  vtkIdType ptId = 640 * xBeginPx.y + xBeginPx.x;
  double* prevVtkPoint = new double[3];
  vtkDepthData->GetPoint(ptId, prevVtkPoint);
  int count = 0;

  // find direction of x_axis in spatial coordinates
  if (xBeginPx.x < xEndPx.x)
  {
    // x axis goes left of image to right
    for (int xPx = xBeginPx.x + 1; xPx <= xEndPx.x; xPx++)
    {
      int yPx = xBeginPx.y + mPx*(xPx - xBeginPx.x);
      // color in image for visualization
      cv::Vec3b color = image.at<cv::Vec3b>(cv::Point(xPx, yPx));
      color[0] = 0;
      color[1] = 0;
      color[2] = 255;
      image.at<cv::Vec3b>(cv::Point(xPx, yPx)) = color;

      vtkIdType ptId = 640 * yPx + xPx;
      double vtkPoint[3];
      vtkDepthData->GetPoint(ptId, vtkPoint);

      if (vtkPoint[2] > 50 && vtkPoint[2] < 2000 && prevVtkPoint[2] > 50 && prevVtkPoint[2] < 2000)
      {
        x_axis[0] += vtkPoint[0] - prevVtkPoint[0];
        x_axis[1] += vtkPoint[1] - prevVtkPoint[1];
        x_axis[2] += vtkPoint[2] - prevVtkPoint[2];
        count++;
      }
      prevVtkPoint = vtkPoint;
    }
  }
  else
  {
    // x axis goes right of image to left
    for (int xPx = xBeginPx.x - 1; xPx >= xEndPx.x; xPx--)
    {
      int yPx = xBeginPx.y + mPx*(xPx - xBeginPx.x);
      // color in image for visualization
      cv::Vec3b color = image.at<cv::Vec3b>(cv::Point(xPx, yPx));
      color[0] = 0;
      color[1] = 255;
      color[2] = 0;
      image.at<cv::Vec3b>(cv::Point(xPx, yPx)) = color;

      vtkIdType ptId = 640 * yPx + xPx;
      double vtkPoint[3];
      vtkDepthData->GetPoint(ptId, vtkPoint);

      if (vtkPoint[2] > 50 && vtkPoint[2] < 2000 && prevVtkPoint[2] > 50 && prevVtkPoint[2] < 2000)
      {
        x_axis[0] += vtkPoint[0] - prevVtkPoint[0];
        x_axis[1] += vtkPoint[1] - prevVtkPoint[1];
        x_axis[2] += vtkPoint[2] - prevVtkPoint[2];
        count++;
      }
      prevVtkPoint = vtkPoint;
    }

    x_axis[0] /= count;
    x_axis[1] /= count;
    x_axis[2] /= count;
  }

  return x_axis;
}

//----------------------------------------------------------------------------
cv::Point2d vtkPlusOpticalMarkerTracker::GetMarkerCenter(std::vector<cv::Point2d> corners) {
  cv::Point2d center;
  // slope of lines
  // TODO: handle special case where corners have same x coordinate -> inf slope -> crash
  double m02 = (corners[0].y - corners[2].y) / (corners[0].x - corners[2].x);
  double m13 = (corners[1].y - corners[3].y) / (corners[1].x - corners[3].x);
  // points on lines
  int x0 = corners[0].x, y0 = corners[0].y;
  int x1 = corners[1].x, y1 = corners[1].y;

  float x_center = (m02*x0 - m13*x1 + y1 - y0) / (m02 - m13);
  center.x = round(x_center);
  center.y = round(m02*(x_center - x0) + y0);

  //LOG_INFO("center x: " << center.x << "center y: " << center.y);

  //LOG_INFO("0+2x: " << (corners[0].x + corners[2].x) / 2);
  //LOG_INFO("0+2y: " << (corners[0].y + corners[2].y) / 2);
  //LOG_INFO("1+3x: " << (corners[1].x + corners[3].x) / 2);
  //LOG_INFO("1+3y: " << (corners[1].y + corners[3].y) / 2);

  return center;
}

//----------------------------------------------------------------------------
double* vtkPlusOpticalMarkerTracker::MarkerCenterImageToSpatial(vtkSmartPointer<vtkPolyData> vtkDepthData, cv::Point2d imageCenter, cv::Mat image)
{
  //cv::circle(image, markerImageCenter, 2, cv::Scalar(0, 0, 255), -1);
  return new double[3];
}

//----------------------------------------------------------------------------
float vtkPlusOpticalMarkerTracker::VectorAngleDeg(vnl_vector<double> xAxis, vnl_vector<double> zAxis)
{
  float dotProduct = xAxis(0)*zAxis(0) + xAxis(1)*zAxis(1) + xAxis(2)*zAxis(2);
  return abs(acos(dotProduct) * 180 / PI); 
}

//----------------------------------------------------------------------------
/*inline*/ void vtkPlusOpticalMarkerTracker::ComputePlaneTransform(
  vtkSmartPointer<vtkMatrix4x4> MarkerToDepthCamera,
  double x_axis[],
  double z_axis[],
  double center[])
{
  //vnl_matrix<double> CameraPoints(3, 3, 0.0);
  //vnl_matrix<double> MarkerPoints(3, 3, 0.0);
  // TODO: add homogenous coordinate to all raw array vectors
  double z_expected[4] = { 0, 0, -1, 0 };
  z_axis[1] = -z_axis[1]; // left-handed to right-handed coord sys conversion
  float ZtoZAngle = vtkMath::Dot(z_expected, z_axis);
  LOG_INFO("ZtoZangle: " << ZtoZAngle);
  if (ZtoZAngle < 0)
  {
    // normal is pointing towards back of marker, flip it, as below
    z_axis[0] *= -1;
    z_axis[1] *= -1;
    z_axis[2] *= -1;
    z_axis[3] *= -1; // does the homogenous coordinate need to be flipped?
  }
  vnl_vector<double> xGuess(3, 3, x_axis);
  vnl_vector<double> zAxis(3, 3, z_axis);
  xGuess.normalize();
  zAxis.normalize();

  double x_theoretical[3] = { 1, 0, 0 };
  vnl_vector<double> xTheoretical(3, 3, x_theoretical);
  double y_theoretical[3] = { 0, 1, 0 };
  vnl_vector<double> yTheoretical(3, 3, y_theoretical);

  vnl_vector<double> yAxis;
  vnl_vector<double> xAxis;

  LOG_ERROR(VectorAngleDeg(xGuess, zAxis));
  if (VectorAngleDeg(xGuess, zAxis) > 10)
  {
    LOG_ERROR("Using x_axis from aruco");
    yAxis = vnl_cross_3d(zAxis, xGuess);
    xAxis = vnl_cross_3d(yAxis, zAxis);
  }
  else if (VectorAngleDeg(xTheoretical, zAxis) > 10)
  {
    LOG_ERROR("using theoretical x_axis");
    yAxis = vnl_cross_3d(zAxis, xTheoretical);
    xAxis = vnl_cross_3d(yAxis, zAxis);
  }
  else
  {
    LOG_ERROR("Using theoretical y_axis as perpendicular to Z");
    xAxis = vnl_cross_3d(yTheoretical, zAxis);
    yAxis = vnl_cross_3d(zAxis, xAxis);
  }

  vnl_matrix<double> Rotation(3, 3);
  Rotation.set_column(0, xAxis);
  Rotation.set_column(1, yAxis);
  Rotation.set_column(2, zAxis);
  /*
  // camera x axis
  CameraPoints.put(0, 0, 1);
  CameraPoints.put(0, 1, 0);
  CameraPoints.put(0, 2, 0);
  // camera z axis
  CameraPoints.put(1, 0, 0);
  CameraPoints.put(1, 1, 1);
  CameraPoints.put(1, 2, 0);
  // camera origin
  CameraPoints.put(2, 0, 0);
  CameraPoints.put(2, 1, 0);
  CameraPoints.put(2, 2, 1);

  // marker x axis
  MarkerPoints.put(0, 0, xAxis(0));
  MarkerPoints.put(0, 1, xAxis(1));
  MarkerPoints.put(0, 2, xAxis(2));
  // marker z axis (n)
  MarkerPoints.put(1, 0, yAxis(0));
  MarkerPoints.put(1, 1, yAxis(1));
  MarkerPoints.put(1, 2, yAxis(2));

  // marker origin (center of mass of marker)
  MarkerPoints.put(2, 0, zAxis(0));
  MarkerPoints.put(2, 1, zAxis(1));
  MarkerPoints.put(2, 2, zAxis(2));

  //MarkerPoints.put(2, 0, center[0]);
  //MarkerPoints.put(2, 1, center[1]);
  //MarkerPoints.put(2, 2, center[2]);

  vnl_svd<double> CameraToMarker(CameraPoints.transpose() * MarkerPoints);
  vnl_matrix<double> V = CameraToMarker.V();
  vnl_matrix<double> U = CameraToMarker.U();
  vnl_matrix<double> Rotation = V * U.transpose();

  LOG_WARNING("ROTATION: ");
  cout << Rotation;
  */
  //TODO: generate 4x4 transform here
  MarkerToDepthCamera->Identity();
  for (int row = 0; row <= 2; row++)
  {
    
    for (int col = 0; col <= 2; col++)
      MarkerToDepthCamera->SetElement(row, col, Rotation(row, col));
  }
  MarkerToDepthCamera->SetElement(0, 3, center[0]);
  MarkerToDepthCamera->SetElement(1, 3, -center[1]);
  MarkerToDepthCamera->SetElement(2, 3, center[2]);

  vtkSmartPointer<vtkMatrix4x4> DepthAxisToPlusTrackerAxis = vtkSmartPointer<vtkMatrix4x4>::New();
  DepthAxisToPlusTrackerAxis->Zero();
  DepthAxisToPlusTrackerAxis->SetElement(0, 0, 1);
  DepthAxisToPlusTrackerAxis->SetElement(1, 1, 1);
  DepthAxisToPlusTrackerAxis->SetElement(2, 2, 1);
  DepthAxisToPlusTrackerAxis->SetElement(3, 3, 1);

  //vtkMatrix4x4::Multiply4x4(DepthAxisToPlusTrackerAxis, MarkerToDepthCamera, MarkerToDepthCamera);
}

//----------------------------------------------------------------------------
PlusStatus vtkPlusOpticalMarkerTracker::InternalUpdate()
{
  if (this->TrackingMethod == TRACKING_OPTICAL)
  {
    if (this->InputChannels.size() != 1)
    {
      LOG_ERROR("OpticalMarkerTracker device requires exactly 1 input stream (that contains video data). Check configuration.");
      return PLUS_FAIL;
    }
  }
  else if (this->TrackingMethod = TRACKING_OPTICAL_AND_DEPTH)
  {
    if (this->InputChannels.size() != 2)
    {
      LOG_ERROR("OpticalMarkerTracker device requires exactly 2 input streams (that contains video data and depth data). Check configuration.");
      return PLUS_FAIL;
    }
  }

  // check if data is ready
  if (!this->InputChannels[CHANNEL_INDEX_VIDEO]->GetVideoDataAvailable())
  {
    LOG_TRACE("OpticalMarkerTracker is not tracking, video data is not available yet. Device ID: " << this->GetDeviceId());
    return PLUS_SUCCESS;
  }
  if (!this->InputChannels[CHANNEL_INDEX_POLYDATA]->GetBulkDataAvailable())
  {
    LOG_TRACE("OpticalMarkerTracker is not tracking, video data is not available yet. Device ID: " << this->GetDeviceId());
    return PLUS_SUCCESS;
  }

  // get timestamp of frame to process from PolyData (as it is added to the buffers after video)
  double oldestTrackingTimestamp(0);
  if (this->InputChannels[CHANNEL_INDEX_POLYDATA]->GetLatestTimestamp(oldestTrackingTimestamp) == PLUS_SUCCESS)
  {
    if (this->LastProcessedInputDataTimestamp > oldestTrackingTimestamp)
    {
      LOG_INFO("Processed image generation started. No tracking data was available between " << this->LastProcessedInputDataTimestamp << "-" << oldestTrackingTimestamp <<
        "sec, therefore no processed images were generated during this time period.");
      this->LastProcessedInputDataTimestamp = oldestTrackingTimestamp;
    }
  }

  // grab tracked frames to process from buffer
  PlusTrackedFrame trackedVideoFrame;
  PlusTrackedFrame trackedPolyDataFrame;
  if (this->TrackingMethod == TRACKING_OPTICAL || this->TrackingMethod == TRACKING_OPTICAL_AND_DEPTH)
  {
    // get optical video data
    if (this->InputChannels[CHANNEL_INDEX_VIDEO]->GetTrackedFrame(trackedVideoFrame) != PLUS_SUCCESS)
    {
      LOG_ERROR("Error while getting latest tracked frame. Last recorded timestamp: " << std::fixed << this->LastProcessedInputDataTimestamp << ". Device ID: " << this->GetDeviceId());
      this->LastProcessedInputDataTimestamp = vtkPlusAccurateTimer::GetSystemTime(); // forget about the past, try to add frames that are acquired from now on
      return PLUS_FAIL;
    }
  }
  if (this->TrackingMethod = TRACKING_OPTICAL_AND_DEPTH)
  {
    // get depth PolyData
    if (this->InputChannels[CHANNEL_INDEX_POLYDATA]->GetTrackedFrame(oldestTrackingTimestamp, trackedPolyDataFrame) != PLUS_SUCCESS)
    {
      LOG_ERROR("Error while getting latest tracked frame. Last recorded timestamp: " << std::fixed << this->LastProcessedInputDataTimestamp << ". Device ID: " << this->GetDeviceId());
      this->LastProcessedInputDataTimestamp = vtkPlusAccurateTimer::GetSystemTime(); // forget about the past, try to add frames that are acquired from now on
      return PLUS_FAIL;
    }
  }

  //TODO: for testing visaulize polydata
  if (false) {
    vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputData(trackedPolyDataFrame.GetPolyData());
    vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);
    vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
    vtkSmartPointer<vtkRenderWindow> renderWindow = vtkSmartPointer<vtkRenderWindow>::New();
    renderWindow->AddRenderer(renderer);
    vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
    renderWindowInteractor->SetRenderWindow(renderWindow);
    renderer->AddActor(actor);
    renderer->SetBackground(.2, .3, .4);
    vtkSmartPointer<vtkAxesActor> axes = vtkSmartPointer<vtkAxesActor>::New();
    vtkSmartPointer<vtkOrientationMarkerWidget> widget = vtkSmartPointer<vtkOrientationMarkerWidget>::New();
    widget->SetOutlineColor(0.93, 0.57, 0.13);
    widget->SetOrientationMarker(axes);
    widget->SetInteractor(renderWindowInteractor);
    widget->SetViewport(0, 0, 0.4, 0.4);
    widget->SetEnabled(1);
    widget->InteractiveOn();
    renderer->ResetCamera();
    renderWindow->Render();
    renderWindowInteractor->Start();
  }

  // get dimensions & data
  FrameSizeType dim = trackedVideoFrame.GetFrameSize();
  PlusVideoFrame* frame = trackedVideoFrame.GetImageData();

  cv::Mat image(dim[1], dim[0], CV_8UC3);
  cv::Mat temp(dim[1], dim[0], CV_8UC3);

  //TODO: Flip image so that it's input to openCV in the correct orientation
  PlusVideoFrame *uprightOcvImage;
  PlusVideoFrame::FlipInfoType flip;

  // Plus image uses RGB and OpenCV uses BGR, swapping is only necessary for colored markers
  //PixelCodec::RgbBgrSwap(dim[0], dim[1], (unsigned char*)frame->GetScalarPointer(), temp.data);
  image.data = (unsigned char*)frame->GetScalarPointer();

  vtkSmartPointer<vtkPolyData> markerPolyData = trackedPolyDataFrame.GetPolyData();



  // detect markers in frame
  this->Internal->MarkerDetector->detect(image, this->Internal->Markers);
  const double unfilteredTimestamp = vtkPlusAccurateTimer::GetSystemTime();
  // iterate through tools updating tracking
  for (vector<TrackedTool>::iterator toolIt = begin(this->Internal->Tools); toolIt != end(this->Internal->Tools); ++toolIt)
  {
    // for SPIE 2017 to output both depth and optical transforms simultaneously
    // ignores depth named transforms
    if (toolIt->ToolSourceId.find("Depth") != string::npos)
    {
      continue;
    }

    bool toolInFrame = false;
    for (vector<aruco::Marker>::iterator markerIt = begin(this->Internal->Markers); markerIt != end(this->Internal->Markers); ++markerIt)
    {
      if (toolIt->MarkerId == markerIt->id) {
        //marker is in frame
        toolInFrame = true;

        if (toolIt->MarkerPoseTracker.estimatePose(*markerIt, *this->Internal->CameraParameters, toolIt->MarkerSizeMm / MM_PER_M, 4))
        {
          // UPDATE OPTICAL TRANSFORM
          cv::Mat Rvec = toolIt->MarkerPoseTracker.getRvec();
          cv::Mat Tvec = toolIt->MarkerPoseTracker.getTvec();
          cv::Mat Rmat(3, 3, CV_32FC1);
          cv::Rodrigues(Rvec, Rmat);
          this->Internal->BuildTransformMatrix(toolIt->OpticalMarkerToCamera, Rmat, Tvec);

          // UPDATE DEPTH TRANSFORM
          // get marker corners
          std::vector<cv::Point2d> corners;
          corners = markerIt->getCornersPx();

          // copy data from inside the marker into data structure for RANSAC plane algorithm
          std::vector<itk::Point<double, 3>> itkPlane;
          GenerateItkData(markerPolyData, itkPlane, corners, dim, image);

          cv::circle(image, corners[0], 2, cv::Scalar(0, 0, 255), -1);
          cv::circle(image, corners[1], 2, cv::Scalar(255, 0, 0), -1);

          // find plane normal and distance using RANSAC
          std::vector<double> ransacParameterResult;
          typedef itk::PlaneParametersEstimator<3> PlaneEstimatorType;
          typedef itk::RANSAC<itk::Point<double, 3>, double> RANSACType;

          //create and initialize the parameter estimator
          double maximalDistanceFromPlane = 0.5;
          PlaneEstimatorType::Pointer planeEstimator = PlaneEstimatorType::New();
          planeEstimator->SetDelta(maximalDistanceFromPlane);
          planeEstimator->LeastSquaresEstimate(itkPlane, ransacParameterResult);

          //create and initialize the RANSAC algorithm
          double desiredProbabilityForNoOutliers = 0.90;
          RANSACType::Pointer ransacEstimator = RANSACType::New();

          try
          {
            ransacEstimator->SetData(itkPlane);
          }
          catch (std::exception& e)
          {
            LOG_DEBUG(e.what());
            return PLUS_SUCCESS;
          }

          try
          {
            ransacEstimator->SetParametersEstimator(planeEstimator.GetPointer());
          }
          catch (std::exception& e)
          {
            LOG_DEBUG(e.what());
            return PLUS_SUCCESS;
          }

          // RANSAC causes massive pauses in tracking... Is it worth it?

          /*try
          {
            ransacEstimator->Compute(ransacParameterResult, desiredProbabilityForNoOutliers);
          }
          catch (std::exception& e)
          {
            LOG_DEBUG(e.what());
            return PLUS_SUCCESS;
          }*/

          // print results of least squares / RANSAC plane fit
          if (ransacParameterResult.empty())
          {
            LOG_WARNING("Unable to fit line through points with least squares estimation");
            return PLUS_SUCCESS;
          }
          else
          {
            LOG_INFO("Least squares line parameters (n, a):");
            for (unsigned int i = 0; i < (2 * 3); i++)
            {
              LOG_INFO(" RANSAC parameter: " << ransacParameterResult[i]);
            }
          }

          double zAxis[4];
          zAxis[0] = ransacParameterResult[0];
          zAxis[1] = ransacParameterResult[1];
          zAxis[2] = ransacParameterResult[2];
          zAxis[3] = 0;

          double xAxis[4];
          xAxis[0] = Rmat.at<float>(0, 0);
          xAxis[1] = Rmat.at<float>(1, 0);
          xAxis[2] = Rmat.at<float>(2, 0);
          xAxis[3] = 0;

          // center is currently computed using the center of mass of the plane from least squares,
          // this could be updated to be the intersection of a line from (0,0,0) to the plane center
          double center[4];
          center[0] = ransacParameterResult[3];
          center[1] = ransacParameterResult[4];
          center[2] = ransacParameterResult[5];
          center[3] = 0;

          ComputePlaneTransform(toolIt->DepthMarkerToCamera, xAxis, zAxis, center);
          ToolTimeStampedUpdate(toolIt->ToolSourceId, toolIt->OpticalMarkerToCamera, TOOL_OK, this->FrameNumber, unfilteredTimestamp);
          ToolTimeStampedUpdate("Depth" + toolIt->ToolSourceId, toolIt->DepthMarkerToCamera, TOOL_OK, this->FrameNumber, unfilteredTimestamp);
        }
        else
        {
          // pose estimation failed
          // TODO: add frame num, marker id, etc. Make this error more helpful.  Is there a way to handle it?
          LOG_ERROR("Pose estimation failed. Tool " << toolIt->ToolSourceId << " with marker " << toolIt->MarkerId << ".");
        }
        break;
      }
    }
    if (!toolInFrame) {
      // tool not in frame
      ToolTimeStampedUpdate(toolIt->ToolSourceId, toolIt->OpticalMarkerToCamera, TOOL_OUT_OF_VIEW, this->FrameNumber, unfilteredTimestamp);
      ToolTimeStampedUpdate("Depth" + toolIt->ToolSourceId, toolIt->DepthMarkerToCamera, TOOL_OUT_OF_VIEW, this->FrameNumber, unfilteredTimestamp);
    }
  }

  // for testing
  PixelCodec::ConvertToBmp24(PixelCodec::ComponentOrder_RGB, PixelCodec::PixelEncoding::PixelEncoding_BGR24, dim[0], dim[1], image.data, (unsigned char*)frame->GetScalarPointer());
  vtkPlusDataSource* videoSource;
  if (GetVideoSource("Optical", videoSource) != PLUS_SUCCESS)
  {
    LOG_ERROR("OMT video source failed");
  }

  videoSource->SetPixelType(VTK_UNSIGNED_CHAR);
  videoSource->SetImageType(US_IMG_RGB_COLOR);
  videoSource->SetNumberOfScalarComponents(3);
  videoSource->SetInputFrameSize(dim);

  videoSource->AddItem(frame, FrameNumber, unfilteredTimestamp);

  //TODO: add logging for frame rate
  this->Modified();
  this->FrameNumber++;

  return PLUS_SUCCESS;
}