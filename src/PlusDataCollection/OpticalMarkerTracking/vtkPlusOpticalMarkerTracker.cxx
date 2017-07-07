/*=Plus=header=begin======================================================
Program: Plus
Copyright (c) Laboratory for Percutaneous Surgery. All rights reserved.
See License.txt for details.
=========================================================Plus=header=end*/

#define MM_PER_M 1000

#include "PlusConfigure.h"
#include "vtkPlusOpticalMarkerTracker.h"
#include "PlusVideoFrame.h"
#include "vtkImageData.h"
#include "vtkImageImport.h"
#include "vtkMath.h"
#include "vtkObjectFactory.h"
#include "vtkExtractVOI.h"
#include <fstream>
#include <iostream>
#include <set>
#include "vtkPlusDataSource.h"
#include "vtkMatrix4x4.h"
#include "PixelCodec.h"
#include "RANSAC.h"
#include "PlaneParametersEstimator.h"

// aruco
#include "dictionary.h"

// OpenCV
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
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"

static const int CHANNEL_INDEX_VIDEO = 0;
static const int CHANNEL_INDEX_POLYDATA = 1;

vtkStandardNewMacro(vtkPlusOpticalMarkerTracker);
//----------------------------------------------------------------------------
vtkPlusOpticalMarkerTracker::TrackedTool::TrackedTool(int MarkerId, float MarkerSizeMm, std::string ToolSourceId)
{
  ToolMarkerType = SINGLE_MARKER;
  this->MarkerId = MarkerId;
  this->MarkerSizeMm = MarkerSizeMm;
  this->ToolSourceId = ToolSourceId;
}

vtkPlusOpticalMarkerTracker::TrackedTool::TrackedTool(std::string MarkerMapFile, std::string ToolSourceId)
{
  ToolMarkerType = MARKER_MAP;
  this->MarkerMapFile = MarkerMapFile;
  this->ToolSourceId = ToolSourceId;
}

//----------------------------------------------------------------------------

class vtkPlusOpticalMarkerTracker::vtkInternal
{
public:
  vtkPlusOpticalMarkerTracker *External;


  vtkInternal(vtkPlusOpticalMarkerTracker* external)
    : External(external)
  {
  }

  virtual ~vtkInternal()
  {
  }
};

//----------------------------------------------------------------------------
vtkPlusOpticalMarkerTracker::vtkPlusOpticalMarkerTracker()
: vtkPlusDevice()
{
  this->FrameNumber = 0;
  this->StartThreadForInternalUpdates = true;
}

//----------------------------------------------------------------------------
vtkPlusOpticalMarkerTracker::~vtkPlusOpticalMarkerTracker() 
{

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

  XML_READ_CSTRING_ATTRIBUTE_REQUIRED(CameraCalibrationFile, deviceConfig);
  XML_READ_ENUM2_ATTRIBUTE_OPTIONAL(TrackingMethod, deviceConfig, "OPTICAL", OPTICAL, "OPTICAL_AND_DEPTH", OPTICAL_AND_DEPTH);
  XML_READ_CSTRING_ATTRIBUTE_REQUIRED(MarkerDictionary, deviceConfig);

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
      Tools.push_back(newTool);
    }
    else if (toolDataElement->GetAttribute("MarkerMapFile") != NULL)
    {
      // this tool is tracked by a marker map
      // TODO: Implement marker map tracking.
    }
    else {
      LOG_ERROR("Incorrectly formatted tool data source.");
    }
  }

  return PLUS_SUCCESS;
}

//----------------------------------------------------------------------------
PlusStatus vtkPlusOpticalMarkerTracker::WriteConfiguration(vtkXMLDataElement* rootConfigElement)
{
  XML_FIND_DEVICE_ELEMENT_REQUIRED_FOR_WRITING(deviceConfig, rootConfigElement);

  XML_WRITE_STRING_ATTRIBUTE(CameraCalibrationFile, deviceConfig);
  // no write enum method
  XML_WRITE_STRING_ATTRIBUTE(MarkerDictionary, deviceConfig);

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
  std::string calibFilePath = vtkPlusConfig::GetInstance()->GetDeviceSetConfigurationPath(this->CameraCalibrationFile);
  LOG_INFO("Use aruco camera calibration file located at: " << calibFilePath);
  if (!vtksys::SystemTools::FileExists(calibFilePath.c_str(), true))
  {
    LOG_ERROR("Unable to find aruco camera calibration file at: " << calibFilePath);
    return PLUS_FAIL;
  }

  // TODO: Need error handling for this?
  CP.readFromXMLFile(calibFilePath);
  MDetector.setDictionary(MarkerDictionary);
  // threshold tuning numbers from aruco_test
  aruco::MarkerDetector::Params params;
  params._thresParam1 = 7;
  params._thresParam2 = 7;
  params._thresParam1_range = 2;
  this->Internal->MarkerDetector->setParams(params);

  bool lowestRateKnown = false;
  double lowestRate = 30; // just a usual value (FPS)
  for (ChannelContainerConstIterator it = this->InputChannels.begin(); it != this->InputChannels.end(); ++it)
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
void vtkPlusOpticalMarkerTracker::BuildTransformMatrix(vtkSmartPointer<vtkMatrix4x4> transformMatrix, cv::Mat Rvec, cv::Mat Tvec)
{
  transformMatrix->Identity();
  cv::Mat Rmat(3, 3, CV_32FC1);
  cv::Rodrigues(Rvec, Rmat);

  for (int x = 0; x <= 2; x++)
  {
    transformMatrix->SetElement(x, 3, MM_PER_M * Tvec.at<float>(x, 0));
    for (int y = 0; y <= 2; y++)
      transformMatrix->SetElement(x, y, Rmat.at<float>(x, y));
  }
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
void vtkPlusOpticalMarkerTracker::CopyToItkData(int top, int bottom, int *leftBoundary, int *rightBoundary)
{
  for (int yPx = top; yPx <= bottom; yPx++)
  {
    for (int xPx = leftBoundary[yPx]; xPx <= rightBoundary[yPx]; xPx++)
    {
      // copy to ITK data
    }
  }
}

//----------------------------------------------------------------------------
float vtkPlusOpticalMarkerTracker::DetermineSlope(cv::Point2d corner1, cv::Point2d corner2)
{
  if (corner1.x == corner2.x || corner1.y == corner2.y)
  {
    return 0.0;
  }
  else
  {
    return (float)(corner1.x - corner2.x) / (corner1.y - corner2.y);
  }
}

//----------------------------------------------------------------------------
void vtkPlusOpticalMarkerTracker::GenerateBoundary(int* boundary, std::vector<cv::Point2d> corners, int top, int bottom, bool isRight)
{
  int currentUpper = 0, currentLower = 1;
  // for line of type x=ym+b (x is dependent variable, y is dependent)
  int mPx = DetermineSlope(corners[0], corners[1]), bPx = corners[0].x;
  
  for (int yPx = top; yPx < bottom; yPx++)
  {
    if (yPx == corners[currentLower].y)
    {
      // we have reached the next corner on the boundary path
      boundary[yPx] = corners[currentLower].x;
      currentUpper++;
      currentLower++;
      mPx = DetermineSlope(corners[currentUpper], corners[currentLower]);
      bPx = corners[currentUpper].x;
    }
    else if (corners[currentUpper].y == corners[currentLower].y)
    {
      // horizontal line
      if (isRight)
      {
        // right boundary, select rightmost corner
        boundary[yPx] = (corners[currentUpper].x > corners[currentLower].x) ? corners[currentUpper].x : corners[currentLower].x;
      }
      else 
      {
        // left boundary, select leftmost corner
        boundary[yPx] = (corners[currentUpper].x < corners[currentLower].x) ? corners[currentUpper].x : corners[currentLower].x;
      }
    }
    else
    {
      // vertical or skew line
      boundary[yPx] = mPx * yPx + bPx;
    }
  }
}

//----------------------------------------------------------------------------
void vtkPlusOpticalMarkerTracker::GenerateSkewLeftItkData(
  vtkSmartPointer<vtkPolyData> vtkDepthData,
  std::vector<itk::Point<double, 3>> &itkData,
  std::vector<cv::Point2d> corners
  /*for testing
  unsigned int dim[],
  cv::Mat image*/)
{
  const char TOP = 0, BOTTOM = 1, LOWER_LEFT = 2, UPPER_LEFT = 3;
  int top = corners[TOP].y, bottom = corners[BOTTOM].y;
  int height = bottom - top;

  // generate left boundary
  int* leftBoundary = new int[height];
  std::vector<cv::Point2d> leftPath;
  leftPath.push_back(corners[TOP]);
  leftPath.push_back(corners[UPPER_LEFT]);
  leftPath.push_back(corners[LOWER_LEFT]);
  leftPath.push_back(corners[BOTTOM]);
  GenerateBoundary(leftBoundary, leftPath, top, bottom, false);

  // generate right boundary
  int* rightBoundary = new int[height];
  std::vector<cv::Point2d> rightPath;
  rightPath.push_back(corners[TOP]);
  rightPath.push_back(corners[BOTTOM]);
  GenerateBoundary(rightBoundary, rightPath, top, bottom, true);

  // copy vtk->itk
}

//----------------------------------------------------------------------------
void vtkPlusOpticalMarkerTracker::GenerateSkewRightItkData(
  vtkSmartPointer<vtkPolyData> vtkDepthData,
  std::vector<itk::Point<double, 3>> &itkData,
  std::vector<cv::Point2d> corners
  /*for testing
  unsigned int dim[],
  cv::Mat image*/)
{
  const char TOP = 0, UPPER_RIGHT = 1, LOWER_RIGHT = 2, BOTTOM = 3;
  int top = corners[TOP].y, bottom = corners[BOTTOM].y;
  int height = bottom - top;

  // generate left boundary
  int* leftBoundary = new int[height];
  std::vector<cv::Point2d> leftPath;
  leftPath.push_back(corners[TOP]);
  leftPath.push_back(corners[BOTTOM]);
  GenerateBoundary(leftBoundary, leftPath, top, bottom, false);

  // generate right boundary
  int* rightBoundary = new int[height];
  std::vector<cv::Point2d> rightPath;
  rightPath.push_back(corners[TOP]);
  rightPath.push_back(corners[UPPER_RIGHT]);
  rightPath.push_back(corners[LOWER_RIGHT]);
  rightPath.push_back(corners[BOTTOM]);
  GenerateBoundary(rightBoundary, rightPath, top, bottom, true);

  // copy vtk->itk
}

//----------------------------------------------------------------------------
void vtkPlusOpticalMarkerTracker::GenerateRotatedItkData(
  vtkSmartPointer<vtkPolyData> vtkDepthData,
  std::vector<itk::Point<double, 3>> &itkData,
  std::vector<cv::Point2d> corners
  /*for testing
  unsigned int dim[],
  cv::Mat image*/)
{
  const char TOP = 0, RIGHT = 1, BOTTOM = 2, LEFT = 3;
  int top = corners[TOP].y, bottom = corners[BOTTOM].y;
  int height = bottom - top;

  // generate left boundary
  int* leftBoundary = new int[height];
  //std::vector<cv::Point2d> leftPath;
  //leftPath.push_back(corners[TOP]);
  //leftPath.push_back(corners[LEFT]);
  //leftPath.push_back(corners[BOTTOM]);
  //GenerateBoundary(leftBoundary, leftPath, top, bottom, false);

  //// generate right boundary
  //int* rightBoundary = new int[height];
  //std::vector<cv::Point2d> rightPath;
  //rightPath.push_back(corners[TOP]);
  //rightPath.push_back(corners[RIGHT]);
  //rightPath.push_back(corners[BOTTOM]);
  //GenerateBoundary(rightBoundary, rightPath, top, bottom, true);
  //// copy vtk->itk
}

//----------------------------------------------------------------------------
void vtkPlusOpticalMarkerTracker::GenerateItkData(
  vtkSmartPointer<vtkPolyData> vtkDepthData,
  std::vector<itk::Point<double, 3>> &itkData,
  std::vector<cv::Point2d> corners
  /*for testing
  unsigned int dim[],
  cv::Mat image*/)
{
  //LOG_WARNING("corners before:");
  //for (int i = 0; i < 4; i++)
  //{
  //  LOG_WARNING("#" << i << " x: " << corners[i].x << " y: " << corners[i].y);
  //}

  MARKER_ORIENTATION orientation = DetermineMarkerOrientation(corners);

  //LOG_WARNING("corners after:");
  //for (int i = 0; i < 4; i++)
  //{
  //  LOG_WARNING("#" << i << " x: " << corners[i].x << " y: " << corners[i].y);
  //}

  switch (orientation)
  {
  case SKEW_LEFT:
    GenerateSkewLeftItkData(vtkDepthData, itkData, corners);
    return;
  case ROTATED:
    GenerateRotatedItkData(vtkDepthData, itkData, corners);
    return;
  case SKEW_RIGHT:
    GenerateSkewRightItkData(vtkDepthData, itkData, corners);
    return;
  }
}

//----------------------------------------------------------------------------
PlusStatus vtkPlusOpticalMarkerTracker::InternalUpdate()
{
  LOG_INFO("----OMT BEGINS HERE----");
  //TODO: refactor this to check for optical & depth data, or simply optical
  if (this->InputChannels.size() != 2)
  {
    LOG_ERROR("ImageProcessor device requires exactly 1 input stream (that contains video data). Check configuration.");
    return PLUS_FAIL;
  }

  // Get image to tracker transform from the tracker (only request 1 frame, the latest)

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



  if (!this->InputChannels[CHANNEL_INDEX_VIDEO]->GetVideoDataAvailable())
  {
    LOG_TRACE("Processed data is not generated, as no video data is available yet. Device ID: " << this->GetDeviceId());
    return PLUS_SUCCESS;
  }

  //double oldestTrackingTimestamp(0);
  //if (this->InputChannels[CHANNEL_INDEX_VIDEO]->GetOldestTimestamp(oldestTrackingTimestamp) == PLUS_SUCCESS)
  //{
  //  if (this->LastProcessedInputDataTimestamp > oldestTrackingTimestamp)
  //  {
  //    LOG_INFO("Processed image generation started. No tracking data was available between " << this->LastProcessedInputDataTimestamp << "-" << oldestTrackingTimestamp <<
  //      "sec, therefore no processed images were generated during this time period.");
  //    this->LastProcessedInputDataTimestamp = oldestTrackingTimestamp;
  //  }
  //}

  PlusTrackedFrame trackedFrame;
  if (this->InputChannels[CHANNEL_INDEX_VIDEO]->GetTrackedFrame(trackedFrame) != PLUS_SUCCESS)
  {
    LOG_ERROR("Error while getting latest tracked frame. Last recorded timestamp: " << std::fixed << this->LastProcessedInputDataTimestamp << ". Device ID: " << this->GetDeviceId());
    this->LastProcessedInputDataTimestamp = vtkPlusAccurateTimer::GetSystemTime(); // forget about the past, try to add frames that are acquired from now on
    return PLUS_FAIL;
  }
 
  PlusTrackedFrame polyDataTrackedFrame;
  if (this->InputChannels[CHANNEL_INDEX_POLYDATA]->GetTrackedFrame(oldestTrackingTimestamp, polyDataTrackedFrame) != PLUS_SUCCESS)
  {
    LOG_ERROR("Error while getting latest tracked frame. Last recorded oldestTrackingTimestamp: " << std::fixed << this->LastProcessedInputDataTimestamp << ". Device ID: " << this->GetDeviceId());
    this->LastProcessedInputDataTimestamp = vtkPlusAccurateTimer::GetSystemTime(); // forget about the past, try to add frames that are acquired from now on
    return PLUS_FAIL;
  }


  vtkPolyData* poly = polyDataTrackedFrame.GetPolyData();
  if (poly != NULL)
  {
    //LOG_INFO("OMT verts: " << poly->GetNumberOfVerts());
  }
  else
  {
    LOG_INFO("NULL polydata");
  }



  //LOG_INFO("VIDEO TIME: " << trackedFrame.GetTimestamp());
  //LOG_INFO("POLY TIME: " << polyDataTrackedFrame.GetTimestamp());
  //poly->GetNumberOfVerts();

  ////////vtksmartpointer<vtkpolydata> polydata = vtksmartpointer<vtkpolydata>::new();
  //////////todo: read polydata from file (replace this by reading from buffers)
  ////////ostringstream vtpfilename;
  ////////vtpfilename << "poly" << framenumber << ".vtp";
  ////////vtksmartpointer<vtkxmlpolydatareader> reader = vtksmartpointer<vtkxmlpolydatareader>::new();
  ////////reader->setfilename(vtpfilename.str().c_str());
  ////////reader->update();
  ////////polydata = reader->getoutput();

  ////TODO: for testing visaulize polydata

  if (false) {
    vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputData(poly);
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


/*










  // get polydata from buffers
  StreamBufferItem item;
  //this->InputChannels[CHANNEL_INDEX_POLYDATA]->GetStreamBufferitem

  // get dimensions & data
  FrameSizeType dim = trackedVideoFrame.GetFrameSize();
  PlusVideoFrame* frame = trackedVideoFrame.GetImageData();

  cv::Mat image(dim[1], dim[0], CV_8UC3);

  
  // Plus image uses RGB and OpenCV uses BGR, swapping is only necessary for colored markers
  //PixelCodec::RgbBgrSwap(dim[0], dim[1], (unsigned char*)frame->GetScalarPointer(), image.data);
  image.data = (unsigned char*)frame->GetScalarPointer();
  //TODO: for testing only, pulling data from files
  ////////ostringstream filename;
  ////////filename << "img" << framenumber << ".png";
  ////////image = cv::imread(filename.str());
  ////////// end testing


  // detect markers in frame
  MDetector.detect(image, markers);

  // iterate through tools updating tracking
  for (std::vector<TrackedTool>::iterator toolIt = begin(this->Internal->Tools); toolIt != end(this->Internal->Tools); ++toolIt)
  {
    bool toolInFrame = false;
    const double unfilteredTimestamp = vtkPlusAccurateTimer::GetSystemTime();
    for (std::vector<aruco::Marker>::iterator markerIt = begin(this->Internal->Markers); markerIt != end(this->Internal->Markers); ++markerIt)
    {
      if (toolIt->MarkerId == markerIt->id) {
        //marker is in frame
        toolInFrame = true;

        // get marker corners
        std::vector<cv::Point2d> corners;
        corners = markerIt->getCornersPx();

        vtkSmartPointer<vtkPolyData> polydata;
        // copy data from inside the marker into data structure for RANSAC plane algorithm
        std::vector<itk::Point<double, 3>> itkPlane;
        std::vector<cv::Point2d> corners_duplicate = corners;
        GenerateItkData(polydata, itkPlane, corners_duplicate);

        //TODO: for testing generate the vtkPolyData plane and show to user
        //end testing


        // find plane normal and distance using RANSAC
        std::vector<double> planeParameters;
        double desiredProbablilityForNoOutluiers = 0.95;
        double maxDistanceFromPlane = 2;
        typedef itk::PlaneParametersEstimator<3> PlaneEstimatorType;
        typedef itk::RANSAC<itk::Point<double, 3>, double> RANSACType;

        PlaneEstimatorType::Pointer planeEstimator = PlaneEstimatorType::New();
        //planeEstimator->SetDelta(maxDistanceFromPlane);
        //planeEstimator->LeastSquaresEstimate(itkPlane, planeParameters);
        //if (planeParameters.empty())
        //  std::cout << "Least squares estimate failed, degenerate configuration?\n";
        //else
        //{
        //  std::cout << "Least squares hyper(plane) parameters: [n,a]\n\t [ ";
        //  int i;
        //  for (i = 0; i < (2 * 3 - 1); i++)
        //    std::cout << planeParameters[i] << ", ";
        //  std::cout << planeParameters[i] << "]\n\n";
        //  //cos(theta), theta is the angle between the two unit normals
        //}


        //TODO: for testing draw plane point & normal on RGB image
        //end testing

        //TODO: for testing draw corners and normals on image
        cv::Point_<int> corner;
        for (int i = 0; i < 4; i++)
        {
          corner.x = corners[i].x;
          corner.y = corners[i].y;
          cv::circle(image, corner, 2, cv::Scalar(0,0,255), -1);
        }
       // cv::imshow("image", image);
       // cv::waitKey();
        //end testing




        if (toolIt->MarkerPoseTracker.estimatePose(*markerIt, CP, toolIt->MarkerSizeMm / MM_PER_M, 4))
        {
          // pose successfully estimated, update transform
          cv::Mat Rvec = toolIt->MarkerPoseTracker.getRvec();
          cv::Mat Tvec = toolIt->MarkerPoseTracker.getTvec();
          BuildTransformMatrix(toolIt->transformMatrix, Rvec, Tvec);
          ToolTimeStampedUpdate(toolIt->ToolSourceId, toolIt->transformMatrix, TOOL_OK, this->FrameNumber, unfilteredTimestamp);
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
      ToolTimeStampedUpdate(toolIt->ToolSourceId, toolIt->transformMatrix, TOOL_OUT_OF_VIEW, this->FrameNumber, unfilteredTimestamp);
    }
  }







  */




  //TODO: add logging for frame rate
  
  this->FrameNumber++;

  return PLUS_SUCCESS;
}