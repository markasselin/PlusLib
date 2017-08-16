/*=Plus=header=begin======================================================
  Progra  : Plus
  Copyright (c) Laboratory for Percutaneous Surgery. All rights reserved.
  See License.txt for details.
=========================================================Plus=header=end*/

#ifndef __vtkPlusOpticalMarkerTracker_h
#define __vtkPlusOpticalMarkerTracker_h

// Local includes
#include "vtkPlusDataCollectionExport.h"
#include "vtkPlusDevice.h"

/*!
  \class vtkPlusOpticalMarkerTracker
  \brief Virtual device that tracks fiducial markers on the input channel in real time.
  \ingroup PlusLibDataCollection
*/
class vtkPlusDataCollectionExport vtkPlusOpticalMarkerTracker : public vtkPlusDevice
{
public:

  /*! Defines whether or not depth stream is used. */
  enum TRACKING_METHOD
  {
    TRACKING_OPTICAL,
    TRACKING_OPTICAL_AND_DEPTH
  };

  static vtkPlusOpticalMarkerTracker* New();
  vtkTypeMacro(vtkPlusOpticalMarkerTracker,vtkPlusDevice);
  virtual void PrintSelf(ostream& os, vtkIndent indent) VTK_OVERRIDE;

  /*! Read main config settings from XML. */
  virtual PlusStatus ReadConfiguration(vtkXMLDataElement*);

  /*! Write current main config settings to XML. */
  virtual PlusStatus WriteConfiguration(vtkXMLDataElement*);

  /*! Probe to see if the tracking system is present. */
  PlusStatus Probe();

  /*! Connect to the tracker hardware */
  PlusStatus InternalConnect();

  /*! Disconnect from the tracker hardware */
  PlusStatus InternalDisconnect();

  /*!
  Get an update from the tracking system and push the new transforms to the tools.
  */
  virtual PlusStatus InternalUpdate();

  /* This device is a virtual tracker. */
  virtual bool IsTracker() const { return true; }
  virtual bool IsVirtual() const { return true; }

protected:
  vtkPlusOpticalMarkerTracker();
  ~vtkPlusOpticalMarkerTracker();

  /*! Start the tracking system. */
  PlusStatus InternalStartRecording();

  /*! Stop the tracking system and bring it back to its initial state. */
  PlusStatus InternalStopRecording();

  class vtkInternal;
  vtkInternal* Internal;

  unsigned int FrameNumber;
  double LastProcessedInputDataTimestamp;

private:
  vtkPlusOpticalMarkerTracker(const vtkPlusOpticalMarkerTracker&);
  void operator=(const vtkPlusOpticalMarkerTracker&);




  private:
  enum MARKER_ORIENTATION
  {
    //ALIGNED, //marker sides are parallel to image frame
    SKEW_LEFT, //skewed back towards left of image (low x values)
    SKEW_RIGHT, //skewed back towards right of image (high x values)
    ROTATED //marker has unique top, right, bottom and left corners
  };

  private:
  // TODO: move this to vtkInternal
  TRACKING_METHOD TrackingMethod;


  /*
   * Determines if the marker is ALIGNED, SKEW_LEFT, SKEW_RIGHT or ROTATED
   * with respect to the image frame.  If marker is SKEW_LEFT, SKEW_RIGHT
   * or ROTATED sets the corner on the top of the image to be at index 0
   * and places the remaining corners in clockwise order in index 1 to 3.
   */
  MARKER_ORIENTATION DetermineMarkerOrientation(std::vector<cv::Point2d>& corners);

  /*
  *
  */
  void vtkPlusOpticalMarkerTracker::CopyToItkData(
    vtkSmartPointer<vtkPolyData> vtkDepthData,
    std::vector<itk::Point<double, 3>> &itkData,
    int top,
    int bottom,
    int *leftBoundary,
    int *rightBoundary,
    cv::Mat image
  );

  /*
   * Computes the slope of the line x=my+b between corners 1 & 2.
   * If corenrs have the same x or y values then returns special value 0.0.
   */
  float DetermineSlope(cv::Point2d corner1, cv::Point2d corner2);

  /*
   *
   */
  void GenerateBoundary(int* boundary, std::vector<cv::Point2d> corners, int top, bool isRight);

  // /*
  // *
  // */
  //void GenerateAlignedItkData(
  //  vtkSmartPointer<vtkPolyData> vtkDepthData,
  //  std::vector<itk::Point<double, 3>> &itkData,
  //  std::vector<cv::Point2d> corners,
  //  /*for testing*/
  //  int dim[],
  //  cv::Mat image
  //);

  /*
   *
   */
  void GenerateSkewLeftItkData(
    vtkSmartPointer<vtkPolyData> vtkDepthData,
    std::vector<itk::Point<double, 3>> &itkData,
    std::vector<cv::Point2d> corners,
    /*for testing*/
    unsigned int dim[],
    cv::Mat image
  );

  /*
   *
   */
  void GenerateSkewRightItkData(
    vtkSmartPointer<vtkPolyData> vtkDepthData,
    std::vector<itk::Point<double, 3>> &itkData,
    std::vector<cv::Point2d> corners,
    /*for testing*/
    unsigned int dim[],
    cv::Mat image
  );

  /*
   *
   */
  void GenerateRotatedItkData(
    vtkSmartPointer<vtkPolyData> vtkDepthData,
    std::vector<itk::Point<double, 3>> &itkData,
    std::vector<cv::Point2d> corners,
    /*for testing*/
    unsigned int dim[],
    cv::Mat image
  );

  /*
  *
  */
  void GenerateItkData(
    vtkSmartPointer<vtkPolyData> vtkDepthData,
    std::vector<itk::Point<double, 3>> &itkData,
    std::vector<cv::Point2d> corners,
    /*for testing*/
    unsigned int dim[],
    cv::Mat image
  );
  
  void ComputePlaneTransform(
    vtkSmartPointer<vtkMatrix4x4> depthTransform,
    double x_axis[],
    double z_axis[],
    double center[]);

  double* GetXAxis(
    vtkSmartPointer<vtkPolyData> vtkDepthData,
    std::vector<cv::Point2d> corners,
    cv::Mat image);

  /* Computes center of marker by finding the intersection of lines
     drawn between opposing corners */
  cv::Point2d GetMarkerCenter(std::vector<cv::Point2d> corners);

  double* MarkerCenterImageToSpatial(vtkSmartPointer<vtkPolyData> vtkDepthData, cv::Point2d imageCenter, cv::Mat image);

  float VectorAngleDeg(vnl_vector<double> xAxis, vnl_vector<double> zAxis);

};
#endif
