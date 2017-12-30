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
  enum INPUT_TYPE
  {
    INPUT_RGB_ONLY,
    INPUT_RGB_AND_DEPTH
  };

  /*! Defines orientation of marker in frame */
  enum MARKER_ORIENTATION
  {
    SKEW_LEFT,  //skewed back towards left of image (low x values)
    SKEW_RIGHT, //skewed back towards right of image (high x values)
    ROTATED     //marker has unique top, right, bottom and left corners
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
};
#endif
