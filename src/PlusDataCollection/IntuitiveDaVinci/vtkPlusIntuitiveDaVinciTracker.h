#ifndef __vtkPlusIntuitiveDaVinciTracker_h
#define __vtkPlusIntuitiveDaVinciTracker_h

#include "vtkPlusDataCollectionExport.h"

#include "time.h"
#include "vtkDoubleArray.h"
#include "vtkImageFlip.h"
#include "vtkImageImport.h"
#include "vtkIntArray.h"
#include "vtkLongArray.h"
#include "vtkMath.h"
#include "vtkMatrix4x4.h"
#include "vtkObjectFactory.h"
#include "vtkPlusDevice.h"
#include "vtkTimerLog.h"
#include "vtkPlusDataSource.h"
#include "vtkTransform.h"
#include "vtkUnsignedCharArray.h"
#include <ctype.h>
#include <float.h>
#include <iomanip>
#include <limits.h>
#include <math.h>

#include "IntuitiveDaVinci.h"
#include "isi_api.h"

class IntuitiveDaVinci;
class vtkPlusIntuitiveDaVinciTracker;
/* 
This class talks with the da Vinci Surgical System.

*/

namespace vtkPlusIntuitiveDaVinciTrackerUtilities{
 void ISICALLBACK streamCB(void *userdata);
 void ISICALLBACK eventCB(ISI_MANIP_INDEX mid, ISI_EVENT_ID event_id, ISI_INT args[ISI_NUM_EVENT_ARGS],  void *userdata);
};

class vtkPlusDataCollectionExport vtkPlusIntuitiveDaVinciTracker : public vtkPlusDevice
{
public:

  static vtkPlusIntuitiveDaVinciTracker *New();

  vtkTypeMacro(vtkPlusIntuitiveDaVinciTracker,vtkPlusDevice);

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

  /*! Get the status of the MicronTracker (Tracking or not) */
  vtkGetMacro(IsDaVinciTrackingInitialized, int);

  /*! Read da Vinci configuration and update the tracker settings accordingly */
  virtual PlusStatus ReadConfiguration( vtkXMLDataElement* config );

  /*! Write current da Vinci configuration settings to XML */
  virtual PlusStatus WriteConfiguration(vtkXMLDataElement* rootConfigElement);

  /*! Connect to the tracker hardware */
  PlusStatus InternalConnect();

  /*! Disconnect from the tracker hardware */
  PlusStatus InternalDisconnect();

  IntuitiveDaVinci* GetDaVinci() { return this->mDaVinci; };

  vtkGetMacro(IpAddr, std::string);
  vtkGetMacro(Port, unsigned int);
  vtkGetMacro(Password, std::string);

  vtkSetMacro(IpAddr, std::string);
  vtkSetMacro(Port, unsigned int);
  vtkSetMacro(Password, std::string);

protected:
  vtkPlusIntuitiveDaVinciTracker();
  ~vtkPlusIntuitiveDaVinciTracker();

  /*!
  Start the tracking system. 
  */
  PlusStatus InternalStartRecording();

  /*! Stop the tracking system and bring it back to its initial state. */
  PlusStatus InternalStopRecording();

  ///*! Refresh the loaded markers by loading them from the Markers directory */
  //PlusStatus RefreshMarkerTemplates();

  /*! Pointer to the MicronTrackerInterface class instance */
  IntuitiveDaVinci* mDaVinci;

  /*! Non-zero if the tracker has been initialized */
  bool IsDaVinciTrackingInitialized;

  /*! Index of the last frame number. This is used for providing a frame number when the tracker doesn't return any transform */
  double LastFrameNumber;

  unsigned int FrameNumber;

#ifdef USE_DAVINCI_TIMESTAMPS
  double TrackerTimeToSystemTimeSec; // time_System = time_Tracker + TrackerTimeToSystemTimeSec
  bool TrackerTimeToSystemTimeComputed; // the time offset is always computed when the first frame is received after start tracking
#endif

  std::string IpAddr;
  unsigned int Port;
  std::string Password;

private:
  vtkPlusIntuitiveDaVinciTracker(const vtkPlusIntuitiveDaVinciTracker&);
  void operator=(const vtkPlusIntuitiveDaVinciTracker&);  

  void StreamCallback(void);

  friend void vtkPlusIntuitiveDaVinciTrackerUtilities::streamCB(void* userData);
  friend void vtkPlusIntuitiveDaVinciTrackerUtilities::eventCB(ISI_MANIP_INDEX mid, ISI_EVENT_ID event_id, ISI_INT args[ISI_NUM_EVENT_ARGS],  void *userdata);

  ISI_MANIP_INDEX getManipIndexFromName(std::string toolName);
  void setVtkMatrixFromISITransform(vtkMatrix4x4* vtkMatrix, ISI_TRANSFORM* isiMatrix);

  static const int CONNECT_RETRY_DELAY_SEC = 1.0;
  static const int MAX_ATTEMPTS = 5;
};



#endif
