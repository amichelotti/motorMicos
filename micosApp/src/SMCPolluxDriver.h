/*
FILENAME...   SMCPolluxDriver.h
USAGE...      Motor driver support for the Micos SMC Pollux controller.

*/

#ifndef __SMCPOLLUX__
#define __SMCPOLLUX__

#include "SMCPolluxAxis.h"

#define MAX_SMCPOLLUX_AXES 2

// Controller-specific parameters
#define NUM_SMCPOLLUX_PARAMS 1

/** drvInfo strings for extra parameters that the SMC Hydra controller supports */
#define SMCPolluxRegulatorModeString "SMCPOLLUX_REGULATOR_MODE"

class epicsShareClass SMCPolluxController : public asynMotorController {
public:
  SMCPolluxController(const char *portName, const char *SMCPolluxPortName, int numAxes, double movingPollPeriod, double idlePollPeriod);

  void report(FILE *fp, int level);
  SMCPolluxAxis* getAxis(asynUser *pasynUser);
  SMCPolluxAxis* getAxis(int axisNo);
  asynStatus changeResolution(int axisNo, double newResolution);

protected:
  int SMCPolluxRegulatorMode_;    /** Regulator mode parameter index */

friend class SMCPolluxAxis;
};

#endif

