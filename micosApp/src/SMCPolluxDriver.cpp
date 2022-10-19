/*
FILENAME... SMCPolluxDriver.cpp
USAGE...    Motor driver support for the Micos SMC Pollux controller.

Note: This driver was tested with the Micos SMC Pollux CM and 
      motor forms 0 (stepper) and 1 (linear).

*/


#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include <iocsh.h>
#include <epicsThread.h>
#include <asynOctetSyncIO.h>
#include <epicsExport.h>

#include "SMCPolluxDriver.h"

#define NINT(f) (int)((f)>0 ? (f)+0.5 : (f)-0.5)

/** Creates a new SMCPolluxController object.
  * \param[in] portName          The name of the asyn port that will be created for this driver
  * \param[in] SMCPolluxPortName  The name of the drvAsynSerialPort that was created previously to connect to the SMC Pollux controller 
  * \param[in] numAxes           The number of axes that this controller supports 
  * \param[in] movingPollPeriod  The time between polls when any axis is moving 
  * \param[in] idlePollPeriod    The time between polls when no axis is moving 
  */
SMCPolluxController::SMCPolluxController(const char *portName, const char *SMCPolluxPortName, int numAxes, 
                                 double movingPollPeriod, double idlePollPeriod)
  :  asynMotorController(portName, numAxes, NUM_SMCPOLLUX_PARAMS, 
                         0, // No additional interfaces beyond those in base class
                         0, // No additional callback interfaces beyond those in base class
                         ASYN_CANBLOCK | ASYN_MULTIDEVICE, 
                         1, // autoconnect
                         0, 0)  // Default priority and stack size
{
  int axis;
  asynStatus status;
  SMCPolluxAxis *pAxis;
  static const char *functionName = "SMCPolluxController::SMCPolluxController";

  // Create controller-specific parameters
  createParam(SMCPolluxRegulatorModeString, asynParamInt32, &SMCPolluxRegulatorMode_);

  /* Connect to SMC Pollux controller */
  status = pasynOctetSyncIO->connect(SMCPolluxPortName, 0, &pasynUserController_, NULL);
  if (status) {
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, 
      "%s: cannot connect to SMC Pollux controller\n",
      functionName);
  }
  for (axis=0; axis<numAxes; axis++) {
    pAxis = new SMCPolluxAxis(this, axis);
  }

  startPoller(movingPollPeriod, idlePollPeriod, 2);
}

/** Change the resolution of an axis
  * \param[in] axisNo The index of the axis
  * \param[in] newResolution The new resolution
  */
asynStatus SMCPolluxController::changeResolution(int axisNo, double newResolution)
{
  SMCPolluxAxis* pAxis;
  asynStatus status;
  
  pAxis = this->getAxis(axisNo);
  
  status = pAxis->changeResolution(newResolution);
  
  return status;
}

/** Reports on status of the driver
  * \param[in] fp The file pointer on which report information will be written
  * \param[in] level The level of report detail desired
  *
  * If details > 0 then information is printed about each axis.
  * After printing controller-specific information it calls asynMotorController::report()
  */
void SMCPolluxController::report(FILE *fp, int level)
{
  fprintf(fp, "SMC Pollux motor driver %s, numAxes=%d, moving poll period=%f, idle poll period=%f\n", 
    this->portName, numAxes_, movingPollPeriod_, idlePollPeriod_);

  // Call the base class method
  asynMotorController::report(fp, level);
}

/** Returns a pointer to an SMCPolluxAxis object.
  * Returns NULL if the axis number encoded in pasynUser is invalid.
  * \param[in] pasynUser asynUser structure that encodes the axis index number. */
SMCPolluxAxis* SMCPolluxController::getAxis(asynUser *pasynUser)
{
  return static_cast<SMCPolluxAxis*>(asynMotorController::getAxis(pasynUser));
}

/** Returns a pointer to an SMCPolluxAxis object.
  * Returns NULL if the axis number encoded in pasynUser is invalid.
  * \param[in] axisNo Axis index number. */
SMCPolluxAxis* SMCPolluxController::getAxis(int axisNo)
{
  return static_cast<SMCPolluxAxis*>(asynMotorController::getAxis(axisNo));
}

/** Creates a new SMCPolluxController object.
  * Configuration command, called directly or from iocsh
  * \param[in] portName          The name of the asyn port that will be created for this driver
  * \param[in] SMCPolluxPortName  The name of the drvAsynIPPPort that was created previously to connect to the SMC Pollux controller 
  * \param[in] numAxes           The number of axes that this controller supports 
  * \param[in] movingPollPeriod  The time in ms between polls when any axis is moving
  * \param[in] idlePollPeriod    The time in ms between polls when no axis is moving 
  */
extern "C" int SMCPolluxCreateController(const char *portName, const char *SMCPolluxPortName, int numAxes, 
                                   int movingPollPeriod, int idlePollPeriod)
{
  SMCPolluxController *pSMCPolluxController
    = new SMCPolluxController(portName, SMCPolluxPortName, numAxes, movingPollPeriod/1000., idlePollPeriod/1000.);
  pSMCPolluxController = NULL;
  return(asynSuccess);
}

/** Specify a new resolution for an SMC Pollux axis.
  * Configuration command, called directly or from iocsh
  * \param[in] SMCPolluxPortName  The name of the drvAsynIPPPort that was created previously to connect to the SMC Pollux controller 
  * \param[in] axisNo            Index of the desired axis 
  * \param[in] newResolution     The new resolution of the specified axis
  */
extern "C" int SMCPolluxChangeResolution(const char *SMCPolluxPortName, int axisNo, double newResolution)
{
  SMCPolluxController *pC;
  static const char *functionName = "SMCPolluxChangeResolution";
  
  pC = (SMCPolluxController*) findAsynPortDriver(SMCPolluxPortName);
  if (!pC) {
    printf("SMCPolluxDriver.cpp:%s: Error port %s not found\n",
           functionName, SMCPolluxPortName);
    return asynError;
  }
  
  pC->lock();
  pC->changeResolution(axisNo, newResolution);
  pC->unlock();
  
  return(asynSuccess);
}

/** Code for iocsh registration */
static const iocshArg SMCPolluxCreateControllerArg0 = {"Port name", iocshArgString};
static const iocshArg SMCPolluxCreateControllerArg1 = {"SMC Pollux port name", iocshArgString};
static const iocshArg SMCPolluxCreateControllerArg2 = {"Number of axes", iocshArgInt};
static const iocshArg SMCPolluxCreateControllerArg3 = {"Moving poll period (ms)", iocshArgInt};
static const iocshArg SMCPolluxCreateControllerArg4 = {"Idle poll period (ms)", iocshArgInt};
static const iocshArg * const SMCPolluxCreateControllerArgs[] = {&SMCPolluxCreateControllerArg0,
                                                             &SMCPolluxCreateControllerArg1,
                                                             &SMCPolluxCreateControllerArg2,
                                                             &SMCPolluxCreateControllerArg3,
                                                             &SMCPolluxCreateControllerArg4};
static const iocshFuncDef SMCPolluxCreateControllerDef = {"SMCPolluxCreateController", 5, SMCPolluxCreateControllerArgs};
static void SMCPolluxCreateControllerCallFunc(const iocshArgBuf *args)
{
  SMCPolluxCreateController(args[0].sval, args[1].sval, args[2].ival, args[3].ival, args[4].ival);
}

static const iocshArg SMCPolluxChangeResolutionArg0 = {"SMC Pollux port name", iocshArgString};
static const iocshArg SMCPolluxChangeResolutionArg1 = {"Axis number", iocshArgInt};
static const iocshArg SMCPolluxChangeResolutionArg2 = {"Axis resolution", iocshArgDouble};
static const iocshArg * const SMCPolluxChangeResolutionArgs[] = {&SMCPolluxChangeResolutionArg0,
                                                             &SMCPolluxChangeResolutionArg1,
                                                             &SMCPolluxChangeResolutionArg2};
static const iocshFuncDef SMCPolluxChangeResolutionDef = {"SMCPolluxChangeResolution", 3, SMCPolluxChangeResolutionArgs};
static void SMCPolluxChangeResolutionCallFunc(const iocshArgBuf *args)
{
  SMCPolluxChangeResolution(args[0].sval, args[1].ival, args[2].dval);
}

static void SMCPolluxRegister(void)
{
  iocshRegister(&SMCPolluxCreateControllerDef, SMCPolluxCreateControllerCallFunc);
  iocshRegister(&SMCPolluxChangeResolutionDef, SMCPolluxChangeResolutionCallFunc);
}

extern "C" {
epicsExportRegistrar(SMCPolluxRegister);
}
