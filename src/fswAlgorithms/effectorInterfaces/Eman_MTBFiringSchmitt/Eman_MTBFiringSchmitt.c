/*
 ISC License

 Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

 Permission to use, copy, modify, and/or distribute this software for any
 purpose with or without fee is hereby granted, provided that the above
 copyright notice and this permission notice appear in all copies.

 THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

 */
/*
	MTB Firing Schmitt

 */

#include "Eman_MTBFiringSchmitt.h"
#include "architecture/utilities/macroDefinitions.h"
#include <stdio.h>
#include <string.h>
#include "architecture/msgPayloadDefC/MTBArrayOnTimeCmdMsgPayload.h"

/*!
 \verbatim embed:rst
	This method initializes the configData for this module.  It creates a single output message of type
	:ref:`MTBArrayOnTimeCmdMsgPayload`.
 \endverbatim
 @return void
 @param configData The configuration data associated with this module
 @param moduleID The ID associated with the configData
 */
void SelfInit_MTBFiringSchmitt(MTBFiringSchmittConfig *configData, int64_t moduleID)
{
	MTBArrayOnTimeCmdMsg_C_init(&configData->onTimeOutMsg);
}

/*! This method performs a complete reset of the module.  Local module variables that retain
 time varying states between function calls are reset to their default values.
 @return void
 @param configData The configuration data associated with the module
 @param callTime The clock time at which the function was called (nanoseconds)
 @param moduleID The ID associated with the configData
 */
void Reset_MTBFiringSchmitt(MTBFiringSchmittConfig *configData, uint64_t callTime, int64_t moduleID)
{
	//MTBArrayConfigMsgPayload localMTBData; /* local copy of the thruster data message */
	int i;

	configData->prevCallTime = 0;

	// check if the required input messages are included
	if (!MTBArrayConfigMsg_C_isLinked(&configData->MTBConfInMsg))
	{
		_bskLog(configData->bskLogger, BSK_ERROR, "Error: thrFiringSchmitt.thrConfInMsg wasn't connected.");
	}
	if (!MTBArrayCmdForceMsg_C_isLinked(&configData->MTBDipoleInMsg))
	{
		_bskLog(configData->bskLogger, BSK_ERROR, "Error: thrFiringSchmitt.thrForceInMsg wasn't connected.");
	}
    if(!MTBArrayConfigMsg_C_isLinked(&configData->mtbParamsInMsg)){
        _bskLog(configData->bskLogger, BSK_ERROR, "Error: MTBFiringSchmitt.mtbParamsInMsg is not connected.");
    }

	/*! - Zero and read in the support messages */
	//localMTBData = MTBArrayConfigMsg_C_read(&configData->MTBConfInMsg);

	/*! - store the number of installed thrusters */
	//configData->numMTB = localMTBData.numMTB;
    configData->mtbConfigParams = MTBArrayConfigMsg_C_read(&configData->mtbParamsInMsg);


	/*! - loop over all thrusters and for each copy over maximum thrust, set last state to off */
	// for (i = 0; i < configData->numMTB; i++)
	// {
	// 	configData->maxMtbDipoles[i] = configData->GtMatrix_B[i].maxMtbDipoles;
	// 	configData->lastMTBState[i] = BOOL_FALSE;
	// }
}

/*! This method maps the input thruster command forces into thruster on times using a remainder tracking logic.
 @return void
 @param configData The configuration data associated with the module
 @param callTime The clock time at which the function was called (nanoseconds)
 @param moduleID The ID associated with the configData
 */
void Update_MTBFiringSchmitt(MTBFiringSchmittConfig *configData, uint64_t callTime, int64_t moduleID)
{
	int i;
	double level;							  /* [-] duty cycle fraction */
	double controlPeriod;					  /* [s] control period */
	double onTime[MAX_EFF_CNT];				  /* [s] array of commanded on time for thrusters */
	MTBCmdMsgPayload MTBForceIn;	          /* -- copy of the thruster force input message */
	MTBArrayOnTimeCmdMsgPayload MTBOnTimeOut; /* -- copy of the thruster on-time output message */
    int numMTB = configData->mtbConfigParams.numMTB;


	/*! - zero the output message */
	MTBOnTimeOut = MTBArrayOnTimeCmdMsg_C_zeroMsgPayload();

	/*! - the first time update() is called there is no information on the time step.  Here
	 return either all thrusters off or on depending on the baseThrustState state */
	if (configData->prevCallTime == 0)
	{
		configData->prevCallTime = callTime;

		for (i = 0; i < configData->numMTB; i++)
		{
			MTBOnTimeOut.OnTimeRequest[i] = (double)(configData->baseMTBState) * 2.0;
		}

		MTBArrayOnTimeCmdMsg_C_write(&MTBOnTimeOut, &configData->onTimeOutMsg, moduleID, callTime);
		return;
	}

	/*! - compute control time period Delta_t */
	controlPeriod = ((double)(callTime - configData->prevCallTime)) * NANO2SEC;
	configData->prevCallTime = callTime;

	/*! - read the input thruster force message */
	MTBForceIn = MTBCmdMsg_C_read(&configData->MTBDipoleInMsg);

	/*! - Loop through thrusters */
	for (i = 0; i < configData->numMTB; i++)
	{

		/*! - Correct for off-pulsing if necessary.  Here the requested force is negative, and the maximum thrust
		 needs to be added.  If not control force is requested in off-pulsing mode, then the thruster force should
		 be set to the maximum thrust value */
		if (configData->baseMTBState == 1)
		{
			MTBForceIn.mtbDipoleCmds[i] += configData->maxMtbDipoles[i];
		}

		/*! - Do not allow thrust requests less than zero */
		if (MTBForceIn.mtbDipoleCmds[i] < 0.0)
		{
			MTBForceIn.mtbDipoleCmds[i] = 0.0;
		}
		/*! - Compute T_on from thrust request, max thrust, and control period */
		onTime[i] = MTBForceIn.mtbDipoleCmds[i] / configData->maxMtbDipoles[i] * controlPeriod;

		/*! - Apply Schmitt trigger logic */
		if (onTime[i] < configData->MTBMinFireTime)
		{
			/*! - Request is less than minimum fire time */
			level = onTime[i] / configData->MTBMinFireTime;
			if (level >= configData->level_on)
			{
				configData->lastMTBState[i] = BOOL_TRUE;
				onTime[i] = configData->MTBMinFireTime;
			}
			else if (level <= configData->level_off)
			{
				configData->lastMTBState[i] = BOOL_FALSE;
				onTime[i] = 0.0;
			}
			else if (configData->lastMTBState[i] == BOOL_TRUE)
			{
				onTime[i] = configData->MTBMinFireTime;
			}
			else
			{
				onTime[i] = 0.0;
			}
		}
		else if (onTime[i] >= controlPeriod)
		{
			/*! - Request is greater than control period then oversaturate onTime */
			configData->lastMTBState[i] = BOOL_TRUE;
			onTime[i] = 1.1 * controlPeriod; // oversaturate to avoid numerical error
		}
		else
		{
			/*! - Request is greater than minimum fire time and less than control period */
			configData->lastMTBState[i] = BOOL_TRUE;
		}

		/*! Set the output data */
		MTBOnTimeOut.OnTimeRequest[i] = onTime[i];
	}

	MTBArrayOnTimeCmdMsg_C_write(&MTBOnTimeOut, &configData->onTimeOutMsg, moduleID, callTime);

	return;
}
