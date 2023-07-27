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

#ifndef _MTB_FIRING_SCHMITT_H
#define _MTB_FIRING_SCHMITT_H

#include <stdint.h>
#include <fswDefinitions.h>

#include "cMsgCInterface/MTBArrayConfigMsg_C.h"
#include "cMsgCInterface/MTBCmdMsg_C.h"
#include "cMsgCInterface/MTBMsg_C.h"
#include "cMsgCInterface/MTBArrayOnTimeCmdMsg_C.h"

#include "architecture/utilities/macroDefinitions.h"
#include "architecture/utilities/bskLogging.h"




/*! @brief Top level structure for the sub-module routines. */
typedef struct {
    /* declare module public variables */
    double              level_on;                               //!< [-] ON duty cycle fraction
    double              level_off;                              //!< [-] OFF duty cycle fraction 
    double              MTBMinFireTime;                         //!< [s] Minimum ON time for thrusters
    int                 baseMTBState;                        //!< [-] Indicates on-pulsing (0) or off-pusling (1)

    /* declare module private variables */
	int                 numMTB;							//!< [-] The number of thrusters available on vehicle
    double              GtMatrix_B[3*MAX_EFF_CNT];   //!< [-] magnetic torque bar alignment matrix in Body frame components, must be provided in row-major format
	double				maxMtbDipoles[MAX_EFF_CNT];					//!< [N] Max thrust
	boolean_t			lastMTBState[MAX_EFF_CNT];			//!< [-] ON/OFF state of thrusters from previous call
    MTBArrayConfigMsgPayload mtbConfigParams;       //!< configuration for MTB layout

	uint64_t			prevCallTime;							//!< callTime from previous function call

    /* declare module IO interfaces */
    MTBArrayConfigMsg_C mtbParamsInMsg;             //!< input message for MTB layout
    MTBCmdMsg_C MTBDipoleInMsg; //!< The name of the Input message
    MTBArrayOnTimeCmdMsg_C onTimeOutMsg;  //!< The name of the output message*, onTimeOutMsgName
    MTBArrayConfigMsg_C MTBConfInMsg;	//!< The name of the thruster cluster Input message

  BSKLogger *bskLogger;                             //!< BSK Logging

}MTBFiringSchmittConfig;

#ifdef __cplusplus
extern "C" {
#endif
    
    void SelfInit_MTBFiringSchmitt(MTBFiringSchmittConfig *configData, int64_t moduleID);
    void Update_MTBFiringSchmitt(MTBFiringSchmittConfig *configData, uint64_t callTime, int64_t moduleID);
    void Reset_MTBFiringSchmitt(MTBFiringSchmittConfig *configData, uint64_t callTime, int64_t moduleID);
    
#ifdef __cplusplus
}
#endif


#endif
