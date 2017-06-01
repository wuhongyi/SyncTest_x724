/******************************************************************************
* 
* CAEN SpA - Front End Division
* Via Vetraia, 11 - 55049 - Viareggio ITALY
* +390594388398 - www.caen.it
*
***************************************************************************//**
* \note TERMS OF USE:
* This program is free software; you can redistribute it and/or modify it under
* the terms of the GNU General Public License as published by the Free Software
* Foundation. This program is distributed in the hope that it will be useful, 
* but WITHOUT ANY WARRANTY; without even the implied warranty of 
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. The user relies on the 
* software, documentation and results solely at his own risk.
******************************************************************************/

#ifndef __USERPARAMS_H
#define __USERPARAMS_H

#include <CAENDigitizer.h>

//****************************************************************************
// Start Modes
//****************************************************************************
#define START_SW_CONTROLLED   0
#define START_HW_CONTROLLED   1

//****************************************************************************
// Sync Modes
//****************************************************************************
#define COMMONT_EXTERNAL_TRIGGER_TRGIN_TRGOUT   0
#define INDIVIDUAL_TRIGGER_SIN_TRGOUT			1
#define TRIGGER_ONE2ALL_EXTOR					2

//****************************************************************************
// Variables for the user parameters
//****************************************************************************
typedef struct {
    int ConnectionType[2];
    int LinkNum[2];
    int ConetNode[2];
    uint32_t BaseAddress[2];
    
    int RefChannel[2];
	uint16_t TriggerThreshold[2];
    uint16_t DCoffset[2];
    uint32_t PostTrigger[2];
    
	int RecordLength;
	int TriggerEdge;
	int MatchingWindow;
	int DesMode;
	int TestPattern;
	int IOlevel;
	int SyncMode;
	int StartMode;
	int EnableLog;

	int HistoNbins;
	double HistoOffset;
	double HistoBinSize;
} UserParams_t;

//****************************************************************************
// Functions
//****************************************************************************
void SetUserParams(UserParams_t *Params);

#endif // __USERPARAMS_H