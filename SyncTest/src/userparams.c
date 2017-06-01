#include "userparams.h"

//****************************************************************************
// Functions
//****************************************************************************
void SetUserParams(UserParams_t *Params)
{
	// CONNECTION PARAMETERS:
	// ConnectionType: can be CAEN_DGTZ_USB, CAEN_DGTZ_PCI_OpticalLink (A2818) or CAEN_DGTZ_PCIE_OpticalLink (A3818)
	// LinkNum: USB or PCI/PCIe enumeration (typ=0)
	// ConetNode: position in the optical daisy chain
	// BaseAddress: only for VME access (otherwise 0)

  Params->ConnectionType[0]	= CAEN_DGTZ_PCI_OpticalLink;//CAEN_DGTZ_USB;
	Params->LinkNum[0]			= 0;
	Params->ConetNode[0]		= 0;
	Params->BaseAddress[0]		= 0;//0x32100000;

	Params->ConnectionType[1]	= CAEN_DGTZ_PCI_OpticalLink;//CAEN_DGTZ_USB;
	Params->LinkNum[1]			= 0;
	Params->ConetNode[1]		= 1;//0;
	Params->BaseAddress[1]		= 0;//0x32110000;


	// CHANNEL SETTINGS
	Params->RefChannel[0]		= 7;		// Channel used for the acquisition
	Params->TriggerThreshold[0]	= 100;		// Trigger threshold (for self triggering)
	Params->PostTrigger[0]		= 70;		// Post trigger in percent of the acquisition window
	Params->DCoffset[0]			= 50000;   // input DC offset adjust (DAC value)

	Params->RefChannel[1]		= 7;		// Channel used for the acquisition
	Params->TriggerThreshold[1]	= 100;		// Trigger threshold (for self triggering)
	Params->PostTrigger[1]		= 70;		// Post trigger in percent of the acquisition window
	Params->DCoffset[1]			= 50000;   // input DC offset adjust (DAC value)

	// Trigger edge (CAEN_DGTZ_TriggerOnRisingEdge, CAEN_DGTZ_TriggerOnFallingEdge)
	Params->TriggerEdge			= CAEN_DGTZ_TriggerOnFallingEdge;


	// Number of samples in the acquisition windows
	Params->RecordLength		= 1000;

	// Max. distance between the trigger time tags in order to consider a valid coincidence
	Params->MatchingWindow		= 2000;

	// Front Panel LEMO I/O level (NIM or TTL). Options: CAEN_DGTZ_IOLevel_NIM, CAEN_DGTZ_IOLevel_TTL
	Params->IOlevel				= CAEN_DGTZ_IOLevel_TTL;


	// ***************************************************************************************************
	// Parameters for the time distribution histogram
	// ***************************************************************************************************
	Params->HistoNbins			= 4000;		// Number of bins of the histogram
	Params->HistoBinSize		= 0.1;		// Bin size in ns
	Params->HistoOffset			= -200.0;	// Lower value of the histogram in ns


	// ***************************************************************************************************
	// Start Mode: 
	// ***************************************************************************************************
	// Options: START_SW_CONTROLLED, START_HW_CONTROLLED
	Params->StartMode = START_HW_CONTROLLED;

	// ***************************************************************************************************
	// Sync Mode: 
	// ***************************************************************************************************
	// Options: COMMONT_EXTERNAL_TRIGGER_TRGIN_TRGOUT, INDIVIDUAL_TRIGGER_SIN_TRGOUT, TRIGGER_ONE2ALL_EXTOR
	Params->SyncMode = INDIVIDUAL_TRIGGER_SIN_TRGOUT;

	/*
	----------------------------------------------------------------------------
	CASE 2: INDIVIDUAL_TRIGGER_SIN_TRGOUT
	----------------------------------------------------------------------------
	N boards with independent triggers generated as OR of the local channel triggers

	SETUP:
	daisy chain SIN-TRGOUT to propagate the start of run

	START OF RUN: 
	1.	All boards: set start mode = start on SIN high level; 
	2.	All boards: set Trg Mask = Channels self trigger
	3.	All boards: set propagation of SIN to TRG-OUT
	4.	All boards armed to start
	5.	Send SW Start to 1st board
	6.	The RUN signal is propagate through the daisy chain SIN-TRGOUT and start all the boards. 
		Delay can be compensated by means of RUN_DELAY register
	7.	Once started, each board is triggered by the OR of the channel self-triggers

	NOTE1: Start and Stop can also be controlled by an external signal going into SIN of the 1st board
	NOTE2: the stop is also synchronous:  when the 1st board is stopped (by SW command), the stop is propagated to the other boards 
		   through the daisy chain.
	NOTE3: in this configuration it is also possible to trigger the boards with individual external triggers (one per board) going into TRGIN
	*/
}


