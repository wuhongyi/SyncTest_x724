/*
  Synctest application is a simple piece of software to demonstrates
  multiboard synchronization with CAEN digitizers.
  It only depends on CAENDigitizer library.

  So far, this software has been tested with V1724, V1720, V1721/31 and V1740
  the V1751 is not yet supported.

  Synctest can be adapted to different synchronization setup by changing 
  the parameter SyncMode. See UserParams.c for more details
*/

#include <stdio.h>
#include <math.h>
#include <float.h>
#include <CAENDigitizer.h>

#include "userparams.h"
#include "synctest.h"


#ifdef WIN32
#define GNUPLOTEXE  "pgnuplot"
#else
#define GNUPLOTEXE  "gnuplot"
#endif


/* ============================================================================== */
/* Get time in milliseconds from the computer internal clock */
long get_time()
{
  long time_ms;
#ifdef WIN32
  struct _timeb timebuffer;
  _ftime( &timebuffer );
  time_ms = (long)timebuffer.time * 1000 + (long)timebuffer.millitm;
#else
  struct timeval t1;
  struct timezone tz;
  gettimeofday(&t1, &tz);
  time_ms = (t1.tv_sec) * 1000 + t1.tv_usec / 1000;
#endif
  return time_ms;
}


/* ============================================================================== */
double  getSamplingPeriod(CAEN_DGTZ_BoardInfo_t BoardInfo, int DesMode)
{
  double Ts;
  switch(BoardInfo.FamilyCode) {
  case CAEN_DGTZ_XX724_FAMILY_CODE: Ts = 10.0; break;
  case CAEN_DGTZ_XX720_FAMILY_CODE: Ts =  4.0; break;
  case CAEN_DGTZ_XX721_FAMILY_CODE: Ts =  2.0; break;
  case CAEN_DGTZ_XX731_FAMILY_CODE: Ts =  2.0; break;
  case CAEN_DGTZ_XX751_FAMILY_CODE: Ts =  1.0; break;
  case CAEN_DGTZ_XX740_FAMILY_CODE: Ts = 16.0; break;
  default: return 0.0;
  }

  if (DesMode && (((BoardInfo.FamilyCode == CAEN_DGTZ_XX751_FAMILY_CODE) ||
		   (BoardInfo.FamilyCode == CAEN_DGTZ_XX731_FAMILY_CODE) )))
    Ts /= 2;

  return Ts;

}

/* ============================================================================== */
double getTriggerClockPeriod(CAEN_DGTZ_BoardInfo_t BoardInfo)
{
  double Tt;
  switch(BoardInfo.FamilyCode) {
  case CAEN_DGTZ_XX724_FAMILY_CODE: Tt = 10.0; break;
  case CAEN_DGTZ_XX720_FAMILY_CODE: Tt =  8.0; break;
  case CAEN_DGTZ_XX721_FAMILY_CODE: Tt =  8.0; break;
  case CAEN_DGTZ_XX731_FAMILY_CODE: Tt =  8.0; break;
  case CAEN_DGTZ_XX751_FAMILY_CODE: Tt =  8.0; break;
  case CAEN_DGTZ_XX740_FAMILY_CODE: Tt =  8.0; break;
  default: return 0.0;
  }
  return Tt;
}

/* ============================================================================== */
double interpolate(uint16_t* data, unsigned int length, int threshold, int edge, double Tstart) 
{
  unsigned int i;
  double crosspoint = -1.0;

  for (i=(int)(Tstart); i < length-1; ++i) {
    if ((edge == CAEN_DGTZ_TriggerOnFallingEdge) && (data[i] >= threshold) && (data[i+1] < threshold)) {
      crosspoint = i + ((double)(data[i] - threshold)/(double)(data[i]-data[i+1]));
      break;
    }
    if ((edge == CAEN_DGTZ_TriggerOnRisingEdge) && (data[i] <= threshold) && (data[i+1] > threshold)) {
      crosspoint = i + ((double)(threshold - data[i])/(double)(data[i+1]-data[i])) ;
      break;
    }
  }
  return crosspoint;
}

/* ============================================================================== */
int ConfigureDigitizers(int handle[2], CAEN_DGTZ_BoardInfo_t Binfo[2], UserParams_t Params) 
{
  int i, ret = 0, ch, mask = 0;
  uint32_t rdata;

  for(i=0; i<2; i++) {
    /* Reset all board registers */
    ret |= CAEN_DGTZ_Reset(handle[i]);

    ret |= CAEN_DGTZ_WriteRegister(handle[i], 0x8000, 0x01000114);  // Channel Control Reg (indiv trg, seq readout) ??
    ret |= CAEN_DGTZ_SetDPPAcquisitionMode(handle[i], CAEN_DGTZ_DPP_ACQ_MODE_Mixed, CAEN_DGTZ_DPP_SAVE_PARAM_EnergyAndTime);


    
    ret |= CAEN_DGTZ_SetRecordLength(handle[i], Params.RecordLength);
    ret |= CAEN_DGTZ_SetPostTriggerSize(handle[i], Params.PostTrigger[i]);
    ret |= CAEN_DGTZ_SetIOLevel(handle[i], Params.IOlevel);
    ret |= CAEN_DGTZ_SetMaxNumEventsBLT(handle[i], MAX_EVENTS_XFER);


    ret |= CAEN_DGTZ_SetChannelEnableMask(handle[i], 0xFF);//1<<Params.RefChannel[i]
    for(ch = 0; ch < (int)Binfo[i].Channels; ch++) {
      ret |= CAEN_DGTZ_SetChannelDCOffset(handle[i], ch, Params.DCoffset[i]); // IModify was Params.DCoffset[ch]
      ret |= CAEN_DGTZ_SetTriggerPolarity(handle[i], ch, Params.TriggerEdge);
      ret |= CAEN_DGTZ_SetChannelTriggerThreshold(handle[i], ch, Params.TriggerThreshold[i]); // IModify Params.TriggerThreshold[ch]

      ret |= CAEN_DGTZ_SetChannelPulsePolarity(handle[i], ch, CAEN_DGTZ_PulsePolarityPositive);
    }

    ret |= CAEN_DGTZ_SetDPP_PHA_VirtualProbe(handle[i], CAEN_DGTZ_DPP_VIRTUALPROBE_DUAL, CAEN_DGTZ_DPP_PHA_VIRTUALPROBE1_Delta2, CAEN_DGTZ_DPP_PHA_VIRTUALPROBE2_Input, CAEN_DGTZ_DPP_PHA_DIGITAL_PROBE_TRGHoldoff);



  }
  return ret;
}

/* ============================================================================== */
int SetSyncMode(int handle[2], CAEN_DGTZ_BoardInfo_t Binfo[2], UserParams_t Params) 
{
  int i, ret=0;
  uint32_t reg;

  for(i=0; i<2; i++) {

    if (i > 0)  // Run starts with S-IN on the 2nd board
      ret |= CAEN_DGTZ_WriteRegister(handle[i], ADDR_ACQUISITION_MODE, RUN_START_ON_SIN_LEVEL);

    
    ret |= CAEN_DGTZ_WriteRegister(handle[i], ADDR_GLOBAL_TRG_MASK, 0x40000000 + (1<<(int)Params.RefChannel[i]));  //  accept EXT TRGIN or trg from selected channel

    ret |= CAEN_DGTZ_WriteRegister(handle[i], ADDR_TRG_OUT_MASK, 0);   // no trigger propagation to TRGOUT
    ret |= CAEN_DGTZ_WriteRegister(handle[i], ADDR_RUN_DELAY, 2*(1-i));   // Run Delay decreases with the position (to compensate for run the propagation delay)

    // Set TRGOUT=RUN to propagate run through S-IN => TRGOUT daisy chain
    ret |= CAEN_DGTZ_ReadRegister(handle[i], ADDR_FRONT_PANEL_IO_SET, &reg);
    reg = reg & 0xFFF0FFFF | 0x00010000;
    ret |= CAEN_DGTZ_WriteRegister(handle[i], ADDR_FRONT_PANEL_IO_SET, reg);
      
  }
  return ret; 
}

/* ============================================================================== */
int StartRun(int handle[2], int SyncMode, int StartMode)
{
  switch(SyncMode) {
  case COMMONT_EXTERNAL_TRIGGER_TRGIN_TRGOUT:
  case TRIGGER_ONE2ALL_EXTOR:
    // Start on first software trigger
    if (StartMode == START_SW_CONTROLLED)
      CAEN_DGTZ_SendSWtrigger(handle[0]);
    if (SyncMode == COMMONT_EXTERNAL_TRIGGER_TRGIN_TRGOUT) 
      CAEN_DGTZ_WriteRegister(handle[0], ADDR_EXT_TRG_INHIBIT, 0); // Enable TRGIN of the first board
    break;
  case INDIVIDUAL_TRIGGER_SIN_TRGOUT:
    if (StartMode == START_SW_CONTROLLED) {
      CAEN_DGTZ_WriteRegister(handle[0], ADDR_ACQUISITION_MODE, 0x4);
    } else {
      CAEN_DGTZ_WriteRegister(handle[0], ADDR_ACQUISITION_MODE, 0x5);
      printf("Run starts/stops on the S-IN high/low level\n");
    }
    break;
  default:
    return -1;
  }
  return 0;
}


/* ============================================================================== */
int StopRun(int handle[2], int SyncMode)
{
  int i;
  switch(SyncMode) {
  case COMMONT_EXTERNAL_TRIGGER_TRGIN_TRGOUT:
  case TRIGGER_ONE2ALL_EXTOR:
    for(i=0; i<2; i++)
      CAEN_DGTZ_WriteRegister(handle[i], ADDR_ACQUISITION_MODE, 0);
    break;
  case INDIVIDUAL_TRIGGER_SIN_TRGOUT:
    CAEN_DGTZ_WriteRegister(handle[0], ADDR_ACQUISITION_MODE, 0x0);
    break;
  default:
    return -1;
  }
  return 0;
}


/* ============================================================================== */
int ForceClockSync(int handle)
{    
  int ret;
  SLEEP(500);
  /* Force clock phase alignment */
  ret = CAEN_DGTZ_WriteRegister(handle, ADDR_FORCE_SYNC, 1);
  /* Wait an appropriate time before proceeding */
  SLEEP(1000);
  return ret;
}




/* ########################################################################### */
/* MAIN                                                                        */
/* ########################################################################### */
int main(int argc, char *argv[])
{
  UserParams_t  Params;
  int i, ret=0, Nbit, Nch, error=1, running=0;
  int GetNextEvent[2]={1,1};
  int QuitAcquisition=0, plot=0, PlotHist=0, ContinousHistPlot=0, AlignTT=0;
  int handle[2];
  char *buffer[2]={NULL, NULL}, *EventPtr[2]={NULL, NULL};
  CAEN_DGTZ_BoardInfo_t  BoardInfo[2];
  CAEN_DGTZ_EventInfo_t  EventInfo[2];
  CAEN_DGTZ_UINT16_EVENT_t  *Event16[2] = {NULL, NULL}; /* generic event struct with 16 bit data (10, 12, 14 and 16 bit digitizers */
  CAEN_DGTZ_UINT8_EVENT_t   *Event8[2]  = {NULL, NULL}; /* generic event struct with 8 bit data (only for 8 bit digitizers) */
  uint16_t *Wave[2];
  uint32_t Ns[2], EIndx[2]={0,0}, NumEvents[2]={0,0}, BufferSize[2]={0,0}, TrgCnt[2]={0,0}, missingEdge[2]={0,0};
  uint32_t Nb=0, MatchingEvents=0;
  uint32_t *histoT = NULL;
  uint64_t CurrentTime, PrevRateTime, ElapsedTime, NsT=0, NsTTT=0, Nroll[2]={0,0};
  char c;
  double Ts, Tt;
  double TTT[2], PrevTTT[2]={0,0}, DeltaT, DeltaTTT, edgeTime[2];
  double MeanT=0.0, MeanTTT=0.0, SigmaT=0.0, SigmaTTT=0.0;

  FILE *log        = NULL;
  FILE *event_file = NULL;
  FILE *plotter;


  printf("\n");
  printf("**************************************************************\n");
  printf(" CAEN Digitizer Multiboard synchronization test %s\n", SyncTest_Release);
  printf("**************************************************************\n");

  SetUserParams(&Params);

  /* *************************************************************************************** */
  /* OPEN DIGITIZERS                                                                         */
  /* *************************************************************************************** */
  for (i = 0; i < 2; i++) {
    ret = CAEN_DGTZ_OpenDigitizer(Params.ConnectionType[i], Params.LinkNum[i], Params.ConetNode[i], Params.BaseAddress[i], &handle[i]);
    if (ret) {
      printf("Can't open digitizer n. %d\n", i);
      goto QuitProgram;
    }
  }

  /* *************************************************************************************** */
  /* GET BOARD INFO AND FW REVISION                                                          */
  /* *************************************************************************************** */
  for (i = 0; i < 2; i++) {
    int MajorNumber;
    ret = CAEN_DGTZ_GetInfo(handle[i], &BoardInfo[i]);
    if (ret) {
      printf("Can't read board info for digitizer n. %d\n", i);
      goto QuitProgram;
    }
    /* Check firmware rivision (DPP firmwares are not supported) */
    sscanf(BoardInfo[i].AMC_FirmwareRel, "%d", &MajorNumber);
    printf("MajorNumber: %d\n",MajorNumber);
    /* if (MajorNumber >= 128) { */
    /*     printf("This digitizer has a DPP firmware; TestSync doesn't support it\n"); */
    /*     goto QuitProgram; */
    /* } */
    printf("Connected to CAEN Digitizer Model %s\n", BoardInfo[i].ModelName);
    printf("ROC FPGA Release is %s\n", BoardInfo[i].ROC_FirmwareRel);
    printf("AMC FPGA Release is %s\n\n", BoardInfo[i].AMC_FirmwareRel);
  }

  /* Get num of channels, num of bit, num of group of the board from first board */
  Nbit = BoardInfo[0].ADC_NBits;
  Nch  = BoardInfo[0].Channels;
  Ts = getSamplingPeriod(BoardInfo[0], Params.DesMode);
  Tt = getTriggerClockPeriod(BoardInfo[0]);
  if ((Ts == 0.0 ) || (Tt == 0.0)) {
    printf("Invalid board type\n");
    goto QuitProgram;
  }


  /* *************************************************************************************** */
  /* BOARD CONFIGURATION                                                                     */
  /* *************************************************************************************** */
  // Set registers for the acquisition (record length, channel mask, etc...)
  ret = ConfigureDigitizers(handle, BoardInfo, Params);
  // Set registers for the synchronization (start mode, trigger masks, signals propagation, etc...)
  ret |= SetSyncMode(handle, BoardInfo, Params);
  /* if (ret) { */
  /*   printf("Errors occurred during digitizer configuration\n"); */
  /*   goto QuitProgram; */
  /* } */


  /* *************************************************************************************** */
  /* MEMORY ALLOCATION                                                                       */
  /* *************************************************************************************** */    
  for(i = 0; i < 2; ++i) {
    int AllocatedSize;
    /* Memory allocation for event buffer and readout buffer */
    if(Nbit == 8)    
      ret = CAEN_DGTZ_AllocateEvent(handle[i], &Event8[i]);
    else             
      ret = CAEN_DGTZ_AllocateEvent(handle[i], &Event16[i]);
    /* for 8 bit digitizers, need buffer to copy waveform from 8 to 16 bit memory space */
    if(Nbit == 8)    
      Wave[i] = malloc(Params.RecordLength * sizeof(uint16_t));
         
    /* NOTE : This malloc must be done after the digitizer programming */ 
    ret |= CAEN_DGTZ_MallocReadoutBuffer(handle[i], &buffer[i], &AllocatedSize); 
    if (ret) {
      printf("Can't allocate memory for the acquisition\n");
      goto QuitProgram;
    }
  }
  // Allocate memory for the histograms
  histoT = malloc(Params.HistoNbins * sizeof(uint32_t));
  memset(histoT, 0, Params.HistoNbins * sizeof(uint32_t));

  /* *************************************************************************************** */
  /* OPEN FILES                                                                              */
  /* *************************************************************************************** */
  if (Params.EnableLog)  
    log = fopen("events.txt", "w");  // Open Output Files
  plotter = popen(GNUPLOTEXE, "w");    // Open plotter pipe (gnuplot)

  /* *************************************************************************************** */
  /* CHECK CLOCK ALIGNMENT                                                                   */
  /* *************************************************************************************** */
  printf("Boards Configured. Press [s] to start run or [c] to check clock alignment\n\n");
  c = getch();
  if (c == 'c') {
    uint32_t rdata[2];
    // propagate CLK to trgout on both boards
    for(i=0; i<2; i++) {
      CAEN_DGTZ_ReadRegister(handle[i], ADDR_FRONT_PANEL_IO_SET, &rdata[i]);
      CAEN_DGTZ_WriteRegister(handle[i], ADDR_FRONT_PANEL_IO_SET, 0x00050000);
    }
    printf("Trigger Clk is now output on TRGOUT.\n");
    printf("Press [r] to reload PLL config, [s] to start acquisition, any other key to quit\n");
    while( (c=getch()) == 'r') {
      CAEN_DGTZ_WriteRegister(handle[0], ADDR_RELOAD_PLL, 0);
      ForceClockSync(handle[0]);
      printf("PLL reloaded\n");
    }
    for(i=0; i<2; i++) 
      CAEN_DGTZ_WriteRegister(handle[i], ADDR_FRONT_PANEL_IO_SET, rdata[i]);
  }
  if (c != 's')
    goto QuitProgram;

  /* *************************************************************************************** */
  /* START RUN                                                                               */
  /* *************************************************************************************** */
  ForceClockSync(handle[0]);            // Force clock sync in board 0
  StartRun(handle, Params.SyncMode, Params.StartMode);  // Start Run
  running = 1;
  printf("Run started\n");

  /* *************************************************************************************** */
  /* READOUT LOOP                                                                            */
  /* *************************************************************************************** */
  PrevRateTime = get_time();
  while(!QuitAcquisition) {

    // --------------------------------------------
    // check for keyboard commands
    // --------------------------------------------
    if (kbhit()) {
      c = getch();
      if (c == 'q')   
	QuitAcquisition = 1;
      if (c == 'p') { 
	plot = 1;
	AlignTT = 0;
      }
      if (c == 't') {
	plot = 1;
	AlignTT = 1;
      }
      if (c == 'h') {
	PlotHist = 1;
	ContinousHistPlot = 0;
      }
      if (c == 'H') {
	PlotHist = 1;
	ContinousHistPlot = 1;
      }
      if (c == 's') {
	if (!running) {
	  GetNextEvent[0]=1;
	  GetNextEvent[1]=1;
	  NumEvents[0]=0;
	  NumEvents[1]=0;
	  SetSyncMode(handle, BoardInfo, Params);
	  StartRun(handle, Params.SyncMode, Params.StartMode);
	  running=1;
	  printf("Acquisition started\n\n");
	} else {
	  StopRun(handle, Params.SyncMode);
	  running=0;
	  printf("Acquisition stopped. Press s to restart\n\n");
	}
      }
      if (c == 'r') {
	memset(histoT, 0, Params.HistoNbins * sizeof(uint32_t));
	MeanT=0.0; 
	MeanTTT=0.0;
	SigmaT=0.0;
	SigmaTTT=0.0;
	NsT=0;
	NsTTT=0;
      }
    }

    // ----------------------------------------------------------------
    // Calculate and print throughput and trigger rate (every second)
    // ----------------------------------------------------------------
    CurrentTime = get_time();
    ElapsedTime = CurrentTime - PrevRateTime;
    if (ElapsedTime > 1000) {
      printf("Readout Rate=%.2f MB\n", (float)Nb/((float)ElapsedTime*1048.576f));
      for(i=0; i<2; i++) {
	if (TrgCnt[i]>0) {
	  printf("Board %d:\tTrgRate=%.2f KHz. Matching Events=%.2f%%; ", i, (float)TrgCnt[i]/(float)ElapsedTime, 100*(float)MatchingEvents/(float)TrgCnt[i], 100*(float)missingEdge[i]/(float)MatchingEvents);
	  if (MatchingEvents>0)   
	    printf("Missing Edges=%.2f%%\n", 100*(float)missingEdge[i]/(float)MatchingEvents);
	  else
	    printf("No edge found\n");
	} else {
	  printf("Board %d:\tNo Data\n", i);
	}
	TrgCnt[i]=0;
	missingEdge[i]=0;
      }
      if (NsT == 0){
	printf("DeltaT edges:    mean= -----  sigma= -----\n");
	printf("DeltaT time tag: mean= -----  sigma= -----\n");
      }
      else{
	printf("DeltaT edges:    mean= %.4f  sigma= %.4f\n", MeanT/NsT, sqrt(SigmaT/NsT-(MeanT*MeanT)/(NsT*NsT)));
	printf("DeltaT time tag: mean= %.4f  sigma= %.4f\n", MeanTTT/NsTTT, sqrt(SigmaTTT/NsTTT-(MeanTTT*MeanTTT)/(NsTTT*NsTTT)));			
      }
      Nb = 0;
      MatchingEvents=0;
      if (PlotHist) {
	FILE *hist_file;
	if (!ContinousHistPlot) 
	  PlotHist = 0;
	hist_file = fopen("hist.txt", "w");
	for (i = 0; i < Params.HistoNbins; ++i)
	  fprintf(hist_file, "%f\t%d\n", (float)(i*Params.HistoBinSize + Params.HistoOffset), histoT[i]);
	fclose(hist_file);
	if (plotter) {
	  fprintf(plotter, "set xlabel 'Time Difference [ns]' \n");
	  fprintf(plotter, "plot 'hist.txt' w boxes fs solid 0.7\n");
	  fflush(plotter);
	}
      }            
      PrevRateTime = CurrentTime;
      printf("\n\n");
    }

    // ----------------------------------------------------------------
    // Read data
    // ----------------------------------------------------------------
    for(i=0; i<2; i++) {
      if (GetNextEvent[i]) {
	/* read a new data block from the board if there are no more events to use in the readout buffer */
	if (EIndx[i] >= NumEvents[i]) {
	  EIndx[i] = 0;
	  /* Read a data block from the board */
	  ret = CAEN_DGTZ_ReadData(handle[i], CAEN_DGTZ_SLAVE_TERMINATED_READOUT_MBLT, buffer[i], &BufferSize[i]);
	  Nb += BufferSize[i];
	  ret |= CAEN_DGTZ_GetNumEvents(handle[i], buffer[i], BufferSize[i], &NumEvents[i]);
	  if (ret) {
	    printf("Readout Error\n");
	    goto QuitProgram;
	  }
	}
	/* Get one event from the readout buffer */
	if (NumEvents[i]) {
	  ret = CAEN_DGTZ_GetEventInfo(handle[i], buffer[i], BufferSize[i], EIndx[i], &EventInfo[i], &EventPtr[i]);
	  TrgCnt[i]++;
	  if (ret) {
	    printf("Event build error\n");
	    goto QuitProgram;
	  }
	  GetNextEvent[i]=0;
	} 
      }
    }
    if (GetNextEvent[0] || GetNextEvent[1])  // missing data from one or both boards
      continue;

    // ----------------------------------------------------------------
    // Analyze data
    // ----------------------------------------------------------------
    // calculate extended Trigger Time Tag (take roll over into account)
    for(i=0; i<2; i++) {
      TTT[i] = ((Nroll[i]<<31) + (EventInfo[i].TriggerTimeTag & 0x7FFFFFFF))*Tt;
      if (TTT[i] < PrevTTT[i]) {
	Nroll[i]++;
	TTT[i] += (1<<31)*Tt;
      }
      PrevTTT[i] = TTT[i];
    }

    // use only events whose time stamp differ of less than the matching window:
    // CASE1: board 0 is behind board 1; keep event from 1 and take next event from 0
    if (TTT[0] < (TTT[1] - Params.MatchingWindow*Tt)) {
      EIndx[0]++; 
      GetNextEvent[0]=1;
      continue;
      // CASE2: board 1 is behind board 0; keep event from 0 and take next event from 1
    } else if (TTT[1] < (TTT[0] - Params.MatchingWindow*Tt)) {
      EIndx[1]++; 
      GetNextEvent[1]=1;
      continue;
      // CASE3: trigger time tags match: calculate DeltaT between edges
    } else {   
      MatchingEvents++;
      for(i=0; i<2; i++) {
	EIndx[i]++;
	GetNextEvent[i]=1;
	/* Decode event of both boards */
	ret = CAEN_DGTZ_DecodeEvent(handle[i],EventPtr[i],&Event16[i]);
	Ns[i] = Event16[i]->ChSize[Params.RefChannel[i]];
	Wave[i] = Event16[i]->DataChannel[Params.RefChannel[i]];

	/* calculate threshold crossing time by interpolation */
	edgeTime[i] = interpolate(Wave[i], Ns[i], Params.TriggerThreshold[i], Params.TriggerEdge, 0);
	if (edgeTime[i] < 0) 
	  missingEdge[i]++;
      }
            
      // if both edge times have been found, calculate deltaT and update statistics
      if ((edgeTime[0] >= 0) && (edgeTime[1] >= 0)) {
	int bin;
	// Calculate Delta T on the time tags
	DeltaTTT = TTT[0] - TTT[1];
	MeanTTT += DeltaTTT;
	SigmaTTT += (DeltaTTT*DeltaTTT);
	NsTTT++;

	// Calculate Delta T between the edges
	DeltaT =  (TTT[0] + edgeTime[0]*Ts) - (TTT[1] + edgeTime[1]*Ts);
	MeanT += DeltaT;
	SigmaT += (DeltaT*DeltaT);
	NsT++;
	bin = (int)((DeltaT-Params.HistoOffset)/Params.HistoBinSize);
	if ((bin>=0) && (bin<Params.HistoNbins))    
	  histoT[bin]++;
      }
                          
      // Plot Waveforms
      if (plot) {
	plot = 0;
	event_file = fopen("event.txt", "w");
	if (AlignTT) {
	  for(i=0; i < Params.RecordLength; i++) {
	    int j = (int)(DeltaTTT/Ts);
	    if (((i+j)>=0) && ((i+j)<Params.RecordLength))
	      fprintf(event_file, "%d\t%d\n", Wave[0][i], Wave[1][i+j]);
	  }
	} else {
	  for(i=0; i < Params.RecordLength; i++) {
	    fprintf(event_file, "%d\t%d\n", Wave[0][i], Wave[1][i]);
	  }
	}
	fclose(event_file);
	fprintf(plotter, "set xlabel 'Samples' \n");
	fprintf(plotter, "plot 'event.txt' using ($0):($1) title 'Board 0' with lines 1, 'event.txt' using ($0):($2) title 'Board 1' with lines 2\n");
	fflush(plotter);
	SLEEP(1);
      }

      // Write log file
      if (Params.EnableLog) {
	fprintf(log, "EvCnt0=%d  EvCnt1=%d  TTT0=%d  TTT1=%d  DeltaTTT=%f  DeltaT=%f\n", 
		EventInfo[0].EventCounter, EventInfo[1].EventCounter,
		EventInfo[0].TriggerTimeTag, EventInfo[1].TriggerTimeTag,
		DeltaTTT, DeltaT);
	fflush(log);
      }            

    }
            
  } /* End of readout loop */
  error=0;

 QuitProgram:
  if (error)
    _getch();

  /* *************************************************************************************** */
  /* FINAL CLEANUP                                                                           */
  /* *************************************************************************************** */
  for (i = 0; i < 2; i++) {
    /* stop the acquisition */
    CAEN_DGTZ_SWStopAcquisition(handle[i]);
    /* close the device and free the buffers */
    if(Event8[i])    CAEN_DGTZ_FreeEvent(handle[i], (void**)&Event8[i]);
    if(Event16[i])    CAEN_DGTZ_FreeEvent(handle[i], (void**)&Event16[i]);
    CAEN_DGTZ_FreeReadoutBuffer(&buffer[i]);
    /* close connection to boards */
    CAEN_DGTZ_CloseDigitizer(handle[i]);
  }
  if (histoT)    free(histoT);
  /* close open files */
  if (log)       fclose(log);
  if (plotter)   pclose(plotter);
  return 0;
}
