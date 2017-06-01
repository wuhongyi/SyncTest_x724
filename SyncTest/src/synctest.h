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

#ifndef __SYNCTEST_H
#define __SYNCTEST_H

#define SyncTest_Release        "1.2.1"
#include <CAENDigitizer.h>
#include <math.h>
#include "keyb.h"

/*! < redefine POSIX 'deprecated' */
#ifndef _getch
#define _getch getch
#endif

#ifndef _kbhit
#define _kbhit kbhit
#endif

#ifdef WIN32

    #include <time.h>
    #include <sys/timeb.h>
    #include <conio.h>
    #include <process.h>

	#define	_PACKED_
	#define	_INLINE_		
    #define popen  _popen    /* redefine POSIX 'deprecated' popen as _popen */
    #define pclose  _pclose  /* redefine POSIX 'deprecated' pclose as _pclose */

    #define SLEEP(x) Sleep(x)

#else
    #include <sys/time.h> /* struct timeval, select() */
    #include <termios.h> /* tcgetattr(), tcsetattr() */
    #include <stdlib.h> /* atexit(), exit(), C99 compliant compilers: uint64_t */
    #include <unistd.h> /* read() */
    #include <stdio.h> /* printf() */
    #include <string.h> /* memcpy() */
    #include <ctype.h>    /* toupper() */

    #define UINT64_T uint64_t
    #define UINT32_T uint32_t

	#define		_PACKED_		__attribute__ ((packed, aligned(1)))
	#define		_INLINE_		__inline__ 

    #define SLEEP(x) usleep(x*1000)

#endif

//****************************************************************************
// Some register addresses
//****************************************************************************
#define ADDR_GLOBAL_TRG_MASK     0x810C
#define ADDR_TRG_OUT_MASK        0x8110
#define ADDR_FRONT_PANEL_IO_SET  0x811C
#define ADDR_ACQUISITION_MODE    0x8100
#define ADDR_EXT_TRG_INHIBIT     0x817C
#define ADDR_RUN_DELAY           0x8170
#define ADDR_FORCE_SYNC			 0x813C
#define ADDR_RELOAD_PLL			 0xEF34
#define ADDR_GROUP_TRG_MASK      0x10A8

//****************************************************************************
// Run Modes
//****************************************************************************
// start on software command 
#define RUN_START_ON_SOFTWARE_COMMAND     0xC 
// start on S-IN level (logical high = run; logical low = stop)
#define RUN_START_ON_SIN_LEVEL            0xD
// start on first TRG-IN or Software Trigger 
#define RUN_START_ON_TRGIN_RISING_EDGE    0xE
// start on LVDS I/O level
#define RUN_START_ON_LVDS_IO              0xF

//****************************************************************************
// Max number of event to read in a Block Transfer
//****************************************************************************
#define MAX_EVENTS_XFER   255

#endif // __SYNCTEST_H