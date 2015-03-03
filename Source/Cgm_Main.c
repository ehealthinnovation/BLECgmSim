/*!
\file		Cgm_Main.c
\brief		 This file contains the main and callback functions for
                  the CGM simulator application.
\author		Harry Qiu
\version        53
\date		2015-Feb-03
\copyright	MIT License (MIT)\n
 Copyright (c) 2014-2015 Center for Global ehealthinnovation

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions: The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

/**************************************************************************************************
 *                                           Includes
 **************************************************************************************************/
/* Hal Drivers */
#include "hal_types.h"
#include "hal_key.h"
#include "hal_timer.h"
#include "hal_drivers.h"
#include "hal_led.h"

/* OSAL */
#include "OSAL.h"
#include "OSAL_Tasks.h"
#include "OSAL_PwrMgr.h"
#include "osal_snv.h"
#include "OnBoard.h"

/**
 @brief       Start of application.
 @return      none
 */
int main(void)
{
  /* Initialize hardware */
  HAL_BOARD_INIT();
  // Initialize board I/O
  InitBoard( OB_COLD );
  /* Initialze the HAL driver */
  HalDriverInit();
  /* Initialize NV system */
  osal_snv_init();
  /* Initialize LL */
  /* Initialize the operating system */
  osal_init_system();
  /* Enable interrupts */
  HAL_ENABLE_INTERRUPTS();
  // Final board initialization
  InitBoard( OB_READY );
  #if defined ( POWER_SAVING )
  osal_pwrmgr_device( PWRMGR_BATTERY );
  #endif
  /* Start OSAL */
  osal_start_system(); // No Return from here
  return 0;
}
