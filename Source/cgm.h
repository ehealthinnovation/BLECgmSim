/**************************************************************************************************
  Filename:       cgm.h
  Revised:        2015-Feb-03
  Revision:       1

  Description:    This file contains the CGM simulator application
                  definitions and prototypes.

The MIT License (MIT)

Copyright (c) 2014-2015 Center for Global ehealthinnovation

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
**************************************************************************************************/

#ifndef CGM_H
#define CGM_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */

/*********************************************************************
 * CONSTANTS
 */
// CGM application level constant
#define CGM_MEAS_DB_SIZE                              10	///< The size for the RACP history database
// CGM Task Events
#define START_DEVICE_EVT                              0x0001	///< The task to be carried out by the application layer: Start device event
#define NOTI_TIMEOUT_EVT                              0x0002	///< The task to be carried out by the application layer: timeout event for the next glucose notification
#define RACP_IND_SEND_EVT			      0x0004	///< The task to be carried out by the application layer: send RACP indication
// Message event  
#define CTL_PNT_MSG                                   0xE0	///< The event message past by the OS: OPCP message
#define RACP_MSG				      0xE1	///< The event message past by the OS: RACP message 
//RACP Search Function Response Code
#define RACP_SEARCH_RSP_SUCCESS			0x01		///< The search is successful
#define RACP_SEARCH_RSP_NO_RECORD		0x06		///< There is no record matching the criteria
#define	RACP_SEARCH_RSP_INVALID_OPERAND		0x05		///< The operand is not valid
#define RACP_SEARCH_RSP_NOT_COMPLETE		0x08		///< The procedure is not completed for a reason
  
/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * FUNCTIONS
 */
  
/*********************************************************************
 * GLOBAL VARIABLES
 */

/* @brief The routine to iniialize the CGM simulator
 * @param task_id the task id associated with the CGM application
 */
extern void CGM_Init( uint8 task_id );

/* @brief The routine to enable CGM application to process the system event
 * @param task_id the task id associated with the CGM application
 * @param events the event received from the os
 */
extern uint16 CGM_ProcessEvent( uint8 task_id, uint16 events );

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* CGM_H */
