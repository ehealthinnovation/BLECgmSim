/**************************************************************************************************
  Filename:       cgm.c

  Revised:        $Date: $
  Revision:       $Revision:  $

  Description:    This file contains the CGM sensor simulator application
                  for use with the CC2540 Bluetooth Low Energy Protocol Stack.

  Copyright 2011-2013 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED “AS IS” WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.
**************************************************************************************************/


/*********************************************************************
 * INCLUDES
 */

#include "bcomdef.h"
#include "OSAL.h"
#include "OSAL_PwrMgr.h"
#include "OnBoard.h"
#include "hal_adc.h"
#include "hal_led.h"
#include "hal_key.h"
#include "gatt.h"
#include "hci.h"
#include "gapgattserver.h"
#include "gattservapp.h"
#include "gatt_profile_uuid.h"
#include "linkdb.h"
#include "peripheral.h"
#include "gapbondmgr.h"
#include "cgmservice.h"
#include "devinfoservice.h"
#include "cgm.h"
#include "OSAL_Clock.h"
#include "CGM_Service_values.h"
#include "battservice.h"
#include "cgmsimdata.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

// Fast advertising interval in 625us units
#define DEFAULT_FAST_ADV_INTERVAL             32

// Duration of fast advertising duration in sec
#define DEFAULT_FAST_ADV_DURATION             30

// Slow advertising interval in 625us units
#define DEFAULT_SLOW_ADV_INTERVAL             1600

// Duration of slow advertising duration in sec
#define DEFAULT_SLOW_ADV_DURATION             30

// Whether to enable automatic parameter update request when a connection is formed
#define DEFAULT_ENABLE_UPDATE_REQUEST         FALSE

// Minimum connection interval (units of 1.25ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     200

// Maximum connection interval (units of 1.25ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     1600

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_DESIRED_SLAVE_LATENCY         1

// Supervision timeout value (units of 10ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_CONN_TIMEOUT          1000

// Whether to enable automatic parameter update request when a connection is formed
#define DEFAULT_ENABLE_UPDATE_REQUEST         FALSE

// Default passcode
#define DEFAULT_PASSCODE                      19655

// Default GAP pairing mode
#define DEFAULT_PAIRING_MODE                   GAPBOND_PAIRING_MODE_INITIATE //GAPBOND_PAIRING_MODE_WAIT_FOR_REQ //GAPBOND_PAIRING_MODE_INITIATE [use this]

// Default MITM mode (TRUE to require passcode or OOB when pairing)
#define DEFAULT_MITM_MODE                     TRUE

// Default bonding mode, TRUE to bond
#define DEFAULT_BONDING_MODE                  TRUE

// Default GAP bonding I/O capabilities
#define DEFAULT_IO_CAPABILITIES               GAPBOND_IO_CAP_DISPLAY_ONLY

// Notification period in ms
#define DEFAULT_NOTI_PERIOD                   1000



/*********************************************************************
 * TYPEDEFS
 */

// contains the data of control point command

//NEW
//the simplest measurement result
typedef struct {
  uint8         size;
  uint8         flags;
  uint16        concentration;
  uint16        timeoffset;
} cgmMeas_t;

//complete measurement result data structure
typedef struct {
  uint8         size;
  uint8         flags;
  uint16        concentration;
  uint16        timeoffset;
  uint24        annunication;
  uint16        trend;
  uint16        quality;
} cgmMeasC_t;

//NEW
typedef struct {
  uint24 cgmFeature;
  uint8  cgmTypeSample;
} cgmFeature_t;

//NEW
typedef struct {
  uint16 timeOffset;
  uint24 cgmStatus;
} cgmStatus_t;

//NEW
typedef struct {
  UTCTimeStruct startTime;
  int8         timeZone;
  uint8         dstOffset;
} cgmSessionStartTime_t;

//NEW
 typedef struct {
  osal_event_hdr_t hdr; //!< MSG_EVENT and status
  uint8 len;
  uint8 data[CGM_CTL_PNT_MAX_SIZE];
} cgmCtlPntMsg_t;

 typedef struct {
  osal_event_hdr_t hdr; //!< MSG_EVENT and status
  uint8 len;
  uint8 data[CGM_RACP_MAX_SIZE];
} cgmRACPMsg_t;

/*********************************************************************
 * GLOBAL VARIABLES
 */

// Task ID
uint8 cgmTaskId;

// Connection handle
uint16 gapConnHandle;

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

// GAP State
static gaprole_States_t gapProfileState = GAPROLE_INIT;

// GAP Profile - Name attribute for SCAN RSP data
static uint8 scanData[] =
{
  0x08,   // length of this data
  0x09,   // AD Type = Complete local name
  'C',
  'G',
  'M',
  ' ',
  'S',
  'i',
  'm'
};

static uint8 advertData[] =
{
  // flags
  0x02,
  GAP_ADTYPE_FLAGS,
  GAP_ADTYPE_FLAGS_LIMITED | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,
  // service UUIDs
  0x05,
  GAP_ADTYPE_16BIT_MORE,
  LO_UINT16(CGM_SERV_UUID),
  HI_UINT16(CGM_SERV_UUID),
  LO_UINT16(DEVINFO_SERV_UUID),
  HI_UINT16(DEVINFO_SERV_UUID)
};

// Device name attribute value
static uint8 attDeviceName[GAP_DEVICE_NAME_LEN] = "CGM Simulator";

// Bonded state
static bool cgmBonded = FALSE;

// Bonded peer address
static uint8 cgmBondedAddr[B_ADDR_LEN];

// GAP connection handle
static uint16 gapConnHandle;

// Indication structures for cgm
//NEW
static attHandleValueInd_t cgmCtlPntRsp;
static attHandleValueNoti_t  CGMMeas;
static attHandleValueInd_t   cgmRACPRsp;
static attHandleValueNoti_t   cgmRACPRspNoti;

// Advertising user-cancelled state
static bool cgmAdvCancelled = FALSE;

//the most current measurement
static cgmMeasC_t        cgmCurrentMeas;


//static cgmMeasC_t	cgmPreviousMeas;

//new all the variables needed for the cgm simulator
//the support feature
static uint16                   cgmCommInterval=1000;//the communication interval in ms
static cgmFeature_t             cgmFeature={ CGM_FEATURE_TREND_INFO, BUILD_UINT8(CGM_TYPE_ISF,CGM_SAMPLE_LOC_SUBCUT_TISSUE)};
static cgmStatus_t              cgmStatus={0x1234,0x567890}; //for testing purpose only
static cgmSessionStartTime_t    cgmStartTime={{0,0,0,0,0,2000},TIME_ZONE_UTC_M5,DST_STANDARD_TIME}; 


//Time related
//static UTCTimeStruct            cgmCurrentTime;
static UTCTime                 	 cgmCurrentTime_UTC;     //the UTC format of the current system time.
static UTCTime			 cgmStartTime_UTC;	 // the UTC format of the start time.
static bool			 cgmStartTimeConfigIndicator;
// The time offset since the session start time
static uint16			 cgmTimeOffset;


//RACP Related Variables
static cgmMeasC_t *	cgmMeasDB;
static uint8		cgmMeasDBWriteIndx;
static uint8		cgmMeasDBCount;
static uint8		cgmMeasDBOldestIndx;
static uint8		cgmMeasDBSearchStart; //the starting index record meeting the search criteria
static uint8		cgmMeasDBSearchEnd;   //the end index of record meeting the search criteria
static uint16		cgmMeasDBSearchNum;   //the resulting record number that matches the criteria
static uint8		cgmMeasDBSendIndx;   //the index of the next record to be sent
static bool		cgmRACPSendInProgress; //indicate if the sensor is sending out data

/*
static UTCTimeStruct            cgmCurrentTime;
static uint16                   cgmOffsetTime;//this can be derived from subtracting start time from current time
*/
static uint16                   cgmSessionRunTime=0x00A8;
static bool                     cgmSessionStartIndicator=false;
/*
static bool                     cgmSensorMalfunctionIndicator=false;
static uint8                    cgmBatteryLevel=95;//battery level in percentage
static uint16                   cgmCurrentMeas=0x0123;//the most recent cgm measurement
static uint16                   cgmCalibration=0x0222;//the most recent calibration
static uint16                   cgmHypoAlert=0x0333;//the alert level for hypo condition
static uint16                   cgmHyperAlert=0x0444;//the alert level for hypo condition
static uint16                   cgmHighAlert=0x0555;//the alert level for patient high condition
static uint16                   cgmLowAlert=0x0666;//the alert level for patient low condition
static uint16                   cgmChangeRate=0x0777;//the rate of change of cgm
static uint16                   cgmSensorTemperature=0x888;//the temperature of sensor
static uint16                   cgmQuality=0x0999;      //the quality of CGM data
*/

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void cgm_ProcessOSALMsg( osal_event_hdr_t *pMsg );
static void cgmProcessCtlPntMsg( cgmCtlPntMsg_t* pMsg);
static void cgmGapStateCB( gaprole_States_t newState );
static void cgm_HandleKeys( uint8 shift, uint8 keys );
static void cgmMeasSend(void);
static uint8 cgmVerifyTime(UTCTimeStruct* pTime);
static uint8 cgmVerifyTimeZone( int8 input);
static uint8 cgmVerifyDSTOffset( uint8 input);
static void cgmCtlPntResponse(uint8 opcode, uint8 *roperand,uint8 roperand_len);
static void cgmservice_cb(uint8 event, uint8* valueP, uint8 len, uint8 * result);
static void cgmPasscodeCB( uint8 *deviceAddr, uint16 connectionHandle,uint8 uiInputs, uint8 uiOutputs );
static void cgmNewGlucoseMeas(cgmMeasC_t * pMeas);                                      //this function loads the structure with the most recent glucose reading while upadting the internal record database
static void cgmSimulationAppInit();							//initialize the simulation app
static void cgmPairStateCB( uint16 connHandle, uint8 state, uint8 status );
//RACP Related Function
static void cgmSimAPPInit();							//initialize the simulation a
static uint8 cgmSearchMeasDB(uint8 filter,uint16 operand1, uint16 operand2); // seach for the required range
static void cgmAddRecord(cgmMeasC_t *cgmCurrentMeas);
static void cgmProcessRACPMsg( cgmRACPMsg_t * pMsg);
static void cgmRACPSendNextMeas();
static void cgmResetMeasDB();





/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static gapRolesCBs_t cgm_PeripheralCBs =
{
  cgmGapStateCB,  // Profile State Change Callbacks
  NULL                // When a valid RSSI is read from controller
};

// Bond Manager Callbacks
static const gapBondCBs_t cgmBondCB =
{
  cgmPasscodeCB,
  cgmPairStateCB
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      CGM_Init
 *
 * @brief   Initialization function for the CGM App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notificaiton ... ).
 *
 * @param   task_id - the ID assigned by OSAL.  This ID should be
 *                    used to send messages and set timers.
 *
 * @return  none
 */
void CGM_Init( uint8 task_id )
{
  cgmTaskId = task_id;

  // Setup the GAP Peripheral Role Profile
  {
    #if defined( CC2540_MINIDK )
      // For the CC2540DK-MINI keyfob, device doesn't start advertising until button is pressed
      uint8 initial_advertising_enable = TRUE;
    #else
      // For other hardware platforms, device starts advertising upon initialization
      uint8 initial_advertising_enable = TRUE;
    #endif
    // By setting this to zero, the device will go into the waiting state after
    // being discoverable for 30.72 second, and will not being advertising again
    // until the enabler is set back to TRUE
    uint16 gapRole_AdvertOffTime = 0;

    uint8 enable_update_request = DEFAULT_ENABLE_UPDATE_REQUEST;
    uint16 desired_min_interval = DEFAULT_DESIRED_MIN_CONN_INTERVAL;
    uint16 desired_max_interval = DEFAULT_DESIRED_MAX_CONN_INTERVAL;
    uint16 desired_slave_latency = DEFAULT_DESIRED_SLAVE_LATENCY;
    uint16 desired_conn_timeout = DEFAULT_DESIRED_CONN_TIMEOUT;

    // Set the GAP Role Parameters
    GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &initial_advertising_enable );
    GAPRole_SetParameter( GAPROLE_ADVERT_OFF_TIME, sizeof( uint16 ), &gapRole_AdvertOffTime );

    GAPRole_SetParameter( GAPROLE_SCAN_RSP_DATA, sizeof ( scanData ), scanData );
    GAPRole_SetParameter( GAPROLE_ADVERT_DATA, sizeof( advertData ), advertData );

    GAPRole_SetParameter( GAPROLE_PARAM_UPDATE_ENABLE, sizeof( uint8 ), &enable_update_request );
    GAPRole_SetParameter( GAPROLE_MIN_CONN_INTERVAL, sizeof( uint16 ), &desired_min_interval );
    GAPRole_SetParameter( GAPROLE_MAX_CONN_INTERVAL, sizeof( uint16 ), &desired_max_interval );
    GAPRole_SetParameter( GAPROLE_SLAVE_LATENCY, sizeof( uint16 ), &desired_slave_latency );
    GAPRole_SetParameter( GAPROLE_TIMEOUT_MULTIPLIER, sizeof( uint16 ), &desired_conn_timeout );
  }

  // Set the GAP Characteristics
  GGS_SetParameter( GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName );

  // Setup the GAP Bond Manager
  {
    uint32 passkey = DEFAULT_PASSCODE;
    uint8 pairMode = DEFAULT_PAIRING_MODE;
    uint8 mitm = DEFAULT_MITM_MODE;
    uint8 ioCap = DEFAULT_IO_CAPABILITIES;
    uint8 bonding = DEFAULT_BONDING_MODE;

    GAPBondMgr_SetParameter( GAPBOND_DEFAULT_PASSCODE, sizeof ( uint32 ), &passkey );
    GAPBondMgr_SetParameter( GAPBOND_PAIRING_MODE, sizeof ( uint8 ), &pairMode );
    GAPBondMgr_SetParameter( GAPBOND_MITM_PROTECTION, sizeof ( uint8 ), &mitm );
    GAPBondMgr_SetParameter( GAPBOND_IO_CAPABILITIES, sizeof ( uint8 ), &ioCap );
    GAPBondMgr_SetParameter( GAPBOND_BONDING_ENABLED, sizeof ( uint8 ), &bonding );
  }

  // Initialize GATT Client
  VOID GATT_InitClient();

  // Register to receive incoming ATT Indications/Notifications
  GATT_RegisterForInd( cgmTaskId );

  // Initialize GATT attributes
  GGS_AddService( GATT_ALL_SERVICES );         // GAP
  GATTServApp_AddService( GATT_ALL_SERVICES ); // GATT attributes
  CGM_AddService(GATT_ALL_SERVICES);
  DevInfo_AddService( );
  Batt_AddService();                              // Battery Service
    
  // Register for CGM service callback
  CGM_Register ( cgmservice_cb);

  // Register for all key events - This app will handle all key events
  RegisterForKeys( cgmTaskId );

 #if defined( CC2540_MINIDK )
  // makes sure LEDs are off
  HalLedSet( (HAL_LED_1 | HAL_LED_2), HAL_LED_MODE_OFF );

  // For keyfob board set GPIO pins into a power-optimized state
  // Note that there is still some leakage current from the buzzer,
  // accelerometer, LEDs, and buttons on the PCB.

  P0SEL = 0; // Configure Port 0 as GPIO
  P1SEL = 0; // Configure Port 1 as GPIO
  P2SEL = 0; // Configure Port 2 as GPIO

  P0DIR = 0xFC; // Port 0 pins P0.0 and P0.1 as input (buttons),
                // all others (P0.2-P0.7) as output
  P1DIR = 0xFF; // All port 1 pins (P1.0-P1.7) as output
  P2DIR = 0x1F; // All port 1 pins (P2.0-P2.4) as output

  P0 = 0x03; // All pins on port 0 to low except for P0.0 and P0.1 (buttons)
  P1 = 0;   // All pins on port 1 to low
  P2 = 0;   // All pins on port 2 to low

#endif // #if defined( CC2540_MINIDK )

  // Simulation Application Initialization
  cgmSimulationAppInit();

  // Setup a delayed profile startup
  osal_set_event( cgmTaskId, START_DEVICE_EVT );
  osal_start_timerEx( cgmTaskId, NOTI_TIMEOUT_EVT, cgmCommInterval);
  cgmSessionStartIndicator=true;
  
}

/*********************************************************************
 * @fn      CGM_ProcessEvent
 *
 * @brief   CGM Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  events not processed
 */
uint16 CGM_ProcessEvent( uint8 task_id, uint16 events )
{

  VOID task_id; // OSAL required parameter that isn't used in this function

  if ( events & SYS_EVENT_MSG )
  {
    uint8 *pMsg;

    if ( (pMsg = osal_msg_receive( cgmTaskId )) != NULL )
    {
      cgm_ProcessOSALMsg( (osal_event_hdr_t *)pMsg );

      // Release the OSAL message
      VOID osal_msg_deallocate( pMsg );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }

  if ( events & START_DEVICE_EVT )
  {
    // Start the Device
    VOID GAPRole_StartDevice( &cgm_PeripheralCBs );

    // Register with bond manager after starting device
    GAPBondMgr_Register( (gapBondCBs_t *) &cgmBondCB );
    // Start the notification
  
    return ( events ^ START_DEVICE_EVT );
  }
//NEW
  if ( events & NOTI_TIMEOUT_EVT )
  {
    // Send the current value of the CGM reading
    //Generate New Measurement
    cgmNewGlucoseMeas(&cgmCurrentMeas);
    //Add the generated record to database
    cgmAddRecord(&cgmCurrentMeas);
    cgmMeasSend();

    return ( events ^ NOTI_TIMEOUT_EVT );
  }

  if ( events & RACP_IND_SEND_EVT)
  {
	cgmRACPSendNextMeas();
	return (events ^ RACP_IND_SEND_EVT);
  }


  return 0;
}

/*********************************************************************
 * @fn      cgm_ProcessOSALMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void cgm_ProcessOSALMsg( osal_event_hdr_t *pMsg )
{
  switch ( pMsg->event )
  {
  case KEY_CHANGE:
      cgm_HandleKeys( ((keyChange_t *)pMsg)->state, ((keyChange_t *)pMsg)->keys );
      break;

  case CTL_PNT_MSG:
      cgmProcessCtlPntMsg( (cgmCtlPntMsg_t *) pMsg);
      break;

  case RACP_MSG:
      cgmProcessRACPMsg( (cgmRACPMsg_t *) pMsg);
      break;

  default:
      break;
  }
}

/*********************************************************************
 * @fn      cgm_HandleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   shift - true if in shift/alt.
 * @param   keys - bit field for key events. Valid entries:
 *                 HAL_KEY_SW_2
 *                 HAL_KEY_SW_1
 *
 * @return  none
 */
static void cgm_HandleKeys( uint8 shift, uint8 keys )
{
  if ( keys & HAL_KEY_SW_1 )
  {
    
  }

  if ( keys & HAL_KEY_SW_2 )
  {
    // if device is not in a connection, pressing the right key should toggle
    // advertising on and off
    if( gapProfileState != GAPROLE_CONNECTED )
    {
      uint8 status;

      // Set fast advertising interval for user-initiated connections
      GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MIN, DEFAULT_FAST_ADV_INTERVAL );
      GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MAX, DEFAULT_FAST_ADV_INTERVAL );
      GAP_SetParamValue( TGAP_LIM_ADV_TIMEOUT, DEFAULT_FAST_ADV_DURATION );

      // toggle GAP advertisement status
      GAPRole_GetParameter( GAPROLE_ADVERT_ENABLED, &status );
      status = !status;
      GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &status );

      // Set state variable
      if (status == FALSE)
      {
        cgmAdvCancelled = TRUE;
      }
    }
  }
}


/*********************************************************************
 * @fn      cgmProcessCtlPntMsg
 *
 * @brief   Process Control Point messages
 *
 * @return  none
 */
//NEW
static void cgmProcessCtlPntMsg (cgmCtlPntMsg_t * pMsg)
{
  uint8 opcode = pMsg->data[0];
  uint8 ropcode; //the op code in the return char value
  uint8 *operand; //the operand in the input
  uint8 roperand[CGM_CTL_PNT_MAX_SIZE];//the operand in reuturn char value
  uint8 roperand_len;//
  uint8 operand_len; // the operand length
 
  switch(opcode)
  { //currently only implement the set/get communication interval
    case CGM_SPEC_OP_GET_INTERVAL:
          ropcode=CGM_SPEC_OP_RESP_INTERVAL;
	  roperand[0]= (cgmCommInterval/1000)&0xFF;
	  roperand_len=1;
          break;
          
    case CGM_SPEC_OP_SET_INTERVAL:
          ropcode=CGM_SPEC_OP_RESP_CODE;
          operand=pMsg->data+1;
          operand_len=pMsg->len-1;
          if (operand_len!=1) //the input interval assumes 1 byte
	  {
		  roperand[0]=CGM_SPEC_OP_RESP_OPERAND_INVALID;
		  roperand_len=1;
	  }
          else
          {

            if ((*operand)==0) //input value being 0x00 would stop the timer.
	    {
		    osal_stop_timerEx(cgmTaskId,NOTI_TIMEOUT_EVT);

	    }
	    else if((*operand)==0xFF)
	    {
		    cgmCommInterval=1000*1; //fastest
		    if(cgmSessionStartIndicator==true)
		    osal_start_timerEx(cgmTaskId,NOTI_TIMEOUT_EVT,cgmCommInterval);
	    }
            else              
	    {
		    cgmCommInterval=(uint16)1000*(*operand); // in ms
		    if(cgmSessionStartIndicator==true)
		    osal_start_timerEx(cgmTaskId,NOTI_TIMEOUT_EVT,cgmCommInterval);
	    }
            roperand[0]=opcode;
	    roperand[1]=CGM_SPEC_OP_RESP_SUCCESS; 
	    roperand_len=2;
          }
          break;
    case CGM_SPEC_OP_START_SES:
	  //EXTRA if RACP is in transfer, this command is invalid
	  //Reset the sensor state
	  cgmResetMeasDB();
      	  cgmSessionStartIndicator=true;
	  cgmStatus.cgmStatus &= (~CGM_STATUS_ANNUNC_SES_STOP);
	  cgmTimeOffset=0;
	  if(cgmStartTimeConfigIndicator==false)
	  	cgmStartTime_UTC=0;
	  osal_setClock(0);
	  osal_start_timerEx(cgmTaskId,NOTI_TIMEOUT_EVT,cgmCommInterval);
          ropcode=CGM_SPEC_OP_RESP_CODE;
	  roperand[0]=opcode;
	  roperand[1]=CGM_SPEC_OP_RESP_SUCCESS;
	  roperand_len=2;
	  break;
    case CGM_SPEC_OP_STOP_SES:
	  osal_stop_timerEx(cgmTaskId,NOTI_TIMEOUT_EVT);
	  //Change the sensor status
	  cgmStatus.cgmStatus|= CGM_STATUS_ANNUNC_SES_STOP;
      	  cgmSessionStartIndicator=false;
          ropcode=CGM_SPEC_OP_RESP_CODE;
	  roperand[0]=opcode;
	  roperand[1]=CGM_SPEC_OP_RESP_SUCCESS;
	  roperand_len=2;
	  break;
    case  CGM_SPEC_OP_SET_CAL:			
    case  CGM_SPEC_OP_GET_CAL:			
    case  CGM_SPEC_OP_SET_ALERT_HIGH:		
    case  CGM_SPEC_OP_GET_ALERT_HIGH:		
    case  CGM_SPEC_OP_SET_ALERT_LOW:		
    case  CGM_SPEC_OP_GET_ALERT_LOW:		
    case  CGM_SPEC_OP_SET_ALERT_HYPO:		
    case  CGM_SPEC_OP_GET_ALERT_HYPO:		
    case  CGM_SPEC_OP_SET_ALERT_HYPER:		
    case  CGM_SPEC_OP_GET_ALERT_HYPER:		
    case  CGM_SPEC_OP_SET_ALERT_RATE_DECREASE:	
    case  CGM_SPEC_OP_GET_ALERT_RATE_DECREASE:	
    case  CGM_SPEC_OP_SET_ALERT_RATE_INCREASE:	
    case  CGM_SPEC_OP_GET_ALERT_RATE_INCREASE:	
	   roperand[0]=CGM_SPEC_OP_RESP_CODE;
 	   roperand[1]=CGM_SPEC_OP_RESP_OP_NOT_SUPPORT;
	   roperand_len=2;
	   break;
  default:
	  ropcode=CGM_SPEC_OP_RESP_CODE;
	  roperand[0]=opcode;
	  roperand[1]=CGM_SPEC_OP_RESP_OP_NOT_SUPPORT;
	  roperand_len=2;
          break;
  }
  cgmCtlPntResponse(ropcode,roperand,roperand_len);
          
}


/*********************************************************************
 * @fn      cgmGapStateCB
 *
 * @brief   Notification from the profile of a state change.
 *
 * @param   newState - new state
 *
 * @return  none
 */
static void cgmGapStateCB( gaprole_States_t newState )
{
  // if connected
  if ( newState == GAPROLE_CONNECTED )
  {

  }
  // if disconnected
  else if (gapProfileState == GAPROLE_CONNECTED &&
           newState != GAPROLE_CONNECTED)
  {
    uint8 advState = TRUE;

    // stop notification timer
    osal_stop_timerEx(cgmTaskId, NOTI_TIMEOUT_EVT);

    if ( newState == GAPROLE_WAITING_AFTER_TIMEOUT )
    {
      // link loss timeout-- use fast advertising
      GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MIN, DEFAULT_FAST_ADV_INTERVAL );
      GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MAX, DEFAULT_FAST_ADV_INTERVAL );
      GAP_SetParamValue( TGAP_LIM_ADV_TIMEOUT, DEFAULT_FAST_ADV_DURATION );
    }
    else
    {
      // Else use slow advertising
      GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MIN, DEFAULT_SLOW_ADV_INTERVAL );
      GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MAX, DEFAULT_SLOW_ADV_INTERVAL );
      GAP_SetParamValue( TGAP_LIM_ADV_TIMEOUT, DEFAULT_SLOW_ADV_DURATION );
    }

    // Enable advertising
    GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &advState );
  }
  // if advertising stopped
  else if ( gapProfileState == GAPROLE_ADVERTISING &&
            newState == GAPROLE_WAITING )
  {
    // if advertising stopped by user
    if ( cgmAdvCancelled )
    {
      cgmAdvCancelled = FALSE;
    }
    // if fast advertising switch to slow
    else if ( GAP_GetParamValue( TGAP_LIM_DISC_ADV_INT_MIN ) == DEFAULT_FAST_ADV_INTERVAL )
    {
      uint8 advState = TRUE;

      GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MIN, DEFAULT_SLOW_ADV_INTERVAL );
      GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MAX, DEFAULT_SLOW_ADV_INTERVAL );
      GAP_SetParamValue( TGAP_LIM_ADV_TIMEOUT, DEFAULT_SLOW_ADV_DURATION );
      GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &advState );
    }
  }
  // if started
  else if ( newState == GAPROLE_STARTED )
  {
    // Set the system ID from the bd addr
    uint8 systemId[DEVINFO_SYSTEM_ID_LEN];
    GAPRole_GetParameter(GAPROLE_BD_ADDR, systemId);

    // shift three bytes up
    systemId[7] = systemId[5];
    systemId[6] = systemId[4];
    systemId[5] = systemId[3];

    // set middle bytes to zero
    systemId[4] = 0;
    systemId[3] = 0;

    DevInfo_SetParameter(DEVINFO_SYSTEM_ID, DEVINFO_SYSTEM_ID_LEN, systemId);
  }
  gapProfileState = newState;
}

/*********************************************************************
 * @fn      cgmPairStateCB
 *
 * @brief   Pairing state callback.
 *
 * @return  none
 */
static void cgmPairStateCB( uint16 connHandle, uint8 state, uint8 status )
{
  if ( state == GAPBOND_PAIRING_STATE_COMPLETE )
  {
    if ( status == SUCCESS )
    {
      linkDBItem_t  *pItem;

      if ( (pItem = linkDB_Find( gapConnHandle )) != NULL )
      {
        // Store bonding state of pairing
        cgmBonded = ( (pItem->stateFlags & LINK_BOUND) == LINK_BOUND );

        if ( cgmBonded )
        {
          osal_memcpy( cgmBondedAddr, pItem->addr, B_ADDR_LEN );
        }
      }
    }
  }
}

/*********************************************************************
 * @fn      cgmPasscodeCB
 *
 * @brief   Passcode callback.
 *
 * @return  none
 */
static void cgmPasscodeCB( uint8 *deviceAddr, uint16 connectionHandle,
                                        uint8 uiInputs, uint8 uiOutputs )
{
  // Send passcode response
  GAPBondMgr_PasscodeRsp( connectionHandle, SUCCESS, DEFAULT_PASSCODE );
}

/*********************************************************************
 * @fn      CGMMeasSend
 *
 * @brief   Prepare and send a CGM measurement
 *
 * @return  none
 */
//NEW
static void cgmMeasSend(void)
{
  
  //att value notification structure
  uint8 *p=CGMMeas.value;
  uint8 flags=cgmCurrentMeas.flags;
  
  //load data into the package buffer
  *p++ = cgmCurrentMeas.size;
  *p++ = flags;
  *p++ = LO_UINT16(cgmCurrentMeas.concentration);
  *p++ = HI_UINT16(cgmCurrentMeas.concentration);
  *p++ = LO_UINT16(cgmCurrentMeas.timeoffset);
  *p++ = HI_UINT16(cgmCurrentMeas.timeoffset);

  if (flags & CGM_STATUS_ANNUNC_STATUS_OCT)
	  *p++ = (cgmCurrentMeas.annunication) & 0xFF;
  if (flags & CGM_STATUS_ANNUNC_WARNING_OCT)
    	  *p++ = (cgmCurrentMeas.annunication>>16) & 0xFF;
  if (flags & CGM_STATUS_ANNUNC_CAL_TEMP_OCT)
	  *p++ = (cgmCurrentMeas.annunication>>8)  & 0xFF;
  if (flags & CGM_TREND_INFO_PRES)
	{  *p++ = LO_UINT16(cgmCurrentMeas.trend);
	   *p++ = HI_UINT16(cgmCurrentMeas.trend);
	}
  if (flags & CGM_QUALITY_PRES)
  	{ 
		*p++ = LO_UINT16(cgmCurrentMeas.quality);
	        *p++ = HI_UINT16(cgmCurrentMeas.quality);
	}	
  CGMMeas.len=cgmCurrentMeas.size;
  //CGMMeas.len=(uint8)(p-CGMMeas.value);
  //command the GATT service to send the measurement
  CGM_MeasSend(gapConnHandle, &CGMMeas,  cgmTaskId);
  osal_start_timerEx(cgmTaskId, NOTI_TIMEOUT_EVT, cgmCommInterval);

}
  
 

//NEW
/*********************************************************************
 * @fn      cgmCtlPntResponse
 *
 * @brief   Send a record control point response
 *
 * @param   opcode - cgm specific control point opcode
 * @param   rspcode - response code
 *
 * @return  none
 */
static void cgmCtlPntResponse(uint8 opcode, uint8 * roperand, uint8 roperand_len)
{
  cgmCtlPntRsp.value[0]=opcode;
  cgmCtlPntRsp.len=1+roperand_len;
  osal_memcpy(cgmCtlPntRsp.value+1,roperand, roperand_len);
  Glucose_CtlPntIndicate(gapConnHandle, &cgmCtlPntRsp, cgmTaskId);
}


/*********************************************************************
 * @fn      cgmservice_cb
 *
 * @brief   Callback function from app to service.
 *
 * @param   event - service event
 *
 * @return  none
 */

//NEW
static void cgmservice_cb(uint8 event, uint8* valueP, uint8 len, uint8 * result)
{

  switch (event)
  {
  case CGM_MEAS_NTF_ENABLED:
    {
        //osal_start_timerEx(cgmTaskId, NOTI_TIMEOUT_EVT, cgmCommInterval);
        break;
    }
  case CGM_MEAS_NTF_DISABLED:
    {
      //  osal_stop_timerEx(cgmTaskId, NOTI_TIMEOUT_EVT);
        break;
    }
    
   
  case CGM_FEATURE_READ_REQUEST:
    {
        *valueP = cgmFeature.cgmFeature & 0xFF;
        *(++valueP) = (cgmFeature.cgmFeature >> 8) & 0xFF;
        *(++valueP) = (cgmFeature.cgmFeature >> 16) & 0xFF;
        *(++valueP) =  cgmFeature.cgmTypeSample;
	*(++valueP) =  0xFF;
	*(++valueP) =  0xFF;
        break;
    }
   
   case CGM_STATUS_READ_REQUEST:
     {
	cgmStatus.timeOffset=(uint16)osal_getClock();
        *valueP = LO_UINT16(cgmStatus.timeOffset);
        *(++valueP) = HI_UINT16(cgmStatus.timeOffset);
        *(++valueP) = BREAK_UINT32(cgmStatus.cgmStatus,0);
        *(++valueP) = BREAK_UINT32(cgmStatus.cgmStatus,1);
        *(++valueP) = BREAK_UINT32(cgmStatus.cgmStatus,2);
        break;
     }
    
  case CGM_START_TIME_READ_REQUEST:
    {
	osal_ConvertUTCTime( &(cgmStartTime.startTime), cgmStartTime_UTC);
        *valueP = (cgmStartTime.startTime.year & 0xFF);
        *(++valueP) = (cgmStartTime.startTime.year >> 8) & 0xFF;
        *(++valueP) = (cgmStartTime.startTime.month) & 0xFF;
        *(++valueP) = (cgmStartTime.startTime.day) & 0xFF;
        *(++valueP) = (cgmStartTime.startTime.hour) & 0xFF;
        *(++valueP) = (cgmStartTime.startTime.minutes) & 0xFF;
        *(++valueP) = (cgmStartTime.startTime.seconds) & 0xFF;
        *(++valueP) = (cgmStartTime.timeZone) & 0xFF;
        *(++valueP) = (cgmStartTime.dstOffset) & 0xFF;
        break;
    }
   
  case CGM_RUN_TIME_READ_REQUEST:
    {
        *valueP = cgmSessionRunTime & 0xFF;
        *(++valueP) = (cgmSessionRunTime >> 8) & 0xFF;
        break;
    }
    
  case CGM_START_TIME_WRITE_REQUEST:
    {	cgmSessionStartTime_t input;
     	UTCTime input_UTC;	
	
	input.startTime.year=BUILD_UINT16(valueP[0],valueP[1]);
     	input.startTime.month=valueP[2];
     	input.startTime.day=valueP[3];
     	input.startTime.hour=valueP[4];
     	input.startTime.minutes=valueP[5];
     	input.startTime.seconds=valueP[6];
     	input.timeZone=valueP[7];
     	input.dstOffset=valueP[8];
	//test the time value
	if (  cgmVerifyTimeZone(input.timeZone)==false || cgmVerifyDSTOffset(input.dstOffset)==false ) //`cgmVerifyTime(&input.startTime)==false)// ||
	{
		*result=0x04;
	}
	input_UTC=osal_ConvertUTCSecs(&input.startTime);
		
	if( cgmSessionStartIndicator == false)
	{
		cgmStartTime_UTC=input_UTC;
	}
	else
	{
		cgmCurrentTime_UTC=osal_getClock();
		cgmStartTime_UTC= input_UTC-(cgmCurrentTime_UTC); //EXTRA:account for the case when input_time is less than the cuurent-start
	}	
		osal_ConvertUTCTime( &(cgmStartTime.startTime), cgmStartTime_UTC);
		cgmStartTime.timeZone=input.timeZone;
		cgmStartTime.dstOffset=input.dstOffset;
		cgmStartTimeConfigIndicator=true;
	*result=SUCCESS;
     // cgmStartTime.startTime.year=BUILD_UINT16(valueP[0],valueP[1]);
     // cgmStartTime.startTime.month=valueP[2];
     // cgmStartTime.startTime.day=valueP[3];
     // cgmStartTime.startTime.hour=valueP[4];
     // cgmStartTime.startTime.minutes=valueP[5];
     // cgmStartTime.startTime.seconds=valueP[6];
     // cgmStartTime.timeZone=valueP[7];
     // cgmStartTime.dstOffset=valueP[8];
     // cgmCurrentTime_UTC=osal_ConvertUTCSecs(&cgmStartTime.startTime);//convert to second format
     // osal_setClock(cgmCurrentTime_UTC);//set the system clock to the session start time
      break;
    }

  case CGM_CTL_PNT_CMD:
    {
      cgmCtlPntMsg_t* msgPtr;
      // Send the address to the task list
      msgPtr = (cgmCtlPntMsg_t *)osal_msg_allocate( sizeof(cgmCtlPntMsg_t) );
      if ( msgPtr )
      {
        msgPtr->hdr.event = CTL_PNT_MSG;
        msgPtr->len = len;
        osal_memcpy(msgPtr->data, valueP, len);
        osal_msg_send( cgmTaskId, (uint8 *)msgPtr );
      }
    }
      break;
    
  case CGM_RACP_CTL_PNT_CMD:
      {
	cgmRACPMsg_t * msgPtr;
	// Send the address to the task list
	msgPtr = (cgmRACPMsg_t *)osal_msg_allocate( sizeof(cgmRACPMsg_t) );
	if ( msgPtr )
	{
	  msgPtr->hdr.event = RACP_MSG;
	  msgPtr->len = len;
	  osal_memcpy(msgPtr->data, valueP, len);
	  osal_msg_send( cgmTaskId, (uint8 *)msgPtr );
	}
      }
      break;
    
  default:
    break;
  }
}

/*********************************************************************
* @fn cgmVerifyTime
*
* @brief Verify time values are suitable for filtering
*
* @param pTime - UTC time struct
*
* @return true if time is ok, false otherwise
*/
static uint8 cgmVerifyTime(UTCTimeStruct* pTime)
{
  // sanity check year
  if (pTime->year < 2000 || pTime->year > 2111)
  {
    return false;
  }
  // check day range
  if (pTime->day == 0 || pTime->day > 31)
  {
    return false;
  }
  // check month range
  if (pTime->month == 0 || pTime->month > 12)
  {
    return false;
  }

  // adjust month and day; utc time uses 0-11 and 0-30, characteristic uses 1-12 and 1-31
  pTime->day--;
  pTime->month--;

  return true;
}

static uint8 cgmVerifyTimeZone( int8 input)
{
	if ( (input % 2) !=0)
		return false;
	if ( ((input > -48) && (input<56)) || (input==128))
	      	return true;
	return false;
}

static uint8 cgmVerifyDSTOffset( uint8 input)
{
	if ( input==0x02 || input==0x04 || input==0x00 || input==0x08 || input==0xFF)
		return true;
		return false;
}
/*********************************************************************
 * @fn      cgmNewGlucoseMeas
 *
 * @brief   Update with the lastest reading while updating the internal database
 *
 * @param   pMeas - pointer to the input data structure
 * @param   pPMeas - pointer to the structure to store previous measurement
 *
 * @return  none
 */
static void cgmNewGlucoseMeas(cgmMeasC_t * pMeas)
{
  //generate the glucose reading.
  static uint16 glucoseGen=0x0000;
  static uint16 glucosePreviousGen=0;
  static uint8  flag=0;
  uint8         size=6;
  uint16        trend;
  int32		currentGlucose_cal=0;
  int32		previousGlucose_cal=0;
  uint16	offset_dif; //for calculating trend
  int32		trend_cal; //the signed version for calculation
  
  uint16        quality;
  UTCTime currentTime, startTime;
 

  // Store the current CGM measurement
  glucosePreviousGen=glucoseGen;

  //get the glucose information
  glucoseGen =cgmGetNextData();
  glucoseGen =( glucoseGen % 0x07FD);
  pMeas->concentration=glucoseGen;
  
  //get the time offset


  //startTime=osal_ConvertUTCSecs(&cgmStartTime.startTime);
  //if (currentTime>startTime)
  //{     
  //    offset=currentTime-startTime;
  //    if (offset > 0x0000FFFF) //UTCTime counts in second, target is minute
  //      offset=0;
  //    else
  pMeas->timeoffset=cgmTimeOffset & 0xFFFF; //EXTRA needs here second minute conflict
  cgmTimeOffset += cgmCommInterval/1000;
  //}
  
  //get the flag information
   flag |= CGM_TREND_INFO_PRES;
   pMeas->flags= (flag);  
 
  
  //trend information
  if(cgmTimeOffset!=0)
  {
    { /*
	0x07FD 2045
	0x0803 -2045
	maximal difference
	2045-(-2045)=4090 or -4090
	minimal difference
	1 or -1
	normal glucose level 80-110mg/dL
	diabetics level can rise up to 140 mg/dL, even 200mg/dL
	http://www.diabetes.co.uk/diabetes_care/blood-sugar-level-ranges.html
     
	*/
	    currentGlucose_cal=(glucoseGen & 0x07FF);
	    previousGlucose_cal=(glucosePreviousGen & 0x07FF);
	    offset_dif=cgmCommInterval/1000;	//EXTRA: currnt communication interval counts in ms.
	    trend_cal=(currentGlucose_cal-previousGlucose_cal)*10/offset_dif; //elevate the power by 10 to gain 1 digit accuracy, the highest resolution is 0.1mg/dl/min
	    
	    //convert the calculation result back to SFLOAT
	    if (trend_cal>2045) //when exponent is -1, the mantissa can be at most 20e5
		    trend=0x07FE;//+infinity
	    else if (trend_cal<-2045)
		    trend=0x0F02;//-infinity
	    else
		    trend= (trend_cal & 0x0FFF) | 0xF000;
    }
  } 
  
  //quality information
  quality=0xF200; //51.2%
  
  //get the status annunication
  pMeas->annunication=0x00CCDDEE;
  
  //update the annuciation field
  if (flag & CGM_TREND_INFO_PRES)
  {
    size+=2;
    pMeas->trend=trend;
  }
  
  if (flag & CGM_QUALITY_PRES)
  {
    size+=2;
    pMeas->quality=quality;
  }
  //EXTRA: cause consequence reversed, should test the annunication field
  //to get the flag field set/clear
  if (flag & CGM_STATUS_ANNUNC_WARNING_OCT)
    size++;
  
  if (flag & CGM_STATUS_ANNUNC_CAL_TEMP_OCT)
    size++;
  
  if (flag & CGM_STATUS_ANNUNC_STATUS_OCT)
    size++;

  pMeas->size=size;  

  //here update the data base record 
}
  

/*********************************************************************
 * @fn      cgmSimulationAppInit
 *
 * @brief   Update with the lastest reading while updating the internal database
 *
 * @param   pMeas - pointer to the input data structure
 *
 * @return  none
 */

static void cgmSimulationAppInit()				 //initialize the simulation a
{
	cgmMeasDB=(cgmMeasC_t *)osal_mem_alloc(CGM_MEAS_DB_SIZE*sizeof(cgmMeasC_t));
	cgmMeasDBCount=0;
	cgmMeasDBWriteIndx=0;
	cgmMeasDBOldestIndx=0;
	cgmStartTimeConfigIndicator=false;
	cgmSimDataReset();
//	cgmMeasC_t temp;
//	for (uint8 i=0;i<10;i++)
//	{
//		cgmNewGlucoseMeas(&temp);
//		temp->timeoffset++;
//		osal_memcpy(cgmMeasDB+i,&temp,sizeof(cgmMeasC_t));
//	}		
//	temp.size=6;/\
//	temp.flags=0x33;
//	temp.concentration=0x0123;
//	temp.timeoffset=0;
//	for (temp.timeoffset=0;temp.timeoffset<10;temp.timeoffset++)
//	{
//		osal_memcpy(cgmMeasDB+temp.timeoffset,&temp,sizeof(cgmMeasC_t));
//	}
//	cgmMeasDBCount=10;
}	

/*********************************************************************
 * @fn      cgmSearchMeasDB
 *
 * @brief  This function implements the search function for the gluocose measurement. 
 * 	   It assumes the measurement database consist of continous records arranged in ascending order.
 * 	   The current version is based on the continuity of data records.
 *
 * @param   filter - the filter type in searching
 * @param   operand1 - the primary operand to the search operation.
 * @param   operand2 - the scrondary operand to the search operation, it is currently used only in searching for a range of record.
 *
 * @return  the result code
 */
static uint8 cgmSearchMeasDB(uint8 filter,uint16 operand1, uint16 operand2)
{
	uint8 i=0;
        uint8 j=0;
	//EXTRA:test record availability
	if(cgmMeasDBCount==0)
		return  RACP_SEARCH_RSP_NO_RECORD;
        
	switch (filter)
        {
                case CTL_PNT_OPER_ALL:
                        {
                                cgmMeasDBSearchStart=cgmMeasDBOldestIndx;
                                cgmMeasDBSearchEnd=(cgmMeasDBOldestIndx+cgmMeasDBCount-1)%CGM_MEAS_DB_SIZE;
                                cgmMeasDBSearchNum=cgmMeasDBCount;
				return RACP_SEARCH_RSP_SUCCESS;

                        }
                case CTL_PNT_OPER_GREATER_EQUAL:
                        {
                                do
                                {
				
					j=(cgmMeasDBOldestIndx+i)%CGM_MEAS_DB_SIZE;
                                        if( (cgmMeasDB+j)->timeoffset >= operand1)
					{
						cgmMeasDBSearchStart=j;
						break;
					}
					i++;
					if (i>=cgmMeasDBCount)
					{
						cgmMeasDBSearchNum=0;
						return RACP_SEARCH_RSP_NO_RECORD;
					}
				}while(1);

				cgmMeasDBSearchEnd=(cgmMeasDBOldestIndx+cgmMeasDBCount-1)%CGM_MEAS_DB_SIZE;
				if(cgmMeasDBSearchEnd>=cgmMeasDBSearchStart)
				{
					cgmMeasDBSearchNum= (cgmMeasDBSearchEnd-cgmMeasDBSearchStart+1);
				}
				else
				{
					cgmMeasDBSearchNum= -(cgmMeasDBSearchStart-cgmMeasDBSearchEnd-1)+CGM_MEAS_DB_SIZE;
				}
				return RACP_SEARCH_RSP_SUCCESS;
			}
		case CTL_PNT_OPER_FIRST:
			{
				cgmMeasDBSearchStart=cgmMeasDBOldestIndx;
				cgmMeasDBSearchEnd=cgmMeasDBOldestIndx;
				cgmMeasDBSearchNum=1;
				return RACP_SEARCH_RSP_SUCCESS;
			}
		case CTL_PNT_OPER_LAST:
			{
				cgmMeasDBSearchStart=(cgmMeasDBOldestIndx+cgmMeasDBCount-1)%CGM_MEAS_DB_SIZE;
				cgmMeasDBSearchEnd=cgmMeasDBSearchStart;
				cgmMeasDBSearchNum=1;
				return RACP_SEARCH_RSP_SUCCESS;
			}
		case CTL_PNT_OPER_LESS_EQUAL:
			{
				do
				{
				j=(cgmMeasDBOldestIndx+i)%CGM_MEAS_DB_SIZE;
				if ( (cgmMeasDB+j)->timeoffset > operand1)
				{
					if(i==0)//no record has a smaller offset value
					{
						cgmMeasDBSearchNum=0;
						return RACP_SEARCH_RSP_NO_RECORD;
					}

					cgmMeasDBSearchEnd=(j+CGM_MEAS_DB_SIZE-1)%CGM_MEAS_DB_SIZE;
					break;
				}
				i++;
				
				if ( i >= cgmMeasDBCount)
				{
					cgmMeasDBSearchEnd=(cgmMeasDBOldestIndx+cgmMeasDBCount-1)%CGM_MEAS_DB_SIZE;
					break;
				}

				}while(1);
				
				cgmMeasDBSearchStart=cgmMeasDBOldestIndx;

				if(cgmMeasDBSearchEnd>=cgmMeasDBSearchStart)
				{
					cgmMeasDBSearchNum= (cgmMeasDBSearchEnd-cgmMeasDBSearchStart+1);
				}
				else
				{
					cgmMeasDBSearchNum= -(cgmMeasDBSearchStart-cgmMeasDBSearchEnd-1)+CGM_MEAS_DB_SIZE;
				}
				return RACP_SEARCH_RSP_SUCCESS;
			}
		case CTL_PNT_OPER_RANGE:
			{
				if (operand1>operand2)
					return RACP_SEARCH_RSP_INVALID_OPERAND;
				uint8 startindx,endindx;
				if (cgmSearchMeasDB(CTL_PNT_OPER_GREATER_EQUAL,operand1,0)!=RACP_SEARCH_RSP_NO_RECORD)
					{
						startindx=cgmMeasDBSearchStart;
					}
				else
				{
					return RACP_SEARCH_RSP_NO_RECORD;
				}

				if(cgmSearchMeasDB(CTL_PNT_OPER_LESS_EQUAL,operand2,0)!= RACP_SEARCH_RSP_NO_RECORD)
				{
					endindx=cgmMeasDBSearchEnd;
				}
				else
				{
					return RACP_SEARCH_RSP_NO_RECORD;
				}
				cgmMeasDBSearchStart=startindx;
				cgmMeasDBSearchEnd=endindx;
				
				if(cgmMeasDBSearchEnd>=cgmMeasDBSearchStart)
				{
					cgmMeasDBSearchNum= (cgmMeasDBSearchEnd-cgmMeasDBSearchStart+1);
				}
				else
				{
					cgmMeasDBSearchNum= -(cgmMeasDBSearchStart-cgmMeasDBSearchEnd-1)+CGM_MEAS_DB_SIZE;
				}
				return RACP_SEARCH_RSP_SUCCESS;
			}

		default:
			break;
	}

	return RACP_SEARCH_RSP_NOT_COMPLETE;	
}

/*********************************************************************
 * @fn     cgmAddRecord 
 *
 * @brief  Add record to the database 
 *
 * @param   cgmCurrentMeas - the current measurement to be added to the database.
 *
 *
 * @return  the result code */
static void cgmAddRecord(cgmMeasC_t *cgmCurrentMeas)
{
	cgmMeasDBWriteIndx=(cgmMeasDBOldestIndx+cgmMeasDBCount)%CGM_MEAS_DB_SIZE;
	if ((cgmMeasDBWriteIndx==cgmMeasDBOldestIndx) && cgmMeasDBCount>0)
		cgmMeasDBOldestIndx=(cgmMeasDBOldestIndx+1)%CGM_MEAS_DB_SIZE;
	osal_memcpy(cgmMeasDB+cgmMeasDBWriteIndx,cgmCurrentMeas,sizeof(cgmMeasC_t));
	cgmMeasDBCount++;
	if (cgmMeasDBCount>CGM_MEAS_DB_SIZE)
		cgmMeasDBCount=CGM_MEAS_DB_SIZE;
}	

static void cgmProcessRACPMsg (cgmRACPMsg_t * pMsg)
{
	uint8 opcode=pMsg->data[0];
	uint8 operator=pMsg->data[1];
	uint8 filter;
	uint16 operand1=0,operand2=0;
	uint8 reopcode=0;

	switch (opcode)
	{
		case CTL_PNT_OP_REQ:
		case CTL_PNT_OP_GET_NUM:
	//		if ( filter=pMsg->data[2] != 0x01)
	//		{
	//			cgmRACPRsp.value[3]=CTL_PNT_RSP_OPER_NOT_SUPPORTED;
	//			cgmRACPRsp.value[0]=CTL_PNT_OP_REQ_RSP;
	//	 		cgmRACPRsp.value[1]=CTL_PNT_OPER_NULL;
	//			cgmRACPRsp.value[2]=opcode;
	//			cgmRACPRsp.len=4;
  	//			CGM_RACPIndicate(gapConnHandle, &cgmRACPRsp, cgmTaskId);
	//l			return;
	//		}

			if (operator==CTL_PNT_OPER_LESS_EQUAL 
					|| operator==CTL_PNT_OPER_GREATER_EQUAL 
					|| operator==CTL_PNT_OPER_RANGE)
				operand1=BUILD_UINT16(pMsg->data[3],pMsg->data[4]);
			if (operator==CTL_PNT_OPER_RANGE)
				operand2=BUILD_UINT16(pMsg->data[5],pMsg->data[6]);
			if ((reopcode=cgmSearchMeasDB(operator,operand1,operand2))==RACP_SEARCH_RSP_SUCCESS)
			{
				if (opcode==CTL_PNT_OP_REQ){
				cgmMeasDBSendIndx=0;
				osal_start_timerEx(cgmTaskId,RACP_IND_SEND_EVT,500); //start the data transfer event  				   
				CGM_SetSendState(true);
				return;}
				else if (opcode==CTL_PNT_OP_GET_NUM)
				{
					
					cgmRACPRsp.value[0]=CTL_PNT_OP_NUM_RSP;
				 	cgmRACPRsp.value[1]=CTL_PNT_OPER_NULL;
					cgmRACPRsp.value[2]=LO_UINT16(cgmMeasDBSearchNum); //EXTRA: epand to uint16 
      					cgmRACPRsp.value[3]=HI_UINT16(cgmMeasDBSearchNum);
					cgmRACPRsp.len=4;
  					CGM_RACPIndicate(gapConnHandle, &cgmRACPRsp, cgmTaskId);
				}
			}
			else
			{
				if (reopcode==RACP_SEARCH_RSP_NO_RECORD && opcode==CTL_PNT_OP_GET_NUM)
				{
					cgmRACPRsp.value[3]=0x00;
					cgmRACPRsp.value[0]=CTL_PNT_OP_NUM_RSP;
			 		cgmRACPRsp.value[1]=CTL_PNT_OPER_NULL;
					cgmRACPRsp.value[2]=0x00;
					cgmRACPRsp.len=4;
				}
				else{
					cgmRACPRsp.value[3]=reopcode;
					cgmRACPRsp.value[0]=CTL_PNT_OP_REQ_RSP;
			 		cgmRACPRsp.value[1]=CTL_PNT_OPER_NULL;
					cgmRACPRsp.value[2]=opcode;
					cgmRACPRsp.len=4;
				}
  				CGM_RACPIndicate(gapConnHandle, &cgmRACPRsp,  cgmTaskId);
			}
			break;

		case CTL_PNT_OP_ABORT:
			{
				osal_stop_timerEx(cgmTaskId,RACP_IND_SEND_EVT);	
				CGM_SetSendState(false);
		      		cgmRACPRsp.value[0]=CTL_PNT_OP_REQ_RSP;
				cgmRACPRsp.value[1]=CTL_PNT_OPER_NULL;
  				cgmRACPRsp.value[2]=opcode;
  				cgmRACPRsp.value[3]=CTL_PNT_RSP_SUCCESS;	
				cgmRACPRsp.len=4;
  				CGM_RACPIndicate(gapConnHandle, &cgmRACPRsp,  cgmTaskId);
			}
		
			
		default:
			break;
	}

	//send the response to the RACP by GATT indication
}	
			

static void cgmRACPSendNextMeas(){

	if (cgmMeasDBSendIndx < cgmMeasDBSearchNum)
	{
		cgmMeasC_t *currentRecord=cgmMeasDB+((cgmMeasDBSearchStart+cgmMeasDBSendIndx)%CGM_MEAS_DB_SIZE);
		//att value notification structure
	  	uint8 *p=cgmRACPRspNoti.value;
	  	uint8 flags=currentRecord->flags;
  
 		 //load data into the package buffer
 		 *p++ = currentRecord->size;
	 	 *p++ = flags;
		 *p++ = LO_UINT16(currentRecord->concentration);
	 	 *p++ = HI_UINT16(currentRecord->concentration);
		 *p++ = LO_UINT16(currentRecord->timeoffset);
		 *p++ = HI_UINT16(currentRecord->timeoffset);

		  if (flags & CGM_STATUS_ANNUNC_STATUS_OCT)
			  *p++ = (currentRecord->annunication) & 0xFF;
		  if (flags & CGM_STATUS_ANNUNC_WARNING_OCT)
    			  *p++ = (currentRecord->annunication>>16) & 0xFF;
		  if (flags & CGM_STATUS_ANNUNC_CAL_TEMP_OCT)
			  *p++ = (currentRecord->annunication>>8)  & 0xFF;
		  if (flags & CGM_TREND_INFO_PRES)
			{  
				*p++ = LO_UINT16(currentRecord->trend);
	   			*p++ = HI_UINT16(currentRecord->trend);
			}
	  	if (flags & CGM_QUALITY_PRES)
  		{ 
			*p++ = LO_UINT16(currentRecord->quality);
	        	*p++ = HI_UINT16(currentRecord->quality);
		}	
	  	cgmRACPRspNoti.len=currentRecord->size;
		cgmMeasDBSendIndx++;
  		//CGMMeas.len=(uint8)(p-CGMMeas.value);
  		//command the GATT service to send the measurement
  		CGM_MeasSend(gapConnHandle, &cgmRACPRspNoti, cgmTaskId);
	  	osal_start_timerEx(cgmTaskId, RACP_IND_SEND_EVT, 1000); //EXTRA: set the timer to be smaller to improve speed
	}
	else
	{
		//The current RACP transfer is finished
		osal_stop_timerEx(cgmTaskId, RACP_IND_SEND_EVT);
		cgmRACPRsp.len=4;
		cgmRACPRsp.value[0]=CTL_PNT_OP_REQ_RSP;
		cgmRACPRsp.value[1]=CTL_PNT_OPER_NULL;
		cgmRACPRsp.value[2]=CTL_PNT_OP_REQ;
		cgmRACPRsp.value[3]=CTL_PNT_RSP_SUCCESS;
		CGM_RACPIndicate(gapConnHandle, &cgmRACPRsp, cgmTaskId);
		CGM_SetSendState(false);
	}

}


static void cgmResetMeasDB()
{
	cgmMeasDBOldestIndx=0;
	cgmMeasDBCount=0;
}
	
/*********************************************************************
*********************************************************************/
