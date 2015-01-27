/**************************************************************************************************
Filename:       cgm.c

Revised:        Date: 2015-01-24
Revision:       Revision:  1

Description:    This file contains the CGM sensor simulator application
for use with the CC2540 Bluetooth Low Energy Protocol Stack.

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
#define DEFAULT_FAST_ADV_INTERVAL             32 	/// Fast advertising interval in 625us units
#define DEFAULT_FAST_ADV_DURATION             30	/// Duration of fast advertising duration in sec
#define DEFAULT_SLOW_ADV_INTERVAL             1600	/// Slow advertising interval in 625us units
#define DEFAULT_SLOW_ADV_DURATION             30	/// Duration of slow advertising duration in sec
#define DEFAULT_ENABLE_UPDATE_REQUEST         FALSE	/// Whether to enable automatic parameter update request when a connection is formed
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     200	/// Minimum connection interval (units of 1.25ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     1600	/// Maximum connection interval (units of 1.25ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_SLAVE_LATENCY         1		/// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_DESIRED_CONN_TIMEOUT          1000	/// Supervision timeout value (units of 10ms) if automatic parameter update request is enabled
#define DEFAULT_ENABLE_UPDATE_REQUEST         FALSE	/// Whether to enable automatic parameter update request when a connection is formed
#define DEFAULT_PASSCODE                      19655	/// Default passcode
#define DEFAULT_PAIRING_MODE                  GAPBOND_PAIRING_MODE_INITIATE
#define DEFAULT_MITM_MODE                     TRUE	/// Default MITM mode (TRUE to require passcode or OOB when pairing)
#define DEFAULT_BONDING_MODE                  TRUE	/// Default bonding mode, TRUE to bond
#define DEFAULT_IO_CAPABILITIES               GAPBOND_IO_CAP_DISPLAY_ONLY	/// Default GAP bonding I/O capabilities
#define DEFAULT_NOTI_PERIOD                   1000	/// Notification period in ms


/*********************************************************************
 * TYPEDEFS
 */
///Container for CGM measurement result data
typedef struct {
	uint8         size;				///<The number of bytes inside the CGM measrement entries. This is not the size of the structure itself.
	uint8         flags;				///<Indicates the presene of optional data fields.  						
	uint16        concentration;			///<The concentration of glucose eastimate, in the SFLOAT data type.
	uint16        timeoffset;			///<The timeoffset from the session start time from the record in the unit of minute.
	uint24        annunication;			///<Annunciation of relevant status of the sensor or the record.
	uint16        trend;				///<The rate of increase or decrease, in the SFLOAT data type. It has the unit of mg/dL/min
	uint16        quality;				///<The quality of the CGM measurement,
} cgmMeasC_t;
///Container for the CGM support feature characteristic
typedef struct {
	uint24 cgmFeature;				///<The CGM supported features
	uint8  cgmTypeSample;				///<The measurement sample type and location
} cgmFeature_t;
///Container for the CGM sensor status characteristic
typedef struct {
	uint16 timeOffset;				///<The time offset from the session start time.
	uint24 cgmStatus;				///<The CGM sensor status.
} cgmStatus_t;
///Container for the CGM session start time characteristic
typedef struct {
	UTCTimeStruct startTime;			///<The date-time of the start time
	int8         timeZone;			///<The time zone associated with the current start time
	uint8         dstOffset;			///<The daylight saving time status associated with the current start time
} cgmSessionStartTime_t;
///The container for receiving CGMCP data message from the CGM service layer
typedef struct {
	osal_event_hdr_t hdr; 			///< MSG_EVENT and status from the CGM service layer
	uint8 len;					///< The length of the data being passed
	uint8 data[CGM_CTL_PNT_MAX_SIZE];		///< The value of the data being passed
} cgmCtlPntMsg_t;
///The container for receiving RACP data message from the CGM service layer
typedef struct {
	osal_event_hdr_t hdr; 			///< MSG_EVENT and status
	uint8 len;					///< The length of the data being passed
	uint8 data[CGM_RACP_MAX_SIZE];		///< The value of the data being passed
} cgmRACPMsg_t;				

/*********************************************************************
 * GLOBAL VARIABLES
 */
uint8 cgmTaskId;				///< The task ID associated with the CGM simulator application. It is used to schedule task in the OS layer.
uint16 gapConnHandle;				///< The handle for the current GAP connection.

/*********************************************************************
 * EXTERNAL VARIABLES
 */


/*********************************************************************
 * EXTERNAL FUNCTIONS
 */


/*********************************************************************
 * LOCAL VARIABLES
 */
static gaprole_States_t gapProfileState = GAPROLE_INIT;			///< The connection state of the GAP layer
/// GAP Profile - Name attribute for SCAN RSP data
static uint8 scanData[] =
{
	0x08,   ///< length of this data
	0x09,   ///< AD Type = Complete local name
	'C',  'G',  'M',  ' ',  'S',  'i',  'm'
};
/// Define the Advertizing data
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
static uint8 attDeviceName[GAP_DEVICE_NAME_LEN] = "CGM Simulator";
static bool cgmBonded = FALSE;						///< Local variable storing the current bonding stage of the sensor.
static uint8 cgmBondedAddr[B_ADDR_LEN];					///< Local variable storing the address of the bonded peer.
static uint16 gapConnHandle;						///< Local version of the GAP connection handle.
// Indication structures for cgm
static attHandleValueInd_t	cgmCtlPntRsp;				///< Container for holding CGMCP response indication message, which would be passed down to the GATT service layer
static attHandleValueNoti_t	CGMMeas;				///< Container for holding CGM measurement notification message, which would be passed down to the GATT service layer
static attHandleValueInd_t 	cgmRACPRsp;				///< Container for holding the RACP response indication message, which would be passed down to the GATT service layer
static attHandleValueNoti_t   cgmRACPRspNoti;				///< Container for holding the RACP notification message, which would be passed down to the GATT service layer. The notification will appear as the CGM measurement notification.
static bool cgmAdvCancelled = FALSE;					///< Denote the advertising state.
//CGM Simulator configureation variables

// ==================================START OF EXCERCISE REGION===============================================
//EXERCISE STEP 1: Here we need to define a variable to store hyperglycemia glucose threshold. 
//Also, we declare the hyperglycemia alert spport in the feature variable to inform collector about tha availability of hyperglycemia alert.
// The feature bit value definition macros can be found in ./CGM_Service_values.h, under the comment "value for CGM feature flag".
static cgmFeature_t             cgmFeature={ CGM_FEATURE_TREND_INFO, BUILD_UINT8(CGM_TYPE_ISF,CGM_SAMPLE_LOC_SUBCUT_TISSUE)};	///<The features supported by the CGM simulator

// ==================================END OF EXCERCISE REGION==================================================

static uint16                   cgmCommInterval=1000;			///<The glucose measurement update interval in ms
static cgmStatus_t              cgmStatus={0x1234,0x567890}; 		///<The status of the CGM simulator. Default value is for testing purpose.
static cgmSessionStartTime_t    cgmStartTime={{0,0,0,0,0,2000},TIME_ZONE_UTC_M5,DST_STANDARD_TIME};
									///<The start time of the current session. The default value is 
static cgmMeasC_t        	cgmCurrentMeas;				///< Local variable storing the most current glucose estimate.
//Time related local variables
static UTCTime                 	cgmCurrentTime_UTC;			///<The UTC format of the current system time. Appear as the number of seconds from 2000-01-01-00:00:00
static UTCTime			cgmStartTime_UTC;			///<The UTC format of the start time. Appear as the number of seconds from 2000-01-01-00:00:00
static uint16			cgmTimeOffset;				///<The time offset from the session start time. 
static uint16                   cgmSessionRunTime=0x00A8;		///<The run time of the current sensor. Default value is 7 days.
static bool                     cgmSessionStartIndicator=false;		///<Indicate whether the sesstion has been started
static bool			cgmStartTimeConfigIndicator=false;	///<Indicate whether the session start time has been configured before with the set start time CGMCP command.
//RACP Related Variables
static cgmMeasC_t *	cgmMeasDB;					///<Pointer to the glucose measurement history database.
static uint8		cgmMeasDBWriteIndx;				///<Hold the array index of the next place to write record.
static uint8		cgmMeasDBCount;					///<The number of records being stored into the database.
static uint8		cgmMeasDBOldestIndx;				///<The index pointing to the oldest record in the database.
static uint8		cgmMeasDBSearchStart; 				///<The starting index records meeting the search criterion.
static uint8		cgmMeasDBSearchEnd;				///<The ending index of records meeting the search criterion.
static uint16		cgmMeasDBSearchNum;   				///<The resulting record number that matches the criterion.
static uint8		cgmMeasDBSendIndx;   				///<The index of the next record to be sent. It is used in RACP reporting record function.
static bool		cgmRACPSendInProgress;				///<Indicate if the sensor is sending out data via RACP.


/*********************************************************************
 * LOCAL FUNCTIONS
 */
//Connection related functions
static void cgmPairStateCB( uint16 connHandle, uint8 state, uint8 status );
static void cgmPasscodeCB( uint8 *deviceAddr, uint16 connectionHandle,uint8 uiInputs, uint8 uiOutputs );
static void cgmGapStateCB( gaprole_States_t newState );
static void cgm_HandleKeys( uint8 shift, uint8 keys );
//Time related functions
static uint8 cgmVerifyTime(UTCTimeStruct* pTime);
static uint8 cgmVerifyTimeZone( int8 input);
static uint8 cgmVerifyDSTOffset( uint8 input);
//CGMCP related functions
static void cgmCtlPntResponse(uint8 opcode, uint8 *roperand,uint8 roperand_len);
static void cgmProcessCtlPntMsg( cgmCtlPntMsg_t* pMsg);
//RACP realted functions
static uint8 cgmSearchMeasDB(uint8 filter,uint16 operand1, uint16 operand2);
static void cgmAddRecord(cgmMeasC_t *cgmCurrentMeas);
static void cgmProcessRACPMsg( cgmRACPMsg_t * pMsg);
static void cgmRACPSendNextMeas();
static void cgmResetMeasDB();
//CGM measurement related functions
static void cgmMeasSend(void);
static void cgmNewGlucoseMeas(cgmMeasC_t * pMeas);
//CGM application level functions
static void cgmservice_cb(uint8 event, uint8* valueP, uint8 len, uint8 * result);
static void cgmSimulationAppInit();
static void cgm_ProcessOSALMsg( osal_event_hdr_t *pMsg );


/*********************************************************************
 * PROFILE CALLBACKS
 */
///Variable for registerint the  GAP Role Callbacks
static gapRolesCBs_t cgm_PeripheralCBs =
{
	cgmGapStateCB,  	///< Profile State Change Callbacks
	NULL                	///< When a valid RSSI is read from controller
};
///Bond Manager Callbacks
static const gapBondCBs_t cgmBondCB =
{
	cgmPasscodeCB,		///< Call back when passcode is entered from the lower level
	cgmPairStateCB		///< Call back when the pair state is changed from the lower level
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
	GGS_AddService( GATT_ALL_SERVICES );       	// GAP service
	GATTServApp_AddService( GATT_ALL_SERVICES ); 	// Add GATT service
	CGM_AddService(GATT_ALL_SERVICES);		// Add CGM service
	DevInfo_AddService( );				// Add device information service
	Batt_AddService();                              // Add battery Service
	
	// Register for CGM application level service callback
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
	P2SEL = 0; // Configure Port 2 as GPIO`
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

	//this command starts the CGM measurement record generation right after device reset
	osal_start_timerEx( cgmTaskId, NOTI_TIMEOUT_EVT, cgmCommInterval);
	cgmSessionStartIndicator=true;
}

/*********************************************************************
 * @fn      CGM_ProcessEvent
 * @brief   CGM Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 * @details This function is registered into the OS task scheduler.
 *	    It will be run periodically. If System message arrives, it will
 *	    be fed into this function for processing. 
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 * @return  events not processed
 */
uint16 CGM_ProcessEvent( uint8 task_id, uint16 events )
{

	VOID task_id; // OSAL required parameter that isn't used in this function
	
	//The event of receiving a system event message.
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
	
	//The event of starting the device
	if ( events & START_DEVICE_EVT )
	{
		// Start the Device
		VOID GAPRole_StartDevice( &cgm_PeripheralCBs );
		// Register with bond manager after starting device
		GAPBondMgr_Register( (gapBondCBs_t *) &cgmBondCB );
		// Start the notification
		return ( events ^ START_DEVICE_EVT );
	}
	
	//The event to send CGM measurement
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

	//The event to send RACP record to the collector
	if ( events & RACP_IND_SEND_EVT)
	{
		cgmRACPSendNextMeas();
		return (events ^ RACP_IND_SEND_EVT);
	}
	return 0;
}

/*********************************************************************
 * @fn      cgm_ProcessOSALMsg
 * @brief   Process an incoming task message.
 * @param   pMsg - message to process
 * @return  none
 */
static void cgm_ProcessOSALMsg( osal_event_hdr_t *pMsg )
{
	switch ( pMsg->event )
	{
		case KEY_CHANGE:
			cgm_HandleKeys( ((keyChange_t *)pMsg)->state, ((keyChange_t *)pMsg)->keys );
			break;
		case CTL_PNT_MSG:// Receive CGMCP write message from the lower GATT layer
			cgmProcessCtlPntMsg( (cgmCtlPntMsg_t *) pMsg);
			break;
		case RACP_MSG:	//Receive RACP write message from the lower GATT layer
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
 * @brief   Process Control Point messages
 * @param   pMsg - The input CGM control point message data structure
 * @return  none
 */
static void cgmProcessCtlPntMsg (cgmCtlPntMsg_t * pMsg)
{
	//Variables for holding the input control point message
	uint8 opcode = pMsg->data[0];
	uint8 *operand; //the operand from the input
	uint8 operand_len; // the operand length
	//Variables for holding the responding control point message
	uint8 ropcode; //the op code in the return char value
	uint8 roperand[CGM_CTL_PNT_MAX_SIZE];//the operand in reuturn char value
	uint8 roperand_len;//length of the response operand

	switch(opcode)
	{ 	//Implement the set/get communication interval
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

		//Implement the start/stop session command
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

// ==================================START OF EXCERCISE REGION===============================================
//EXERCISE STEP (Optional): Implement the procedure for setting and getting the value of hyperglymecia threshold we previously declared.
//Afterwards, prepare the response message (response opcode, response operand).
//The response code value definition macros can be found in ./CGM_Service_values.h, under
//comment "CGM specific op code - response codes"


// ==================================END OF EXCERCISE REGION==================================================
		//Other functions are not implemented
		case  CGM_SPEC_OP_SET_CAL:			
		case  CGM_SPEC_OP_GET_CAL:			
		case  CGM_SPEC_OP_SET_ALERT_HIGH:		
		case  CGM_SPEC_OP_GET_ALERT_HIGH:		
		case  CGM_SPEC_OP_SET_ALERT_LOW:		
		case  CGM_SPEC_OP_GET_ALERT_LOW:		
		case  CGM_SPEC_OP_SET_ALERT_HYPO:		
		case  CGM_SPEC_OP_GET_ALERT_HYPO:		
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
 * @brief   Notification from the profile of a state change.
 * @param   newState - new state
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
 * @brief   Passcode callback.
 * @return  none
 */
static void cgmPasscodeCB( uint8 *deviceAddr, uint16 connectionHandle, uint8 uiInputs, uint8 uiOutputs )
{
	// Send passcode response
	GAPBondMgr_PasscodeRsp( connectionHandle, SUCCESS, DEFAULT_PASSCODE );
}

/*********************************************************************
 * @fn      CGMMeasSend
 * @brief   Send the most current record stored in cgmCurrentMeas as a GATT notification to the CGM measurement characteristic.
 * @return  none
 */
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
	//The following portion is optionally present depending on the flag field of the record
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
	CGM_MeasSend(gapConnHandle, &CGMMeas,  cgmTaskId);
	//Start timing for the next update cycle.
	osal_start_timerEx(cgmTaskId, NOTI_TIMEOUT_EVT, cgmCommInterval);
}

/*********************************************************************
 * @fn      cgmCtlPntResponse
 * @brief   Send a record control point response
 * @param   opcode - response opcode 
 * @param   roperand - address of the array storing the response operand
 * @param   roperand_len - the length of the response operand array
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
 * @brief   The callback function in the application layer when the GATT service layer receives
 * 	    read/write operation to one of the CGM service characteristic
 * @param   event - service event. Enumeration can be found in cgmservice.h  
 * @param   valueP - data value past from the GATT layer to the Application Layer, or vice versa.
 * @param   result - the address to the memory to pass the processing result back to the GATT layer
 * @return  none
 */
static void cgmservice_cb(uint8 event, uint8* valueP, uint8 len, uint8 * result)
{
	switch (event)
	{
		//when CGM measurement characteristic notification is enabled/disabled by the collector APP
		case CGM_MEAS_NTF_ENABLED:
		case CGM_MEAS_NTF_DISABLED:
				break;
		//when the CGM feture characteristic is read by the collector APP
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
		//when the CGM status characteristic is read by the collector APP
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
		//when the CGM session start time characteristic is read by the collector APP
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
		//when the CGM sensor run time characteristic is read by the collector APP
		case CGM_RUN_TIME_READ_REQUEST:
			{
				*valueP = cgmSessionRunTime & 0xFF;
				*(++valueP) = (cgmSessionRunTime >> 8) & 0xFF;
				break;
			}
		//when the CGM start time characteristic is written by the collector APP
		case CGM_START_TIME_WRITE_REQUEST:
			{	
				cgmSessionStartTime_t input;
				UTCTime input_UTC;	
				//Loaded the written value to the local buffer	
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
				//Configure the sensor start time
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
				break;
			}
		//When the CGMCP is written by the collector APP
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
		//When the RACP is written by the collector APP
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
 * @brief Verify time values are suitable for filtering
 * @param pTime - UTC time struct
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

/*********************************************************************
 * @fn cgmVerifyTimeZone	
 * @brief Verify time zone values are compliant to the standard.
 * @param input - the time zone code to be tested
 * @return true if time is ok, false otherwise
 */
static uint8 cgmVerifyTimeZone( int8 input)
{
	if ( (input % 2) !=0)
		return false;
	if ( ((input > -48) && (input<56)) || (input==128))
		return true;
	return false;
}

/*********************************************************************
 * @fn cgmVerifyDSTOffset
 * @brief Verify the daylight saving time input is compliant to the standard.
 * @param input - the dayligt saving code to be tested
 * @return true if time is ok, false otherwise
 */
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
 * @param   pMeas - address to store the generated cgm measurement.
 * @return  none
 */
static void cgmNewGlucoseMeas(cgmMeasC_t * pMeas)
{
	//generate the glucose reading.
	static uint16	glucoseGen=0x0000;	//The current glucose being generated	
	static uint16	glucosePreviousGen=0;	//The previous glucose value
	uint8		flag=0;			//The flag field of the glucose measurement characteristic
	uint8		size=6;			//The size field of the glucose measurement characteristic
	uint16		trend;			//The trend field of the glucose measurement characteristic
	uint16		quality;		//The quality field of the glucose measurement characteristic
	uint32		annunciation=0;		//The annunciation field of the current glucose measurement characteristic
	int32		currentGlucose_cal=0;	//The signed version of the current glucose value for caluculation of trend
	int32		previousGlucose_cal=0;	//The signed version of the previous glucose value for calculation of trend 
	uint16		offset_dif;		//The offset between current and previous record calculating trend 
	int32		trend_cal;		//The signed version for trend calculation, which will be later converted to SFLOAT
	UTCTime		currentTime, startTime; //The current time and the session start time

	//Prepare the CGM measurement concentration value
	glucosePreviousGen=glucoseGen;		// Store the current CGM measurement, which will be the previous value in the next call
	glucoseGen =cgmGetNextData();		// Call the function to generate the next glucose value data point. Currently it is a simulation program drawing glucose value from a patient database
	glucoseGen =( glucoseGen % 0x07FD); 	//Make sure the generated value fit into SFLOAT. In this application we fix the exponent of the SFLOAT to be 0
	pMeas->concentration=glucoseGen;	//Write the value into the buffer 
	
	//Prepare the time offset 
	pMeas->timeoffset=cgmTimeOffset & 0xFFFF;
	cgmTimeOffset += cgmCommInterval/1000;	//Update the time offset for the next call. 

	//Prepare the flag
	flag |= CGM_TREND_INFO_PRES;

	//Prepare the trend field
	if(cgmTimeOffset!=0)
	{
		{ 
			currentGlucose_cal=(glucoseGen & 0x07FF);
			previousGlucose_cal=(glucosePreviousGen & 0x07FF);
			offset_dif=cgmCommInterval/1000;	//EXTRA: currnt communication interval counts in ms.
			trend_cal=(currentGlucose_cal-previousGlucose_cal)*10/offset_dif; //elevate the power by 10 to gain 1 digit accuracy, the highest resolution is 0.1mg/dl/min
			//convert the calculation result to SFLOAT. For simplicity, we fix the exponent to be -1.
			if (trend_cal>2045) 		//when exponent is -1, the mantissa can be at most 2045
				trend=0x07FE;		//representing +infinity
			else if (trend_cal<-2045)	//repeat the above for the negative region
				trend=0x0F02;//-infinity
			else
				trend= (trend_cal & 0x0FFF) | 0xF000;
		}
	} 

	//Prepare the measurement status annunication
	//==================================START OF EXCERCISE REGION===============================================
	//EXERCISE STEP 2: Here it is a good place to test whether the newly generated glucose value exceeds the 
	//hyperglymecia threshold. Set the corresponding status annunciation field bit to indicate hyperglycemia
	//condition. The bit definition can be found in the Bluetooth Continous Glucose Service websit. Additionally,
	//the implementation bit value definition macros can be found in ./CGM_Service_values.h, under
	//the comment "value for CGM status annunciation field"
	


	// ==================================END OF EXCERCISE REGION==================================================
	
	//update the annuciation field
	pMeas->annunication=annunciation;
	//Update the flag field
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
	if (flag & CGM_STATUS_ANNUNC_WARNING_OCT)
		size++;
	if (flag & CGM_STATUS_ANNUNC_CAL_TEMP_OCT)
		size++;
	if (flag & CGM_STATUS_ANNUNC_STATUS_OCT)
		size++;
	pMeas->size=size;  
	pMeas->flags= (flag);  
}

/*********************************************************************
 * @fn      cgmSimulationAppInit
 * @brief   Initialize the CGM simulator application. Reset the historical database
 * @return  none
 */
static void cgmSimulationAppInit()				
{
	cgmMeasDB=(cgmMeasC_t *)osal_mem_alloc(CGM_MEAS_DB_SIZE*sizeof(cgmMeasC_t));
	cgmMeasDBCount=0;
	cgmMeasDBWriteIndx=0;
	cgmMeasDBOldestIndx=0;
	cgmStartTimeConfigIndicator=false;
	cgmSimDataReset();
}	

/*********************************************************************
 * @fn      cgmSearchMeasDB
 * @brief   This function implements the search function for the gluocose measurement. 
 * 	    It assumes the measurement database consists of continous records arranged in ascending order.
 * 	    The current version is based on the sequential search for demo purpose. Therefore, we assume
 * 	    records are arranged in ascending order based on offset time.
 * @param   filter - the filter type in searching
 * @param   operand1 - the primary operand to the search operation.
 * @param   operand2 - the scrondary operand to the search operation, it is currently used only in searching for a range of record.
 * @return  the result code
 */
static uint8 cgmSearchMeasDB(uint8 filter,uint16 operand1, uint16 operand2)
{
	uint8 i=0;
	uint8 j=0;
	
	if(cgmMeasDBCount==0)
		return  RACP_SEARCH_RSP_NO_RECORD;

	switch (filter)
	{
		// All records
		case CTL_PNT_OPER_ALL:
			{
				cgmMeasDBSearchStart=cgmMeasDBOldestIndx;
				cgmMeasDBSearchEnd=(cgmMeasDBOldestIndx+cgmMeasDBCount-1)%CGM_MEAS_DB_SIZE;
				cgmMeasDBSearchNum=cgmMeasDBCount;
				return RACP_SEARCH_RSP_SUCCESS;
			}
		// Records greater or equal to operand1
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
		// The first record
		case CTL_PNT_OPER_FIRST:
			{
				cgmMeasDBSearchStart=cgmMeasDBOldestIndx;
				cgmMeasDBSearchEnd=cgmMeasDBOldestIndx;
				cgmMeasDBSearchNum=1;
				return RACP_SEARCH_RSP_SUCCESS;
			}
		// The last record
		case CTL_PNT_OPER_LAST:
			{
				cgmMeasDBSearchStart=(cgmMeasDBOldestIndx+cgmMeasDBCount-1)%CGM_MEAS_DB_SIZE;
				cgmMeasDBSearchEnd=cgmMeasDBSearchStart;
				cgmMeasDBSearchNum=1;
				return RACP_SEARCH_RSP_SUCCESS;
			}
		// The records which are less than or equal to operand1
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
		// Records that are with in the range of [operand1, operand2]
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
 * @brief  Add record to the database 
 * @param   cgmCurrentMeas - the add of the current measurement to be added to the database.
 */
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

/*********************************************************************
 * @fn      cgmProcessRACPMsg
 * @brief   Record Access Control Point messages processing
 * @param   pMsg - The input RACP message data structure
 * @return  none
 */
static void cgmProcessRACPMsg (cgmRACPMsg_t * pMsg)
{
	uint8 opcode=pMsg->data[0];
	uint8 operator=pMsg->data[1];
	uint8 filter;
	uint16 operand1=0,operand2=0;
	uint8 reopcode=0;

	switch (opcode)
	{
		//Get the history records or their number count.
		case CTL_PNT_OP_REQ:
		case CTL_PNT_OP_GET_NUM:
			//Parse the input command operand bease on the filter requirement
			if (operator==CTL_PNT_OPER_LESS_EQUAL 
					|| operator==CTL_PNT_OPER_GREATER_EQUAL 
					|| operator==CTL_PNT_OPER_RANGE)
				operand1=BUILD_UINT16(pMsg->data[3],pMsg->data[4]);
			if (operator==CTL_PNT_OPER_RANGE)
				operand2=BUILD_UINT16(pMsg->data[5],pMsg->data[6]);
			//Get the starting and ending index of the record meeting requriement  
			if ((reopcode=cgmSearchMeasDB(operator,operand1,operand2))==RACP_SEARCH_RSP_SUCCESS)
			{
				if (opcode==CTL_PNT_OP_REQ){
					cgmMeasDBSendIndx=0;
					osal_start_timerEx(cgmTaskId,RACP_IND_SEND_EVT,500); //start the data transfer event  				   
					CGM_SetSendState(true);
					return;}
				//If we only need to report the number count, we can prepare the send the packet right away.
				else if (opcode==CTL_PNT_OP_GET_NUM)
				{
					cgmRACPRsp.value[0]=CTL_PNT_OP_NUM_RSP;
					cgmRACPRsp.value[1]=CTL_PNT_OPER_NULL;
					cgmRACPRsp.value[2]=LO_UINT16(cgmMeasDBSearchNum);
					cgmRACPRsp.value[3]=HI_UINT16(cgmMeasDBSearchNum);
					cgmRACPRsp.len=4;
					CGM_RACPIndicate(gapConnHandle, &cgmRACPRsp, cgmTaskId);
				}
			}
			//If search is not successful, indicate the result.
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
}	


/*********************************************************************
 * @fn      cgmRACPSendNextMeas
 * @brief   As part of the RACP operation, this function sends the set of historical
 * 	    measurement data to the collector APP sequencially. The resulting record
 * 	    would be received by the collector through the glucose measurement characteristic
 * 	    notification.
 * @param   pMsg - The input RACP message data structure
 * @return  none
 */
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
		CGM_MeasSend(gapConnHandle, &cgmRACPRspNoti, cgmTaskId);
		osal_start_timerEx(cgmTaskId, RACP_IND_SEND_EVT, 1000); //EXTRA: set the timer to be smaller to improve speed
	}
	else
	{
		//The current RACP transfer is finished. Indicate the a success to the RACP operation
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


/*********************************************************************
 * @fn      cgmResetMeasDB
 * @brief   Reset the cgm measurement history database.
 * @return  none
 */
static void cgmResetMeasDB()
{
	cgmMeasDBOldestIndx=0;
	cgmMeasDBCount=0;
}

/*********************************************************************
 *********************************************************************/
