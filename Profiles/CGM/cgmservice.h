/**************************************************************************************************
  Filename:       cgmservice.h
  Revised:        $Date: 2015-02-03$
  Revision:       $Revision: 59 $

  Description:    This file contains the CGM service definitions and
                  prototypes.

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

#ifndef CGMSERVICE_H
#define CGMSERVICE_H
#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include "OSAL_clock.h"
  
/*********************************************************************
 * CONSTANTS
 */
// Glucose Service bit fields
#define CGM_SERVICE				0x00000001	///< Flag identifier for CGM service

// Characteristic Value sizes
#define CGM_CTL_PNT_MIN_SIZE			1               ///< Minimum size of the CGM specific operation control point buffer
#define CGM_CTL_PNT_MAX_SIZE			20              ///< Maximum size of the CGM specific operation control point buffer
#define CGM_RACP_MIN_SIZE			2               ///< Minimum size of the RACP buffer
#define CGM_RACP_MAX_SIZE			20              ///< Minimum size of the RACP buffer

//CGM  Callback events
#define CGM_MEAS_NTF_ENABLED			1               ///< Measurement Characteristic notification enabled
#define CGM_MEAS_NTF_DISABLED			2               ///< Measurement Characteristic notification disabled
#define CGM_CONTEXT_NTF_ENABLED			3               ///< Context notification enabled
#define CGM_CONTEXT_NTF_DISABLED		4               ///< Context notification disabled
#define CGM_CTL_PNT_IND_ENABLED			5               ///< OPCP indication is enabled
#define CGM_CTL_PNT_IND_DISABLED		6               ///< OPCP indication is disabled
#define CGM_CTL_PNT_CMD				7               ///< OPCP control command received
#define CGM_RACP_CTL_PNT_CMD			8               ///< RACP control command received
#define CGM_SESSION_START_TIME_CHANGED		9               ///< Session start time has changed
#define CGM_FEATURE_READ_REQUEST		10              ///< Feature read request from the lower layer
#define CGM_STATUS_READ_REQUEST			11              ///< Status read request from the lower layer
#define CGM_START_TIME_READ_REQUEST		12              ///< Start time read request from the lower layer
#define CGM_RUN_TIME_READ_REQUEST		13              ///< Run time read request from the lower layer
#define CGM_START_TIME_WRITE_REQUEST		14              ///< Start time write request from the lower layer
#define	CGM_RACP_IND_DISABLED			15              ///< RACP indication is disabled
#define	CGM_RACP_IND_ENABLED			16              ///< RACP indication is enabled

// ATT status values
#define	CGM_ERR_IN_PROGRESS		     	0x80            ///< GATT error, transmission in progress
#define CGM_ERR_CCC_CONFIG		     	0x81            ///< GATT error, incompatible CCC configuration

// CGM Service UUID
#define CGM_SERV_UUID				0x181F	///< GM service

// CGM Service Characteristic UUID
#define CGM_MEAS_UUID				0x2AA7	///< CGM Measurement
#define CGM_FEATURE_UUID			0x2AA8	///< CGM Feature
#define	CGM_STATUS_UUID				0x2AA9	///< CGM Status
#define	CGM_SES_START_TIME_UUID			0x2AAA	///< CGM Session Start Time
#define CGM_SES_RUN_TIME_UUID			0x2AAB	///< CGM Session Run Time
#define REC_ACCESS_CTRL_PT_UUID			0x2A52	///< Record Access Control Point
#define CGM_SPEC_OPS_CTRL_PT_UUID		0x2AAC	///< CGM Specific Ops Control Point

// values for CGM meas flags
#define	CGM_TREND_INFO_PRES			0x01	///< CGM Trend Information Present
#define	CGM_QUALITY_PRES			0x02	///< CGM Quality Present
#define	CGM_STATUS_ANNUNC_WARNING_OCT		0x20	///< Sensor Status Annuciation Field, Warning Octet present
#define	CGM_STATUS_ANNUNC_CAL_TEMP_OCT		0x40	///< Sensor Status Annuciation Field, Cal/Temp-Octet present
#define	CGM_STATUS_ANNUNC_STATUS_OCT		0x80	///< Sensor Status Annuciation Field, Status Octet present

// value for CGM status annuciation field
#define	CGM_STATUS_ANNUNC_SES_STOP		0x000001///< Session Stop
#define	CGM_STATUS_ANNUNC_BAT_LOW		0x000002///< Device Battery Low
#define	CGM_STATUS_ANNUNC_WRONG_SENSOR_TYPE	0x000004///< Sensor type incorrect for device
#define	CGM_STATUS_ANNUNC_SENSOR_MALFUNC	0x000008///< Sensor malfunction
#define	CGM_STATUS_ANNUNC_DEVICE_SPEC_ALERT	0x000010///< Device specific alert
#define	CGM_STATUS_ANNUNC_GNERAL_DEVICE_FAULT	0x000020///< General device fault has occurred in the sensor
#define	CGM_STATUS_ANNUNC_REQ_TIME_SYNC		0x000100///< Time synchronization between sensor and collector required
#define	CGM_STATUS_ANNUNC_NOT_ALLOWED_CAL	0x000200///< Calibration not allowed
#define	CGM_STATUS_ANNUNC_RECOM_CAL		0x000400///< Calibration recommended
#define	CGM_STATUS_ANNUNC_REQ_CAL		0x000800///< Calibration required
#define	CGM_STATUS_ANNUNC_HIGH_TEMP		0x001000///< Sensor temperature too high for valid test/result at the time of measurement
#define	CGM_STATUS_ANNUNC_LOW_TEMP		0x002000///< Sensor temperature too low for valid test/result at the time of measurement
#define	CGM_STATUS_ANNUNC_LOW_PATIENT		0x010000///< Sensor result lower than the patient low level
#define	CGM_STATUS_ANNUNC_HIGH_PATIRNT		0x020000///< Sensor result higher than the patient high level		
#define	CGM_STATUS_ANNUNC_LOW_HYPO		0x040000///< Sensor lower than the hypo level
#define	CGM_STATUS_ANNUNC_HIGH_HYPER		0x080000///< Sensor result higher than the hyper level
#define	CGM_STATUS_ANNUNC_EXCEEDED_DESC_RATE	0x100000///< Sensor rate of decrease exceeded
#define	CGM_STATUS_ANNUNC_EXCEEDED_INCR_RATE	0x200000///< Sensor rate of increase exceeded
#define	CGM_STATUS_ANNUNC_LOW_NIR		0x400000///< Sensor result lower than the device can process
#define	CGM_STATUS_ANNUNC_HIGH_NIR		0x800000///< Sensor result higher than the device can process

// value for CGM feature flag org.bluetooth.characteristic.cgm_feature.xml
#define CGM_FEATURE_CAL				0x000001///< Calibration Support
#define CGM_FEATURE_ALERTS_HIGH_LOW		0x000002///< Patient High/Low Alerts supported
#define CGM_FEATURE_ALERTS_HYPO			0x000004///< Hypo Alerts supported
#define CGM_FEATURE_ALERTS_HYPER		0x000008///< Hyper Alerts supported
#define CGM_FEATURE_ALERTS_INC_DEC		0x000010///< Rate of Increase/Decrease Alerts support
#define CGM_FEATURE_ALERTS_DEVICE_SPEC		0x000020///< Device Specific Alert supported
#define CGM_FEATURE_DETECTION_SENSOR_MALFUNC	0x000040///< Sensor Malfunction Detection support
#define CGM_FEATURE_DETECTION_TEMP_HIGH_LOW	0x000080///< Sensor temperature high-low detection support
#define CGM_FEATURE_DETECTION_RESULT_HIGH_LOW	0x000100///< Sensor result high-low detection support
#define CGM_FEATURE_DETECTION_BAT_LOW		0x000200///< Low battery detection support
#define CGM_FEATURE_DETECTION_SENSOR_TYPE_ERR	0x000400///< Sensor type error detection supported
#define CGM_FEATURE_FAULT_GENERAL_DEVICE	0x000800///< General device fault support
#define CGM_FEATURE_E2E_CRC			0x001000///< E2E-CRC supported
#define CGM_FEATURE_MULTI_BOND			0x002000///< Multiole Bond support
#define CGM_FEATURE_MULTI_SES			0x004000///< Multiple Sessions support
#define CGM_FEATURE_TREND_INFO			0x008000///< CGM Trend information support
#define CGM_FEATURE_QUALITY			0x010000///< CGM quality support

// CGM Type-Sample byte
// values for CGM type information flag 4 bit lower nimble 
#define CGM_TYPE_CAPILLARY_WHOLE_BLOOD		0x01	///< Capillary whole blood
#define CGM_TYPE_CAPILLARY_PLASMA		0x02	///< Capillary plasma
#define CGM_TYPE_CAPILLARY_WHOLE_BLOOD2		0x03	///< Capillary whole blood again, possibly a typo
#define CGM_TYPE_VENOUS_PLASMA			0x04	///< Venous Plasma
#define CGM_TYPE_ARTERIAL_WHOLE_BLOOD		0x05	///< Arterial whole blood
#define CGM_TYPE_ARTERIAL_PLASMA		0x06	///< Arterial Plasma
#define CGM_TYPE_UNDET_WHOLE_BLOOD		0x07	///< Undetermined whole blood
#define CGM_TYPE_UNDET_PLASMA			0x08	///< Undetermined plasma
#define CGM_TYPE_ISF				0x09	///< Interstitial fluid (ISF)
#define CGM_TYPE_CTRL_SOLUTION			0x0A	///< Control solution

// values for CGM sample location 4 bit higher nimble
#define CGM_SAMPLE_LOC_FINGER			0x01	///< Sample location Finger
#define CGM_SAMPLE_LOC_AST			0x02	///< Sample location Alternate Site Test (AST
#define CGM_SAMPLE_LOC_EARLOB			0X03	///< Sample location Earlobe
#define CGM_SAMPLE_LOC_CTRL_SOLUTION		0x04	///< Sample location Control solution
#define CGM_SAMPLE_LOC_SUBCUT_TISSUE		0x05	///< Sample location Subcutaneous tissue
#define CGM_SAMPLE_LOC_UNAVAIL			0x0F	///< Sample location not available

// The time value for start time/ run time etc. is obviously implemented
// https://developer.bluetooth.org/gatt/characteristics/Pages/CharacteristicViewer.aspx?u=org.bluetooth.characteristic.date_time.xml
// Time Zone Infomation sint8
#define TIME_ZONE_UTC_M12			-48	///< TC-12:00
#define TIME_ZONE_UTC_M11			-44	///< TC-11:00
#define TIME_ZONE_UTC_M10			-40	///< TC-10:00
#define TIME_ZONE_UTC_M9p5			-38	///< TC-9:30
#define TIME_ZONE_UTC_M9			-36	///< TC-9:00
#define TIME_ZONE_UTC_M8			-32	///< TC-8:00
#define TIME_ZONE_UTC_M7			-28	///< TC-7:00
#define TIME_ZONE_UTC_M6			-24	///< TC-6:00
#define TIME_ZONE_UTC_M5			-20	///< TC-5:00
#define TIME_ZONE_UTC_M4p5			-18	///< TC-4:30
#define TIME_ZONE_UTC_M4			-16	///< TC-4:00
#define TIME_ZONE_UTC_M3p5			-14	///< TC-3:30
#define TIME_ZONE_UTC_M3			-12	///< TC-3:00
#define TIME_ZONE_UTC_M2			-8	///< TC-2:00
#define TIME_ZONE_UTC_M1			-4	///< TC-1:00
#define TIME_ZONE_UTC_P0			0	///< TC+0:00
#define TIME_ZONE_UTC_P1			4	///< TC+1:00
#define TIME_ZONE_UTC_P2			8	///< TC+2:00
#define TIME_ZONE_UTC_P3			12	///< TC+3:00
#define TIME_ZONE_UTC_P3p5			14	///< TC+3:30
#define TIME_ZONE_UTC_P4			16	///< TC+4:00
#define TIME_ZONE_UTC_P4p5			18	///< TC+4:30
#define TIME_ZONE_UTC_P5			20	///< TC+5:00
#define TIME_ZONE_UTC_P5p5			22	///< TC+5:30
#define TIME_ZONE_UTC_P5p75			23	///< TC+5:45
#define TIME_ZONE_UTC_P6			24	///< TC+6:00
#define TIME_ZONE_UTC_P6p5			26	///< TC+6:30
#define TIME_ZONE_UTC_P7			28	///< TC+7:00
#define TIME_ZONE_UTC_P8			32	///< TC+8:00
#define TIME_ZONE_UTC_P8p75			35	///< TC+8:45
#define TIME_ZONE_UTC_P9			36	///< TC+9:00
#define TIME_ZONE_UTC_P9p5			38	///< TC+9:30
#define TIME_ZONE_UTC_P10			40	///< TC+10:00
#define TIME_ZONE_UTC_P10p5			42	///< TC+10:30
#define TIME_ZONE_UTC_P11			44	///< TC+11:00
#define TIME_ZONE_UTC_P11p5			46	///< TC+11:30
#define TIME_ZONE_UTC_P12			48	///< TC+12:00
#define TIME_ZONE_UTC_P12p75			51	///< TC+12:45
#define TIME_ZONE_UTC_P13			52	///< TC+13:00
#define TIME_ZONE_UTC_P14			56	///< TC+14:00
#define TIME_ZONE_UNKNOWN			-128	///< nknown

// DST Offset information uint8
// https://developer.bluetooth.org/gatt/characteristics/Pages/CharacteristicViewer.aspx?u=org.bluetooth.characteristic.dst_offset.xml
#define DST_STANDARD_TIME			0x00	///< Standard Time
#define	DST_HALF_HOUR_DAYLIGHT			0x02	///< Half An Hour Daylight Time (+0.5h)
#define	DST_ONE_HOUR_DAYLIGHT			0x04	///< Daylight Time (+1h)
#define	DST_DOUBLE_HOUR_DAYLIGHT		0x08	///< Double Daylight Time (+2h)
#define	DST_UNKNOWN				0xFF	///< Daylight Time unknown

// Record Control Access Point 
//Record Control Point values OP Codes
//https://developer.bluetooth.org/gatt/characteristics/Pages/CharacteristicViewer.aspx?u=org.bluetooth.characteristic.record_access_control_point.xml
#define CTL_PNT_OP_REQ                  	0x01	///< eport stored record (Operator: Value from Operator Table)
#define CTL_PNT_OP_CLR                       	0x02	///< elete stored records (Operator: Value from Operator Table)
#define CTL_PNT_OP_ABORT                     	0x03	///< bort operation (Operator: Null 'value of 0x00 from Operator Table')
#define CTL_PNT_OP_GET_NUM                   	0x04	///< eport number of stored records (Operator: Value from Operator Table)
#define CTL_PNT_OP_NUM_RSP                   	0x05	///< umber of stored records response (Operator: Null 'value of 0x00 from Operator Table')
#define CTL_PNT_OP_REQ_RSP                   	0x06	///< esponse Code (Operator: Null 'value of 0x00 from Operator Table')

//Record Control Point operator
#define CTL_PNT_OPER_NULL                   	0x00	///< Null
#define CTL_PNT_OPER_ALL                     	0x01  	///< All records
#define CTL_PNT_OPER_LESS_EQUAL              	0x02	///< Less than or equal to 
#define CTL_PNT_OPER_GREATER_EQUAL           	0x03	///< Greater than or equal to
#define CTL_PNT_OPER_RANGE                   	0x04	///< Within range of (inclusive)
#define CTL_PNT_OPER_FIRST                   	0x05	///< First record(i.e. oldest record)
#define CTL_PNT_OPER_LAST                    	0x06  	///< Last record (i.e. most recent record)

//Record Control Point Response Codes
#define CTL_PNT_RSP_SUCCESS                	0x01	///< Normal response for successful operation
#define CTL_PNT_RSP_OPCODE_NOT_SUPPORTED     	0x02	///< Normal response if unsupported Op Code is received
#define CTL_PNT_RSP_OPER_INVALID             	0x03	///< Normal response if Operator received does not meet the requirements of the service (e.g. Null was expected)
#define CTL_PNT_RSP_OPER_NOT_SUPPORTED       	0x04	///< Normal response if unsupported Operator is received
#define CTL_PNT_RSP_OPERAND_INVALID          	0x05	///< Normal response if Operand received does not meet the requirements of the service
#define CTL_PNT_RSP_NO_RECORDS              	0x06	///< Normal response if request to report stored records or request to delete stored records resulted in no records meeting criteria.
#define CTL_PNT_RSP_ABORT_FAILED             	0x07	///< Normal response if request for Abort cannot be completed
#define CTL_PNT_RSP_PROC_NOT_CMPL            	0x08	///< Normal response if unable to complete a procedure for any reason
#define CTL_PNT_RSP_FILTER_NOT_SUPPORTED     	0x09	///< Normal response if unsupported Operand is received

//Record Control Point Resposne Filter
#define CTL_PNT_FILTER_TIME_OFFSET             	0x01	///< Time Offset
// CGM Specific operation codes
#define CGM_SPEC_OP_SET_INTERVAL		1	///< Set CGM Communication Interval
#define CGM_SPEC_OP_GET_INTERVAL		2	///< Get CGM Communication Interval
#define CGM_SPEC_OP_RESP_INTERVAL		3	///< CGM Communication Interval response
#define CGM_SPEC_OP_SET_CAL			4	///< Set Glucose Calibration Value
#define CGM_SPEC_OP_GET_CAL			5	///< Get Glucose Calibration Value
#define CGM_SPEC_OP_RESP_CAL			6	///< Glucose Calibration Value response
#define CGM_SPEC_OP_SET_ALERT_HIGH		7	///< Set Patient High Alert Level
#define CGM_SPEC_OP_GET_ALERT_HIGH		8	///< Get Patient High Alert Level
#define CGM_SPEC_OP_RESP_ALERT_HIGH		9	///< Patient High Alert Level Response
#define CGM_SPEC_OP_SET_ALERT_LOW		10	///< Set Patient Low Alert Level
#define CGM_SPEC_OP_GET_ALERT_LOW		11	///< Get Patient Low Alert Level
#define CGM_SPEC_OP_RESP_ALERT_LOW		12	///< Patient Low Alert Level Response
#define CGM_SPEC_OP_SET_ALERT_HYPO		13	///< Set Hypo Alert Level
#define CGM_SPEC_OP_GET_ALERT_HYPO		14	///< Get Hypo Alert Level
#define CGM_SPEC_OP_RESP_ALERT_HYPO		15	///< Hypo Alert Level Response
#define CGM_SPEC_OP_SET_ALERT_HYPER		16	///< Set Hyper Alert Level
#define CGM_SPEC_OP_GET_ALERT_HYPER		17	///< Get Hyper Alert Level
#define CGM_SPEC_OP_RESP_ALERT_HYPER		18	///< Hyper Alert Level Response
#define CGM_SPEC_OP_SET_ALERT_RATE_DECREASE	19	///< Set Rate of Decrease Alert Level
#define CGM_SPEC_OP_GET_ALERT_RATE_DECREASE	20	///< Get Rate of Decrease Alert Level
#define CGM_SPEC_OP_RESP_ALERT_RATE_DECREASE	21	///< Rate of Decrease Alert Level Response
#define CGM_SPEC_OP_SET_ALERT_RATE_INCREASE	22	///< Set Rate of Increase Alert Level
#define CGM_SPEC_OP_GET_ALERT_RATE_INCREASE	23	///< Get Rate of Increase Alert Level
#define CGM_SPEC_OP_RESP_ALERT_RATE_INCREASE	24	///< Rate of Increase Alert Level Response
#define CGM_SPEC_OP_RESET_ALERT_DEVICE_SPEC	25	///< Reset Device Specific Alert
#define CGM_SPEC_OP_START_SES			26	///< Start the Session
#define CGM_SPEC_OP_STOP_SES			27	///< Stop the Session
#define CGM_SPEC_OP_RESP_CODE			28	///< Response Code

// CGM specific op code - resposne codes
#define	CGM_SPEC_OP_RESP_SUCCESS		1	///< Success
#define	CGM_SPEC_OP_RESP_OP_NOT_SUPPORT		2	///< Op Code not supported
#define	CGM_SPEC_OP_RESP_OPERAND_INVALID	3	///< Invalid Operand
#define	CGM_SPEC_OP_RESP_PROCEDURE_NOT_COMPLETE	4	///< Procedure not completed
#define	CGM_SPEC_OP_RESP_PARAM_NIR		5	///< Parameter out of range

// CGM Calibration Status
#define	CGM_CALIBRATION_REJECT			0x01	///< Calibration Data rejected (Calibration failed)
#define	CGM_CALIBRATION_NIR			0x02	///< Calibration Data out of range
#define	CGM_CALIBRATION_PENDING			0x04	///< Calibration Process Pending

// Unit UUID 
#define UNIT_MASSDENSITY_MG_PER_DL		0x27B1	///< ass density (milligram per decilitre)
#define UNIT_TIME_MIN				0x2760	///< ime (minute)

// The following definition is inferred from the CGM profile specification
#define CGM_CHAR_VAL_SIZE_FEATURE                   6              ///< he size of the feature characteristic without the CRC 
#define CGM_CHAR_VAL_SIZE_STATUS                    5              ///< he size of the status characteristic without the CRC 
#define CGM_CHAR_VAL_SIZE_START_TIME                9              ///< he size of the start time characteristic without the CRC 
#define CGM_CHAR_VAL_SIZE_RUN_TIME                  2              ///< he size of the run time characteristic without the CRC 

/*********************************************************************
 * TYPEDEFS
 */
// Glucose Service callback function
typedef void (*CGMServiceCB_t)(uint8 event, uint8* data, uint8 dataLen,uint8 *result);

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * Profile Callbacks
 */

/*********************************************************************
 * API FUNCTIONS 
 */

/*
 * CGM_AddService- Initializes the CGM service by registering
 *          GATT attributes with the GATT server.
 * @param   services - services to add. This is a bit map and can
 *                     contain more than one service.
 */
extern bStatus_t CGM_AddService( uint32 services );

/*
 * CGM_Register - Register a callback function with the
 *          CGM Service
 * @param   pfnServiceCB - Callback function.
 */
extern void CGM_Register( CGMServiceCB_t pfnServiceCB);

/*********************************************************************
 * @fn          CGM_MeasSend
 * @brief       Send a CGM measurement.
 * @param       connHandle - connection handle
 * @param       pNoti - pointer to notification structure
 * @return      Success or Failure
 */
extern bStatus_t CGM_MeasSend( uint16 connHandle, attHandleValueNoti_t *pNoti, uint8 taskId );

/*********************************************************************
 * @fn          CGM_ContextSend
 * @brief       Send a CGM measurement context.
 * @param       connHandle - connection handle
 * @param       pNoti - pointer to notification structure
 * @return      Success or Failure
 */
extern bStatus_t CGM_ContextSend( uint16 connHandle, attHandleValueNoti_t *pNoti, uint8 taskId );

/*********************************************************************
 * @fn          CGM_CtlPntIndicate
 * @brief       Send an indication containing a CGM
 *              measurement.
 * @param       connHandle - connection handle
 * @param       pInd - pointer to indication structure
 * @return      Success or Failure
 */
extern bStatus_t CGM_CtlPntIndicate( uint16 connHandle, attHandleValueInd_t *pInd, uint8 taskId );

/*********************************************************************
 * @fn          CGM_RACPIndicate
 * @brief       Send an indication containing RACP operation result to RACP characteristic.
 * @param       connHandle - connection handle
 * @param       pInd - pointer to indication structure
 * @return      Success or Failure
 */
extern bStatus_t CGM_RACPIndicate( uint16 connHandle, attHandleValueInd_t *pInd, uint8 taskId );

/*********************************************************************
 * @fn          CGM_SetSendState
 * @brief       Set the state of the CGM database transmission indicator
 * @param       connHandle - connection handle
 * @param       changeType - type of change
 * @return      none
 */
extern bool CGM_SetSendState(bool input);

/*********************************************************************
*********************************************************************/
#ifdef __cplusplus
}
#endif

#endif /* CGMSERVICE_H */
