/**************************************************************************************************
  Filename:       cgmservice.c
  Revised:        3-Feb-2015
  Revision:       2

  Description:    This file contains the CGM sample service
                  for use with the CGM sample application.

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
#include "linkdb.h"
#include "att.h"
#include "gatt.h"
#include "gatt_uuid.h"
#include "gatt_profile_uuid.h"
#include "gattservapp.h"
#include "gapbondmgr.h"
#include "cgmservice.h"


/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */
// Position of glucose measurement value in attribute array
#define CGM_MEAS_VALUE_POS                     2		///<The position of the value of the CGM measurement charateristic in the attribute array
#define CGM_MEAS_CONFIG_POS                    3		///<The position of the Client Configuration of the CGM measurement charateristic in the attribute array
#define CGM_FEATURE_VALUE_POS                  5		///<The position of the value of the CGM feature charateristic in the attribute array
#define CGM_STATUS_VALUE_POS                   7		///<The position of the value of the CGM status charateristic in the attribute array
#define CGM_SESSION_START_TIME_VALUE_POS       9		///<The position of the value of the session start time  charateristic in the attribute array
#define CGM_SESSION_RUN_TIME_VALUE_POS         11		///<The position of the value of the session run time charateristic in the attribute array
#define CGM_RACP_VALUE_POS                     13		///<The position of the value of the RACP charateristic in the attribute array
#define CGM_RACP_CONFIG_POS                    14   		///<The position of the Client Configuration of the RACP charateristic in the attribute array
#define CGM_CGM_OPCP_VALUE_POS                 16   		///<The position of the value of the CGM Specific Operation Control Point (OPCP) charateristic in the attribute array
#define CGM_CGM_OPCP_CONFIG_POS                17		///<The position of the Client Configuration of the CGM Specific Operation Control Point OPCP) charateristic in the attribute array

/*********************************************************************
 * TYPEDEFS
 */
   
/*********************************************************************
 * GLOBAL VARIABLES
 */
CONST uint8 CGMServiceUUID[ATT_BT_UUID_SIZE] = {LO_UINT16(CGM_SERV_UUID), HI_UINT16(CGM_SERV_UUID)};				///< CGM service UUID Stored as a constant variable
CONST uint8 CGMMeasUUID[ATT_BT_UUID_SIZE] = {LO_UINT16(CGM_MEAS_UUID), HI_UINT16(CGM_MEAS_UUID)};				///< CGM Measurement characteristic UUID stored as a constant variable
CONST uint8 CGMFeatureUUID[ATT_BT_UUID_SIZE] = {LO_UINT16(CGM_FEATURE_UUID), HI_UINT16(CGM_FEATURE_UUID)};			///< CGM Feature UUID stored as a constant
CONST uint8 CGMStatusUUID[ATT_BT_UUID_SIZE] = {LO_UINT16(CGM_STATUS_UUID), HI_UINT16(CGM_STATUS_UUID)};				///< CGM Status UUID stored as a constant
CONST uint8 CGMSessionStartTimeUUID[ATT_BT_UUID_SIZE] = {LO_UINT16(CGM_SES_START_TIME_UUID), HI_UINT16(CGM_SES_START_TIME_UUID)};		///<CGM Session Start stored as a constant
CONST uint8 CGMSessionRunTimeUUID[ATT_BT_UUID_SIZE] = {LO_UINT16(CGM_SES_RUN_TIME_UUID), HI_UINT16(CGM_SES_RUN_TIME_UUID)};			///<CGM Session Run stored as a constant
CONST uint8 recordControlPointUUID[ATT_BT_UUID_SIZE] = {LO_UINT16(REC_ACCESS_CTRL_PT_UUID), HI_UINT16(REC_ACCESS_CTRL_PT_UUID)};		///<Record Control Point stored as a constant
CONST uint8 CGMSpecificOpsControlPointUUID[ATT_BT_UUID_SIZE] = {LO_UINT16(CGM_SPEC_OPS_CTRL_PT_UUID), HI_UINT16(CGM_SPEC_OPS_CTRL_PT_UUID)};	///<CGM Specific Ops Control Point stored as a constant

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
static CGMServiceCB_t CGMServiceCB;		///< The variable to register the CGM service callback function
static bool	      cgmMeasDBSendInProgress;	///< An variable to indicate whether the RACP transmission is in progress. 

/*********************************************************************
 * Profile Attributes - variables
 */
// CGM Service attribute
static CONST gattAttrType_t CGMService = {ATT_BT_UUID_SIZE, CGMServiceUUID };		///< CGM GATT service declaration data structure
// CGM meaaurement Characteristic
static uint8 CGMMeasProps = GATT_PROP_NOTIFY;						///< Variable storing the CGM Measurement Characteristic property
static gattCharCfg_t CGMMeasConfig[GATT_MAX_NUM_CONN];					///< Variable for storing the client configuration for CGM Measurement Characteristic
static uint8 CGMMeassurementDummy=0;							///< This is a dummy variable to register the CGM Measurement characteristic to the ATT server. The actual variable for receiving the CGM measurement is defined in cgm.c
// CGM feature
static uint8  CGMFeatureProps = GATT_PROP_READ;						///< Variable storing the CGM Feature Characteristic property
static uint8  CGMFeatureDummy = 0;							///< This is a dummy variable to register the CGM Feature characteristic to the ATT server. The actual variable for receiving the CGM feature value is defined in cgm.c
// CGM Status
static uint8  CGMStatusProps = GATT_PROP_READ;						///< Variable storing the CGM Status Characteristic property
static uint8  CGMStatusDummy=0;								///< This is a dummy variable to register the CGM Status characteristic to the ATT server. The actual variable for receiving the CGM measurement is defined in cgm.c
// CGM Session Start Time
static uint8  CGMSessionStartTimeProps = GATT_PROP_WRITE|GATT_PROP_READ;		///< Variable storing the CGM Start Time Characteristic property
static uint8  CGMSessionStartTimeDummy=0;						///< This is a dummy variable to register the CGM Start Time characteristic to the ATT server. The actual variable for receiving the CGM measurement is defined in cgm.c
// CGM Session Run Time
static uint8 CGMSessionRunTimeProps = GATT_PROP_READ;					///< Variable storing the CGM Run Time Characteristic property
static uint16  CGMSessionRunTimeDummy=2; 						///< This is a dummy variable to register the CGM Run time characteristic to the ATT server. The actual variable for receiving the CGM measurement is defined in cgm.c
// Record Access Control Point RACP
static uint8 CGMRacpProps = GATT_PROP_WRITE|GATT_PROP_INDICATE;				///< Variable storing the CGM RACP Characteristic property
static gattCharCfg_t    CGMRacpConfig[GATT_MAX_NUM_CONN];				///< Variable for storing the client configuration for RACP Characteristic
static uint8 CGMRacpDummy=0;								///< This is a dummy variable to register the CGM RACP characteristic to the ATT server. The actual variable for receiving the CGM measurement is defined in cgm.c
// CGM Specific Control Point
static uint8 CGMControlProps = GATT_PROP_WRITE|GATT_PROP_INDICATE;			///< Variable storing the CGM OPCP Characteristic property
static gattCharCfg_t    CGMControlConfig[GATT_MAX_NUM_CONN];				///< Variable for storing the client configuration OPCP Characteristic
static uint8 CGMControlDummy=0;								///< This is a dummy variable to register the CGM OPCP characteristic to the ATT server. The actual variable for receiving the CGM measurement is defined in cgm.c

///******************************************************
/// Profile Attributes - Table 
///******************************************************

/// This variable is used to define the attributes of the CGM service
static gattAttribute_t CGMAttrTbl[] =
{
  // CGM Service
  {
    { ATT_BT_UUID_SIZE, primaryServiceUUID }, /* type */
    GATT_PERMIT_READ,                         /* permissions */
    0,                                        /* handle */
    (uint8 *)&CGMService                      /* pValue */
  },
    //////////////////////////////////////////////
    // CGM MEASUREMENT Charaacteristic
    //////////////////////////////////////////////
    
    /// 1. Characteristic Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &CGMMeasProps
    },

    /// 2. Characteristic Value
    {
      { ATT_BT_UUID_SIZE, CGMMeasUUID },
      0, 
      0,
      &CGMMeassurementDummy
    },

    /// 3.Characteristic Configuration
    {
      { ATT_BT_UUID_SIZE, clientCharCfgUUID },
      GATT_PERMIT_AUTHEN_READ | GATT_PERMIT_AUTHEN_WRITE |
      GATT_PERMIT_ENCRYPT_READ | GATT_PERMIT_ENCRYPT_WRITE,
      0,
      (uint8 *)&CGMMeasConfig
    },

    //////////////////////////////////////////////
    // CGM Feature
    //////////////////////////////////////////////

    /// 4.Characteristic Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &CGMFeatureProps
    },

    /// 5.Characteristic Value
    {
      { ATT_BT_UUID_SIZE, CGMFeatureUUID },
      GATT_PERMIT_ENCRYPT_READ | GATT_PERMIT_AUTHEN_READ,
      0,
      &CGMFeatureDummy
    },

    //////////////////////////////////////////////
    // CGM STATUS
    //////////////////////////////////////////////

    /// 6.Characteristic Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &CGMStatusProps
    },

    /// 7.Characteristic Value
    {
      { ATT_BT_UUID_SIZE, CGMStatusUUID },
      GATT_PERMIT_AUTHEN_READ | GATT_PERMIT_ENCRYPT_READ,
      0,
      &CGMStatusDummy
    },

    //////////////////////////////////////////////
    // CGM Session Start Time
    //////////////////////////////////////////////

    /// 8.Characteristic Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      (uint8*)&CGMSessionStartTimeProps
    },

    /// 9.Characteristic Value
    {
      { ATT_BT_UUID_SIZE, CGMSessionStartTimeUUID },
      GATT_PERMIT_AUTHEN_READ |GATT_PERMIT_AUTHEN_WRITE |
      GATT_PERMIT_ENCRYPT_READ | GATT_PERMIT_ENCRYPT_WRITE,
      0,
      &CGMSessionStartTimeDummy
    },

    //////////////////////////////////////////////
    // CGM Session Run Time
    //////////////////////////////////////////////

    /// 10.Characteristic Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &CGMSessionRunTimeProps
    },

    /// 11.Characteristic Value
    {
      { ATT_BT_UUID_SIZE, CGMSessionRunTimeUUID },
      GATT_PERMIT_AUTHEN_READ | GATT_PERMIT_ENCRYPT_READ,
      0,
      (uint8 *)&CGMSessionRunTimeDummy
    },

    //////////////////////////////////////////////
    // RACP Charateristic
    //////////////////////////////////////////////

    /// 12.Characteristic Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &CGMRacpProps
    },

    /// 13.Characteristic Value
    {
      { ATT_BT_UUID_SIZE, recordControlPointUUID },
      GATT_PERMIT_AUTHEN_READ |GATT_PERMIT_AUTHEN_WRITE |
      GATT_PERMIT_ENCRYPT_READ | GATT_PERMIT_ENCRYPT_WRITE,
      0,
      &CGMRacpDummy
    },
    /// 14.Characteristic Configuration
    {
      { ATT_BT_UUID_SIZE, clientCharCfgUUID },
      GATT_PERMIT_AUTHEN_READ |GATT_PERMIT_AUTHEN_WRITE|
      GATT_PERMIT_ENCRYPT_READ |GATT_PERMIT_ENCRYPT_WRITE,
      0,
      (uint8 *)&CGMRacpConfig
    },

    //////////////////////////////////////////////
    // CGM Specific Operation Control Point Charateristic
    //////////////////////////////////////////////

    /// 15.Characteristic Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &CGMControlProps
    },

    /// 16.Characteristic Value
    {
      { ATT_BT_UUID_SIZE, CGMSpecificOpsControlPointUUID },
      GATT_PERMIT_AUTHEN_READ |GATT_PERMIT_AUTHEN_WRITE|
      GATT_PERMIT_ENCRYPT_READ |GATT_PERMIT_ENCRYPT_WRITE,
      0,
      &CGMControlDummy
    },
    
    /// 17.Characteristic Configuration
    {
      { ATT_BT_UUID_SIZE, clientCharCfgUUID },
      GATT_PERMIT_AUTHEN_READ |GATT_PERMIT_AUTHEN_WRITE|
      GATT_PERMIT_ENCRYPT_READ |GATT_PERMIT_ENCRYPT_WRITE,
      0,
      (uint8 *)&CGMControlConfig
    }

};

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static uint8 CGM_ReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr,uint8 *pValue, uint8 *pLen, uint16 offset, uint8 maxLen );
static bStatus_t CGM_WriteAttrCB( uint16 connHandle, gattAttribute_t *pAttr,uint8 *pValue, uint8 len, uint16 offset );
static void CGM_HandleConnStatusCB( uint16 connHandle, uint8 changeType );
bStatus_t CGM_RACPIndicate( uint16 connHandle, attHandleValueInd_t *pInd, uint8 taskId );

/*********************************************************************
 * PROFILE CALLBACKS
 */
/// Service Callbacks
CONST gattServiceCBs_t  CGMCBs =
{
  CGM_ReadAttrCB,   ///< Read callback function pointer
  CGM_WriteAttrCB,  ///< Write callback function pointer
  NULL              ///< Authorization callback function pointer
};


/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      CGM_AddService
 * @brief   Initializes the CGM service by registering
 *          GATT attributes with the GATT server.
 * @param   services - services to add. This is a bit map and can
 * 	    contain more than one service.
 * @return  Success or Failure
 */
bStatus_t CGM_AddService( uint32 services )
{
  uint8 status = SUCCESS;
  // Initialize Client Characteristic Configuration attributes
  GATTServApp_InitCharCfg( INVALID_CONNHANDLE, CGMMeasConfig );
  GATTServApp_InitCharCfg( INVALID_CONNHANDLE, CGMRacpConfig );
  GATTServApp_InitCharCfg( INVALID_CONNHANDLE, CGMControlConfig );
  // Register with Link DB to receive link status change callback
  VOID linkDB_Register( CGM_HandleConnStatusCB );
  if ( services & CGM_SERVICE )
  {
    // Register GATT attribute list and CBs with GATT Server App
    status = GATTServApp_RegisterService( CGMAttrTbl, GATT_NUM_ATTRS( CGMAttrTbl ),&CGMCBs );
  }
  return ( status );
}

/*********************************************************************
 * @fn      CGM_Register
 * @brief   Register a callback function with the CGM Service.
 * @param   pfnServiceCB - Callback function.
 *          pfnCtlPntCB - Callback for control point
 * @return  None.
 */
extern void CGM_Register( CGMServiceCB_t pfnServiceCB)
{
  CGMServiceCB = pfnServiceCB;
}

/*********************************************************************
 * @fn          CGM_MeasSend
 * @brief       Send a CGM measurement via the CGM measurement notification
 * @param       connHandle - connection handle
 * @param       pNoti - pointer to notification structure
 * @return      Success or Failure
 */
bStatus_t CGM_MeasSend( uint16 connHandle, attHandleValueNoti_t *pNoti, uint8 taskId )
{
  uint16 value = GATTServApp_ReadCharCfg( connHandle, CGMMeasConfig );
  // Check if the notification property is enabled for the CGM measurement characteristic
  if ( value & GATT_CLIENT_CFG_NOTIFY )
  {
    // Set the handle. Set the CGM measurement characteristic value as the target to push the notification.
    pNoti->handle = CGMAttrTbl[CGM_MEAS_VALUE_POS].handle;
    // Send the Notiication
    return GATT_Notification( connHandle, pNoti, FALSE );
  }
  return bleNotReady;
}

/*********************************************************************
 * @fn          CGM_CtlPntIndicate
 * @brief       Send an indication containing a control point
 *              message.
 * @param       connHandle - connection handle
 * @param       pInd - pointer to indication structure
 * @return      Success or Failure
 */
bStatus_t CGM_CtlPntIndicate( uint16 connHandle, attHandleValueInd_t *pInd, uint8 taskId )
{
  uint16 value = GATTServApp_ReadCharCfg( connHandle, CGMControlConfig );
  // Check if the indication is enabled for the CGM specific operation control point
  if ( value & GATT_CLIENT_CFG_INDICATE )
  {
    // Set the handle. Set the CGM specific operation control point as the target to push the indication.
    pInd->handle = CGMAttrTbl[CGM_CGM_OPCP_VALUE_POS].handle;
    // Send the Indication
    return GATT_Indication( connHandle, pInd, FALSE, taskId );
  }
  return bleNotReady;
}

/*********************************************************************
 * @fn          CGM_RACPIndicate
 * @brief       Send an indication to the RACP characteristic.
 * @param       connHandle - connection handle
 * @param       pInd - pointer to indication structure
 * @return      Success or Failure
 */
bStatus_t CGM_RACPIndicate( uint16 connHandle, attHandleValueInd_t *pInd, uint8 taskId )
{
  uint16 value = GATTServApp_ReadCharCfg( connHandle, CGMRacpConfig );
  // Check if the RACP characteristic has indication enabled.
  if ( value & GATT_CLIENT_CFG_INDICATE )
  {
    // Set the handle. Set the RACP characteristic as the target to push the indication.
    pInd->handle = CGMAttrTbl[CGM_RACP_VALUE_POS].handle;
    // Send the Indication
    return GATT_Indication( connHandle, pInd, FALSE, taskId );
  }
  return bleNotReady;
}

/*********************************************************************
 * @fn          CGM_ReadAttrCB
 * @brief       The callback function when an attribute is being read by a collector.
 * @param       connHandle - connection message was received on
 * @param       pAttr - pointer to attribute
 * @param       pValue - pointer to data to be read
 * @param       pLen - length of data to be read
 * @param       offset - offset of the first octet to be read
 * @param       maxLen - maximum length of data to be read
 * @return      Success or Failure
 */
static uint8 CGM_ReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                            uint8 *pValue, uint8 *pLen, uint16 offset, uint8 maxLen )
{
  bStatus_t status = SUCCESS;
  // Make sure it's not a blob operation (no attributes in the profile are long)
  if ( offset > 0 )
  {
    return ( ATT_ERR_ATTR_NOT_LONG );
  }
  if ( pAttr->type.len == ATT_BT_UUID_SIZE )
  {
    //get the 16-bit UUID of the attribute being read
    uint16 uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1]);
    switch ( uuid )
    {
      // No need for "GATT_SERVICE_UUID" or "GATT_CLIENT_CHAR_CFG_UUID" cases;
      // gattserverapp handles those types for reads
      // The rest of the cases deal with CGM service specific characteristic read.
      case CGM_FEATURE_UUID:       
        *pLen = CGM_CHAR_VAL_SIZE_FEATURE;
         (*CGMServiceCB)(CGM_FEATURE_READ_REQUEST, pValue, *pLen, (uint8 *)&status );
      break;
     case CGM_STATUS_UUID:
        *pLen = CGM_CHAR_VAL_SIZE_STATUS;
         (*CGMServiceCB)(CGM_STATUS_READ_REQUEST, pValue, *pLen,(uint8 *)&status);
         break;
      case CGM_SES_START_TIME_UUID:
        *pLen = CGM_CHAR_VAL_SIZE_START_TIME;
        (*CGMServiceCB)(CGM_START_TIME_READ_REQUEST, pValue, *pLen,(uint8 *)&status);
      break;
      case CGM_SES_RUN_TIME_UUID:
        *pLen = CGM_CHAR_VAL_SIZE_RUN_TIME;
        (*CGMServiceCB)(CGM_RUN_TIME_READ_REQUEST, pValue, *pLen,(uint8 *)&status);
      break;
      default:
        // Should never get here! (characteristics 3 and 4 do not have read permissions)
        *pLen = 0;
        status = ATT_ERR_ATTR_NOT_FOUND;
        break;
    }
  }
  return ( status );
}

/*********************************************************************
 * @fn      CGM_WriteAttrCB
 * @brief   The CGM service layer callback function when an attribute is being written to by the collector.
 * @param   connHandle - connection message was received on
 * @param   pAttr - pointer to attribute
 * @param   pValue - pointer to data to be written
 * @param   len - length of data
 * @param   offset - offset of the first octet to be written
 * @return  Success or Failure
 */
static bStatus_t CGM_WriteAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                 uint8 *pValue, uint8 len, uint16 offset )
{
  bStatus_t status = SUCCESS;
  // Make sure it's not a blob operation (no attributes in the profile are long)
  if ( offset > 0 )
  {
    return ( ATT_ERR_ATTR_NOT_LONG );
  }
  uint16 uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1]);
  switch ( uuid )
  {
  //When the Client Characteristic Configuration is being written to.
  case  GATT_CLIENT_CHAR_CFG_UUID:
      // Notifications 
      if ((pAttr->handle == CGMAttrTbl[CGM_MEAS_CONFIG_POS].handle)) //Currently in CGM service, only measurement supports notification
      {
	//Change the GATT server variable for the CCC 
        status = GATTServApp_ProcessCCCWriteReq( connHandle, pAttr, pValue, len, offset, GATT_CLIENT_CFG_NOTIFY );
        if ( status == SUCCESS )
        {
          uint16 charCfg = BUILD_UINT16( pValue[0], pValue[1] );
          if(pAttr->handle == CGMAttrTbl[CGM_MEAS_CONFIG_POS].handle)
          {
	    //Invoke the service callback to perform additional tasks in response to the change of CCC. These tasks can be defined in the application layer in cgm.c 
            (*CGMServiceCB)((charCfg == 0) ? CGM_MEAS_NTF_DISABLED : CGM_MEAS_NTF_ENABLED, NULL, NULL,(uint8 *)&status);
          }
        }
      }
      // Indications for RACP Characteristic
      else if ( pAttr->handle == CGMAttrTbl[CGM_RACP_CONFIG_POS].handle ||
              pAttr->handle == CGMAttrTbl[CGM_CGM_OPCP_CONFIG_POS ].handle ) //Currently in CGM service, RACP and OPCP support indication.
      {
	//Change the GATT server variable for CCC
        status = GATTServApp_ProcessCCCWriteReq( connHandle, pAttr, pValue, len,
                                                 offset, GATT_CLIENT_CFG_INDICATE );
        if ( status == SUCCESS )
        {
            uint16 charCfg = BUILD_UINT16( pValue[0], pValue[1] );
	    if(pAttr->handle == CGMAttrTbl[CGM_CGM_OPCP_CONFIG_POS].handle)
	    {
	    //Invoke the service callback to perform additional tasks in response to the change of CCC. These tasks can be defined in the application layer in cgm.c 
            (*CGMServiceCB)((charCfg == 0) ? CGM_CTL_PNT_IND_DISABLED :
                                                 CGM_CTL_PNT_IND_ENABLED, NULL, NULL,(uint8 *)&status);
	    }
	    else if (pAttr->handle == CGMAttrTbl[CGM_RACP_CONFIG_POS].handle)
	    {
	    //Invoke the service callback to perform additional tasks in response to the change of CCC. These tasks can be defined in the application layer in cgm.c 
            	(*CGMServiceCB)((charCfg == 0) ? CGM_RACP_IND_DISABLED :
                                                 CGM_RACP_IND_ENABLED, NULL, NULL,(uint8 *)&status);
	    }
        }
      }
      else
      {
        status = ATT_ERR_INVALID_VALUE;
      }
    break;
    
  case CGM_SES_START_TIME_UUID:
	    //Invoke the service callback to perform tasks in response to value written to start time. These tasks can be defined in the application layer in cgm.c 
       (*CGMServiceCB)(CGM_START_TIME_WRITE_REQUEST, pValue, len, (uint8 *)&status);
    break;
    
  case CGM_SPEC_OPS_CTRL_PT_UUID: //if CGM specific control point is written
      if(len >= CGM_CTL_PNT_MIN_SIZE  && len <= CGM_CTL_PNT_MAX_SIZE)
      {
	//Invoke the service callback to perform tasks in response to value written to OPCP. These tasks can be defined in the application layer in cgm.c 
        (*CGMServiceCB)(CGM_CTL_PNT_CMD, pValue, len, (uint8 *)&status); //call back to APP2SERV to process the command received.
      }
      else
      {
        status = ATT_ERR_INVALID_VALUE_SIZE;
      }
      break;
    
    //If RACP is written to 
  case REC_ACCESS_CTRL_PT_UUID:
      if(len>=CGM_RACP_MIN_SIZE && len<= CGM_RACP_MAX_SIZE)
      {
	      uint8 opcode = pValue[0];
	      //If transfer in progress
	      if (opcode != CTL_PNT_OP_ABORT && cgmMeasDBSendInProgress)
	      {
		      status = CGM_ERR_IN_PROGRESS;
	      }
	      else if ( opcode == CTL_PNT_OP_REQ &&
			      !( GATTServApp_ReadCharCfg( connHandle, CGMRacpConfig) & GATT_CLIENT_CFG_INDICATE))
	      {
		      status = CGM_ERR_CCC_CONFIG;
	      }
	      else
	      {
	    		//Invoke the service callback to perform tasks in response to value written to RACP. These tasks can be defined in the application layer in cgm.c 
	      	      (*CGMServiceCB)(CGM_RACP_CTL_PNT_CMD,pValue,len, (uint8 *) &status);
	      }
      }
      else
      {
	      status = ATT_ERR_INVALID_VALUE_SIZE;
      }
      break;
  default:
      status = ATT_ERR_ATTR_NOT_FOUND;
      break;
  }
  return ( status );
}

/*********************************************************************
 * @fn          glucose_HandleConnStatusCB
 * @brief       Simple Profile link status change handler function.
 * @param       connHandle - connection handle
 * @param       changeType - type of change
 * @return      none
 */
static void CGM_HandleConnStatusCB( uint16 connHandle, uint8 changeType )
{
  // Make sure this is not loopback connection
  if ( connHandle != LOOPBACK_CONNHANDLE )
  {
    // Reset Client Char Config if connection has dropped
    if ( ( changeType == LINKDB_STATUS_UPDATE_REMOVED )      ||
         ( ( changeType == LINKDB_STATUS_UPDATE_STATEFLAGS ) &&
           ( !linkDB_Up( connHandle ) ) ) )
    {
      GATTServApp_InitCharCfg( connHandle, CGMControlConfig );
      GATTServApp_InitCharCfg( connHandle, CGMRacpConfig );
      GATTServApp_InitCharCfg( connHandle, CGMMeasConfig );
    }
  }
}

/*********************************************************************
 * @fn          CGM_SetSendState
 * @brief       Set the state of the CGM database transmission indicator
 * @param       connHandle - connection handle
 * @param       changeType - type of change
 * @return      none
 */
bool CGM_SetSendState(bool input)
{
	cgmMeasDBSendInProgress=input;
	return cgmMeasDBSendInProgress;
}

/*********************************************************************
*********************************************************************/
