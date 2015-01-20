#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

//constants define
#define		uint8		uint8_t
#define		uint16		uint16_t
#define		uint24		uint32_t
#define		uint32		uint32_t
#define		int8		int8_t
#define		int16		int16_t
#define		int32		int32_t
// values for CGM meas flags
#define	CGM_TREND_INFO_PRES			0x01		// CGM Trend Information Present
#define	CGM_QUALITY_PRES			0x02		// CGM Quality Present
#define	CGM_STATUS_ANNUNC_WARNING_OCT		0x20		// Sensor Status Annuciation Field, Warning Octet present
#define	CGM_STATUS_ANNUNC_CAL_TEMP_OCT		0x40		// Sensor Status Annuciation Field, Cal/Temp-Octet present
#define	CGM_STATUS_ANNUNC_STATUS_OCT		0x80		// Sensor Status Annuciation Field, Status Octet present


#define CGM_MEAS_DB_SIZE			254

#define CTL_PNT_OP_REQ                  	0x01		//Report stored record (Operator: Value from Operator Table)
#define CTL_PNT_OP_CLR                       	0x02		//Delete stored records (Operator: Value from Operator Table)
#define CTL_PNT_OP_ABORT                     	0x03		//Abort operation (Operator: Null 'value of 0x00 from Operator Table')
#define CTL_PNT_OP_GET_NUM                   	0x04		//Report number of stored records (Operator: Value from Operator Table)
#define CTL_PNT_OP_NUM_RSP                   	0x05		//Number of stored records response (Operator: Null 'value of 0x00 from Operator Table')
#define CTL_PNT_OP_REQ_RSP                   	0x06		//Response Code (Operator: Null 'value of 0x00 from Operator Table')

//Record Control Point operator
#define CTL_PNT_OPER_NULL                   	0x00		// Null
#define CTL_PNT_OPER_ALL                     	0x01  		// All records
#define CTL_PNT_OPER_LESS_EQUAL              	0x02		// Less than or equal to 
#define CTL_PNT_OPER_GREATER_EQUAL           	0x03		// Greater than or equal to
#define CTL_PNT_OPER_RANGE                   	0x04		// Within range of (inclusive)
#define CTL_PNT_OPER_FIRST                   	0x05		// First record(i.e. oldest record)
#define CTL_PNT_OPER_LAST                    	0x06  		// Last record (i.e. most recent record)

//Record Control Point Response Codes
#define CTL_PNT_RSP_SUCCESS                	0x01		// Normal response for successful operation
#define CTL_PNT_RSP_OPCODE_NOT_SUPPORTED     	0x02		// Normal response if unsupported Op Code is received
#define CTL_PNT_RSP_OPER_INVALID             	0x03		// Normal response if Operator received does not meet the requirements of the service (e.g. Null was expected)
#define CTL_PNT_RSP_OPER_NOT_SUPPORTED       	0x04		// Normal response if unsupported Operator is received
#define CTL_PNT_RSP_OPERAND_INVALID          	0x05		// Normal response if Operand received does not meet the requirements of the service
#define CTL_PNT_RSP_NO_RECORDS              	0x06		// Normal response if request to report stored records or request to delete stored records resulted in no records meeting criteria.
#define CTL_PNT_RSP_ABORT_FAILED             	0x07		// Normal response if request for Abort cannot be completed
#define CTL_PNT_RSP_PROC_NOT_CMPL            	0x08		// Normal response if unable to complete a procedure for any reason
#define CTL_PNT_RSP_FILTER_NOT_SUPPORTED     	0x09		// Normal response if unsupported Operand is received

//Record Control Point Resposne Filter
#define CTL_PNT_FILTER_TIME_OFFSET             	0x01		// Time Offset

//RACP Search Function Response Code
#define RACP_SEARCH_RSP_SUCCESS			0x01		//the search is successful
#define RACP_SEARCH_RSP_NO_RECORD		0x02		//there is no record matching the criteria
#define RACP_SEARCH_RSP_FAIL			0x03		//the search is failed
#define	RACP_SEARCH_RSP_INVALID_OPERAND		0x04		// the operand is not valid





typedef struct {
  uint8         size;
  uint8         flags;
  uint16        concentration;
  uint16        timeoffset;
  uint24        annunication;
  uint16        trend;
  uint16        quality;
} cgmMeasC_t;

//function prototype
static void cgmNewGlucoseMeas(cgmMeasC_t * pMeas);
static void cgmSimAPPInit();							//initialize the simulation a
static uint8_t cgmSearchMeasDB(uint8_t filter,uint16_t operand1, uint16_t operand2); // seach for the required range


//Local Variables
//the most current measurement
static cgmMeasC_t       cgmCurrentMeas;
static cgmMeasC_t	cgmPreviousMeas;

static uint16		cgmCommInterval=1000	;

static cgmMeasC_t *	cgmMeasDB;
static uint8		cgmMeasDBWriteIndx;
static uint8		cgmMeasDBCount;
static uint8		cgmMeasDBOldestIndx;
static uint8		cgmMeasDBSearchStart; //the starting index record meeting the search criteria
static uint8		cgmMeasDBSearchEnd;   //the end index of record meeting the search criteria
static uint8		cgmMeasDBSearchNum;   //the resulting record number that matches the criteria
static uint8		cgmMeasDBSendIndex;   //the index of the next record to be sent



  

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
  uint16	offset_dif=1; //for calculating trend
  int32		trend_cal; //the signed version for calculation
  
  uint16        quality;
 // UTCTime currentTime, startTime;
  static uint32 offset=0;

  // Store the current CGM measurement
  glucosePreviousGen=glucoseGen;

  //get the glucose information
  glucoseGen+=4; 
  glucoseGen =( glucoseGen % 0x07FD);
  pMeas->concentration=glucoseGen;
  
  //get the time offset
 // currentTime=osal_getClock();
  //startTime=osal_ConvertUTCSecs(&cgmStartTime.startTime);
 // if (currentTime>startTime)
 // {     
 //     offset=currentTime-startTime;
 //     if (offset > 0x0000FFFF) //UTCTime counts in second, target is minute
 //       offset=0;
 //     else
       pMeas->timeoffset=(++offset) & 0xFFFF; //EXTRA needs here second minute conflict
//  }
  
  //get the flag information
  flag++; //simulate all flag situation
  flag |= CGM_TREND_INFO_PRES;
   pMeas->flags= (flag);  
 
  
  //trend information
  if(offset!=0)
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
  

static void cgmSimAPPInit()							//initialize the simulation a
{
	cgmMeasDB=(cgmMeasC_t *)malloc(CGM_MEAS_DB_SIZE);
}


static uint8_t cgmSearchMeasDB(uint8_t filter,uint16_t operand1, uint16_t operand2)
{
	uint8_t i=0;
        uint8_t j=0;
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
				uint8_t startindx,endindx,recordnum;
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

	return RACP_SEARCH_RSP_FAIL;	
}
 

int cgmAddRecord()
{
	cgmNewGlucoseMeas( &cgmCurrentMeas);
	cgmMeasDBWriteIndx=(cgmMeasDBOldestIndx+cgmMeasDBCount)%CGM_MEAS_DB_SIZE;
	if ((cgmMeasDBWriteIndx==cgmMeasDBOldestIndx) && cgmMeasDBCount>0)
		cgmMeasDBOldestIndx=(cgmMeasDBOldestIndx+1)%CGM_MEAS_DB_SIZE;
	memcpy(cgmMeasDB+cgmMeasDBWriteIndx,&cgmCurrentMeas,sizeof(cgmMeasC_t));
	cgmMeasDBCount++;
	if (cgmMeasDBCount>CGM_MEAS_DB_SIZE)
		cgmMeasDBCount=CGM_MEAS_DB_SIZE;
	return 0;
}	

void printBuffer(uint8_t * pbuffer, uint8_t len)
{
	uint8_t i;
	for (i=0;i<len;i++)
		printf("%02x ",*(pbuffer+i));
	if (pbuffer==(uint8_t *)(cgmMeasDB+cgmMeasDBSearchStart))
		printf("<------Start");
	if( pbuffer ==(uint8_t *) (cgmMeasDB+cgmMeasDBSearchEnd))
		printf("<------End");
	if( pbuffer ==(uint8_t *) (cgmMeasDB+cgmMeasDBOldestIndx))
		printf("<------History");
	printf("\n");
}		

void printDB(cgmMeasC_t *pcgmDB, uint8_t record_num)
{	
	uint8_t i;
	for (i=0;i<record_num;i++)
		printBuffer((uint8_t *)(pcgmDB+i),sizeof(cgmMeasC_t));
		
	printf("\n");

}	


void testscript1()
{	uint8_t result;
	cgmSimAPPInit();
	uint16_t i;
	for (i=0;i<400;i++)
		{cgmAddRecord();
		}
//test first record
	printf("The first record\n");
	result=cgmSearchMeasDB(CTL_PNT_OPER_FIRST,0,0);
	printDB(cgmMeasDB,CGM_MEAS_DB_SIZE);
	printf("The total number of records returned is %u\n",cgmMeasDBSearchNum);
	printf("Start Index %u\n",cgmMeasDBSearchStart);
	printf("End Inde %u \n", cgmMeasDBSearchEnd);
	printf("return code from search function: %u\n", result);
	printf("Oldest Indx:%u\n",cgmMeasDBOldestIndx);
	printf("Record Count:%u\n",cgmMeasDBCount);
	printf("\n");

//test last record
	printf("The last record\n");
	result=cgmSearchMeasDB(CTL_PNT_OPER_LAST,0,0);
	printDB(cgmMeasDB,CGM_MEAS_DB_SIZE);
	printf("The total number of records returned is %u\n",cgmMeasDBSearchNum);
	printf("Start Index %u\n",cgmMeasDBSearchStart);
	printf("End Inde %u \n", cgmMeasDBSearchEnd);
	printf("return code from search function: %u\n", result);
	printf("Oldest Indx:%u\n",cgmMeasDBOldestIndx);
	printf("Record Count:%u\n",cgmMeasDBCount);
	printf("\n");
	

//test less than
	// All
	printf("All less  record\n");
	result=cgmSearchMeasDB(CTL_PNT_OPER_LESS_EQUAL,405,0);
	printDB(cgmMeasDB,CGM_MEAS_DB_SIZE);
	printf("The total number of records returned is %u\n",cgmMeasDBSearchNum);
	printf("Start Index %u\n",cgmMeasDBSearchStart);
	printf("End Inde %u \n", cgmMeasDBSearchEnd);
	printf("return code from search function: %u\n", result);
	printf("Oldest Indx:%u\n",cgmMeasDBOldestIndx);
	printf("Record Count:%u\n",cgmMeasDBCount);
	printf("\n");

	// Normal
	printf("part of record less  record\n");
	result=cgmSearchMeasDB(CTL_PNT_OPER_LESS_EQUAL,300,0);
	printDB(cgmMeasDB,CGM_MEAS_DB_SIZE);
	printf("The total number of records returned is %u\n",cgmMeasDBSearchNum);
	printf("Start Index %u\n",cgmMeasDBSearchStart);
	printf("End Inde %u \n", cgmMeasDBSearchEnd);
	printf("return code from search function: %u\n", result);
	printf("Oldest Indx:%u\n",cgmMeasDBOldestIndx);
	printf("Record Count:%u\n",cgmMeasDBCount);
	printf("\n");

	// No record
	printf("No record less record\n");
	result=cgmSearchMeasDB(CTL_PNT_OPER_LESS_EQUAL,138,0);
	printDB(cgmMeasDB,CGM_MEAS_DB_SIZE);
	printf("The total number of records returned is %u\n",cgmMeasDBSearchNum);
	printf("Start Index %u\n",cgmMeasDBSearchStart);
	printf("End Inde %u \n", cgmMeasDBSearchEnd);
	printf("return code from search function: %u\n", result);
	printf("Oldest Indx:%u\n",cgmMeasDBOldestIndx);
	printf("Record Count:%u\n",cgmMeasDBCount);
	printf("\n");

//test range
	// All
	printf("All the rand record\n");
	result=cgmSearchMeasDB(CTL_PNT_OPER_RANGE,130,410);
	printDB(cgmMeasDB,CGM_MEAS_DB_SIZE);
	printf("The total number of records returned is %u\n",cgmMeasDBSearchNum);
	printf("Start Index %u\n",cgmMeasDBSearchStart);
	printf("End Inde %u \n", cgmMeasDBSearchEnd);
	printf("return code from search function: %u\n", result);
	printf("Oldest Indx:%u\n",cgmMeasDBOldestIndx);
	printf("Record Count:%u\n",cgmMeasDBCount);
	printf("\n");

	// Normal
	printf("Normal range  record\n");
	result=cgmSearchMeasDB(CTL_PNT_OPER_RANGE,150,300);
	printDB(cgmMeasDB,CGM_MEAS_DB_SIZE);
	printf("The total number of records returned is %u\n",cgmMeasDBSearchNum);
	printf("Start Index %u\n",cgmMeasDBSearchStart);
	printf("End Inde %u \n", cgmMeasDBSearchEnd);
	printf("return code from search function: %u\n", result);
	printf("Oldest Indx:%u\n",cgmMeasDBOldestIndx);
	printf("Record Count:%u\n",cgmMeasDBCount);
	printf("\n");
	
	printf("Normal range  record, Hit Ceiling\n");
	result=cgmSearchMeasDB(CTL_PNT_OPER_RANGE,150,410);
	printDB(cgmMeasDB,CGM_MEAS_DB_SIZE);
	printf("The total number of records returned is %u\n",cgmMeasDBSearchNum);
	printf("Start Index %u\n",cgmMeasDBSearchStart);
	printf("End Inde %u \n", cgmMeasDBSearchEnd);
	printf("return code from search function: %u\n", result);
	printf("Oldest Indx:%u\n",cgmMeasDBOldestIndx);
	printf("Record Count:%u\n",cgmMeasDBCount);
	
	printf("Normal range  record, Hit Floor\n");
	result=cgmSearchMeasDB(CTL_PNT_OPER_RANGE,120,300);
	printDB(cgmMeasDB,CGM_MEAS_DB_SIZE);
	printf("The total number of records returned is %u\n",cgmMeasDBSearchNum);
	printf("Start Index %u\n",cgmMeasDBSearchStart);
	printf("End Inde %u \n", cgmMeasDBSearchEnd);
	printf("return code from search function: %u\n", result);
	printf("Oldest Indx:%u\n",cgmMeasDBOldestIndx);
	printf("Record Count:%u\n",cgmMeasDBCount);

	// No Record
		// all record greater
	printf("No record, all greater  record\n");
	result=cgmSearchMeasDB(CTL_PNT_OPER_RANGE,1,10);
	printDB(cgmMeasDB,CGM_MEAS_DB_SIZE);
	printf("The total number of records returned is %u\n",cgmMeasDBSearchNum);
	printf("Start Index %u\n",cgmMeasDBSearchStart);
	printf("End Inde %u \n", cgmMeasDBSearchEnd);
	printf("return code from search function: %u\n", result);
	printf("Oldest Indx:%u\n",cgmMeasDBOldestIndx);
	printf("Record Count:%u\n",cgmMeasDBCount);
	printf("\n");

		// all records smaller
	printf(" No record all smaller\n");
 result=	cgmSearchMeasDB(CTL_PNT_OPER_RANGE,420,700);
	printDB(cgmMeasDB,CGM_MEAS_DB_SIZE);
	printf("The total number of records returned is %u\n",cgmMeasDBSearchNum);
	printf("Start Index %u\n",cgmMeasDBSearchStart);
	printf("End Inde %u \n", cgmMeasDBSearchEnd);
	printf("return code from search function: %u\n", result);
	printf("Oldest Indx:%u\n",cgmMeasDBOldestIndx);
	printf("Record Count:%u\n",cgmMeasDBCount);
	printf("\n");

}	



int main (){
	uint8_t result;
	cgmSimAPPInit();
	uint16_t i;
	testscript1();
	return 0;
	
}
