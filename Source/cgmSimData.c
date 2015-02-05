/**************************************************************************************************
Filename:       cgmSimData.c

Revised:        Date: 2015-Feb-4
Revision:       Revision:  0

Description:    This file contains the CGM sensor simulator glucose data generation function.

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

#include "cgmsimdata.h"
#define CGM_SIM_DATA_RECORD_NUMBER 	145
#define CGM_SIM_DATA_OFFSET		80

static unsigned char index;

const unsigned char cgmData[CGM_SIM_DATA_RECORD_NUMBER]={
50,92,88,80,74,74,80,88,86,92,92,94,96,100,104,108,114,110,94,76,66,66,62,58,56,58,60,64,64,66,66,68,72,72,74,78,80,84,88,120,128,136,140,144,150,152,152,156,160,164,164,168,172,176,180,184,186,190,192,196,198,200,202,206,204,202,196,186,190,192,208,230,228,226,220,210,206,202,200,200,204,208,208,204,202,200,198,198,200,124,114,120,118,116,112,110,102,92,78,64,52,42,42,56,58,62,68,74,78,80,76,72,68,62,60,52,40,26,14,6,4,6,12,20,30,34,26,22,22,24,26,30,44,50,56,64,66,64,60,56,46,30,22,10,0};

void	 cgmSimDataReset(void)
{
	index=0;
}

unsigned short cgmGetNextData(void)
{	
	unsigned short temp;
	temp=cgmData[index]+CGM_SIM_DATA_OFFSET;
	index++;
	index %= CGM_SIM_DATA_RECORD_NUMBER;
	return temp;
}


	
