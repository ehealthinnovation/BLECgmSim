/*!
\file		crc.h
\brief		This file contains the declarations for CRC16.
\author		Harry Qiu
\version        2
\date		2015-March-13
\copyright	MIT License (MIT)\n
 Copyright (c) 2014-2015 Center for Global ehealthinnovation

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions: The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
*/

#ifndef _CRC16_
#define _CRC16_


/*
  @brief CRC-CITT calculation using the tabular form.
  @param [in] message - the pointer to the message array.
  @param [in] length - the length of the message.
  @return the 16-bit CRC value.*/
unsigned short  ccitt_crc16 (unsigned char * message,short length);
/*
 * @brief Test the input message against its attached CRC16 code.
 * @note Here we assume that the CRC is attached as the last two bytes of the message.
 * @param [in] message - pointer to the message character array
 * @param [in] length - the length of the message array
 * @return the result of the test.
 * <table><th><td>Value</td><td>Meaning</td></th>
 * 	  <tr><td>0</td><td>The CRC test is failed</td></tr>
 * 	  <tr><td>1</td><td>The CRC test is passed</td></tr>
 * </table>*/
char ccitt_crc16_test(unsigned char *message,short length);
#endif
