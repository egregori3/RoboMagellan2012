//---------------------------------------------------------------------------
//    RoboMagellan Motor control + Ultrasonic Control + Steering
//
//
//	  Written for the EMGRobotics Autonomous Vehicle Booster board for the TI Launchpad. 
//
//	  Written by Eric Gregori ( www.buildsmartrobots.com )
//    Copyright (C) 2012  Eric Gregori
//
//    This program is free software: you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation, either version 3 of the License, or
//    (at your option) any later version.
//
//    This program is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
//    The EMGRobotics Autonomous Vehicle Booster Board (AVBB) plugs into the TI Launchpad.
//	  The AVBB takes in data from a GPS, compass, ultrasonic range sensors, and bumper
//    sensors and translates the data into as motion vector.  The motion vector is then 
//    translated into servo commands to drive standard RC servos.
//
//    The AVBB has three TI MSP430G2553 located on the booster board, with an optional
//    fourth being installed in the Launchpad itself.  The AVBB can be used in a stand-alone
//    mode without the TI Launchpad if a seperate 3 volt power supply is provided, 
//    and USB to serial communications is not required.
//
//---------------------------------------------------------------------------

//******************************************************************************
//   MSP430G2xx3 Demo - USCI_A0, 19200 UART Echo ISR, DCO SMCLK
//
//   Description: Echo a received character, RX ISR used. Normal mode is LPM0.
//   USCI_A0 RX interrupt triggers TX Echo.
//   Baud rate divider with 1MHz = 1MHz/19200 = ~52.1
//   ACLK = n/a, MCLK = SMCLK = CALxxx_1MHZ = 1MHz
//
//                MSP430G2xx3
//             -----------------
//         /|\|              XIN|-
//          | |                 |
//          --|RST          XOUT|-
//            |                 |
//            |     P1.2/UCA0TXD|------------>
//            |                 | 19200 - 8N1
//            |     P1.1/UCA0RXD|<------------
//
//   D. Dang
//   Texas Instruments Inc.
//   February 2011
//   Built with CCS Version 4.2.0 and IAR Embedded Workbench Version: 5.10
//******************************************************************************
#include "RoboMagallan_launchpad_Motor_ultrasonic.h"


static unsigned char RxBuf;

void RoboMagellan_InitSerial(void)
{
	RxBuf = 0;
	// SMCLK = 16
	// UCOS16 = 1
	// UCBRx = 104
	// UCBRSx = 0
	// UCBRFx = 3	
  	UCA0CTL1 |= UCSSEL_2;    // SMCLK
  	UCA0BR0 = 208;           // 16MHZ - 4800
  	UCA0BR1 = 0;             // 16MHz - 4800
  	UCA0MCTL = 0x31;         // UCBRFx=3, UCBRSx=0, UCOS16=1 
  	UCA0CTL1 &= ~UCSWRST;                     // **Initialize USCI state machine**
    IE2 |= UCA0RXIE;                          // Enable USCI_A0 RX interrupt
}

void RoboMagellan_SendSerialByte( unsigned char data )
{
  while (!(IFG2 & UCA0TXIFG));         // USCI_A0 TX buffer ready?
  UCA0TXBUF = data;                    // TX data
}

static void RoboMagellan_SendSerialHex( unsigned char hex )
{
	unsigned char data;
	switch( hex )
	{
		case 0: data = 0x30; break;
		case 1: data = 0x31; break;
		case 2: data = 0x32; break;
		case 3: data = 0x33; break;
		case 4: data = 0x34; break;
		case 5: data = 0x35; break;
		case 6: data = 0x36; break;
		case 7: data = 0x37; break;
		case 8: data = 0x38; break;
		case 9: data = 0x39; break;
		case 10: data = 'A'; break;
		case 11: data = 'B'; break;
		case 12: data = 'C'; break;
		case 13: data = 'D'; break;
		case 14: data = 'E'; break;
		case 15: data = 'F'; break;
		default: data = '?'; break;
	}
	RoboMagellan_SendSerialByte( data );
}

void RoboMagellan_SendSerialShort( unsigned short data )
{
//	RoboMagellan_SendSerialByte( '0' );
//	RoboMagellan_SendSerialByte( 'x' );
	RoboMagellan_SendSerialHex( data >> 12 );
	RoboMagellan_SendSerialHex( (data >> 8) & 0x0f );
	RoboMagellan_SendSerialHex( (data >> 4) & 0x0f );
	RoboMagellan_SendSerialHex( data & 0x0f );		
}

void RoboMagellan_SendSerialNewLine( void )
{
	RoboMagellan_SendSerialByte( 0x0d );	
	RoboMagellan_SendSerialByte( 0x0a );
}

unsigned char RoboMagellan_GetSerialByte( void )
{
	return( RxBuf );
}

#pragma vector=USCIAB0RX_VECTOR
__interrupt void USCI0RX_ISR(void)
{
  	unsigned char data;
  	data = UCA0RXBUF;
  	if( data == '@' )
  		RoboMagellan_ResetJoJo();
  	else
  		RxBuf = data;
}
