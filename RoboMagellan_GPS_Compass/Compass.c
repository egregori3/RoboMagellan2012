//---------------------------------------------------------------------------
//    RoboMagellan - Executive / GPS / Compass / Ultrasonic
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
#include "RoboMagellan_GPS_Compass.h"

static volatile unsigned short RisingEdge;
static volatile unsigned short PulseTime;

// Comparator connects to CCI1B on timer 0 

// P2CA4 = 0
// P2CA0 = 1
// P2CA3 = 0
// P2CA2 = 0
// P2CA1 = 0
// CAON = 1
// CAEX = 0
// CARSEL = 1
// CAREFx = 10
// CAF = 0

void RoboMagellan_InitCompass(void)
{
	CACTL1 = CAREF_2 + CARSEL + CAON;
	CACTL2 = P2CA0;

	TA0CCTL1 	= 0x5910;		// RisingEdges, CCI1B, SCS, CAP, IE, 
	TA0CTL 		= 0x02e0; 		// SMCLK(8Mhz), /8, continuous, no overflow interrupt
}

// CMPS03 - 1ms (0 degrees) to 36.99ms (359.9 degrees)
// 100us / degree
// Timer is (SMCLK)8Mhz/8 = 1Mhz = 1us = 1 counts/us
// 100 counts/degree
// 0 degrees     = 1000us  = 1000
// 359.9 degrees = 36990us = 36990
// returns 0-360 = 0 to 180
unsigned char RoboMagellan_ReadCompass(void)
{

	unsigned short Degrees;
	
	if( PulseTime < 1000 ) return 0;
	Degrees = PulseTime - 1000;	
	return( Degrees/200 );
}

#pragma vector=TIMER0_A1_VECTOR
__interrupt void Timer0_A1 (void)
{
	volatile unsigned short ta0iv;
	
	ta0iv = TA0IV;		// Reading TA0IV clears it
	
	if( ta0iv == TA0IV_TACCR1 )
	{
		if( TA0CCTL1 & CM0 ) // Rising edge
		{
			RisingEdge = TA0CCR1;
			TA0CCTL1 &= ~CM0;
			TA0CCTL1 |= CM1;
		}
		else
		{
			PulseTime = TA0CCR1 - RisingEdge;
			TA0CCTL1 &= ~CM1;
			TA0CCTL1 |= CM0;
		}				
	}
}



