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


#define SERVOS			3

static unsigned char  rate[SERVOS];
static unsigned short servo[SERVOS];
static unsigned short input[SERVOS];
static unsigned char  BumperCounter;		
			 
// Servos use Timer 1
void RoboMagellan_InitServos( CLOCK_FREQUENCY clock )
{
	unsigned char i; 
	
  	for( i=0; i<SERVOS; ++i )
  	{ 
  		servo[i] = 6000; 
  		input[i] = 6000; 
  	}   
  	
  	rate[0] = SERVO1_RATE;
	rate[1] = MOTOR_SERVO_RATE;
	rate[2] = SERVO3_RATE;
  		 
	TA1CCR0		= SERVO_NEUTRAL;			// PWM pulse width
	TA1CCR1		= SERVO_NEUTRAL;			// PWM pulse width
	TA1CCR2		= SERVO_NEUTRAL;			// PWM pulse width
	 
	TA1CCTL0 	= 0x0004;		// Mode = 0, OUT = 1
	TA1CCTL1 	= 0x0004;		// Mode = 0, OUT = 1
	TA1CCTL2 	= 0x0004;		// Mode = 0, OUT = 1

	TA1R   		= 0x8FFF;  		// Offset Timer1 Overflow interrupt
	
	// 4 counts / us
	switch( clock )
	{
		case CLOCK_8MHZ:
			TA1CTL 		= 0x0262; 		// Overflow Interrupt Enable, /2, continuous
			break;
		case CLOCK_16MHZ:
			TA1CTL 		= 0x02A2; 		// Overflow Interrupt Enable, /4, continuous
			break;
		case CLOCK_1MHZ:				// 1 Mhz not supported
		default:
			while(1);
	}
	
}

void RoboMagellan_NeutralServo( unsigned char servo )
{
	if( servo < SERVOS ) 
		input[servo] = SERVO_NEUTRAL;
}


void RoboMagellan_CenterSteering( void )
{
	input[STEERING_SERVO] = STEERING_CENTER;
}


// offset is encoded, set 0x80 for minus
// 0x01 = Turn Right
// 0x81 = Turn Left
void RoboMagellan_SetSteeringOffset( unsigned char offset )
{
	volatile unsigned short ServoPosition;
	volatile unsigned short AmplifiedOffset = (offset&0x7f)*STEERING_GAIN;

	ServoPosition = STEERING_CENTER;
	
	if( (STEERING_CENTER + AmplifiedOffset) > STEERING_RIGHT )
		input[STEERING_SERVO] = STEERING_RIGHT;
	else if( (STEERING_CENTER - AmplifiedOffset) < STEERING_LEFT )
		input[STEERING_SERVO] = STEERING_LEFT;
	else if( offset & 0x80 )
		input[STEERING_SERVO] = (STEERING_CENTER + AmplifiedOffset);
	else
		input[STEERING_SERVO] = (STEERING_CENTER - AmplifiedOffset);
		
//	RoboMagellan_SendSerialShort( input[STEERING_SERVO] );
}


void RoboMagellan_InitBumper(void)
{
	// Enable pullup on P2.3
	// 7654|3210
	// 0000|1000
	P2REN |= 0x08;		// Enable Pullus
	P2OUT |= 0x08;		// Enable Pullup
	BumperCounter = 0;
}


unsigned char RoboMagellan_ReadBumper( void )
{
	if( BumperCounter > 5 )
	 return 1;
	return 0; 		
}

// Servos
#pragma vector=TIMER1_A1_VECTOR
__interrupt void Timer1_A1 (void)
{
	volatile unsigned short i;
	
	if( TA1IV == TA1IV_TAIFG )
	{		
		// 7654|3210
		// 0000|1000
		if( P2IN & 0x08 ) // Bumper open
		{
			if( BumperCounter > 0 )
				--BumperCounter;
		}
		else // bumper closed
		{
			if( BumperCounter < 10 )
				++BumperCounter;
		}
			
		// Set to mode 0
		TA1CCTL0 	= 0; //0x0004;
		TA1CCTL1 	= 0; //0x0004;
		TA1CCTL2 	= 0; //0x0004;		
		i = TA1R;
		while( TA1R == i );
		TA1CCR0		= servo[0];
		TA1CCR1		= servo[1];
		TA1CCR2		= servo[2];
		TA1CCTL0 	= 0x0020; // 0x00A4;
		TA1CCTL1 	= 0x0020; // 0x00A4;
		TA1CCTL2 	= 0x0020; // 0x00A4;		

		for( i=0; i<SERVOS; ++i )
  		{ 
  			if( rate[i] > 0 )
  			{
  				if( input[i] > servo[i] )
  					servo[i] = servo[i] + rate[i];
  				if( input[i] < servo[i] )
  					servo[i] = servo[i] - rate[i];
  				if( servo[i] > 10000 ) servo[i] = 10000;
  				if( servo[i] < 2000 ) servo[i] = 2000;
  			}
  			else
  			{
  				servo[i] = input[i];
  			}
  		}
  	}		
}




