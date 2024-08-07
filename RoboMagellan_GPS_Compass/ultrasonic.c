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


static unsigned volatile short AD_Data[8];  // Please note, data goes out of sync when debugging


// Channel 	-> Memory Location
// 7		-> 0
// 6		-> 1
// 5		-> 2
// 4		-> 3
// 3		-> 4
// 2		-> 5
// 1		-> 6
// 0		-> 7
void RoboMagellan_ReadUltraSounds( 	unsigned short *us1, 
									unsigned short *us2,
									unsigned short *us3,
									unsigned short *us4											
								) 
{
	*us1 = AD_Data[AN_CHANNEL4];
	*us2 = AD_Data[AN_CHANNEL5];
	*us3 = AD_Data[AN_CHANNEL6];
	*us4 = AD_Data[AN_CHANNEL7];
}


void RoboMagellan_InitUltrasound(void)
{

	// Init Analog ultrasonic sensors
  	ADC10CTL1 = INCH_7 + CONSEQ_3;            // A7-A0, repeat multi channel, ADC10SC bit=start, ADC10OSC
  	// vcc/gnd, 16 sample/hold, continuous, ADCON
  	ADC10CTL0 = ADC10SHT_2 + MSC + ADC10ON; // VR+=VCC, VR-=GND, continuous,  + ADC10IE;  
 	ADC10DTC0 = ADC10CT;
 	ADC10DTC1 = 8;							// INCH + 1
    ADC10CTL0 &= ~ENC;    
    while (ADC10CTL1 & BUSY);               // Wait if ADC10 core is active
	ADC10SA = (unsigned int)AD_Data;
    ADC10CTL0 |= ENC + ADC10SC;             // Sampling and conversion start
}


// Returns:
// 0    if nothing is seen
// 0x7f if clear on right
// 0xFE if clear on left
// 0xFF if both front sensors are blocked
unsigned char RoboMagellan_GetUltrasoundOffset(void)
{
	unsigned char data, retval;
	
	data   = 0;
	retval = 0;
	
	if( AD_Data[ULTRASOUND_FRONT_LEFT] < ULTRASONIC_THRESHOLD )
		data |= 1;
	if( AD_Data[ULTRASOUND_FRONT_RIGHT] < ULTRASONIC_THRESHOLD )
		data |= 2;
	if( AD_Data[ULTRASOUND_BACK_LEFT] < ULTRASONIC_THRESHOLD )
		data |= 4;
	if( AD_Data[ULTRASOUND_BACK_RIGHT] < ULTRASONIC_THRESHOLD )
		data |= 8;
		
	// Could use a table here as well
	switch( data )
	{
		case 0:		// No detections
			retval = ULTRASOUND_NO_DETECTIONS;
			break;
			
		default:
		case 0x03: // 0011
		case 0x07: // 0111
		case 0x0b: // 1011
		case 0x0f: // 1111
		case 0x06: // 0110 - Front Right, Back Left
		case 0x09: // 1001 - Front Left, Back Right
		case 0x0c: // 1100 - Back Left, Back Right
			// Both fronts blocked
			retval = ULTRASOUND_BACKUP; // Backup
			break;
			
		case 0x01: // 0001 - See something on LEFT
		case 0x04: // 0100
		case 0x05: // 0101
			retval = ULTRASOUND_TURN_RIGHT; // Clear on right
			break;
			
		case 0x02: // 0010 - RIGHT
		case 0x08: // 1000
		case 0x0a: // 1010
			retval = ULTRASOUND_TURN_LEFT; // Clear on left
			break;
	}	
	
	return retval;
}

// 0xff - Cannot see cone
// 0x7F - Cone on Right
// 0xFE - Cone on Left
// 0    - Cone in Front
unsigned char RoboMagellan_ReadAndroidVision( void )
{
	unsigned char data = 0;
	
	EMGRobotics_SetGPIOPin( PIN_27 );
	EMGRobotics_ClearGPIOPin( PIN_26 );
	EMGRobotics_SpinWaitingForIntervalTimer(3);
	if( AD_Data[AN_CHANNEL3] < 0x200 )
		data |= 1;
		 
	EMGRobotics_ClearGPIOPin( PIN_27 );
	EMGRobotics_SetGPIOPin( PIN_26 );
	EMGRobotics_SpinWaitingForIntervalTimer(3);
	if( AD_Data[AN_CHANNEL3] < 0x200 )		
		data |= 2;
		
	if( data == 0x03 ) return 0;
	if( data == 0x01 ) return 0x7F;
	if( data == 0x02 ) return 0xFE;
	return 0xFF; 	
}

