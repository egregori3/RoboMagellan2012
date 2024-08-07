//---------------------------------------------------------------------------
//    RoboMagellan - Executive / GPS / Compass / Ultrasonic
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
//
//---------------------------------------------------------------------------
#include "..\RoboMagellan_Executive_Ultrasonic\RoboMagellan.h"
#include "RoboMagellan_GPS_Compass.h"

const unsigned char GPSCompassPinConfiguration[] = {
											FUNCTION_COMP,				// Pin 2  - CA0(COMPASS)
											FUNCTION_UART,				// Pin 3  - RXD
											FUNCTION_UART,				// Pin 4  - TXD
											FUNCTION_ANALOG,			// Pin 5  - From Vision
											FUNCTION_ANALOG,			// Pin 6  - Ultrasound in (F)
											FUNCTION_ANALOG,			// Pin 7  - Ultrasound in (B)
											FUNCTION_GPIOIN,			// Pin 8  - From Motor
											FUNCTION_GPIOIN,			// Pin 9  - From Motor
											FUNCTION_TIMEROUT,			// Pin 10 - Servo - Steering
											FUNCTION_GPIOIN,			// Pin 11 - From Bumper
											FUNCTION_GPIOOUT,			// Pin 12 - NOT USED
											FUNCTION_GPIOOUT,			// Pin 13 - Beeper
											FUNCTION_ANALOG,			// Pin 14 - Ultrasound in (B)
											FUNCTION_ANALOG,			// Pin 15 - Ultrasound in (F)
											FUNCTION_NA,				// Pin 16 - DEBUG
											FUNCTION_NA,				// Pin 17 - DEBUG
											FUNCTION_GPIOOUT,			// Pin 18 - To Vision
											FUNCTION_GPIOOUT			// Pin 19 - To Vision
										};
	
unsigned char GPS_Bearing;		// 0-180 = 0-360 degrees
unsigned char Compass_Bearing;	// 0-180 = 0-360 degrees
unsigned char Ultrasound_Offset;// 0x01, 0x81 degrees off the nose
unsigned char Vision_Offset;	// 0x01, 0x81 degrees off the nose
unsigned char Steering_Offset;	// +- 127 degrees off the nose
unsigned char Velocity;			// 'f', 'F', 0, 'b'
unsigned char FrontBumper;
GPS_STATUS GPS_Status;			// 

// Lat = 42.40629, Lon = -88.01200  -> 4063, -120
// Lat = 42.40630, Lon = -88.01232	-> 4063, -123
// Lat = 42.40606. Lon = -88.01246  -> 4061, -125
// Lat = 42.40657, Lon = -88.01241  -> 4066, -124					
long DesiredLat					= 4061; 
long DesiredLong				= -125;

WAYPOINT_WTD waypoint_wtd;

#define FOR_TESTING		0

void RoboMagellan_BeeperOn( void )
{
	// 7654|3210
	// 0010|0000
	P2OUT |= 0x20;
}


void RoboMagellan_BeeperOff( void )
{
	// 7654|3210
	// 0010|0000
	P2OUT &= ~0x20;
}
														
void main(void)
{	
	unsigned char i;
//	unsigned short test1, test2;
	
	EMGRobotics_DisableWDT();
	EMGRobotics_WDTInterval();				// At 16Mhz interval is 2ms
	EMGRobotics_SetClock( CLOCK_16MHZ );
	BCSCTL2 = 2;	// SMCLK = 8Mhz
	EMGRobotics_ConfigurePins( (unsigned char*)GPSCompassPinConfiguration );
	
	RoboMagellan_InitGPS( CLOCK_8MHZ );
	RoboMagellan_InitCompass();
	RoboMagellan_InitServos( CLOCK_8MHZ );  // SMCLK = 8Mhz
	RoboMagellan_InitUltrasound();
	RoboMagellan_InitBumper();	
	
	// Prime the pump
// FOR DEBUGGING	waypoint_wtd = RoboMagellan_InitNavigationMap( &DesiredLat, &DesiredLong );
	RoboMagellan_InitRobotStateMachine();
  	EMGRobotics_EnableInterrupts();

	FrontBumper			= 0;

	// Offsets
	Ultrasound_Offset	= 0;

	// Bearings
	GPS_Bearing			= 0;
	Compass_Bearing		= 0;
	GPS_Status			= GPS_STATUS_NO_SIGNAL;	

	// Output
	Steering_Offset		= 0;
	Velocity			= 0;
	
#if FOR_TESTING
		GPS_Bearing = 100/2;	
#endif


// Offset = +-127 degrees angle off the nose
// Bearing = Absolute compass angle in degrees/2 (360 degrees = 180)
// Velocity = 'f', 'F', 0, 'b'
// Loop  	
  	// Start GPS Read
  	// Wait x ms
  	// Read Compass Offset
  	// Read Ultrasound Bearing
  	// Read Vision Offset
  	// Read Stall Status
 	// Read GPS Bearing for next loop
 	
  	// Call Robot State Machine
  	
  	// Set Steering Offset
  	// Send Velocity to Motor
// Goto Loop  	  	
  	  	
  	RoboMagellan_StartGPSRead();  
  	RoboMagellan_CenterSteering();	
  	
  	// Wait 10 seconds before moving
  	RoboMagellan_BeeperOn();
  	for( i=0; i<100; ++i )
 		EMGRobotics_SpinWaitingForIntervalTimer(100/2);
  	 	
	while( 1 )
	{
		RoboMagellan_BeeperOff();
		EMGRobotics_SpinWaitingForIntervalTimer(100/2);
		
		RoboMagellan_SendSerialByte( '@' );		// Kick the dog on the MP
		Compass_Bearing 	= RoboMagellan_ReadCompass();
// ENABLE FOR FLIGHT		Ultrasound_Offset	= RoboMagellan_GetUltrasoundOffset();
		
		if( Ultrasound_Offset != 0 )
			RoboMagellan_BeeperOn();

		if( RoboMagellan_CheckGPSReadDone() )
		{
// TESTING			GPS_Status = RoboMagellan_GetBearing( DesiredLat, DesiredLong, &GPS_Bearing );
			RoboMagellan_StartGPSRead();
		}
		
#if FOR_TESTING
		RoboMagellan_SendSerialByte( '>' );
		RoboMagellan_SendSerialShort( Compass_Bearing );
//		RoboMagellan_SendSerialShort( GPS_Status );
		RoboMagellan_SendSerialShort( GPS_Bearing );
		RoboMagellan_SendSerialShort( Vision_Offset );
//		RoboMagellan_ReadAndroidVision( &test1, &test2 );
#endif		

		Vision_Offset 	= RoboMagellan_ReadAndroidVision( );
		FrontBumper 	= RoboMagellan_ReadBumper();
		RoboMagellan_RobotStateMachine();
		RoboMagellan_SetSteeringOffset( Steering_Offset );
		RoboMagellan_SendSerialByte( Velocity );

#if FOR_TESTING
		RoboMagellan_SendSerialShort( Steering_Offset );
		RoboMagellan_SendSerialNewLine();
#endif
	}
}
