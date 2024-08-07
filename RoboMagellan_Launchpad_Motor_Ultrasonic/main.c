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
#include "..\RoboMagellan_Executive_Ultrasonic\RoboMagellan.h"
#include "RoboMagallan_launchpad_Motor_ultrasonic.h"

typedef enum	{
					STOP,
					FORWARD,
					BACKWARD,
					CHANGEDIR
				} MOTION_CONTROL_STATES;

typedef enum	{
					DO_INIT,
					DO_STOP,
					DO_FORWARD_FULL,
					DO_BACKWARD,
					DO_FORWARD_HALF
				} MOTION_CONTROL_SIGNALS;
				
const unsigned char PinConfiguration[] = {
											FUNCTION_COMP,				// Pin 2  - CA0(SpeedSensor)
											FUNCTION_UART,				// Pin 3  - RXD
											FUNCTION_UART,				// Pin 4  - TXD
											FUNCTION_ANALOG,			// Pin 5  - Ultrasound Reverse
											FUNCTION_ANALOG,			// Pin 6  - Front Bumper
											FUNCTION_GPIOOUT,			// Pin 7  - NOT USED
											FUNCTION_GPIOOUT,			// Pin 8  - TO GUC
											FUNCTION_GPIOOUT,			// Pin 9  - TO GUC
											FUNCTION_TIMEROUT,			// Pin 10 - Servo - Throttle
											FUNCTION_GPIOOUT,			// Pin 11 - NOT USED
											FUNCTION_GPIOOUT,			// Pin 12 - Ultrasonic Control (F)
											FUNCTION_GPIOOUT,			// Pin 13 - Ultrasonic control (F)
											FUNCTION_GPIOOUT,			// Pin 14 - Ultrasonic control (REV)
											FUNCTION_GPIOOUT,			// Pin 15 - NOT USED
											FUNCTION_NA,				// Pin 16 - DEBUG
											FUNCTION_NA,				// Pin 17 - DEBUG
											FUNCTION_GPIOOUT,			// Pin 18 - Ultrasonic control (B)
											FUNCTION_GPIOOUT			// Pin 19 - Ultrasonic control (B)
										};

// returns Desired_Velocity, Input command
// Command is a single ASCII character
// 2 forward speeds, 1 backward speed, stop
// A heartbeat character should be sent every xms to kick the dog
static unsigned char Motion_State_Variable;
static unsigned char WatchDog;
static unsigned char Change_Delay;
unsigned char Motion_State_Machine( unsigned char Command )
{
	MOTION_CONTROL_SIGNALS signal;
	unsigned char Desired_Velocity = 0;
	
	++WatchDog;
	signal = DO_INIT;
	switch( Command )
	{
		case 'f':	// Forward Half
			signal = DO_FORWARD_HALF;
			break;
			
		case 'F':	// Forward Full
			signal = DO_FORWARD_FULL;
			break;
			
		case 'b':	// Backward
			signal = DO_BACKWARD;
			break;	
			
		default:	// stop
			signal = DO_STOP;
	}
#if JOJO_TIMEOUT	
	if( WatchDog >= JOJO_TIMEOUT )
	{
		Motion_State_Variable = STOP;
		WatchDog =  JOJO_TIMEOUT;
	}
#endif	
	switch( Motion_State_Variable )
	{
		default:
			Motion_State_Variable = STOP;
			signal = DO_STOP;
			
		case STOP:
			Change_Delay = 0;
			Desired_Velocity = 0;
			if( (signal == DO_FORWARD_HALF) || (signal == DO_FORWARD_FULL) ) 
				Motion_State_Variable = FORWARD;
			else if( signal == DO_BACKWARD )
				Motion_State_Variable = BACKWARD;
			break;
			
		case FORWARD:
			Change_Delay = 0;
			if( signal == DO_FORWARD_FULL )
				Desired_Velocity = FULL_SPEED;
			else if( signal == DO_FORWARD_HALF )
				Desired_Velocity = HALF_SPEED;
			else if( signal == DO_BACKWARD )
			{
				Desired_Velocity = 0;
				Motion_State_Variable = CHANGEDIR;
			}
			else
			{
				Desired_Velocity = 0;
				Motion_State_Variable = STOP;
			}
			break;				

		case BACKWARD:
			Change_Delay = 0;
			if( signal == DO_BACKWARD )
				Desired_Velocity = HALF_SPEED+0x80;
			else if( (signal == DO_FORWARD_HALF) || (signal == DO_FORWARD_FULL) )
			{
				Desired_Velocity = 0;
				Motion_State_Variable = CHANGEDIR;
			}
			else
			{
				Desired_Velocity = 0;
				Motion_State_Variable = STOP;
			}
			break;				
			
		
		case CHANGEDIR:
			Desired_Velocity = 0;
			// Need a delay
			if( ++Change_Delay > 5 )
				Motion_State_Variable = STOP;
			break;	
	}
	
	return Desired_Velocity;
} 			
	
void RoboMagellan_ResetJoJo( void )	
{
	WatchDog = 0;
}
			
#define FOR_TESTING		0
										
void main(void)
{	
	unsigned short 	Velocity;
	unsigned char 	Desired_Velocity = 10;
	unsigned char 	wait = 0xff;
	
	Motion_State_Variable = STOP;
	Change_Delay		  = 0;
	
	EMGRobotics_DisableWDT();
	EMGRobotics_WDTInterval();				// At 16Mhz interval is 2ms
	EMGRobotics_SetClock( CLOCK_16MHZ );
	EMGRobotics_ConfigurePins( (unsigned char*)PinConfiguration );
//	RoboMagellan_InitI2C();
	RoboMagellan_InitServos( CLOCK_16MHZ );		
	RoboMagellan_InitSerial();
	RoboMagellan_InitVelocityLoop();
  	EMGRobotics_EnableInterrupts();
  	
#if FOR_TESTING
	for( wait=200; wait; --wait )
	{
		RoboMagellan_SendSerialByte( 'W' );
		RoboMagellan_SendSerialByte( 'a' );
		RoboMagellan_SendSerialByte( 'i' );
		RoboMagellan_SendSerialByte( 't' );
		RoboMagellan_SendSerialByte( 'i' );
		RoboMagellan_SendSerialByte( 'n' );
		RoboMagellan_SendSerialByte( 'g' );
		RoboMagellan_SendSerialNewLine();
		EMGRobotics_SpinWaitingForIntervalTimer(100/2);
	}
#endif
	  	
	while( 1 )
	{
		EMGRobotics_SpinWaitingForIntervalTimer(100/2);

		RoboMagellan_SendSerialByte( '>' ); // Beeper tickle
#if FOR_TESTING
		RoboMagellan_DisplayServo();
#endif		
		Desired_Velocity = Motion_State_Machine( RoboMagellan_GetSerialByte() );
		RoboMagellan_Velocity_Loop( Desired_Velocity );

#if FOR_TESTING		
		RoboMagellan_SendSerialShort( Desired_Velocity );
		RoboMagellan_SendSerialByte( ' ' );		
		Velocity = RoboMagellan_GetVelocity();
		RoboMagellan_SendSerialShort( Velocity );
		RoboMagellan_SendSerialByte( ' ' );
#endif		
		RoboMagellan_SendSerialNewLine();  // Beeper tickle
	}
}
