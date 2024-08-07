#ifndef ROBOMAGALLAN_LAUNCHPAD_MOTOR_ULTRASONIC_H_
#define ROBOMAGALLAN_LAUNCHPAD_MOTOR_ULTRASONIC_H_
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
#include "msp430g2553.h"
#include "..\RoboMagellan_Executive_Ultrasonic\EMGRobotics_MSP430G_Init.h"

#define SERVO1_RATE				1
#define MOTOR_SERVO_RATE		0
#define SERVO3_RATE 			1

#define MOTOR_SERVO		1
#define SERVO_NEUTRAL	6000
#define SERVO_MIN		2000	
#define SERVO_MAX		10000

#define BACK_BASE		SERVO_NEUTRAL-((SERVO_NEUTRAL-SERVO_MIN)/4)
#define FORW_BASE		SERVO_NEUTRAL+((SERVO_MAX-SERVO_NEUTRAL)/4)

#define FULL_SPEED		0x10
#define HALF_SPEED		FULL_SPEED/2
 
#define ULTRASOUND_CONTROL_FRONT1		PIN_24
#define ULTRASOUND_CONTROL_FRONT2		PIN_25
#define ULTRASOUND_CONTROL_FRONT3		PIN_26
#define ULTRASOUND_CONTROL_FRONT4		PIN_27
#define ULTRASOUND_CONTROL_BACK			PIN_16

#define JOJO_TIMEOUT	50		// Set to Zero to Disable

void RoboMagellan_NeutralServo( unsigned char servo );
void RoboMagellan_ModServo( unsigned char servo, unsigned char offset );
void RoboMagellan_InitServos( CLOCK_FREQUENCY clock );
void RoboMagellan_ForwThrottleServo( unsigned char offset );
void RoboMagellan_BackThrottleServo( unsigned char offset );
void RoboMagellan_InitForwThrottleServo( void );
void RoboMagellan_InitBackThrottleServo( void );

void RoboMagellan_DisplayServo( void );
									
void RoboMagellan_InitSerial(void);
void RoboMagellan_SendSerialByte( unsigned char data );
void RoboMagellan_SendSerialNewLine( void );
void RoboMagellan_SendSerialShort( unsigned short data );
unsigned char RoboMagellan_GetSerialByte( void );

void RoboMagellan_InitVelocityLoop(void);
unsigned short RoboMagellan_GetVelocity(void);
void RoboMagellan_Velocity_Loop( unsigned char Velocity );

void RoboMagellan_ResetJoJo( void );

#endif /*ROBOMAGALLAN_LAUNCHPAD_MOTOR_ULTRASONIC_H_*/
