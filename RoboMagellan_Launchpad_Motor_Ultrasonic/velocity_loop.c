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
#include "RoboMagallan_launchpad_Motor_ultrasonic.h"

#define 	Pk 	2

//static volatile unsigned short RisingEdge;
//static volatile unsigned short PulseTime;
static unsigned char Edge_Counter, Current_Velocity, OverFlow_Counter;
static unsigned char Previous_Desired_Velocity; 
static unsigned char Ultrasound_Control_Counter;

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

void RoboMagellan_InitVelocityLoop(void)
{
	CACTL1 = CAREF_2 + CARSEL + CAON + CAIE;
	CACTL2 = P2CA0 + CAF;
	
	P1DIR |= 0x80;
	P1SEL |= 0x80;
	P1SEL2 &= ~0x80;

//	TA0CTL 				= 0x02e2; 		// SMCLK(16Mhz), /8, continuous, overflow interrupt
	TA0CTL 				= 0x0222; 		// SMCLK(16Mhz), /1, continuous, overflow interrupt

	Edge_Counter				= 0;
	Current_Velocity			= 0;
	OverFlow_Counter			= 0;
	Previous_Desired_Velocity 	= 0;
	Ultrasound_Control_Counter	= 0;
}


unsigned short RoboMagellan_GetVelocity(void)
{
	return( Current_Velocity );
}


void RoboMagellan_Velocity_Loop( unsigned char Desired_Velocity )
{
	unsigned char Error;
	unsigned char Offset	= 0;
	unsigned char Desired   = Desired_Velocity & 0x7F;
	unsigned char Direction = Desired_Velocity & 0x80;
	
	EMGRobotics_SetGPIOPin( PIN_15 );
	if( Desired_Velocity == 0 )
	{
		RoboMagellan_NeutralServo( MOTOR_SERVO );
		Previous_Desired_Velocity = 0;
		return;
	}
	
	if( Desired_Velocity != Previous_Desired_Velocity )
	{
		if( (Previous_Desired_Velocity > 0) &&
			(Previous_Desired_Velocity < 0x80) &&
			(Desired_Velocity > 0 ) &&
			(Desired_Velocity < 0x80 ) )
		{
			// Change in forward speed
		}
		else
		{
			// From stop to forward or backward
			// From forward to backward
			// From backward to forward
			if( Direction == 0 )
				RoboMagellan_InitForwThrottleServo();
			else
				RoboMagellan_InitBackThrottleServo();
		}
	} 
	
	Previous_Desired_Velocity = Desired_Velocity;
	
	if( Current_Velocity > Desired )
	{
		// Going to fast
		Error = Current_Velocity - Desired;
		Offset = Error * Pk;
		if( Offset >= 0x80 ) Offset = 0x7f;
		Offset |= 0x80;
		RoboMagellan_SendSerialByte( '-' );
	}		
	else if( Current_Velocity < Desired )
	{
		Error = Desired - Current_Velocity;
		Offset = Error * Pk;
		if( Offset >= 0x80 ) Offset = 0x7f;
		RoboMagellan_SendSerialByte( '+' );
	}
	else
	{
		RoboMagellan_SendSerialByte( '=' );
	}
	
	RoboMagellan_SendSerialByte( ' ' );
	RoboMagellan_SendSerialShort( Offset );
	RoboMagellan_SendSerialByte( ' ' );

	if( Direction == 0 )
	{
		RoboMagellan_SendSerialByte( 'f' );
		RoboMagellan_SendSerialByte( ' ' );
		RoboMagellan_ForwThrottleServo( Offset );
	}
	else
	{
		RoboMagellan_SendSerialByte( 'b' );
		RoboMagellan_SendSerialByte( ' ' );
		RoboMagellan_BackThrottleServo( Offset );
	}
		
	EMGRobotics_ClearGPIOPin( PIN_15 );	
}

#pragma vector=COMPARATORA_VECTOR
__interrupt void CompA (void)
{
	++Edge_Counter;
}


#if 0
void RoboMagellan_Velocity_Loop( unsigned char Desired_Velocity )
{
	unsigned char Error;
	unsigned char Offset;
	unsigned char Desired   = Desired_Velocity & 0x7F;
	unsigned char Direction = Desired_Velocity & 0x80;
	
	EMGRobotics_SetGPIOPin( PIN_15 );
	if( Desired_Velocity == 0 )
	{
		RoboMagellan_NeutralServo( MOTOR_SERVO );
		return;
	}
	
	RoboMagellan_SendSerialShort( Acc_Error );
	RoboMagellan_SendSerialByte( ' ' );		

	if( Current_Velocity > Desired )
	{
		// Going to fast
		Error = Current_Velocity - Desired;
		Acc_Error -= (Error/2);
		if( Acc_Error < -100) Acc_Error = -100;	
		Offset = Error * Pk; // + Acc_Error;
		if( Offset >= 0x80 ) Offset = 0x7f;
		Offset |= 0x80;
		RoboMagellan_SendSerialByte( '-' );
		RoboMagellan_SendSerialByte( ' ' );
		RoboMagellan_SendSerialShort( Offset );
		RoboMagellan_SendSerialByte( ' ' );
		RoboMagellan_ForwThrottleServo( Offset );
	}		
	else if( Current_Velocity < Desired )
	{
		Error = Desired - Current_Velocity;
		Acc_Error += (Error/2);
		if( Acc_Error > 100 ) Acc_Error = 100;
		Offset = Error * Pk; // + Acc_Error;
		if( Offset >= 0x80 ) Offset = 0x7f;
		RoboMagellan_SendSerialByte( '+' );
		RoboMagellan_SendSerialByte( ' ' );
		RoboMagellan_SendSerialShort( Offset );
		RoboMagellan_SendSerialByte( ' ' );
		RoboMagellan_ForwThrottleServo( Offset );
	}
	else
	{
		RoboMagellan_SendSerialByte( '=' );
		RoboMagellan_SendSerialByte( ' ' );
		RoboMagellan_SendSerialShort( Offset );
		RoboMagellan_SendSerialByte( ' ' );
		if( Acc_Error > 0 )
		{
			if( Acc_Error > 10 )
			 	Acc_Error -= 10;
			else
				Acc_Error = 0;
		}
		else if( Acc_Error < 0 )
		{
			if( Acc_Error < -10 )
			 	Acc_Error += 10;
			else
				Acc_Error = 0;
		}		
	}
	
	EMGRobotics_ClearGPIOPin( PIN_15 );	
}
#endif

// Timer0 Overflow Interrupt (TA0IV_TAIFG)
// Occurs every 65535/2000000 = 0.0327675 seconds
#pragma vector=TIMER0_A1_VECTOR
__interrupt void Timer0_A1 (void)
{
	volatile unsigned short ta0iv;
	
	ta0iv = TA0IV;		// Reading TA0IV clears it
	
	if( ta0iv == TA0IV_TAIFG )
	{
		if( ++OverFlow_Counter > (3*8) )
		{
			OverFlow_Counter = 0;
			Current_Velocity = Edge_Counter;
			Edge_Counter = 0;
		}
	}
	
	// Inverted Logic, going through transistor
	EMGRobotics_SetGPIOPin( ULTRASOUND_CONTROL_FRONT1 );
	EMGRobotics_SetGPIOPin( ULTRASOUND_CONTROL_FRONT2 );
	EMGRobotics_SetGPIOPin( ULTRASOUND_CONTROL_FRONT3 );
	EMGRobotics_SetGPIOPin( ULTRASOUND_CONTROL_FRONT4 );
	EMGRobotics_SetGPIOPin( ULTRASOUND_CONTROL_BACK );
	
	// Bring high for 20uS or more to command a range reading.
	switch( Ultrasound_Control_Counter++ )
	{
		case 10:
			EMGRobotics_ClearGPIOPin( ULTRASOUND_CONTROL_FRONT1 );
			break;
		case 20:
			EMGRobotics_ClearGPIOPin( ULTRASOUND_CONTROL_FRONT2 );
			break;	
		case 30:
			EMGRobotics_ClearGPIOPin( ULTRASOUND_CONTROL_FRONT3 );
			break;				
		case 40:
			EMGRobotics_ClearGPIOPin( ULTRASOUND_CONTROL_FRONT4 );
			break;				
		case 50:
			EMGRobotics_ClearGPIOPin( ULTRASOUND_CONTROL_BACK );
			Ultrasound_Control_Counter = 0;
			break;			
	}
}

