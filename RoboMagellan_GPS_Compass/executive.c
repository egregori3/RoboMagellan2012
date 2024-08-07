//---------------------------------------------------------------------------
//    RoboMagellan - Executive / GPS / Compass / Ultrasonic
///
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

extern unsigned char GPS_Bearing;		// 0-180 = 0-360 degrees
extern unsigned char Compass_Bearing;	// 0-180 = 0-360 degrees
extern unsigned char Ultrasound_Offset;	// x01, 0x81 degrees off the nose, 0x80 = backup
extern unsigned char Steering_Offset;	// x01, 0x81 degrees off the nose
extern unsigned char Velocity;			// 'f', 'F', 0, 'b'
extern unsigned char GPS_Status;		// 0, 1
extern unsigned char Vision_Offset;		// 0x01, 0x81 degrees off the nose
extern unsigned char FrontBumper;
extern long DesiredLat; 
extern long DesiredLong;
extern WAYPOINT_WTD waypoint_wtd;


// Local Variables
static unsigned char BackupTimer;
static unsigned char RandomNumber;
static ROBOT_STATES RobotState, PreviousState;


void RoboMagellan_InitRobotStateMachine( void )
{
	RobotState 		= DRIVE_TO_COORDINATE;
	PreviousState 	= SAFETY_STOP;
	BackupTimer 	= 0;	
}


void RoboMagellan_RobotStateMachine( void )
{
	unsigned char temp;
	
	++RandomNumber;
	if( BackupTimer > 0 ) --BackupTimer;
	
	switch( RobotState )
	{
		case GET_NEXT_COORDINATE:
			waypoint_wtd = RoboMagellan_GetNextWaypoint( &DesiredLat, &DesiredLong );
			if( waypoint_wtd == WAYPOINT_OH_SHIT )
			{
				RobotState = SAFETY_STOP;
				Velocity   = 0;
				break;
			}	
			RobotState = DRIVE_TO_COORDINATE;
			break;
		
		case DRIVE_TO_COORDINATE:
// Drive to Coordinate
			if( GPS_Status == GPS_STATUS_IN_WAYPOINT )
			{
				switch( waypoint_wtd )
				{
					case WAYPOINT_NEXT:
						RobotState = GET_NEXT_COORDINATE;
						break;
						
					case WAYPOINT_FIND_CONE:
					case WAYPOINT_LAST_CONE:
						RobotState = FIND_CONE;
						Velocity   = 0;
						break;
						
					default:
						RobotState = SAFETY_STOP;
						Velocity   = 0;
						break;						
				}					
			} // if( GPS_Status == GPS_STATUS_IN_WAYPOINT )
			else
			{
// Drive to Coordinate
#if 0
				if( FrontBumper )
				{
					PreviousState = RobotState;
					RobotState = OBSTACLE_DETECTED;
					break;
				}
#endif
				// Ultrasound offset takes priority
				if( Ultrasound_Offset != ULTRASOUND_NO_DETECTIONS )
				{
					// an obstruction has been detected, steer around it
					if( Ultrasound_Offset == ULTRASOUND_BACKUP )
					{
						// We need to backup
						PreviousState = RobotState;
						RobotState = OBSTACLE_DETECTED;
						break;
					}
					Velocity = 'f';
					Steering_Offset = Ultrasound_Offset;
				}
				else
				{
					// Use compass to track bearing from GPS
					// GPS_Bearing;		// 0-180 = 0-360 degrees
					// Compass_Bearing;	// 0-180 = 0-360 degrees
					// GPS_Bearing = 185, compass = 180 -> turn right
					// GPS_Bearing = 0,   compass = 10  -> turn left
					// GPS_Bearing = 0,   compass = 350 -> turn right
					if( GPS_Bearing > Compass_Bearing )
					{
						// Turn Right
						temp = GPS_Bearing - Compass_Bearing;
					}
					else
					{
						// Turn Left
						temp = Compass_Bearing - GPS_Bearing;
					}
					 
					Velocity = 'f';
		if( Compass_Bearing < 90 ) 
			RoboMagellan_SetSteeringOffset( Compass_Bearing ); // Steering_Offset );
		else
			RoboMagellan_SetSteeringOffset( (180-Compass_Bearing) + 0x80 ); // Steering_Offset );

				}
			}	
			break;
		
		case OBSTACLE_DETECTED:
			RobotState    = RANDOM_BACKUP;
			// Set steering to Random Angle
			if( RandomNumber & 0x04 )
				Steering_Offset = 0x7F;
			else
				Steering_Offset = 0xFE;
				
			// Backup for a random amount of time
			BackupTimer = 30 + (1*(RandomNumber&0x0f));
			Velocity    = 'b';
			break;
			
		case RANDOM_BACKUP:
			if( Ultrasound_Offset != ULTRASOUND_BACKUP )
			{
				Velocity   = 0;
				RobotState = PreviousState; 
			}
				
			if( BackupTimer == 0 )
			{
				Velocity   = 0;
				RobotState = PreviousState; 
			}
			break;
					
		case FIND_CONE:
			Velocity = 'f';
// FIX THIS
//			if( Vision_Offset == 0xFF )
//			{
//				// Cannot see cone
//			}
//			else
				Steering_Offset = Vision_Offset;

			if( FrontBumper )
			{
				RobotState = CONE_FOUND;
				Velocity = 0;
			}		
			break;
		
		case CONE_FOUND:
			Velocity   = 0;
			RobotState = GET_NEXT_COORDINATE;
			break;
		
		default:
		case SAFETY_STOP:
			Velocity   = 0;
			RobotState = DRIVE_TO_COORDINATE;
			break;
	}
}


