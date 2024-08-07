#ifndef ROBOMAGELLAN_GPS_COMPASS_H_
#define ROBOMAGELLAN_GPS_COMPASS_H_
#include "msp430g2553.h"
#include "..\RoboMagellan_Executive_Ultrasonic\EMGRobotics_MSP430G_Init.h"


#define SERVO1_RATE				1
#define MOTOR_SERVO_RATE		0
#define SERVO3_RATE 			1

#define STEERING_SERVO			1
#define SERVO_NEUTRAL			6000
#define SERVO_MIN				2000	
#define SERVO_MAX				10000

#define STEERING_LEFT			3500
#define STEERING_CENTER			5000
#define STEERING_RIGHT			6500
#define STEERING_GAIN			((STEERING_RIGHT-STEERING_LEFT)/90)

// Ultrasound
#define AN_CHANNEL7		0
#define AN_CHANNEL6		1
#define AN_CHANNEL5		2
#define AN_CHANNEL4		3
#define AN_CHANNEL3		4
#define AN_CHANNEL2		5
#define AN_CHANNEL1		6
#define AN_CHANNEL0		7

// A4 = Front Left
// A5 = Back Right
// A6 = Back Left
// A7 = Front right
#define ULTRASOUND_FRONT_LEFT	AN_CHANNEL4
#define ULTRASOUND_FRONT_RIGHT	AN_CHANNEL7
#define ULTRASOUND_BACK_LEFT	AN_CHANNEL6
#define ULTRASOUND_BACK_RIGHT	AN_CHANNEL5

#define ULTRASONIC_THRESHOLD	0x100
#define ULTRASOUND_BACKUP		0xFF
#define ULTRASOUND_TURN_RIGHT	0x7F
#define ULTRASOUND_TURN_LEFT	0xFE
#define ULTRASOUND_NO_DETECTIONS 0

typedef enum {
				GET_NEXT_COORDINATE,
				DRIVE_TO_COORDINATE,
				OBSTACLE_DETECTED,
				RANDOM_BACKUP,
				FIND_CONE,
				CONE_FOUND,
				SAFETY_STOP
			} ROBOT_STATES;

typedef enum {
				GPS_STATUS_NO_SIGNAL,
				GPS_STATUS_READ_FAILURE,
				GPS_STATUS_INVALID_DATA,
				GPS_STATUS_IN_WAYPOINT,
				GPS_STATUS_VALID_SIGNAL
			 } GPS_STATUS;

typedef enum {
				WAYPOINT_NEXT,
				WAYPOINT_FIND_CONE,
				WAYPOINT_LAST_CONE,
				WAYPOINT_OH_SHIT
			 } WAYPOINT_WTD;

typedef struct {
					long lat;
					long lon;
					WAYPOINT_WTD  wtd;
				} WAYPOINTS;

// Init
void RoboMagellan_InitCompass(void);
void RoboMagellan_InitGPS( CLOCK_FREQUENCY clock );
void RoboMagellan_InitUltrasound(void);
void RoboMagellan_InitServos( CLOCK_FREQUENCY clock );
WAYPOINT_WTD RoboMagellan_InitNavigationMap( long *lat, long *lon );
void RoboMagellan_InitRobotStateMachine( void );
void RoboMagellan_InitUltrasound(void);
void RoboMagellan_InitBumper(void);

// Read Sensors
unsigned char RoboMagellan_GetUltrasoundOffset(void);
signed char RoboMagellan_GetVisionOffset( void );
unsigned char RoboMagellan_ReadCompass(void);
void RoboMagellan_StartGPSRead(void);
unsigned char RoboMagellan_CheckGPSReadDone(void);
GPS_STATUS RoboMagellan_GetBearing( 	long LatTo, 
										long LonTo,
										unsigned char *bearing );
unsigned char RoboMagellan_GetStallStatus( void );
void RoboMagellan_ReadUltraSounds( 	unsigned short *us1, 
									unsigned short *us2,
									unsigned short *us3,
									unsigned short *us4											
								);
								
unsigned char RoboMagellan_ReadBumper( void );

// Robot State Machine
WAYPOINT_WTD RoboMagellan_GetNextWaypoint( long *lat, long *lon );
void RoboMagellan_RobotStateMachine( void );
				
// Output
void RoboMagellan_SetSteeringOffset( unsigned char );
void RoboMagellan_SendMotorCommand( unsigned char );
void RoboMagellan_CenterSteering(void);					
	
// Serial TX
void RoboMagellan_SendSerialByte( unsigned char data );
void RoboMagellan_SendSerialShort( unsigned short data );
void RoboMagellan_SendSerialNewLine( void );
					
// Vision
unsigned char RoboMagellan_ReadAndroidVision( void );
//void RoboMagellan_ReadAndroidVision( unsigned short *a, unsigned short *b );

#endif /*ROBOMAGELLAN_GPS_COMPASS_H_*/
