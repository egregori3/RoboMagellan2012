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
// The UART is configured fo 4800 baud 8N1
//---------------------------------------------------------------------------
#include "math.h"
#include "RoboMagellan_GPS_Compass.h"

#define START_GPS_READ					0
#define GPS_READ_INPROGRESS				1
#define WAITING_FOR_GGA_HEADER			7
#define GGA_HEADER_RECEIVED				7
#define READ_LAT						0x10
#define WAIT_FOR_LONG					0x20
#define READ_LONG						0x30
#define WAIT_FOR_STATUS					0x40
#define READ_STATUS						0x50
#define GPS_READ_COMPLETE				0xFF


static const char 		gpgga_string[] 	= "$GPGGA,";
static unsigned char 	gps_rx_state 	= 0;
static char 			lat[16];
static char 			lon[16];
static unsigned char    status;


void RoboMagellan_InitGPS( CLOCK_FREQUENCY clock )
{
	// SMCLK = 16
	// UCOS16 = 1
	// UCBRx = 104
	// UCBRSx = 0
	// UCBRFx = 3	
  	UCA0CTL1 |= UCSSEL_2;    // SMCLK
  	
  	switch( clock )
  	{
  		case CLOCK_16MHZ:
  			UCA0BR0 = 208;           // 16MHZ - 4800
  			UCA0BR1 = 0;             // 16MHz - 4800
  			UCA0MCTL = 0x31;         // UCBRFx=3, UCBRSx=0, UCOS16=1
			break;
			
		case CLOCK_8MHZ:
  			UCA0BR0 = 104;           // 8MHZ - 4800
  			UCA0BR1 = 0;             // 8MHz - 4800
  			UCA0MCTL = 0x31;         // UCBRFx=3, UCBRSx=0, UCOS16=1
			break;
		
		default:
			while(1);
  	} 
  	UCA0CTL1 &= ~UCSWRST;    // **Initialize USCI state machine**
    IE2 |= UCA0RXIE;         // Enable USCI_A0 RX interrupt
}


void RoboMagellan_StartGPSRead(void)
{
	gps_rx_state = START_GPS_READ;	
}

unsigned char RoboMagellan_CheckGPSReadDone(void)
{
	if( gps_rx_state == GPS_READ_COMPLETE )
		return 1;
	return 0;
}

// Takes in Lat and Long of desired location
// Output bearing to location
// bearing is degrees/2,  so we output 90, when we actually want 180 degrees.
// $GPGGA,123251.000,4224.3727,N,08800.7229,W,1,08,1.1,229.6,M,-34.1,M,,0000*6B
GPS_STATUS RoboMagellan_GetBearing( 	long LatTo, 
										long LonTo,
										unsigned char *bearing )
{
	volatile long CurrentLat, CurrentLon;
	volatile float degreesTo;
	unsigned short IdegreesTo;
	
#if 1 // This code is required for normal operation
	if( gps_rx_state != GPS_READ_COMPLETE )
		return GPS_STATUS_READ_FAILURE;
		
	if( status == 0 ) return GPS_STATUS_NO_SIGNAL;
	
	if( lat[4] != '.' ) return GPS_STATUS_INVALID_DATA;
	if( lon[5] != '.' ) return GPS_STATUS_INVALID_DATA;
#endif
		
	// calculate bearing to lat/long based on current lat/long
	// Big assumption: integer degrees are same for current and desired position
	// this assumption only applies if navigation is taking place in a small area ( a few blocks )
	// ddmm.mmmm
	// 4224.3727
	CurrentLat  = ((long)(lat[2]-0x30) * (long)100000);
	CurrentLat += ((long)(lat[3]-0x30) * (long)10000);
	CurrentLat += ((long)(lat[5]-0x30) * (long)1000);
	CurrentLat += ((long)(lat[6]-0x30) * (long)100);
	CurrentLat += ((long)(lat[7]-0x30) * (long)10);
	CurrentLat += ((long)(lat[8]-0x30) * (long)1);
	CurrentLat = CurrentLat / (long)60;
	
	// 08800.7229
	CurrentLon  = ((long)(lon[3]-0x30) * (long)100000);
	CurrentLon += ((long)(lon[4]-0x30) * (long)10000);
	CurrentLon += ((long)(lon[6]-0x30) * (long)1000);
	CurrentLon += ((long)(lon[7]-0x30) * (long)100);
	CurrentLon += ((long)(lon[8]-0x30) * (long)10);
	CurrentLon += ((long)(lon[9]-0x30) * (long)1);
	CurrentLon = CurrentLon / (long)60;
	
#if 0 // FOR TEST ONLY!!!!!	
	CurrentLat = 4062;	// REMOVE BEFORE FLIGHT
	CurrentLon = 120;	// REMOVE BEFORE FLIGHT
#endif		

	// Calc difference of current position and desired position
	CurrentLat = LatTo - CurrentLat;
	CurrentLon = LonTo - CurrentLon;
	
	if( (abs(CurrentLat) < 1) && (abs(CurrentLon) < 1) )
		return GPS_STATUS_IN_WAYPOINT;
	
	degreesTo = atan2( CurrentLon, CurrentLat );
	degreesTo *= (180.0/3.14159);
	degreesTo += 360.0;		// Eliminate negative values
	IdegreesTo = degreesTo; 
	IdegreesTo %= 360;
		
	IdegreesTo /= 2.0;
	
	*bearing = (unsigned char)IdegreesTo; 
	return GPS_STATUS_VALID_SIGNAL;
}


// $GPGGA,123251.000,4224.3727,N,08800.7229,W,1,08,1.1,229.6,M,-34.1,M,,0000*6B
#if 1
// Echo back RXed character, confirm TX buffer is ready first
#pragma vector=USCIAB0RX_VECTOR
__interrupt void USCI0RX_ISR(void)
{
  	unsigned char data;
  	data = UCA0RXBUF;
 
 	if( gps_rx_state == GPS_READ_COMPLETE )
 		goto done;
 
 	if( gps_rx_state > GPS_READ_INPROGRESS )
	{
		if( data == '$' )
		{
			gps_rx_state = 0;
			goto done;
		}
	}
 
  	if( gps_rx_state < WAITING_FOR_GGA_HEADER )
  	{
  		if( data != gpgga_string[gps_rx_state++] )
   			gps_rx_state = 0;

		goto done;
  	}

	if( gps_rx_state == GGA_HEADER_RECEIVED )
	{
		if( data == ',' )
		{
			gps_rx_state = READ_LAT;
			goto done;
		}
	}  
	
	if( (gps_rx_state >= READ_LAT) && (gps_rx_state < WAIT_FOR_LONG) )
	{
		if( data == ',' )
		{
			gps_rx_state = WAIT_FOR_LONG;
			goto done;
		}		
		lat[gps_rx_state-READ_LAT] = data;
		gps_rx_state++;
	}
  
	if( gps_rx_state == WAIT_FOR_LONG )
	{
		if( data == ',' )
		{
			gps_rx_state = READ_LONG;
			goto done;
		}
	}  
	
	if( (gps_rx_state >= READ_LONG) && (gps_rx_state < WAIT_FOR_STATUS) )
	{
		if( data == ',' )
		{
			gps_rx_state = WAIT_FOR_STATUS;
			goto done;
		}		
		lon[gps_rx_state-READ_LONG] = data;
		gps_rx_state++;
	}  

	if( gps_rx_state == WAIT_FOR_STATUS )
	{
		if( data == ',' )
		{
			gps_rx_state = READ_STATUS;
			goto done;
		}
	} 
	
	if( gps_rx_state == READ_STATUS )
	{
		status = data;
		gps_rx_state = GPS_READ_COMPLETE;
	}
	
done: 
	data = 0;
}
#endif








