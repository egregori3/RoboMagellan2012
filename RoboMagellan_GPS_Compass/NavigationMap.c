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

static unsigned char MapIndex;
// This file contains the navigation array.
// Each entry is a waypoint containing a lat/long and flags.
// The flags determine what to expect at the waypoint;
//  Nothing at this waypoint
//  A cone at this wypoint
//  This is the last waypoint
// More can be added later if needed

//
// 4 DECIMAL PLACES ONLY!!!!!!!
// NO LEADING ZEROS
// Lat = 42.40629, Lon = -88.01200  -> 4063, -120
// Lat = 42.40630, Lon = -88.01232	-> 4063, -123
// Lat = 42.40606. Lon = -88.01246  -> 4061, -125
// Lat = 42.40657, Lon = -88.01241  -> 4066, -124
static const WAYPOINTS    NavigationMap[] = {  //  lat,    long,    What To Do
										{ 689322, 83123, WAYPOINT_NEXT },
										{ 0,      0,     WAYPOINT_OH_SHIT }
									 };
									 
WAYPOINT_WTD RoboMagellan_InitNavigationMap( long *lat, long *lon )
{
	*lat = NavigationMap[0].lat;
	*lon = NavigationMap[0].lon;
	if( NavigationMap[0].lat == 0 || NavigationMap[0].lon == 0 ||
		NavigationMap[0].wtd == WAYPOINT_OH_SHIT )
		return WAYPOINT_OH_SHIT;
		
	MapIndex = 1;
	
	return NavigationMap[0].wtd;
}									 
									 
WAYPOINT_WTD RoboMagellan_GetNextWaypoint( long *lat, long *lon )
{
	*lat = NavigationMap[MapIndex].lat;
	*lon = NavigationMap[MapIndex].lon;
	if( NavigationMap[MapIndex].lat == 0 || NavigationMap[MapIndex].lon == 0 ||
		NavigationMap[MapIndex].wtd == WAYPOINT_OH_SHIT )
		return WAYPOINT_OH_SHIT;
		
	return NavigationMap[MapIndex++].wtd;
}								 


