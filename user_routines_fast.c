/*******************************************************************************
* FILE NAME: user_routines_fast.c <FRC VERSION>
*
* DESCRIPTION:
*  This file is where the user can add their custom code within the framework
*  of the routines below. 
*
* USAGE:
*  You can either modify this file to fit your needs, or remove it from your 
*  project and replace it with a modified copy. 
*
*
*******************************************************************************/

#include "ifi_aliases.h"
#include "ifi_default.h"
#include "ifi_utilities.h"
#include "user_routines.h"
#include "mort2004.h"


//  M.O.R.T Variables 
int			  left_beacon_count		=	0;			// Variable that contains left beacon count
int			  right_beacon_count	=	0;			// Variable that contains right beacon count
int	  		  left_wheel_count		=	0;			// Variable that contains left wheel count
int	  		  right_wheel_count		=	0;			// Variable that contains right wheel count
int	  		  left_temp_wheel_count		=	0;			// Variable that contains temporary count of left wheel
int	  		  right_temp_wheel_count		=	0;			// Variable that contains temporary count of right wheel
unsigned char left_wheel_status		=	0;			// Variable that contains the status of left wheel
unsigned char right_wheel_status	=	0;			// Variable that contains the status of right wheel

// Defines for possible contents of wheel status variables
#define	SILVER	0
#define	BLACK	1


/*******************************************************************************
* FUNCTION NAME: Process_Data_From_Local_IO
* PURPOSE:       Execute user's realtime code.
* You should modify this routine by adding code which you wish to run fast.
* It will be executed every program loop, and not wait for fresh data 
* from the Operator Interface.
* CALLED FROM:   main.c
* ARGUMENTS:     none
* RETURNS:       void
*******************************************************************************/
void Process_Data_From_Local_IO(void)
{
	track_wheel_position( );		// Update wheel positions
	poll_beacons( );				// Update beacon counts
}


/**************************************************************************************
 Counts the transitions of black to silver markings on each wheel. There are 72 black
 marks so there will be 144 transitions per revolution of each wheel. The wheel diameter
 is 12.5 inches so each pulse represents .273 inch or about 4 transitions per inch.

	circumference = diameter * pi   or  12.5 * 3.14  = 39.25  inches per revolution of wheel
	
	since there are 144 transitions per 39.25 inches, each transition is

	39.25 / 144 = .273 inches
 
-------------------------------------------------------------------------------------*/
void track_wheel_position( void )
{
	// Update position of LEFT WHEEL
	if( left_wheel_status == BLACK )					// If we are looking for BLACK
	{
		if( LEFT_WHEEL_SENSOR == BLACK )				// If sensor is BLACK
		{
			left_temp_wheel_count++;							// Increment tempory count used by test program
			left_wheel_count++;							// Increment left wheel count
			left_wheel_status = SILVER;					// Now we must look for SILVER
		}
	}
	if( left_wheel_status == SILVER )					// If we are looking for SILVER
	{
		if( LEFT_WHEEL_SENSOR == SILVER )				// If sensor is SILVER
		{
			left_temp_wheel_count++;							// Increment tempory count used by test program
			left_wheel_count++;							// Increment left wheel count
			left_wheel_status = BLACK;					// Now we must look for BLACK
		}
	}

	// Update position of RIGHT WHEEL
	if( right_wheel_status == BLACK )					// If we are looking for BLACK
	{
		if( RIGHT_WHEEL_SENSOR == BLACK )				// If sensor is BLACK
		{
			right_temp_wheel_count++;
			right_wheel_count++;						// Increment right wheel count
			right_wheel_status = SILVER;				// Now we must look for SILVER
		}
	}
	if( right_wheel_status == SILVER )					// If we are looking for SILVER
	{
		if( RIGHT_WHEEL_SENSOR == SILVER )				// If sensor is SILVER
		{
			right_temp_wheel_count++;
			right_wheel_count++;						// Increment right wheel count
			right_wheel_status = BLACK;					// Now we must look for black
		}
	}
}


/**************************************************************************************
 Checks if beacon pulses are being received. If sensor is seeing the beacon then
 simply increment a counter. Beacon pulses are only 1 millisecond long for Beacon #1
 and 2 milliseconds for Beacon #2, so checking them in the standard user routine that
 runs every 26.2 milliseconds will miss the pulses. This routine must be called from
 the high speed user_routines_fast function that runs every program loop. Then the slow
 26.2 millisecond loop can check the counts.
-------------------------------------------------------------------------------------*/
void poll_beacons( void )
{
	if( LEFT_IR_SENSOR == 0 )							// If left input is seeing beacon
		left_beacon_count++;							// Increment left beacon count

	if( RIGHT_IR_SENSOR == 0 )							// If right input is seeing beacon
		right_beacon_count++;							// Increment right beacon count
}


/**************************************************************************************
 Clears the beacon counters so high speed routing can increment them when beacon
 pulses are seen.  This function should be called right before exiting the 26.2
 millisecond loop. The next time the 26.2 millisecond user routine runs it can 
 check if the high speed loop incremented the counters
-------------------------------------------------------------------------------------*/
void reset_beacon_counts( void )
{
	left_beacon_count = 0;
	right_beacon_count = 0;
}
