/*******************************************************************************
* MODULE NAME: autonomous_mode.c
*
* DESCRIPTION:
*  This module contains routines used in Autonomous mode
*
* CHANGE LOG:
* 1/15/2004		GROUP		Initial Creation
* 1/30/2004		GROUP		Coded and tested MODE 1 which is Intelligent Drive to 10pt ball
* 2/??/2004		GROUP		Coded and Tested Modes 1, 2 on stage
* 2/??/2004		GROUP		Coded and Tested Modes 3, 4 , played in hallway, inconsistant beacon sensing
* 2/16/2004     GROUP		Coded modes 5 & 6. Tested modes 3 & 4 on rug upstairs, worked out
	`						approach for pivoting robot to knock ball off.. Very Sucessful !!!
* 3/19/2004					Wrote codes 8 and 9 and tweaked
*
*******************************************************************************/
#include "ifi_aliases.h"
#include "ifi_default.h"
#include "ifi_utilities.h"
#include "user_routines.h"
#include "printf_lib.h"
#include "mort2004.h"					// Defines and aliases for I/O used in our robot

/*******************************************************************************/
// Variables needed and used in this module
int auto_mode 				= 	0;		// Used by switch statement to tell us what part of autonomous mode we are in
int arm_motor_counter       =   0;


/*******************************************************************************/
// Defines used for this module


/*******************************************************************************
* FUNCTION NAME: User_Autonomous_Code
* PURPOSE:       Execute user's code during autonomous robot operation.
* You should modify this routine by adding code which you wish to run in
* autonomous mode.  It will be executed every program loop, and not
* wait for or use any data from the Operator Interface.
* CALLED FROM:   main.c file, main() routine when in Autonomous mode
* ARGUMENTS:     none
* RETURNS:       void
*******************************************************************************/
void User_Autonomous_Code(void)
{
	mode_switch = read_selector_switch( );	// Read selector switch into mode_switch variable

	printf("Entering Autonomous mode ");
	if( mode_switch == 1 )
		printf("1\n");
	if( mode_switch == 2 )
		printf("2\n");
	if( mode_switch == 3 )
		printf("3\n");
	if( mode_switch == 4 )
		printf("4\n");
	if( mode_switch == 5 )
		printf("5\n");
	if( mode_switch == 6 )
		printf("6\n");
	if( mode_switch == 7 )
		printf("7\n");

	left_wheel_count = 0;
	right_wheel_count = 0;

	while (autonomous_mode)   				// DO NOT CHANGE!
  	{
		poll_beacons( );					// Count beacon pulses	
		track_wheel_position( );			// Update wheel position counts
    	if (statusflag.NEW_SPI_DATA)      	// 26.2ms loop area 
    	{
        	Getdata(&rxdata);  				// DO NOT DELETE, or you will be stuck here forever!

        	/* Add your own autonomous code here. */
			// M.O.R.T mode detection
			if((mode_switch == 1) || (mode_switch == 2))
				Autonomous_mode_1or2( );
			if((mode_switch == 3) || (mode_switch == 4))
				Autonomous_mode_3or4( );
			if((mode_switch == 5) || (mode_switch == 6))
				Autonomous_mode_5or6( );
			if(mode_switch == 7)
				Autonomous_mode_7( );
			if((mode_switch == 8) ||(mode_switch == 9))
				Autonomous_mode_8or9();


    		reset_beacon_counts( );			// Clear beacon counts
	       	Putdata(&txdata);   			// DO NOT DELETE, or you will get no PWM outputs!
 		}
  	}
}

// M.O.R.T MODES #1 and #2
/*******************************************************************************
* FUNCTION NAME: Autonomous_mode_1or2
* PURPOSE:       Executes a 2 turn maneuver using the wheel position sensors
*				 NOTE:  IR Beacon is NOT used in this function
*       
* CALLED FROM:   main.c
* ARGUMENTS:     none
* RETURNS:       The state of autonomous mode
*				 If we return AUTO_MODE_DONE, the main program will control the robot normally	
*-------------------------------------------------------------------------------
//    *
//      *
// 	      *     FINAL COUNT
//	        *     FINAL SPEED
//			  *    
//			    *        TURN2 COUNT
//		        *  <---  TURN2 SPEED
//		        *
//		        *
//		        *
//		        *	LANE COUNT
//		        *	LANE SPEED
//		        *
//		        *
//		        *         TURN1 COUNT
//		        *  <---   TURN1 SPEED
//		          *
//		  	        *      GATE COUNT
//				      *      GATE SPEED
//						*
//----------------------------------------------------------------------------------------*/

#define MODE1_GATE_SPEED			  1			// Speed out of the gate
#define MODE1_GATE_COUNT  	   	     30			// Number of wheel counts before first turn

#define MODE1_TURN1_SPEED			 120			// Speed to change wheel by in the first turn - one up, one down
#define MODE1_TURN1_LEFT_COUNT 	    130			// Number of wheel counts to turn the robot LEFT  in TURN1
#define MODE1_TURN1_RIGHT_COUNT 	130			// Number of wheel counts to turn the robot RIGHT in TURN1

#define MODE1_LANE_SPEED			  1			// Speed that both wheels travel down lane
#define MODE1_LANE_COUNT			550			// Number of wheel counts during lane travel

#define MODE1_HANG_ARM_HEIGHT        60         // How high the arm moves in auton mode 
#define MODE1_HANG_ARM_SPEED          1         // How fast the arm moves in auton mode

#define MODE1_TURN2_SPEED			 120			// Speed to change wheel by in the second turn - one up, one down
#define MODE1_TURN2_LEFT_COUNT	    130			// Number of wheel counts to turn the robot LEFT  in TURN2
#define MODE1_TURN2_RIGHT_COUNT	    130			// Number of wheel counts to turn the robot RIGHT in TURN2

// defines for GIANT switch statement below
#define MODE1_TURN1_RIGHT		1
#define	MODE1_TURN1_LEFT		2
#define MODE1_LANE				3
#define MODE1_RAISE_ARM         4
#define	MODE1_TURN2_LEFT		5
#define	MODE1_TURN2_RIGHT		6

void Autonomous_mode_1or2(void)
{
	LEFT_WHEEL  = 127;						// Assume we want to stop LEFT_WHEEL.  This value may be changed below
	RIGHT_WHEEL = 127;						// Assume we want to stop RIGHT_WHEEL. This value may be changed below

	switch( auto_mode )						// Which part of autonomous_mode_1 are we in ???
	{
		/*******************************************************************************/
		// Drive ROBOT straight out the gate till GATE_DISTANCE is reached
		case 0:

		LEFT_WHEEL  = MODE1_GATE_SPEED;			// Output GATE_SPEED to LEFT_WHEEL
		RIGHT_WHEEL = MODE1_GATE_SPEED;			// Output GATE_SPEED to RIGHT_WHEEL
		// Wait for robot to reach destination
		if( left_wheel_count > MODE1_GATE_COUNT)	// If we are at destination
		{
			left_wheel_count  = 0;			// Reset left wheel count
			right_wheel_count = 0;			// Reset right wheel count
			if( mode_switch == 1 )			// If we are on the left side of the field
			{	
				auto_mode = MODE1_TURN1_LEFT;	// Turn left
			}
			else
			{
				auto_mode = MODE1_TURN1_RIGHT;		// Turn right
			}
		}
		break;


		/*******************************************************************************/
		// Turn ROBOT to the RIGHT until TURN1_COUNT is reached
		// Used in autonomous mode # 1 right side of field
		case MODE1_TURN1_RIGHT:
		LEFT_WHEEL = 127 + MODE1_TURN1_SPEED;			// Keep LEFT_WHEEL moving at GATE_SPEED
		RIGHT_WHEEL = 127 - MODE1_TURN1_SPEED;			// Slow down RIGHT_WHEEL to turn
		// Wait for robot to finish turning Right
		if( left_wheel_count > MODE1_TURN1_RIGHT_COUNT )// If count has been reached
		{
			left_wheel_count = 0;			// Reset left wheel count
			right_wheel_count = 0;			// Reset right wheel count
			auto_mode = MODE1_LANE;			// Next State for switch statment
		}
		break;								// Exit switch statement


		/*******************************************************************************/
		// Turn ROBOT to the LEFT until TURN1_COUNT is reached
		// Used in autonomous mode # 2 left side of field
		case MODE1_TURN1_LEFT:
		RIGHT_WHEEL = 127 + MODE1_TURN1_SPEED;			// Keep RIGHT_WHEEL moving at GATE_SPEED
		LEFT_WHEEL = 127 - MODE1_TURN1_SPEED;			// Slow down LEFT_WHEEL to turn
		// Wait for robot to finish turning Right
		if( right_wheel_count > MODE1_TURN1_LEFT_COUNT )// If count has been reached
		{
			printf( "Going down the Lane\n");
			left_wheel_count = 0;			// Reset left wheel count
			right_wheel_count = 0;			// Reset right wheel count
			auto_mode = MODE1_LANE;			// Next State for switch statment
		}
		break;								// Exit switch statement


		/********************************************************************************/
		// Send ROBOT Straight ahead down the lane until LANE_COUNT is reached
		case MODE1_LANE:
		LEFT_WHEEL  = MODE1_LANE_SPEED;			// Output LANE_SPEED to LEFT_WHEEL
		RIGHT_WHEEL = MODE1_LANE_SPEED;			// Output LANE SPEED to RIGHT_WHEEL
		// Wait for robot to reach destination
		if( left_wheel_count > MODE1_LANE_COUNT)  // If count has been reached
		{
			left_wheel_count = 0;			// Reset left wheel count
			right_wheel_count = 0;			// Reset right wheel count
			auto_mode = MODE1_RAISE_ARM;	// Turn Left
		}
		break;								// Exit switch statement
		

		case MODE1_RAISE_ARM:
		HANG_MOTOR = MODE1_HANG_ARM_SPEED;
		arm_motor_counter++;
		if(arm_motor_counter > MODE1_HANG_ARM_HEIGHT)
		{
			HANG_MOTOR = 127;
			if( mode_switch == 1 )			// If we are on the left side of the field
			{	
				auto_mode = MODE1_TURN2_RIGHT;	// Turn right
				arm_motor_counter = 0;
			}
			else
			{
				auto_mode = MODE1_TURN2_LEFT;		// Turn Left
				arm_motor_counter = 0;
			}
		} 
		break;
			
		/*******************************************************************************/
		// Turn ROBOT to the LEFT until TURN2_COUNT is reached
		// Used in autonomous mode # 1 left side of field
		case MODE1_TURN2_LEFT:
		RIGHT_WHEEL = 127 + MODE1_TURN2_SPEED;			// Keep RIGHT_WHEEL at LANE_SPEED
		LEFT_WHEEL = 127 - MODE1_TURN2_SPEED;          // Slow down LEFT_WHEEL to turn   
		// Wait for robot to finish turning Left
		if( right_wheel_count > MODE1_TURN2_LEFT_COUNT )// If count has been reached
		{
			left_wheel_count = 0;			// Reset left wheel count
			right_wheel_count = 0;			// Reset right wheel count
			auto_mode = AUTO_MODE_DONE;
		}
		break;								// Exit switch statement


		/*******************************************************************************/
		// Turn ROBOT to the RIGHT until TURN2_COUNT is reached
		// Used in autonomous mode # 2 right side of field
		case MODE1_TURN2_RIGHT:
		LEFT_WHEEL = 127 + MODE1_TURN2_SPEED;			// Keep LEFT_WHEEL at LANE_SPEED
		RIGHT_WHEEL = 127 - MODE1_TURN2_SPEED;			// Slow down RIGHT_WHEEL to turn   
		// Wait for robot to finish turning Right
		if( left_wheel_count > MODE1_TURN2_RIGHT_COUNT )// If count has been reached
		{
			left_wheel_count = 0;			// Reset left wheel count
			right_wheel_count = 0;			// Reset right wheel count
			auto_mode = AUTO_MODE_DONE;			// Next State for switch statement
		}
		break;								// Exit switch statement

	}
}


// M.O.R.T MODES #3 and #4
/*******************************************************************************
* FUNCTION NAME: Autonomous_mode_3or4
* PURPOSE:       Executes a 1 turn maneuver using the wheel position sensors
*				 NOTE:  IR Beacons ARE used in this function
*       
* CALLED FROM:   main.c
* ARGUMENTS:     none
* RETURNS:       The state of autonomous mode
*				 If we return AUTO_MODE_DONE, the main program will control the robot normally	
*-------------------------------------------------------------------------------
//	* 
//	*
//	* ++++
//	* ++++
//	* ++++	Polling For Beacon
//	* ++++
//	* ++++
//	*
//	*
//	* <---- SLOWDOWN_SPEED @ SLOWDOWN_COUNT
//	*
//	*
//	*
//	*
//	*
//	* 		FINAL COUNT @ FINAL SPEED
//	*
//	*
//	*
//	*
//	*
//	*
//	*
//	*
//	*
//  *         ARC RIGHT COUNT
//  *  <---   ARC SPEED
//   *
//     *
//       *      
//		   *      GATE2 COUNT
//           *      GATE2 SPEED
//     	       *
//		         *
//		           *      
//		   	         *       
//				       *

//----------------------------------------------------------------------------------------*/


//Defines for robot movements in auton modes 3 and 4

	#define MODE3_GATE_SPEED		      1			// Speed out of the gate
	#define MODE3_GATE_COUNT    		1050		// Number of wheel counts before first turn
	#define MODE3_HANG_ARM_SPEED         90
	#define MODE3_HANG_ARM_HEIGHT       70

	#define MODE3_TURN1_SPEED			120			// Speed reduced to wheel to turn robot for TURN1
	#define MODE3_TURN1_LEFT_COUNT   	120			// Number of wheel counts to turn the robot LEFT  in TURN1
	#define MODE3_TURN1_RIGHT_COUNT 	120			// Number of wheel counts to turn the robot RIGHT in TURN1

//Defines for switch
	#define MODE3_TURN1_LEFT       1
	#define MODE3_TURN1_RIGHT      2


void Autonomous_mode_3or4( void )
{
	LEFT_WHEEL  = 127;						// Assume we want to stop LEFT_WHEEL.  This value may be changed below
	RIGHT_WHEEL = 127;						// Assume we want to stop RIGHT_WHEEL. This value may be changed below

	switch( auto_mode )						// Which part of autonomous_mode_1 are we in ???
	{
		/*******************************************************************************/
		// Drive ROBOT straight out the gate till GATE2_COUNT is reached
		case 0:
		LEFT_WHEEL  = MODE3_GATE_SPEED;			// Output GATE2_SPEED to LEFT_WHEEL
		RIGHT_WHEEL = MODE3_GATE_SPEED;			// Output GATE2_SPEED to RIGHT_WHEEL
		HANG_MOTOR = 127;
		arm_motor_counter++;
		printf("counter at %d\n", arm_motor_counter);
		if(arm_motor_counter < MODE3_HANG_ARM_HEIGHT)
		{
			HANG_MOTOR = MODE3_HANG_ARM_SPEED;
		}
		else
			{
			HANG_MOTOR = 127;
			}
		if ( HANG_BOTTOM_LIMIT == 0 && HANG_JOYSTICK > 127 )
			HANG_MOTOR = 127;

		if ( HANG_TOP_LIMIT == 0 && HANG_JOYSTICK < 127 )
			HANG_MOTOR = 127;

		// Wait for robot to reach destination
		if( left_wheel_count > MODE3_GATE_COUNT)	// If we are at destination
		{
			HANG_MOTOR= 127;

			left_wheel_count = 0;			// Reset left wheel count
			right_wheel_count = 0;			// Reset right wheel count
			if( mode_switch == 5 )			// If we are on the left side of the field
				{
				auto_mode = MODE3_TURN1_RIGHT;		// Turn right
				}
			else
				{
				auto_mode = MODE3_TURN1_LEFT;		// Turn Left
				}
		}
		break;


		/*******************************************************************************/
		// Turn ROBOT to the RIGHT until ARC_RIGHT_COUNT is reached
		case MODE3_TURN1_RIGHT:
		LEFT_WHEEL = 127 + MODE3_TURN1_SPEED;			// Keep LEFT_WHEEL moving at GATE_SPEED
		RIGHT_WHEEL = 127 - MODE3_TURN1_SPEED;					// Stop right wheel
        HANG_MOTOR= 127;
		// Wait for robot to finish turning Right
		if( left_wheel_count > MODE3_TURN1_RIGHT_COUNT )// If count has been reached
		{
			printf("Arcing Complete\nTraveling fast\n");
			left_wheel_count = 0;			// Reset left wheel count
			right_wheel_count = 0;			// Reset right wheel count
			LEFT_WHEEL = 127;
			RIGHT_WHEEL = 127;
			auto_mode = AUTO_MODE_DONE;			// Next State for switch statment
		}
		break;								// Exit switch statement


		/*******************************************************************************/
		// Turn ROBOT to the LEFT until ARC_LEFT_COUNT is reached
		case MODE3_TURN1_LEFT:
		RIGHT_WHEEL = 127 + MODE3_TURN1_SPEED;			// Keep RIGHT_WHEEL moving at GATE_SPEED
		LEFT_WHEEL = 127 - MODE3_TURN1_SPEED;            // Stop Left wheel
		HANG_MOTOR= 127;
		// Wait for robot to finish turning Right
		if( right_wheel_count > MODE3_TURN1_LEFT_COUNT )// If count has been reached
		{
			printf("Arcing Complete\nTraveling fast\n");
			left_wheel_count = 0;			// Reset left wheel count
			right_wheel_count = 0;			// Reset right wheel count
			LEFT_WHEEL = 127;
			RIGHT_WHEEL = 127;
			auto_mode = AUTO_MODE_DONE;			// Next State for switch statment
		}
		break;								// Exit switch statement
	}
}


// M.O.R.T MODES #5 and #6
/*******************************************************************************
* FUNCTION NAME: Autonomous_mode_5or6
* PURPOSE:       Knock the ball of by pointing robot in proper direction ("Point and Pray")
*       
* CALLED FROM:   main.c
* ARGUMENTS:     none
* RETURNS:       The state of autonomous mode
*				 If we return AUTO_MODE_DONE, the main program will control the robot normally	
*-------------------------------------------------------------------------------
// 
//
//	* 
//	 * 
//    * 
//	   * 
//	    * 
// 		 *
//		  *
//		   *
//			*
//			 *
//			  *
//			   *
//			    *
//				 *	 	
//				  *
//				   *
//			 		*
//			  		 *
//					  *
//					   *
//					    *	
//						 *
//					* * * *  <=  hits our ball
//----------------------------------------------------------------------------------------*/







	#define MODE5_GATE_SPEED		      1			// Speed out of the gate
	#define MODE5_GATE_COUNT    		1000			// Number of wheel counts before first turn
	#define MODE5_HANG_ARM_SPEED        90
	#define MODE5_HANG_ARM_HEIGHT       70

	#define MODE5_TURN1_SPEED			120			// Speed reduced to wheel to turn robot for TURN1
	#define MODE5_TURN1_LEFT_COUNT   	120			// Number of wheel counts to turn the robot LEFT  in TURN1
	#define MODE5_TURN1_RIGHT_COUNT 	120			// Number of wheel counts to turn the robot RIGHT in TURN1

//Defines for switch
	#define MODE5_TURN1_LEFT       1
	#define MODE5_TURN1_RIGHT      2

void Autonomous_mode_5or6( void )
{
	LEFT_WHEEL  = 127;						// Assume we want to stop LEFT_WHEEL.  This value may be changed below
	RIGHT_WHEEL = 127;						// Assume we want to stop RIGHT_WHEEL. This value may be changed below

	switch( auto_mode )						// Which part of autonomous_mode_1 are we in ???
	{
		/*******************************************************************************/
		// Drive ROBOT straight out the gate till GATE2_COUNT is reached
		case 0:
		LEFT_WHEEL  = MODE5_GATE_SPEED;			// Output GATE2_SPEED to LEFT_WHEEL
		RIGHT_WHEEL = MODE5_GATE_SPEED;			// Output GATE2_SPEED to RIGHT_WHEEL
		HANG_MOTOR = 127;
		arm_motor_counter++;
		if(arm_motor_counter < MODE5_HANG_ARM_HEIGHT)
		{
			HANG_MOTOR = MODE5_HANG_ARM_SPEED;
		}
		else
		{
			HANG_MOTOR = 127;
		}



		if ( HANG_BOTTOM_LIMIT == 0 && HANG_JOYSTICK > 127 )
			HANG_MOTOR = 127;

		if ( HANG_TOP_LIMIT == 0 && HANG_JOYSTICK < 127 )
			HANG_MOTOR = 127;

		// Wait for robot to reach destination
		if( left_wheel_count > MODE5_GATE_COUNT)	// If we are at destination
		{
			HANG_MOTOR= 127;

			left_wheel_count = 0;			// Reset left wheel count
			right_wheel_count = 0;			// Reset right wheel count
			if( mode_switch == 5 )			// If we are on the left side of the field
				{
				auto_mode = MODE5_TURN1_LEFT;		// Turn right
				}
			else
				{
				auto_mode = MODE5_TURN1_RIGHT;		// Turn Left
				}
		}
		break;


		/*******************************************************************************/
		// Turn ROBOT to the RIGHT until ARC_RIGHT_COUNT is reached
		case MODE5_TURN1_RIGHT:
		LEFT_WHEEL = 127 + MODE5_TURN1_SPEED;			// Keep LEFT_WHEEL moving at GATE_SPEED
		RIGHT_WHEEL = 127 - MODE5_TURN1_SPEED;					// Stop right wheel
        HANG_MOTOR= 127;
		// Wait for robot to finish turning Right
		if( left_wheel_count > MODE5_TURN1_RIGHT_COUNT )// If count has been reached
		{
			printf("Arcing Complete\nTraveling fast\n");
			left_wheel_count = 0;			// Reset left wheel count
			right_wheel_count = 0;			// Reset right wheel count
			LEFT_WHEEL = 127;
			RIGHT_WHEEL = 127;
			auto_mode = AUTO_MODE_DONE;			// Next State for switch statment
		}
		break;								// Exit switch statement


		/*******************************************************************************/
		// Turn ROBOT to the LEFT until ARC_LEFT_COUNT is reached
		case MODE5_TURN1_LEFT:
		RIGHT_WHEEL = 127 + MODE5_TURN1_SPEED;			// Keep RIGHT_WHEEL moving at GATE_SPEED
		LEFT_WHEEL = 127 - MODE5_TURN1_SPEED;            // Stop Left wheel
		HANG_MOTOR= 127;
		// Wait for robot to finish turning Right
		if( right_wheel_count > MODE5_TURN1_LEFT_COUNT )// If count has been reached
		{
			printf("Arcing Complete\nTraveling fast\n");
			left_wheel_count = 0;			// Reset left wheel count
			right_wheel_count = 0;			// Reset right wheel count
			LEFT_WHEEL = 127;
			RIGHT_WHEEL = 127;
			auto_mode = AUTO_MODE_DONE;			// Next State for switch statment
		}
		break;								// Exit switch statement
	}
}


#define _20_FEET 960			// 48 transitions per foot
#define GATE7_SPEED 60	

void Autonomous_mode_7( void )
{
	/*******************************************************************************/
	// Drive ROBOT straight out the gate for 20 feet
	LEFT_WHEEL  = 127 - GATE7_SPEED;			// Output GATE2_SPEED to LEFT_WHEEL
	RIGHT_WHEEL = 127 - GATE7_SPEED;			// Output GATE2_SPEED to RIGHT_WHEEL
	HANG_MOTOR = 127;

	// Wait for robot to reach destination
	if( left_wheel_count > _20_FEET)	// If we are at destination
	{
		LEFT_WHEEL  = 127;
		RIGHT_WHEEL = 127;			// Output GATE2_SPEED to RIGHT_WHEEL
	}
}


#define MODE8_GATE_SPEED         60
#define MODE8_GATE_LENGTH       1008-290  //16 ft			48 counts/ft

#define MODE8_TURN_LEFT     270
#define MODE8_TURN_RIGHT    270
#define MODE8_TURN_SPEED    120



void Autonomous_mode_8or9(void)
{
switch( auto_mode )						// Which part of autonomous_mode_1 are we in ???
	{
		/*******************************************************************************/
		// Drive ROBOT straight out the gate till GATE2_COUNT is reached
		case 0:
		LEFT_WHEEL  = MODE8_GATE_SPEED;			// Output MODE3_GATE_SPEED to LEFT_WHEEL
		RIGHT_WHEEL = MODE8_GATE_SPEED;			// Output MODE3_GATE_SPEED to RIGHT_WHEEL
		// Wait for robot to reach destination
		if( left_wheel_count > MODE8_GATE_LENGTH)	// If we are at destination
		{
			left_wheel_count = 0;			// Reset left wheel count
			right_wheel_count = 0;			// Reset right wheel count
			if( mode_switch == 8 )			// If we are on the left side of the field
				{
				auto_mode = 2;		// Turn left
				}
			else
				{
				auto_mode = 1;		// Turn right
				}

		}
		break;


		/*******************************************************************************/
		// Turn ROBOT to the RIGHT
		case 2:
		LEFT_WHEEL = 127 + MODE8_TURN_SPEED;			// Keep LEFT_WHEEL moving at GATE_SPEED
		RIGHT_WHEEL = 127 - MODE8_TURN_SPEED;					// Stop right wheel
		// Wait for robot to finish turning Right
		if( left_wheel_count > MODE8_TURN_RIGHT )// If count has been reached
		{
			left_wheel_count = 0;			// Reset left wheel count
			right_wheel_count = 0;			// Reset right wheel count
			LEFT_WHEEL = 127;
			RIGHT_WHEEL = 127;
			auto_mode = 10;			// Next State for switch statment
		}
		break;								// Exit switch statement


		/*******************************************************************************/
		// Turn ROBOT to the LEFT
		case 1:
		RIGHT_WHEEL = 127 + MODE3_TURN1_SPEED;			// Keep RIGHT_WHEEL moving at GATE_SPEED
		LEFT_WHEEL =  127 - MODE3_TURN1_SPEED;					// Stop Left wheel
		// Wait for robot to finish turning Right
		if( right_wheel_count > MODE8_TURN_LEFT )// If count has been reached
		{
			left_wheel_count = 0;			// Reset left wheel count
			right_wheel_count = 0;			// Reset right wheel count
			LEFT_WHEEL = 127;
			RIGHT_WHEEL = 127;
			auto_mode = 10;			// Next State for switch statment
		}
		break;			

		case 10:
		RIGHT_WHEEL = 127;
		LEFT_WHEEL  = 127;

		break;			

	}
}



					// Exit switch statement
// End Of File



