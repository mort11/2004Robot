/*******************************************************************************
* FILE NAME: user_routines.c <FRC VERSION>
*
* DESCRIPTION:
*  This file contains the default mappings of inputs  
*  (like switches, joysticks, and buttons) to outputs on the RC. 
*
* USAGE:
*  You can either modify this file to fit your needs, or remove it from your 
*  project and replace it with a modified copy. 
*
* CHANGE LOG:
* 1/18/2004		GROUP		Ported and tested Compensate_joysticks( ) function
*
*
*******************************************************************************/
#include "ifi_aliases.h"
#include "ifi_default.h"
#include "ifi_utilities.h"
#include "user_routines.h"
#include "printf_lib.h"
#include "mort2004.h"

/* M.O.R.T Variables 
EXAMPLES: (see MPLAB C18 User's Guide, p.9 for all types)
unsigned char  (can vary from 0 to 255)
unsigned int   (can vary from 0 to 65,535)
int            (can vary from -32,768 to 32,767)
unsigned long  (can vary from 0 to 4,294,967,295)
*/
unsigned char table_compensation	=	1;		// Set to 1 for table compensation
unsigned char dual_joysticks		=	1;		// Set to 1 for dual joystick control
unsigned char Accelleration 		=	0;		// Set to non ZERO to enable accell limiting, smaller value will slow accell/decell.
int			  right_wheel_save		= 	127;
int			  left_wheel_save		=	127;
int			  mode_switch			=	0;		// holds value of autonomous mode selection, ready only once during initialization
int 		  mode_switch_save		=	0;
int left_sensor_count_save			=	0;		// Saves current left  wheel position. 
											// Positions can be larger than 255 so we must use an int, not a char
int right_sensor_count_save			=	0;		// Saves current right wheel position
int loop_timer 						= 	0;

/* Lookup table for joystick output if table_compensation = 1 */
unsigned char Table[128] =
				{
				0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,2,2,2,
				3,3,4,4,4,5,5,5,6,6,6,6,7,7,7,8,8,8,
				9,9,9,10,10,10,11,11,11,12,12,13,13,
				13,14,14,15,15,16,16,17,17,17,18,18,
				19,20,20,21,21,22,22,23,24,24,25,25,
				26,27,28,28,29,30,31,31,32,33,34, 35,
				36,37,38,39,40,41,42,43,44,45,46,48,
				49,50,52,53,55,56,58,60,61,63,65,67,
				69,71,73,75,78,80,83,86,88,91,95,98,
				101,105,109,113,117,127,127
				};

/*******************************************************************************
* FUNCTION NAME: Process_Data_From_Master_uP
* PURPOSE:       Executes every 26.2ms when it gets new data from the master 
*                microprocessor.
* CALLED FROM:   main.c
* ARGUMENTS:     none
* RETURNS:       void
*******************************************************************************/
void Process_Data_From_Master_uP(void)
{
	Getdata(&rxdata);   				// Get fresh data from the master microprocessor.
		
  	// M.O.R.T. code begins here

	if(loop_timer++ > 38)				// If 38 26.2 Millisecond loops have been counted
	{
		loop_timer=0;					// Reset loop counter to start counting over
		Every_second( );				// Call this routine every second
	}

	mode_switch = read_selector_switch( ); // Read Autonomous mode selector switch and save
										   // The contents in the mode_switch variable

	Service_end_effectors( );			// Control all end effectors

	// Turn ON Compressor
  	COMPRESSOR_FWD = !COMPRESSOR_SWITCH;// Power pump only if pressure switch is off
  	COMPRESSOR_REV = 0;

	check_center( );					// Display neutral position on OI 

	if(table_compensation == 1)			// If non linear joystick output is required
		Compensate_joysticks( );		// Adjust joystick values 
	
	if(dual_joysticks == 1)				// If using dual joysticks	
	{
		LEFT_WHEEL =  p2_y;				// Simply output p2_y to LEFT wheel
		RIGHT_WHEEL = p4_y;				// Simply output p4_y to RIGHT wheel
	}
	else								// else combine X-Y to single joystick drive
	{
		RIGHT_WHEEL = Limit_Mix(2000 + p2_y + p2_x - 127);
  		LEFT_WHEEL  = Limit_Mix(2000 + p2_y - p2_x + 127);
	}

	if( Accelleration != 0 )			// If accelleration limiting is required
		SoftStartWheels( );				// Prevent rapid changes @ wheels to reduce slippage

	// Output the same PWM values from the Chippuas to the Drill motors
	LEFT_DRILL  = LEFT_WHEEL;
	RIGHT_DRILL = RIGHT_WHEEL;
//if ((CURRENT_SENSOR_DISABLE1 == 0) && (CURRENT_SENSOR_DISABLE2 == 0))
//	{
		limit_current( );					// Adjust PWM values for Drills if overcurrent	
//}
	


	RIGHT_DRILL= 255 - RIGHT_DRILL;
	LEFT_DRILL = 255 - LEFT_DRILL;
	RIGHT_WHEEL= 255 - RIGHT_WHEEL;
	LEFT_WHEEL = 255 - LEFT_WHEEL;
	
	if((SLOW_BUTTON1 == 1) || (SLOW_BUTTON2 == 1))
	{

//RIGHT DRILL COMPENSATION
		if (RIGHT_DRILL>127)
			RIGHT_DRILL = (RIGHT_DRILL - 127) / 6 + 127;
						
		if (RIGHT_DRILL<127)
			RIGHT_DRILL = 127 - ((127 - RIGHT_DRILL) / 6) ;

//LEFT DRILL COMPENSATION
		if (LEFT_DRILL>127)
			LEFT_DRILL = (LEFT_DRILL - 127) / 6 + 127;
						
		if (LEFT_DRILL<127)
			LEFT_DRILL = 127 - ((127 - LEFT_DRILL) / 6) ;

//RIGHT WHEEL COMPENSATION
		if (RIGHT_WHEEL>127)
			RIGHT_WHEEL = (RIGHT_WHEEL - 127) / 6 + 127;
						
		if (RIGHT_WHEEL<127)
			RIGHT_WHEEL = 127 - ((127 - RIGHT_WHEEL) / 6) ;

//LEFT_WHEEL COMPENSATION

		if (LEFT_WHEEL>127)
			LEFT_WHEEL = (LEFT_WHEEL - 127) / 6 + 127;
						
		if (LEFT_WHEEL<127)
			LEFT_WHEEL = 127 - ((127 - LEFT_WHEEL) / 6) ;

	}


	reset_beacon_counts( );				// Clear beacon counts so high speed routine can increment them
	
	Putdata(&txdata);					// Output new data to the master microprocessor
}


/* ported from basic */
/*============================================================================
 Converts joystick linear output to exponential output. This makes controlling the 
 robot easier by providing a lot of joystick motion at low/med speeds and less motion
 at high speeds where the direction is not rapidly changing. Using a 128 byte lookup
 table allows tuning the response to the mass and acceleration of the robot. 
 The table also has allows a deadband by having the first few entries set to zero

	ARGUMENTS:
		NONE
		
	RETURNS:
		NONE

=============================================================================*/
void Compensate_joysticks(void) 
{
	// Always do joystick #1
	if (p2_x < 127) 
	{			
		/*Table starts at memory locations 0-127 and represents positive joystick positions
		128-254. Since positive and negative response is identical, we can position from
		the end of the curve to find the negative value of the joystick */
        p2_x = Table[127 - p2_x];
        p2_x = 127 - p2_x;
	}
	else
	{ 				
		/* Subtract 127 from joystick value which is 127 or greater to get to the start of
		the table, Index into the adjusted value from the table, then add 127 to get
		back to 127 */ 
		p2_x = Table[p2_x - 127];
		p2_x = p2_x + 127;
	}
	if (p2_y < 127) 
	{			
        p2_y = Table[127 - p2_y];
        p2_y = 127 - p2_y;
	}
	else
	{ 				
		p2_y = Table[p2_y - 127];
		p2_y = p2_y + 127;
	}
	
	// Now check if using 2 joysticks
	if(dual_joysticks == 1)
	{
		if (p4_x < 127) 
		{			
	        p4_x = Table[127 - p4_x];
	        p4_x = 127 - p4_x;
		}
		else
		{ 				
			p4_x = Table[p4_x - 127];
			p4_x = p4_x + 127;
		}
		if (p4_y < 127) 
		{			
	        p4_y = Table[127 - p4_y];
	        p4_y = 127 - p4_y;
		}
		else
		{ 				
			p4_y = Table[p4_y - 127];
			p4_y = p4_y + 127;
		}
	}
}


/*============================================================================
Limit wheel pwm accelleration and decelleration of robot by ramping up and down to target
velocity. Prevents wheel slippage, reduces current peaks and saves wear and tear on drive
train. If accelleration limiting is NOT needed, set the variable accelleration = 0 to disable

NOTE: 	This routine must be called AFTER the combine_joysticks( ) and compensate_jowstick( )
		routines. It should be called just before the output function.
=============================================================================*/
void SoftStartWheels( void )
{
	// Check for accell limit
	if (LEFT_WHEEL > left_wheel_save) 						// If we are accellerating
	{
		if (LEFT_WHEEL > (left_wheel_save+Accelleration)) 	// If we must limit accell
			left_wheel_save+=Accelleration;					// Step to next higher speed
		else
			left_wheel_save = LEFT_WHEEL;
	}

	// Check for decell limit
	if (LEFT_WHEEL < left_wheel_save) 						// If we are decellerating
	{
		if (LEFT_WHEEL < (left_wheel_save-Accelleration)) 	// If we must limit decell
			left_wheel_save-=Accelleration;					// Step to next lower speed
		else
			left_wheel_save = LEFT_WHEEL;
	}
	

	// Check for accell limit
	if (RIGHT_WHEEL > right_wheel_save) 						// If we are accellerating
	{
		if (RIGHT_WHEEL > (right_wheel_save+Accelleration)) 	// If we must limit accell
			right_wheel_save+=Accelleration;					// Step to next higher speed
		else
			right_wheel_save = RIGHT_WHEEL;
	}

	// Check for decell limit
	if (RIGHT_WHEEL < right_wheel_save) 						// If we are decellerating
	{
		if (RIGHT_WHEEL < (right_wheel_save-Accelleration)) 	// If we must limit decell
			right_wheel_save-=Accelleration;					// Step to next lower speed
		else
			right_wheel_save = RIGHT_WHEEL;
	}
	
	LEFT_WHEEL  = left_wheel_save;
	RIGHT_WHEEL = right_wheel_save;
}




//**********************************************************************
//
// Bypasses the Accell/Decell and brings the robot to a skidding halt
//
//----------------------------------------------------------------------
void stop_short( void )
{
	LEFT_WHEEL = 127;
	RIGHT_WHEEL = 127;
	left_wheel_save = 127;
	right_wheel_save = 127;
}


void check_center( void )
{
	// Turn on LEDs to inform operator if joystick #1 is neutral
    Pwm1_green  = 0;   		// Assume X axis is not neutral
    Pwm1_red    = 0;      	// Assume Y axis is not neutral
	// If p2_x is close to neutral
    if (p2_x >= 124 && p2_x <= 130)
      Pwm1_red  = 1;    	// Turn PWM1 RED LED - ON */
	// If p2_y is close to neutral
    if (p2_y >= 124 && p2_y <= 130)
      Pwm1_green  = 1;    	// Turn PWM1 GREEN LED - ON */

	
	// Turn on LEDs to inform operator if joystick #1 is neutral
	Pwm2_green  = 0;   		// Assume X axis is not neutral
	Pwm2_red    = 0;      	// Assume Y axis is not neutral
	// If p4_x is close to neutral
	if (p4_x >= 124 && p4_x <= 130)
		Pwm2_red  = 1;    	// Turn PWM2 RED LED - ON */
	// If p4_y is close to neutral
	if (p4_y >= 124 && p4_y <= 130)
		Pwm2_green  = 1;    	// Turn PWM2 GREEN LED - ON */


	// Update LEDS on OI to display status of beacons
	// If we see the left beacon,  turn on the GREEN Relay1 LED
	// If we see the right beacon, turn on the RED   Relay1 LED
    if( left_beacon_count > NEAR_BEACON_COUNT )
		Relay1_red = 1;
	else
		Relay1_red = 0;

    if( right_beacon_count > NEAR_BEACON_COUNT )
		Relay1_green = 1;
	else
		Relay1_green = 0;
}	


int read_selector_switch( void )
{
	int x = 0;
	
	if( SELECTOR_SW_BIT8 == 0 )
		x += 8;
	if( SELECTOR_SW_BIT4 == 0 )
		x += 4;
	if( SELECTOR_SW_BIT2 == 0 )
		x += 2;
	if( SELECTOR_SW_BIT1 == 0 )
		x += 1;
	return x;
}


// END M.O.R.T. Code



/*******************************************************************************
* FUNCTION NAME: Limit_Switch_Max
* PURPOSE:       Sets a PWM value to neutral (127) if it exceeds 127 and the
*                limit switch is on.
* CALLED FROM:   this file
* ARGUMENTS:     
*     Argument       Type             IO   Description
*     --------       -------------    --   -----------
*     switch_state   unsigned char    I    limit switch state
*     *input_value   pointer           O   points to PWM byte value to be limited
* RETURNS:       void
*******************************************************************************/
void Limit_Switch_Max(unsigned char switch_state, unsigned char *input_value)
{
  if (switch_state == CLOSED)
  { 
    if(*input_value > 127)
      *input_value = 127;
  }
}

/*******************************************************************************
* FUNCTION NAME: Limit_Switch_Min
* PURPOSE:       Sets a PWM value to neutral (127) if it's less than 127 and the
*                limit switch is on.
* CALLED FROM:   this file
* ARGUMENTS:     
*     Argument       Type             IO   Description
*     --------       -------------    --   -----------
*     switch_state   unsigned char    I    limit switch state
*     *input_value   pointer           O   points to PWM byte value to be limited
* RETURNS:       void
*******************************************************************************/
void Limit_Switch_Min(unsigned char switch_state, unsigned char *input_value)
{
  if (switch_state == CLOSED)
  { 
    if(*input_value < 127)
      *input_value = 127;
  }
}

/*******************************************************************************
* FUNCTION NAME: Limit_Mix
* PURPOSE:       Limits the mixed value for one joystick drive.
* CALLED FROM:   Default_Routine, this file
* ARGUMENTS:     
*     Argument             Type    IO   Description
*     --------             ----    --   -----------
*     intermediate_value    int    I    
* RETURNS:       unsigned char
*******************************************************************************/
unsigned char Limit_Mix (int intermediate_value)
{
  static int limited_value;
  
  if (intermediate_value < 2000)
  {
    limited_value = 2000;
  }
  else if (intermediate_value > 2254)
  {
    limited_value = 2254;
  }
  else
  {
    limited_value = intermediate_value;
  }
  return (unsigned char) (limited_value - 2000);
}

/*******************************************************************************
* FUNCTION NAME: User_Initialization
* PURPOSE:       This routine is called first (and only once) in the Main function.  
*                You may modify and add to this function.
* CALLED FROM:   main.c
* ARGUMENTS:     none
* RETURNS:       void
*******************************************************************************/
void User_Initialization (void)
{
  rom const	char *strptr = "IFI User Processor Initialized ...";

  Set_Number_of_Analog_Channels(SIXTEEN_ANALOG);    /* DO NOT CHANGE! */

/* FIRST: Set up the I/O pins you want to use as digital INPUTS. */
  digital_io_01 = digital_io_02 = digital_io_03 = digital_io_04 = INPUT;
  digital_io_05 = digital_io_06 = digital_io_07 = digital_io_08 = INPUT;
  digital_io_09 = digital_io_10 = digital_io_11 = digital_io_12 = INPUT;
  digital_io_13 = digital_io_14 = digital_io_15 = digital_io_16 = INPUT;
  digital_io_17 = digital_io_18 = INPUT;  /* Used for pneumatic pressure switch. */
    /* 
     Note: digital_io_01 = digital_io_02 = ... digital_io_04 = INPUT; 
           is the same as the following:

           digital_io_01 = INPUT;
           digital_io_02 = INPUT;
           ...
           digital_io_04 = INPUT;
    */

/* FOURTH: Set your initial PWM values.  Neutral is 127. */
  pwm01 = pwm02 = pwm03 = pwm04 = pwm05 = pwm06 = pwm07 = pwm08 = 127;
  pwm09 = pwm10 = pwm11 = pwm12 = pwm13 = pwm14 = pwm15 = pwm16 = 127;

/* FIFTH: Set your PWM output types for PWM OUTPUTS 13-16.
  /*   Choose from these parameters for PWM 13-16 respectively:               */
  /*     IFI_PWM  - Standard IFI PWM output generated with Generate_Pwms(...) */
  /*     USER_CCP - User can use PWM pin as digital I/O or CCP pin.           */
  Setup_PWM_Output_Type(IFI_PWM,IFI_PWM,IFI_PWM,IFI_PWM);

  /* 
     Example: The following would generate a 40KHz PWM with a 50% duty cycle on the CCP2 pin:

         CCP2CON = 0x3C;
         PR2 = 0xF9;
         CCPR2L = 0x7F;
         T2CON = 0;
         T2CONbits.TMR2ON = 1;

         Setup_PWM_Output_Type(USER_CCP,IFI_PWM,IFI_PWM,IFI_PWM);
  */

  /* Add any other initialization code here. */

  Initialize_Serial_Comms();   
  
  Putdata(&txdata);             		/* DO NOT CHANGE! */

  User_Proc_Is_Ready();         		/* DO NOT CHANGE! - last line of User_Initialization */
	
	printf( "%s\n", strptr);

}


/*********************************************************************************************
*
*	Current Sensor Code
*
*
*--------------------------------------------------------------------------------------------*/
#define NEUTRAL_CURRENT 506
#define MAX_CURRENT 300   //200 previously  									//40 amps, each count equals .025 amps
//define MAX_CURRENT 156  									//30 amps, each count equals .025 amps

#define Current_Max_Reverse NEUTRAL_CURRENT - MAX_CURRENT 
#define Current_Max_Forward NEUTRAL_CURRENT + MAX_CURRENT
int Current_sensor_right  = 0;
int Current_sensor_left  = 0;

void limit_current( void )
{
	Current_sensor_right =	Get_Analog_Value (2);
	Current_sensor_left  =	Get_Analog_Value (1);


//RIGHT DRILL FORWARD
//*********************************************************************************************

	if (RIGHT_DRILL>127)
	{
		if (Current_sensor_right >  Current_Max_Forward)
		{
			if(RIGHT_DRILL - ((Current_sensor_right-NEUTRAL_CURRENT)/3)>127)
				RIGHT_DRILL-= ((Current_sensor_right-NEUTRAL_CURRENT)/3);
			else
				RIGHT_DRILL = 127;

			printf("RIGHT_DRILL Excessive Forward: %d Amps....New Speed = %d\n",(Current_sensor_right-NEUTRAL_CURRENT)/5, (int)RIGHT_DRILL);
		}
	}


//RIGHT DRILL REVERSE
//*********************************************************************************************
	if (RIGHT_DRILL<127)
	{
		if (Current_sensor_right < Current_Max_Reverse)
		{
			if(RIGHT_DRILL + ((NEUTRAL_CURRENT-Current_sensor_right)/3)<127)
				RIGHT_DRILL+= ((NEUTRAL_CURRENT-Current_sensor_right)/3);
			else
				RIGHT_DRILL = 127;

			printf("RIGHT_DRILL Excessive Reverse: %d Amps....New Speed = %d\n", (NEUTRAL_CURRENT-Current_sensor_right)/5 , (int)RIGHT_DRILL);
		}
	}

//LEFT DRILL FORWARD
//*********************************************************************************************
	if (LEFT_DRILL>127)
	{
		if (Current_sensor_left >  Current_Max_Forward)
		{
			if(LEFT_DRILL - ((Current_sensor_left-NEUTRAL_CURRENT)/3)>127)
				LEFT_DRILL-= ((Current_sensor_left-NEUTRAL_CURRENT)/3);
			else
				LEFT_DRILL = 127;

			printf("LEFT_DRILL Excessive Forward: %d Amps....New Speed = %d\n",(Current_sensor_left-NEUTRAL_CURRENT)/5, (int)LEFT_DRILL);
		}
	}

//LEFT DRILL REVERSE
//*********************************************************************************************
	if (LEFT_DRILL<127)
	{
		if (Current_sensor_left < Current_Max_Reverse)
		{
			if(LEFT_DRILL + ((NEUTRAL_CURRENT-Current_sensor_left)/3)<127)
				LEFT_DRILL+= ((NEUTRAL_CURRENT-Current_sensor_left)/3);
			else
				LEFT_DRILL = 127;

			printf("LEFT_DRILL Excessive Reverse: %d Amps....New Speed = %d\n", (NEUTRAL_CURRENT-Current_sensor_left)/5 , (int)LEFT_DRILL);
		}
	}

}

//END OF AMP SENSING
//*********************************************************************************************
		








void Every_second( void )
{
	
	int left_feet_per_second 	= 0;
	int right_feet_per_second 	= 0;
//	printf("calling function %d%n", temp_wheel_count);
	// There are 72 "teeth" on the sensor disk so there will be 144 transitions of the sensor per revolution of each wheel.
	// The wheel diameter is 12.25 inches, so each transision, or pulse, represents .267 inch or about 3.75 transitions per inch.
	
	//	circumference = diameter * pi  OR  12.25 * 3.14  = 38.465  inches per revolution of wheel
		
	//	since there are 144 transitions per 38.465 inches, each transition = 38.465 / 144 = .267 inches
	 		
	//  In order to perform fractional math, the computer must represent numbers in floating point notation,
	// 	which is very slow and requires a lot of code space. Ideally we would want to multiply the number of wheel 
	// 	pulses by the distance per pulse to get the number of inches travelled for this 1 second period.
	//
	//	inches_per_second = left_wheel_count * .267
	//
	//	What can avoid using floating point by multiplying the wheel count by 267 NOT .267 and then divide the result by 1000
	//
	//	inches_per_second  left_wheel_count * 267
	//
	//	Now divide by 1000 to get the correct result
	//
	//	inches_per_second = inches_per_second / 1000;
	//
	// 	To convert to feet per second, simply divide by 12 and store the result in the variable feet_per_second
	//
	//	feet_per_second = inches_per_second / 12;
	//
	// Another way of calculating feet per second is to divide the number of pulses travelled in 1 second by the number 
	// of pulses in 1 foot of travel. Since there are 3.75 pulses per inch there must be 3.75 * 12  OR  45 pulses per foot.
	// SO......We can calculate feet per second with 1 single line of code:
	if( left_temp_wheel_count)
	{
		left_feet_per_second = left_temp_wheel_count / 45;
		// Printout number of pulses that occurred this second and printout our calculated feet_per_second
		printf( "Left Pulses Per Second = %d   Left Feet per second = %d\n", left_temp_wheel_count, left_feet_per_second );
		// Reset wheel count to start from zero for the next time around
		left_temp_wheel_count = 0;
	}

	if( right_temp_wheel_count)
	{
		right_feet_per_second = right_temp_wheel_count / 45;
		// Printout number of pulses that occurred this second and printout our calculated feet_per_second
		printf( "Right Pulses Per Second = %d   Right Feet per second = %d\n", right_temp_wheel_count, right_feet_per_second );
		// Reset wheel count to start from zero for the next time around
		right_temp_wheel_count = 0;
	}

	// Display any changes to mode selector switch
	if( mode_switch != mode_switch_save)
		printf("Autonomous Mode = %d\r", mode_switch);
	mode_switch_save = mode_switch;

	// Display any changes to wheel counts
//	if( left_sensor_count_save != left_wheel_count || right_sensor_count_save != right_wheel_count )
//		printf( "LeftWheelCount = %d    RightWheelCount = %d\n", left_wheel_count, right_wheel_count );
	left_sensor_count_save  = left_wheel_count;
	right_sensor_count_save = right_wheel_count;

	// Display any counts received from Beacons
//	if( left_beacon_count != 0  || right_beacon_count != 0 )
//		printf( "Left beacon_count  = %d    Right_beacon_count = %d\n", left_beacon_count, right_beacon_count );
}

