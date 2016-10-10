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
*
* CHANGE LOG:
* 1/15/2004		P.Roth		Initial Creation
* 1/18/2004		GROUP		Ported and tested Compensate_joysticks( ) function
*
*
*******************************************************************************/

#include "ifi_aliases.h"
#include "ifi_default.h"
#include "ifi_utilities.h"
#include "user_routines.h"
#include "printf_lib.h"

/* FIRST prototypes */
void Limit_Switch_Max(unsigned char switch_state, unsigned char *input_value);
void Limit_Switch_Min(unsigned char switch_state, unsigned char *input_value);
unsigned char Limit_Mix (int intermediate_value);

/* M.O.R.T Prototypes */
void Compensate_joysticks(void);
void Combine_joysticks(void);

/* M.O.R.T. defines */
#define LEFT_WHEEL 		pwm01			// PWM for left wheel
#define RIGHT_WHEEL 	pwm02			// PWM for right wheel
#define ARM_MOTOR 		pwm03			// PWM that ARM motor is connected to

#define HOME_SWITCH			rc_dig_in03  	// ARM Home switch input


/* Defines for speed values 0 to 255 that are used for PWMs.
/* Remember 127 = STOPPED, 0 = Full REV, 255 = Full FWD */
#define HOME_SPEED  	100				// Slow speed DOWN when homing
#define GRAB_SPEED  	180				// Slow speed up to GRAB position

/* defines for state of home switch */
#define ACTIVE 			1				// Home switch is active (at home)
#define NOT_ACTIVE 		0 				// Home switch not active (not at home)

/* Defines for the current state of what the grabber arm is doing */
#define POS_HOMING		1				// ARM is going down to home position
#define POS_AT_HOME		2				// ARM is stopped at the home position
#define POS_GRABBING    3				// ARM is going up to GRAB position
#define POS_AT_GRAB     4				// ARM is at GRAB position

/* Defines for grabber ARM positions */
#define GRAB_POSITION   13				// Positions grabber ARM 15" off ground
#define MAX_POSITION  	150				// Maximum allowable position

/* M.O.R.T Variables 
EXAMPLES: (see MPLAB C18 User's Guide, p.9 for all types)
unsigned char  (can vary from 0 to 255)
unsigned int   (can vary from 0 to 65,535)
int            (can vary from -32,768 to 32,767)
unsigned long  (can vary from 0 to 4,294,967,295)
*/
unsigned char table_compensation	=	1;		// Set to 1 for table compensation
unsigned char single_joystick		=	1;		// Set to 1 for single joystick control
unsigned char Acceleration 			=	3;		// Set to non ZERO to enable accell limiting, smaller value will slow accell/decell.
unsigned char p1_x_save    			=	127;	// Saves last value of p1_x when acceleration is enabled
unsigned char p1_y_save   			=   127;	// Saves last value of p1_y when acceleration is enabled
unsigned char p2_x_save    			=   127;	// Saves last value of p2_x when acceleration is enabled
unsigned char p2_y_save   			=   127;	// Saves last value of p2_y when acceleration is enabled
int left_count						=	0;
int right_count						=	0;
unsigned auto_mode = 0;


extern int right_wheel_count;
extern int left_wheel_count;

/* defines used for grabber ARM */
unsigned char current_position		=	0;
int tooth_counter					=	0;
unsigned char timer;


/* Lookup table for joystick output if table_compensation = 1 */
unsigned char Table[128] =
				{
				0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,2,2,2,
				3,3,4,4,4,5,5,5,6,6,6,6,7,7,7,8,8,8,
				9,9,9,10,10,10,11,11,11,12,12,13,13,
				13,14,14,15,15,16,16,17,17,17,18,18,
				19,20,20,21,21,22,22,23,24,24,25,25,
				26,27,28,28,29,30,31,31,32,33,34,35,
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
	Getdata(&rxdata);   		// Get fresh data from the master microprocessor.
//	Default_Routine( );


//	if( timer == 0 )
//		printf( "BEFORE   X Joystick = %d     Y Joystick = %d\n", (int)p1_x, (int)p1_y );

	if( p1_y > 120 && p1_y < 134 )
		p1_y = 127;
	if( p1_x > 120 && p1_x < 134 )
		p1_x = 127;
	
//	if( timer == 0 )
//		printf( "AFTER    X Joystick = %d     Y Joystick = %d\n", (int)p1_x, (int)p1_y );


	if( left_count != left_wheel_count || right_count != right_wheel_count )
			printf( "LeftWheelCount = %d    RightWheelCount = %d\n", left_wheel_count, right_wheel_count );
	left_count = left_wheel_count;
	right_count = right_wheel_count;

			LEFT_WHEEL = 127;
			RIGHT_WHEEL = 127;


#define COUNTS_PER_REV		72 
#define WHEEL_DIAMETER		6
#define WHEEL_RESOLUTION	WHEEL_DIAMETER * 3.14 / COUNTS_PER_REV


#define GATE_DISTANCE		36				// Inches out of the gate
#define GATE_SPEED			200

#define TURN1_COUNTS		40				// Number of counts to turn the robot
#define TURN2_COUNTS		40				// Number of counts to turn the robot
#define TURN_SPEED			160

#define LANE_DISTANCE		120				// Inches down the lane
#define LANE_SPEED			200

#define FINAL_DISTANCE		36				// Inches to ball
#define FINAL_SPEED			200				// Speed to ball

#define END_AUTO 			99
#define WAIT_FOR_TRIG 		98


	switch( auto_mode )
	{
		case 0:
		if( p1_y >= 254 )			// Begin autonomous 
			{
			auto_mode = 3;
			}
		break;

		// Straight ahead down the lane ---------------------------------------
		case 3:
		LEFT_WHEEL  = LANE_SPEED;
		RIGHT_WHEEL = LANE_SPEED;
		if( p1_sw_trig == 1 )				// If panic stop is needed
		{
			auto_mode = WAIT_FOR_TRIG;		// go to WAIT_FOR_TRIG state
			break;							// Exit switch statement
		}	
		// Wait for robot to reach destination
		if( left_wheel_count > (LANE_DISTANCE / WHEEL_RESOLUTION) )
		{
			left_wheel_count = 0;			// Reset left wheel count
			right_wheel_count = 0;			// Reset right wheel count
			auto_mode = WAIT_FOR_TRIG;					// Next state
		}
		break;


		// Execute left turn ------------------------------------------------
		case 4:
		LEFT_WHEEL = 127;					// Stop LEFT wheel
		RIGHT_WHEEL = TURN_SPEED;			// Keep left wheel turning
		if( p1_sw_trig == 1 )				// If panic stop is needed
		{
			auto_mode = WAIT_FOR_TRIG;		// go to WAIT_FOR_TRIG state
			break;							// Exit switch statement
		}	
		// Wait for robot to finish turning Right
		if( right_wheel_count > TURN2_COUNTS )
		{
			left_wheel_count = 0;			// Reset left wheel count
			right_wheel_count = 0;			// Reset right wheel count
			auto_mode++;					// Next state
		}
		break;


		// Go straight towards ball ------------------------------------------- 
		case 5:
		if( p1_sw_trig == 1 )				// If panic stop is needed
		{
			auto_mode = WAIT_FOR_TRIG;		// go to WAIT_FOR_TRIG state
			break;							// Exit switch statement
		}	
		LEFT_WHEEL  = FINAL_SPEED;			// Turn LEFT  wheel ON
		RIGHT_WHEEL = FINAL_SPEED;			// Turn RIGHT wheel ON
		// Wait for robot to reach destination
		if( left_wheel_count > (FINAL_DISTANCE / WHEEL_RESOLUTION) )
		{
			auto_mode = WAIT_FOR_TRIG;		// go to WAIT_FOR_TRIG state
		}
		break;



	
  	// M.O.R.T. code begins here
	// Convert joystick values using table and limit accelleration
//	Compensate_joysticks( );
//	if(single_joystick == 1)

		case WAIT_FOR_TRIG:
		LEFT_WHEEL = 127;
		RIGHT_WHEEL = 127;
		if( p1_sw_trig == 1)
			auto_mode = END_AUTO;
		break;

	}

		if( auto_mode == END_AUTO )
			Combine_joysticks();
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
  digital_io_18 = INPUT;  /* Used for pneumatic pressure switch. */
    /* 
     Note: digital_io_01 = digital_io_02 = ... digital_io_04 = INPUT; 
           is the same as the following:

           digital_io_01 = INPUT;
           digital_io_02 = INPUT;
           ...
           digital_io_04 = INPUT;
    */

/* SECOND: Set up the I/O pins you want to use as digital OUTPUTS. */
  digital_io_17 = OUTPUT;    /* Example - Not used in Default Code. */

/* THIRD: Initialize the values on the digital outputs. */
  rc_dig_out17 = 0;

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

  printf("%s\n", strptr);       		/* Optional - Print initialization message. */

	if( HOME_SWITCH == NOT_ACTIVE )		/* If home switch is NOT active */
	{
		ARM_MOTOR = HOME_SPEED;			/* Output home speed to grabber arm motor */
		current_position = POS_HOMING;	/* Next state */
	}

  User_Proc_Is_Ready();         		/* DO NOT CHANGE! - last line of User_Initialization */

}


/*******************************************************************************
* FUNCTION NAME: Default_Routine
* PURPOSE:       Performs the default mappings of inputs to outputs for the
*                Robot Controller.
* CALLED FROM:   this file, Process_Data_From_Master_uP routine
* ARGUMENTS:     none
* RETURNS:       void
*******************************************************************************/
void Default_Routine(void)
{
  
 /*---------- Analog Inputs (Joysticks) to PWM Outputs-----------------------
  *--------------------------------------------------------------------------
  *   This maps the joystick axes to specific PWM outputs.
  */
  pwm01 = p1_y;   
  pwm02 = p2_y;   
  pwm03 = p3_y;   
  pwm04 = p4_y;   
  pwm05 = p1_x;   
  pwm06 = p2_x;   
  pwm07 = p3_x;   
  pwm08 = p4_x;   
  pwm09 = p1_wheel;
  pwm10 = p2_wheel;   
  pwm11 = p3_wheel;   
  pwm12 = p4_wheel;   
  
 /*---------- 1 Joystick Drive ----------------------------------------------
  *--------------------------------------------------------------------------
  *  This code mixes the Y and X axis on Port 1 to allow one joystick drive. 
  *  Joystick forward  = Robot forward
  *  Joystick backward = Robot backward
  *  Joystick right    = Robot rotates right
  *  Joystick left     = Robot rotates left
  *  Connect the right drive motors to PWM13 and/or PWM14 on the RC.
  *  Connect the left  drive motors to PWM15 and/or PWM16 on the RC.
  */  
  pwm13 = pwm14 = Limit_Mix(2000 + p1_y + p1_x - 127);
  pwm15 = pwm16 = Limit_Mix(2000 + p1_y - p1_x + 127);
  
 /*---------- Buttons to Relays----------------------------------------------
  *--------------------------------------------------------------------------
  *  This default code maps the joystick buttons to specific relay outputs.  
  *  Relays 1 and 2 use limit switches to stop the movement in one direction.
  *  The & used below is the C symbol for AND                                
  */
  relay1_fwd = p1_sw_trig & rc_dig_in01;  /* FWD only if switch1 is not closed. */
  relay1_rev = p1_sw_top  & rc_dig_in02;  /* REV only if switch2 is not closed. */
  relay2_fwd = p2_sw_trig & rc_dig_in03;  /* FWD only if switch3 is not closed. */
  relay2_rev = p2_sw_top  & rc_dig_in04;  /* REV only if switch4 is not closed. */
  relay3_fwd = p3_sw_trig;
  relay3_rev = p3_sw_top;
  relay4_fwd = p4_sw_trig;
  relay4_rev = p4_sw_top;
  relay5_fwd = p1_sw_aux1;
  relay5_rev = p1_sw_aux2;
  relay6_fwd = p3_sw_aux1;
  relay6_rev = p3_sw_aux2;
  relay7_fwd = p4_sw_aux1;
  relay7_rev = p4_sw_aux2;
  relay8_fwd = !rc_dig_in18;  /* Power pump only if pressure switch is off. */
  relay8_rev = 0;
  
  /*---------- PWM outputs Limited by Limit Switches  ------------------------*/
  
  Limit_Switch_Max(!rc_dig_in05, &pwm03);
  Limit_Switch_Min(!rc_dig_in06, &pwm03);
  Limit_Switch_Max(!rc_dig_in07, &pwm04);
  Limit_Switch_Min(!rc_dig_in08, &pwm04);
  Limit_Switch_Max(!rc_dig_in09, &pwm09);
  Limit_Switch_Min(!rc_dig_in10, &pwm09);
  Limit_Switch_Max(!rc_dig_in11, &pwm10);
  Limit_Switch_Min(!rc_dig_in12, &pwm10);
  Limit_Switch_Max(!rc_dig_in13, &pwm11);
  Limit_Switch_Min(!rc_dig_in14, &pwm11);
  Limit_Switch_Max(!rc_dig_in15, &pwm12);
  Limit_Switch_Min(!rc_dig_in16, &pwm12);
  
 /*---------- ROBOT FEEDBACK LEDs------------------------------------------------
  *------------------------------------------------------------------------------
  *   This section drives the "ROBOT FEEDBACK" lights on the Operator Interface.
  *   The lights are green for joystick forward and red for joystick reverse.
  *   Both red and green are on when the joystick is centered.  Use the
  *   trim tabs on the joystick to adjust the center.     
  *   These may be changed for any use that the user desires.                       
  */	
  
  if (user_display_mode == 0) /* User Mode is Off */
    
  { /* Check position of Port 1 Joystick */
    if (p1_y >= 0 && p1_y <= 56)
    {                     /* Joystick is in full reverse position */
      Pwm1_green  = 0;    /* Turn PWM1 green LED - OFF */
      Pwm1_red  = 1;      /* Turn PWM1 red LED   - ON  */
    }
    else if (p1_y >= 125 && p1_y <= 129)
    {                     /* Joystick is in neutral position */
      Pwm1_green  = 1;    /* Turn PWM1 green LED - ON */
      Pwm1_red  = 1;      /* Turn PWM1 red LED   - ON */
    }
    else if (p1_y >= 216 && p1_y <= 255)
    {                     /* Joystick is in full forward position*/
      Pwm1_green  = 1;    /* Turn PWM1 green LED - ON  */
      Pwm1_red  = 0;      /* Turn PWM1 red LED   - OFF */
    }
    else
    {                     /* In either forward or reverse position */
      Pwm1_green  = 0;    /* Turn PWM1 green LED - OFF */
      Pwm1_red  = 0;      /* Turn PWM1 red LED   - OFF */
    }  /*END Check position of Port 1 Joystick
    
    /* Check position of Port 2 Y Joystick 
           (or Port 1 X in Single Joystick Drive Mode) */
    if (p2_y >= 0 && p2_y <= 56)
    {                     /* Joystick is in full reverse position */
      Pwm2_green  = 0;    /* Turn pwm2 green LED - OFF */
      Pwm2_red  = 1;      /* Turn pwm2 red LED   - ON  */
    }
    else if (p2_y >= 125 && p2_y <= 129)
    {                     /* Joystick is in neutral position */
      Pwm2_green  = 1;    /* Turn PWM2 green LED - ON */
      Pwm2_red  = 1;      /* Turn PWM2 red LED   - ON */
    }
    else if (p2_y >= 216 && p2_y <= 255)
    {                     /* Joystick is in full forward position */
      Pwm2_green  = 1;    /* Turn PWM2 green LED - ON  */
      Pwm2_red  = 0;      /* Turn PWM2 red LED   - OFF */
    }
    else
    {                     /* In either forward or reverse position */
      Pwm2_green  = 0;    /* Turn PWM2 green LED - OFF */
      Pwm2_red  = 0;      /* Turn PWM2 red LED   - OFF */
    }  /* END Check position of Port 2 Joystick */
    
    /* This drives the Relay 1 and Relay 2 "Robot Feedback" lights on the OI. */
    Relay1_green = relay1_fwd;    /* LED is ON when Relay 1 is FWD */
    Relay1_red = relay1_rev;      /* LED is ON when Relay 1 is REV */
    Relay2_green = relay2_fwd;    /* LED is ON when Relay 2 is FWD */
    Relay2_red = relay2_rev;      /* LED is ON when Relay 2 is REV */

    Switch1_LED = !(int)rc_dig_in01;
    Switch2_LED = !(int)rc_dig_in02;
    Switch3_LED = !(int)rc_dig_in03;
    
  } /* (user_display_mode = 0) (User Mode is Off) */
  
  else  /* User Mode is On - displays data in OI 4-digit display*/
  {
    User_Mode_byte = backup_voltage*10; /* so that decimal doesn't get truncated. */
  }   

} /* END Default_Routine(); */





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
	// If table compensation is required always do joystick #1
	if(table_compensation == 1)
	{
		if (p1_x < 127) 
		{			
			/*Table starts at memory locations 0-127 and represents positive joystick positions
			128-254. Since positive and negative response is identical, we can position from
			the end of the curve to find the negative value of the joystick */
	        p1_x = Table[127 - p1_x];
	        p1_x = 127 - p1_x;
		}
		else
		{ 				
			/* Subtract 127 from joystick value which is 127 or greater to get to the start of
			the table, Index into the adjusted value from the table, then add 127 to get
			back to 127 */ 
			p1_x = Table[p1_x - 127];
			p1_x = p1_x + 127;
		}
		if (p1_y < 127) 
		{			
	        p1_y = Table[127 - p1_y];
	        p1_y = 127 - p1_y;
		}
		else
		{ 				
			p1_y = Table[p1_y - 127];
			p1_y = p1_y + 127;
		}
		
		// Now check if using 2 joysticks
		if(single_joystick = 0)
		{
			if (p2_x < 127) 
			{			
		        p2_x = Table[127 - p2_x];
		        p2_x = 127 - p2_x;
			}
			else
			{ 				
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
		}
	}
	
	/*============================================================================
	Now that the joysticks are compensated by getting new values from the table, 
	limit acceleration and deceleration of robot by ramping up and down to target
	velocity. Saves wear and tear on drive train and makes the robot easier to control.
	Acceleration can be controlled by the variable "acceleration" which is the maximum amount
	the speed can change for every loop execution. If no acceleration limiting is needed,
	then just set variable to ZERO, the code below still runs, but the value is ZERO
	=============================================================================*/
	if (p1_x > p1_x_save) 			/* If target is faster than current speed */
	    {
		p1_x_save = (p1_x_save + Acceleration);
        if (p1_x_save >= p1_x)
            {
			p1_x_save = p1_x;
            }
	    }
	else
	   {
	   if (p1_x < p1_x_save)  		/* If target is slower than current speed */
	      {
		  if (p1_x_save > Acceleration)
		 	{
			p1_x_save = (p1_x_save - Acceleration);
			}
	      else
			{
		 	p1_x_save = p1_x;
			}
          }
        }	
	if (p1_y > p1_y_save)			/* If target is faster than current speed */
	   {
		p1_y_save = (p1_y_save + Acceleration);
		if (p1_y_save >= p1_y)
			{
			p1_y_save = p1_y;
			}
	   }
	else
	   {
	   if  (p1_y < p1_y_save)  		/* If target is slower than current speed */
            {   
			if (p1_y_save > Acceleration)
			  {
	          p1_y_save = (p1_y_save - Acceleration);
			  }
	        else
			  {
		  	  p1_y_save = p1_y;
			  }
	       	}
        }	
	p1_x  = p1_x_save;
	p1_y = p1_y_save;

	// Now check if using 2 joysticks
	if(single_joystick == 0)
	{
		if (p2_x > p2_x_save) 			/* If target is faster than current speed */
		    {
			p2_x_save = (p2_x_save + Acceleration);
	        if (p2_x_save >= p2_x)
	            {
				p2_x_save = p2_x;
	            }
		    }
		else
		   {
		   if (p2_x < p2_x_save)  		/* If target is slower than current speed */
		      {
			  if (p2_x_save > Acceleration)
			 	{
				p2_x_save = (p2_x_save - Acceleration);
				}
		      else
				{
			 	p2_x_save = p2_x;
				}
	          }
	        }	
		if (p2_y > p2_y_save)			/* If target is faster than current speed */
		   {
			p2_y_save = (p2_y_save + Acceleration);
			if (p2_y_save >= p2_y)
				{
				p2_y_save = p2_y;
				}
		   }
		else
		   {
		   if  (p2_y < p2_y_save)  		/* If target is slower than current speed */
	            {   
				if (p2_y_save > Acceleration)
				  {
		          p2_y_save = (p2_y_save - Acceleration);
				  }
		        else
				  {
			  	  p2_y_save = p2_y;
				  }
		       	}
	        }	
		}
	p2_x  = p2_x_save;
	p2_y = p2_y_save;
}




 /*---------- 1 Joystick Drive ----------------------------------------------
  *--------------------------------------------------------------------------
  *  This code mixes the Y and X axis on Port 1 to allow one joystick drive. 
  *  Joystick forward  = Robot forward
  *  Joystick backward = Robot backward
  *  Joystick right    = Robot rotates right
  *  Joystick left     = Robot rotates left
  */  
void Combine_joysticks(void)
{
  RIGHT_WHEEL = Limit_Mix(2000 + p1_y + p1_x - 127);
  LEFT_WHEEL = Limit_Mix(2000 + p1_y - p1_x + 127);
}


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



//  printf("Joystick Y axis = %n\n", (int)p1_y);       		/* Optional - Print initialization message. */
	
//	switch(current_position)
//	{
		// This state runs if we are homing the grabber arm
//		case POS_HOMING:
//			if( HOME_SWITCH == ACTIVE )				/* If home switch activated */
//			{
//				ARM_MOTOR = 127;					/* Stop the grabber arm */
//				current_position = POS_AT_HOME;		/* Next state */
//			}
//		break;

//		// This state runs when the grabber arm is home
//		case POS_AT_HOME:
//			if( p3_y > 250 )						/* If joystick is pushed forward */
//			{
//				ARM_MOTOR = GRAB_SPEED;				/* Start Arm moving up */
//				current_position = POS_GRABBING;	/* Next state */
//			}
//		break;
	
//		// This state runs when the grabber arm is moving to the GRAB POSITION
//		case POS_GRABBING:
//			if( tooth_counter > GRAB_POSITION )		/* If ARM is 15 inches above ground */
//			{
//				ARM_MOTOR = 127;					/* Stop motor */
//				current_position = POS_AT_GRAB;		/* Next state */
//			}
//		break;
//	}




	Putdata(&txdata);			// Output new data to the master microprocessor
}


