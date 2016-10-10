/*******************************************************************************
* FILE NAME: mort2004.h
*
* DESCRIPTION: 
*  This is the include file that defines all I/O used in the MORT 2004 Robot
*  It contains aliases and function prototypes used in all mort 'C' routines.
*
*******************************************************************************/

#ifndef __mort2004_h_
#define __mort2004_h_


// PWM Output Aliases
#define LEFT_WHEEL 			pwm01			// PWM for left wheel 	(chippua motor)
#define LEFT_DRILL			pwm02			// PWM for left wheel 	(drill motor)
#define RIGHT_WHEEL			pwm03			// PWM for right wheel 	(chippua motor)
#define RIGHT_DRILL			pwm04			// PWM for right wheel 	(drill motor)
#define GRAB_MOTOR      	pwm05       	// PWM for BALL Grabber Arm Motor
#define HANG_MOTOR 			pwm06			// PWM for HOOK Hanging ARM motor

// Relay OUT Aliases
#define BALL_GRABBER_FWD    relay1_fwd      // Relay for the ball grabbers forward ( close )
#define BALL_GRABBER_REV    relay1_rev      // Relay for the ball grabbers reverse ( open )

#define HANG_HOOK_FWD       relay3_fwd      // Relay for the hang hook forward
#define HANG_HOOK_REV       relay3_rev      // Relay for the hang hook grabbers reverse


#define COMPRESSOR_FWD		relay8_fwd		// 1= Turn Compressor ON	0=Turn Compressor OFF
#define COMPRESSOR_REV		relay8_rev		// 0= Turn Compressor ON	1=Turn Compressor OFF


// Digital Input Assignments
#define LEFT_WHEEL_SENSOR	rc_dig_in01  	// Left Wheel Sensor
#define RIGHT_WHEEL_SENSOR	rc_dig_in02  	// Right Wheel Sensor
#define HOME_SWITCH			rc_dig_in03  	// ARM Home switch input
#define SPARE_DIG_IN04		rc_dig_in04		// **** SPARE ****
#define GRAB_TOP_LIMIT      rc_dig_in05     // BALL Grabber Arm Top Limit
#define SPARE_DIG_IN06		rc_dig_in06		// **** SPARE ****
#define HANG_ARM_SENSOR     rc_dig_in07		// HOOK Hanging Arm Motion Sensor
#define GRAB_BOTTOM_LIMIT  	rc_dig_in08     // BALL Grabber Arm Bottom Limit
#define HANG_TOP_LIMIT      rc_dig_in09     // HOOK Hanging Arm Top Limit
#define GRAB_ARM_SENSOR     rc_dig_in10		// BALL Grabber Arm Motion Sensor
#define HANG_BOTTOM_LIMIT   rc_dig_in11     // HOOK Hanging Arm Bottom Limit
#define RIGHT_IR_SENSOR		rc_dig_in12		// Right IR Sensor
#define LEFT_IR_SENSOR      rc_dig_in13     // Left IR Sensor
#define SELECTOR_SW_BIT1	rc_dig_in14		// Autonomous Mode BIT 1
#define SELECTOR_SW_BIT2	rc_dig_in15		// Autonomous Mode BIT 2
#define SELECTOR_SW_BIT4	rc_dig_in16		// Autonomous Mode BIT 4
#define SELECTOR_SW_BIT8	rc_dig_in17		// Autonomous Mode BIT 8
#define COMPRESSOR_SWITCH	rc_dig_in18		// Compressor pressure switch 0=FULL



// Operator inteface aliases
#define GRAB_JOYSTICK       p1_y         	// Grabbing arm joystick alias
#define GRAB_TRIG_PULL      p1_sw_trig   	// Grabbing arm trigger alias
#define BALL_GRABBER_OPEN   p1_sw_aux1      // Button to open ball grabbers
#define BALL_GRABBER_CLOSE  p1_sw_top      // Button to close ball grabbers


#define HANG_JOYSTICK       p1_y            // Hanging arm joystick alias
#define HANG_TRIG_PULL      p3_sw_trig      // Hanging arm trigger alias
#define HANG_HOOK_EXTEND    p3_sw_aux1      // Button to extend hanging hook
#define HANG_HOOK_RETRACT   p3_sw_aux2      // Button to retract hanging hook

#define KILL_SWITCH1        p2_sw_trig      // Button to neutralize robot
#define KILL_SWITCH2        p4_sw_trig      // Button to neutralize robot
#define SLOW_BUTTON1		p2_sw_top
#define SLOW_BUTTON2		p4_sw_top

/* FIRST prototypes */
void Limit_Switch_Max(unsigned char switch_state, unsigned char *input_value);
void Limit_Switch_Min(unsigned char switch_state, unsigned char *input_value);
unsigned char Limit_Mix (int intermediate_value);

/* M.O.R.T Function Prototypes */
unsigned char autonomous_mode_1( void );
void Autonomous_mode_7( void );
void Compensate_joysticks(void);
void SoftStartWheels(void);
void Service_end_effectors(void);
void track_wheel_position(void);
void check_center( void );
void reset_beacon_counts( void );
void poll_beacons( void );
void Autonomous_mode_1or2(void);
void Autonomous_mode_3or4(void);
void Autonomous_mode_5or6(void);
int read_selector_switch(void);
void tune_wheels( void );
void display_velocity( void );
void limit_current(void);
void Every_second( void );
void Autonomous_mode_8or9(void);

// references to variables so all 'C' files can use them
extern int right_wheel_count;
extern int left_wheel_count;
extern int left_temp_wheel_count;
extern int right_temp_wheel_count;
extern int right_beacon_count;
extern int left_beacon_count;
extern int auto_mode;
extern int mode_switch;

// miscellaneous defines
#define AUTO_MODE_DONE   		99			// Tells everyone we are done with autonomous mode

#define ON 		1
#define OFF 	0	

#define NEAR_BEACON_COUNT 		100

#endif
