/*
Names: Alex Belsten, Kristian Gutenmann
	Section: 03
	Side: B
	Date: 11/10/17
	Description:
		1. Integration of the compass and ranger systems developed in the previous lab to control the
		steering and driving. SecureCRT through the wireless serial link will be used to collect data
		and set some options in the program.
		2. Use Analog to Digital Conversion as was implemented in Lab 2 to set the steering
		proportional feedback gain (optional: add a voltage divider to measure battery voltage).
		3. Display control information on LCD display screen or SecureCRT and enter variables using
		either the keypad or the terminal keyboard.
*/

#include <c8051_SDCC.h>
#include <stdio.h>
#include <stdlib.h>
#include <i2c.h>

//-----------------------------------------------------------------------------
// Function Prototypes
//-----------------------------------------------------------------------------
unsigned char ADC_result(unsigned char pin); 
unsigned char selectGain  (void);
unsigned int  ReadDistance(void);
unsigned int  ReadCompass (void);
unsigned int  pickRange   (void);		
void  setServoPWM	    (void);
void  pickHeading     (void);				
void  ObstacleTracking(void);			
void  setRangePWM  	  (void);	
void  ADC_Init     	  (void); 
void  Interrupt_Init  (void);
void  PCA_Init        (void);
void  Port_Init       (void);
void  SMB0_Init       (void);
void  PCA_ISR(void) __interrupt 9;

//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------
char input;
//sbits
__sbit __at 0xB7 RUN;				 					//slideswitch associated with port 3 pin 7
//ADC pins
unsigned char battery_pin = 4;				//port 1 pin 6
unsigned char gain_pin    = 6;	  		//port 1 pin 4

//PWM Global Variables
unsigned int PCA_start     = 28672;
//======= motor =======
unsigned int PW_MAX_ranger = 3502;		//maximum pulsewidth of 1.9ms
unsigned int PW_NEUT       = 2764;		//neutral pulsewidth of 1.5ms
unsigned int PW_MIN_ranger = 2027;		//minimum pulsewidth of 1.1ms
unsigned int PW_ranger     = 0;
int range_adj = 0; 								  	//correction value from ranger
//======= servo =======
unsigned int PW_right  = 3871;				//maximum pulsewidth of 2.1ms
unsigned int PW_center = 2765; 				//neutral pulsewidth of 1.5ms
unsigned int PW_left   = 1659;				//minimum pulsewidth of 0.9ms
unsigned int PW_servo  = 0;
unsigned int desired_heading;					//desired heading
int compass_adj = 0; 									//correction value from compass


//Counters and Flags
unsigned int count = 0;
//======= print =======
unsigned char print_count  = 0;			//overflow counter used for printing
unsigned char print_flag   = 0;			//flag for printing
//======= motor =======
unsigned char r_count      = 0;			//overflow counter used for ranger
unsigned char new_distance = 0;			//flag for calling ReadDistance function
unsigned char ranger_gain  = 0;
unsigned int  range;								//current range  
//======= servo =======
unsigned char h_count      = 0;			//overflow counter used or compass
unsigned char new_heading  = 0;			//flag for calling ReadCompass function
unsigned int  heading;							//current heading
unsigned char compass_gain;					

//-----------------------------------------------------------------------------
// Main Function
//-----------------------------------------------------------------------------
void main() {
	unsigned char run_stop; 								//flag for slide switch		
	unsigned int Voltage_input;	
	//initialize board
	putchar('\n');
	Sys_Init(); 
	Port_Init();
	Interrupt_Init();
	PCA_Init();
	ADC_Init();
	SMB0_Init();

	printf("STARTING\r\n");

	compass_gain    = selectGain();					//unsigned char between 1 & 10
 	ranger_gain     = pickRange();					//unsigned char between 4 & 40
	printf("COMPASS GAIN: %d RANGER GAIN: %d\r\n",compass_gain, ranger_gain);
	pickHeading();													//sets global variable desired_heading
	printf("INFINTE LOOP\r\n");
	while (1) {
		run_stop = 0;
		while (!RUN) { 												//check if the slideswitch is off
			if (run_stop == 0) {								//stay in loop until switch is in run position
				compass_gain    = selectGain();		//unsigned char between 1 & 10
  				ranger_gain     = pickRange();		//unsigned char between 4 & 40
  				pickHeading();										//sets global variable desired_heading
				run_stop = 1; 										//only try to update desired heading once
			}
		}
		if (new_heading) {										//enough overflows for a new heading
			heading = ReadCompass();						//get current heading
			setServoPWM();											//adjust servo PWM for current heading
			new_heading = 0;										//clear flag
		}
		if (new_distance) { 									//enough overflows for a new distance
			range = ReadDistance(); 						//get distance
			if (range < 50) {	  								//if an obstacle is detected within 50cm 
				ObstacleTracking(); 						  //move into obstacle tracking routine
				setRangePWM(); 									  
			}
			new_distance = 0;										//clear flag
		}
		if (print_flag) {
			//print battery voltage to lcd 
			//print heading and range to lcd

			//AD_value = ADC_result(battery_pin);
			Voltage_input = ((ADC_result(battery_pin)*2.4)/(512))*1000;		//convert to voltage in mV
			//lcd_clear();																								//clear current lcd text
			//lcd_print("BATTERY: %dmV  HEADING: %d\nRANGE: %d", heading, heading, range);
			printf("BATTERY: %dmV  HEADING: %d\nRANGE: %d\r\n", heading, heading, range);
			print_flag = 0;							//clear print flag
		}
	}
}

//-----------------------------------------------------------------------------
// ReadDistance
//-----------------------------------------------------------------------------
unsigned int ReadDistance() {
	unsigned char Data[2];								//Data is an array with a size of 2 unsigned chars
	unsigned char addr = 0xE0; 						//the address of the ranger is 0xE0
	i2c_read_data(addr, 2, Data, 2);			//read two bytes, starting at reg 2

	//ping the ranger
	Data[0] = 0x51;											//set to read centimeters
	i2c_write_data(addr, 0, Data, 1);		//write one byte of data to register 0 at addr
	return (((unsigned int)Data[0] << 8) | Data[1]);	//return echo bytes as unsigned int
}

//-----------------------------------------------------------------------------
// ReadCompass
//-----------------------------------------------------------------------------
//the unsigned int returned is between 0 and 3599 in degrees x10
unsigned int ReadCompass() {
	unsigned char addr = 0xC0;				//the address of the Compass is 0xC0
	unsigned char Data[2]; 						//Data is an array with a length of 2
	i2c_read_data(addr, 2, Data, 2);	//starting at reg 2, read two bytes 
	return (((unsigned int)Data[0] << 8) | Data[1]);  	//return read bytes as an unsigned int
}

//-----------------------------------------------------------------------------
// ObstacleTracking
//-----------------------------------------------------------------------------
//routine for when there is an object within the distance threshold
void ObstacleTracking() {
	input = '-';
	PCA0CP2 = 0xFFFF - PW_NEUT;				//stop the car

	//get input on which way to drive
	printf("Press 'l' to go left or 'r' to go right\r\n");
	input = '-';
	while(input != 'r' || input != 'l') input = getchar();		//wait for r or l

	//turn the cars wheels based on the input
	if (input == 'r') PCA0CP0 = 0xFFFF - PW_right;
	else PCA0CP0 = 0xFFFF - PW_left;

	PCA0CP2 = 0xFFFF - //PW_MAX_ranger; <== should involve the user set gain
	printf("Press enter to continue towards heading\r\n");
	while(input != '\n');			//wait for enter to be pressed to continue towards heading

	while(1) {														//continue towards heading until an object is detected
		if (new_heading) {									//enough overflows for a new heading
				heading = ReadCompass();				//get current heading
				setServoPWM(); 									//if new data, adjust servo PWM 
				new_heading = 0;								//clear flag
			}
		if (new_distance) { 								//enough overflows for a new distance
			range = ReadDistance(); 					//get distance
			if (range < 10) { 								//if an obstacle is detected
				PCA0CP2 = 0xFFFF - PW_NEUT;			//stop the car
				break;
			}
			new_distance = 0;									//clear flag
		}
	}

	//wait for powerswitch to be toggled to continue
	while(RUN);
	count = 0;
	while(count < 1);
	while(!RUN);
}

//-----------------------------------------------------------------------------
// setRanageAdj
//-----------------------------------------------------------------------------
//set the motor PWM based on the distance from an object based on the gain
void setRangePWM() {
	PW_ranger = PW_ranger + ranger_gain*(45 - heading);
	if (PW_ranger > PW_MAX_ranger) { PW_ranger = PW_MAX_ranger; }
	if (PW_ranger < PW_MIN_ranger) { PW_ranger = PW_MIN_ranger; }
	PCA0CP2 = 0xFFFF - PW_ranger;

}

//-----------------------------------------------------------------------------
// setServoPWM
//-----------------------------------------------------------------------------
//sets the servo to drive toward the desired heading
void setServoPWM() {
	int difference = 0;
	unsigned int PWM_error = 0;
	difference = desired_heading - heading;				//calculate the difference

	//adjust the difference for +/- 180 degrees
	if (difference < 1800) difference = difference + 3600;
	if (difference > 1800) difference = difference - 3600;

	//calculate PW based on gain
	PWM_error = PW_center + (compass_gain * difference);

	//make sure the PW isnt above or below the max or min
	if (PWM_error < PW_left) PWM_error  = PW_left;
	if (PWM_error > PW_right) PWM_error = PW_right;

	PW_servo = PWM_error;
	PCA0CP0 = 0xFFFF - PW_servo; 								//change pulse width
	new_heading = 0;
}

//-----------------------------------------------------------------------------
// pickHeading
//-----------------------------------------------------------------------------
//this function will allow the user to to choose the heading or enter their own
//it will use the lcd keyboard as input
//user first picks 0, 90, 180, 270
//if they dont like those, get an int from the keyboard ranging from 0 - 359 
void pickHeading() {
	lcd_print("Choose one of the four headings:\n a) 0  b) 90  c)180  d) 270  e) Pick your own\r\n");
	input = getchar();
	if(input == 'a')
		desired_heading = 0;
	if(input == 'b')
		desired_heading = 90;
	if(input == 'c')
		desired_heading = 180;
	if(input == 'd')
		desired_heading = 270;

	if(input == 'e') {
		//lcd_print("Chose a heading between 0 and 359");
		printf("\r\nChose a heading between 0 and 359");
		desired_heading = kpd_input(0);
		if(desired_heading > 359)
			desired_heading = 359;
	}
	//lcd_print("\r\nThe chosen heading is %u", desired_heading);
	printf("\r\nThe chosen heading is %u", desired_heading);
	desired_heading *= 10;
}

//-----------------------------------------------------------------------------
// pickRanger
//-----------------------------------------------------------------------------
//function allows user to enter a desired speed gain through the lcd keyboard
//gain ranges from 1-10
unsigned int pickRange() {
	printf("Enter desired speed between 1 & 10 on keypad/r/n");
	//input = kpd_input(1);
	input = getchar();
	if (input < 1)  input = 1;
	if (input > 10) input = 10;
	printf("Speed setting: %d\r\n", input);
	return input * 4;				//set a neutral gain of 5 to an actual gain of 20
}

//-----------------------------------------------------------------------------
// selectGain
//-----------------------------------------------------------------------------
//this function allows the user to adjust the servo gain through the potentiometer
//current gain is printed to the lcd as it is adjusted, once the user is happy 
//with the gain, they will press a button on the lcd keypad to continue 
unsigned char selectGain() {
	unsigned char gain = 0;
	while (getchar() != '\n'){
	// while(read_keypad() == 0xFF){				//check if there was input from the keypad
		gain = ADC_result(gain_pin) / 25.0;										//calculate gain
		// lcd_print("CURRENT SERVO GAIN: %d  \n", gain );		//print current gain
		// lcd_print("PRESS '#' TO CONTINUE");
		printf("CURRENT SERVO GAIN: %d  \r\n", gain );		//print current gain
		printf("PRESS '#' TO CONTINUE\r\n");
		count = 0;
		while(count < 2);			//wait 40ms before checking the keypad		
	}
	return gain;
}	

//-----------------------------------------------------------------------------
// ADC_Init
//-----------------------------------------------------------------------------
//Initialize conversion function 
void ADC_Init(void) {
    REF0CN = 0x03;      //set Vref to use internal reference voltage of 2.4v
    ADC1CF |= 0x01;     //set gain to 1
    ADC1CN = 0x80;      //enable ADC1
}

//-----------------------------------------------------------------------------
// ADC_Result
//-----------------------------------------------------------------------------
//function that completes an A/D conversion
unsigned char ADC_result(unsigned char pin) {
    AMX1SL = pin;            		//set to appropriate port 1 pin input
    ADC1CN &= ~0x20;        		//clear flag
    ADC1CN |= 0x10;         		//start conversion
    while (!(ADC1CN & 0x20));   //wait for converion to complete
    return ADC1;            		//return digital value in ADC1 register
}

//-----------------------------------------------------------------------------
// Interrupt_Init
//-----------------------------------------------------------------------------
// Set up ports for input and output
void Interrupt_Init() {
    EA = 1;					//enable all interupts
    EIE1 |= 0x08;		//enable PCA0 interupt
}


//-----------------------------------------------------------------------------
// PCA_Init
//-----------------------------------------------------------------------------
// Set up Programmable Counter Array
void PCA_Init() {
    PCA0MD = 0x81;   	//SYSCLK/12, enable CF interrupts, suspend when idle
    PCA0CPM2 = 0xC2; 	//16 bit, enable compare, enable PWM
    PCA0CN |= 0x40;  	//enable PCA
}

//-----------------------------------------------------------------------------
// Port_Init
//-----------------------------------------------------------------------------
// Set up ports for input and output
void Port_Init() {
	//configure crossbar for 
	//SDA -> port 0.6 
	//SCL -> port 0.7
	XBR0 = 0x27;  	
	//port 1
 	P1MDIN &= ~0x50;				// read analog input from port 1.3 & 1.1
  	P1MDOUT |= 0x0D; 				//set output pins for CEX0, CEX2, CEX3 in push-pull mode
	P1MDOUT &= ~0x50;
	P1 |= 0x50;

	//port 3
	P3MDOUT &= ~0x80;				//set port 3 slideswitch input bit low
	P3 |= 0x80; 						//set port 3 slideswitch input bit high
}

//-----------------------------------------------------------------------------
// SMB0_Init
//-----------------------------------------------------------------------------
// Set up the SMBus
void SMB0_Init() {
	SMB0CR = 0x93;				//set SCL to 100KHz
	SMB0CN |= 0x40;				//enable SMBus
}

//-----------------------------------------------------------------------------
// PCA_ISR
//-----------------------------------------------------------------------------
// Interrupt Service Routine for Programmable Counter Array Overflow Interrupt
void PCA_ISR () __interrupt 9 {
	if (CF) {
		CF = 0;							//clear overflow flag
		PCA0 = PCA_start;		//set PCA0 to PCA_start
		//increment counters for heading read and ranger
		r_count++;
		h_count++;
		print_count++;
		count++;

		//check if its been 80ms since last distance read
		if (r_count >= 4) {
			new_distance = 1; //throw new distance flag 
			r_count = 0;			//clear the distance counter
		}
		//check if its been 40ms since last heading read
		if (h_count >= 2) {
			new_heading = 1;	//throw new heading flag
			h_count = 0;			//clear the heading counter
		}
		//check if it has been 0.5s since last print
		if (print_count >= 25) {
		  print_flag = 1;		//throw new print flag
		  print_count = 0;	//clear the print counter
	  }
	}
	PCA0CN &= 0xC0;			//handle other PCA interupt sources
}