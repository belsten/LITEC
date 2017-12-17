/*
Names: Alex Belsten, Kristian Gutenmann, James Naylor
	Section: 03
	Side: B
	Date: 11/20/17
	Description:
        Lab 6: Gondola Control
		Use the thrust fans and compass to contol the
		gondola and make it point in the direction
		a desired heading. kD contol must be used as
		the gondola has no dampening. Allow the distance
		of an object above the gondola to advust the desired
		heading from +180 to -180 degrees.
*/

#include <c8051_SDCC.h>
#include <stdio.h>
#include <stdlib.h>
#include <i2c.h>

//-----------------------------------------------------------------------------
// Function Prototypes
//-----------------------------------------------------------------------------
unsigned char ADC_result(void);
unsigned int ReadDistance(void);
unsigned int ReadCompass(void);
void SMB0_Init(void);
void ADC_Init(void);
void pickHeading(void);
void pickGains(void);
void PCA_Init(void);
void Port_Init(void);
void setServoPWM(void);
void setFanAngle(void);

//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------
//PCA values
//======== constants ========
__xdata unsigned int PCA_start = 28672;
__xdata unsigned int PW_max = 3502;
__xdata unsigned int PW_neutral = 2765;
__xdata unsigned int PW_min = 2028;

//======== pw variables ========
signed int error = 0;
signed int previous_error = 0;
signed long tmp_pw;             //must be a long to handle both large positive or negative calculations
unsigned int PW_left;
unsigned int PW_right;
unsigned int PW_angle = 2405;

//counters, headings, ranges & flags
unsigned char count;
unsigned char input;
//========== ranger ==========
unsigned char r_count      = 0;			//overflow counter used for ranger
unsigned char new_distance = 0;			//flag for calling ReadDistance function
unsigned int current_range;

//========== compass ==========
unsigned char h_count      = 0;			//overflow counter used or compass
unsigned char new_heading  = 0;			//flag for calling ReadCompass function
signed int desired_heading;
unsigned int original_heading;
signed int current_heading;

//========== print ==========
unsigned char print_flag  = 0;
unsigned char print_count = 0;

//========== gains ==========
float kp;                     //proportional gain
unsigned int kd;              //derivative gain

//sbits
__sbit __at 0xB7 adjust_heading;				//slideswitch associated with port 3 pin 7

//-----------------------------------------------------------------------------
// Main Function
//-----------------------------------------------------------------------------
void main() {
	unsigned char mVbattery;
	unsigned int time = 0;
	//initialize board
	Sys_Init();
	putchar('\n');
	SMB0_Init();
	Port_Init();
	ADC_Init();
	PCA_Init();

	PCA0CP1 = 0xFFFF - PW_angle;
	PCA0CP0, PCA0CP2, PCA0CP3 = 0xFFFF - PW_neutral;    //set servos to neutral
	lcd_clear();
	lcd_print("Hello world\n");
	//wait for 1 second
	count = 0;
	while (count < 250);

	//get user's desired gains and heading
	pickHeading();
	pickGains();

	//move the thrust fans to a vertical position before the main
	setFanAngle();

	printf("Gains: kd: %d\r\n", kd);
	printf_fast_f("       kp: %2.2f\r\n",kp);
	while(1) {
	    if (new_heading) {
	      //update servo PWs based on current_heading, current_range, desired_heading, and gains
	      current_heading = ReadCompass();

	      //check if the adjust_heading slideswitch is on and adjust heading accordingly
		if (adjust_heading) {
			desired_heading = original_heading + 30 * current_range - 1800;
			if (desired_heading < 0) desired_heading += 3600;
			if (desired_heading > 3600) desired_heading -=3600;
		}
		else desired_heading = original_heading;

			setServoPWM();         //set the thrust servos PW
			new_heading = 0;       //clear flag
	    }
	    if (new_distance) {
			//only need to update the distance
			current_range = ReadDistance();
			new_distance = 0;      //clear flag
	    }
	    if (print_flag) {

			//voltage read by the ADC is 0.236·Vbattery (ADC_result / 0.236 = mVbattery, in other words, the actual battery voltage is 4.235·ADC voltage).
			mVbattery =  ADC_result() * 40;
			printf("TIME: %d.%d RIGHT: %d LEFT: %d tmp_pw: %ld CURRENT HEADING: %d ERROR: %d\r\n", time/10, time%10,  PW_right, PW_left, tmp_pw, current_heading, error);
			if (adjust_heading) {
			printf("DESIRED HEADING %d DISTANCE: %d\r\n", desired_heading, current_range);
			}
			lcd_clear();
			lcd_print("Heading: %d\nBattery: %d\n", current_heading, mVbattery);
			print_flag = 0;       //clear flag
			time += 5;
    	}
	}
}

//-----------------------------------------------------------------------------
// setServoPWM
//-----------------------------------------------------------------------------
void setServoPWM() {
	//center the heading
	error = desired_heading - current_heading;
	if (error < -1800)  error += 3600;
	if (error > 1800)   error -= 3600;
	//calcualte the adjustment PW
	tmp_pw = (signed long)kp*(signed long)error + (signed long)kd*(signed long)(error - previous_error);
	previous_error = error;

	//make sure the PWs are within the max and min values
	if (tmp_pw > 1000) tmp_pw = 1000;
	if (tmp_pw < -1000)tmp_pw = -1000;
	PW_left = PW_neutral + (int)tmp_pw;
	PW_right = PW_neutral - (int)tmp_pw;
	if (PW_left < PW_min) PW_left = PW_min;
	if (PW_right < PW_min) PW_right = PW_min;
	if (PW_left > PW_max) PW_left = PW_max;
	if (PW_right > PW_max) PW_right = PW_max;

	//set motors
	PCA0CP2 = 0xFFFF - PW_left;
	PCA0CP3 = 0xFFFF - PW_right;
}

//-----------------------------------------------------------------------------
// pickHeading
//-----------------------------------------------------------------------------
//this function will allow the user to to choose the heading or enter their own
//it will use the lcd keyboard as input
//user first picks 0, 90, 180, 270
//if they dont like those, get an int from the keyboard ranging from 0 - 359
void pickHeading() {
	printf("GET HEADING\r\n");
	while (1){
		lcd_clear();
		lcd_print("Choose heading:\n1)0 2)90 3)180 4)2705)other\n");
		//input = getchar();
		input = kpd_input(1);
		if(input == 1) {
			desired_heading = 0;
			break;
		}
	  if(input == 2) {
			desired_heading = 90;
			break;
		}
		if(input == 3) {
			desired_heading = 180;
			break;
		}
		if(input == 4) {
			desired_heading = 270;
			break;
		}
		if (input == 5) {
			lcd_clear();
			lcd_print("Chose a heading between 0 and 359\n");
			desired_heading = kpd_input(1);
			if (desired_heading > 359) desired_heading = 359;
			break;
		}
		else {
			//print an error statement to the lcd for 2 seconds
			lcd_clear();
			lcd_print("ERROR:\nInvalid reponse.\n");
			count = 0;
			while (count < 50);
		}
	}
	//display the users chosen heading
	lcd_clear();
	lcd_print("The chosen heading  is %u degrees\n", desired_heading);
	count = 0;
	while (count < 50);
	lcd_clear();
	desired_heading *= 10;				//multiply users desired heading by ten
 	original_heading = desired_heading;
}

//-----------------------------------------------------------------------------
// pickGains
//-----------------------------------------------------------------------------
//let the user pick the kP & kD
//kP should have a min value of .1 and a max of about 15 according to lab pdf
//kD should range from 0 to about 200 according to lab pdf
//(explicitly need to provide data for kD values 0-18 & kP values from 0.1 to 12)
void pickGains() {

  while (1) {   //get kD
		lcd_clear();
		lcd_print("Enter desired kD\nbetween 0 & 100\n");
		kd = kpd_input(1);
		if (kd <= 100) break;
		else {
			//print an error statement to the lcd for 2 seconds
			lcd_clear();
			lcd_print("ERROR:\nkD not in range.\nTry again.\n");
			count = 0;
			while (count < 50);
		}
	}

	while (1) {   //get kP
		lcd_clear();
		lcd_print("Enter desired kP*10\nbetween 1 & 150\n");
		input = kpd_input(1);
		if (input < 150) break;
		else {
		  //print an error statement to the lcd for 2 seconds
		  lcd_clear();
		  lcd_print("ERROR:\nkP not in range.\nTry again.\n");
		  count = 0;
		  while (count < 50);
    	}
  	}

	//display chosen gains to user
	lcd_clear();
	lcd_print("kP : %d.%d\nkD: %d", input / 10, input % 10, kd);
	count = 0;
	while (count < 75);
	kp = (float)input / 10;
}


//-----------------------------------------------------------------------------
// setFanAngle
//-----------------------------------------------------------------------------
//function allows user to set the thrust fans to a vertical position
void setFanAngle() {
	lcd_clear();
	lcd_print("Set the fan angle.");
	count = 0;
	while (count < 100);
	while(1) {
		lcd_clear();
		lcd_print("Press '2' for up\nPress '5' for down\nPress '8' when done\n");
		while(read_keypad() == 0xFF);
		input = read_keypad();
		printf("%c, %d\r\n",input, PW_angle);
		if(input == '2') PW_angle -= 15;
		if(input == '5') PW_angle += 15;
		if(input == '8') break;
		if(PW_angle > 4025) PW_angle = 4025;
		if(PW_angle < 1600) PW_angle = 1600;
		PCA0CP1 = 0xFFFF - PW_angle;
		count = 0;
		while (count < 2);
	}
	printf("PW_angle: %u\r\n", PW_angle);
	lcd_clear();
}

//-----------------------------------------------------------------------------
// ReadDistance
//-----------------------------------------------------------------------------
//function that updates the global variable range, the distance from a object
//infront of the car in cm
unsigned int ReadDistance() {
  	unsigned int range;
	unsigned char Data[2];								//Data is an array with a size of 2 unsigned chars
	unsigned char addr = 0xE0; 						// the address of the ranger is 0xE0
	i2c_read_data(addr, 2, Data, 2);			// read two bytes, starting at reg 2
	range = (((unsigned int)Data[0] << 8) | Data[1]);	//store echo bytes to global variable range

	//ping the ranger
	Data[0] = 0x51;											//set to read centimeters
	i2c_write_data(addr, 0, Data, 1);		//write one byte of data to register 0 at addr
  	return range;
}

//-----------------------------------------------------------------------------
// ReadCompass
//-----------------------------------------------------------------------------
//function that gets the current direction the car is facing. Used to adjust servo
//a unsigned int returned is between 0 and 3599 in degrees x10
unsigned int ReadCompass() {
	unsigned char addr = 0xC0;				//the address of the Compass is 0xC0
	unsigned char Data[2]; 						//Data is an array with a length of 2
	i2c_read_data(addr, 2, Data, 2);	//starting at reg 2, read two bytes
	return (((unsigned int)Data[0] << 8) | Data[1]);  	//return read bytes as an unsigned int
}

//-----------------------------------------------------------------------------
// ADC_Result
//-----------------------------------------------------------------------------
//function that completes an A/D conversion
//provides the voltage of the battery on the Blimp.
//it is suggested values be displayed in mV (4 digits).
unsigned char ADC_result(void) {
    AMX1SL = 3;            		  //set to port 1 pin 3
    ADC1CN &= ~0x20;        		//clear flag
    ADC1CN |= 0x10;         		//start conversion
    while (!(ADC1CN & 0x20));   //wait for converion to complete
    return ADC1;            		//return digital value in ADC1 register
}

//-----------------------------------------------------------------------------
// Port_Init
//-----------------------------------------------------------------------------
// Set up ports for input and output
void Port_Init() {
	//configure crossbar for
	//SDA -> port 0.2
	//SCL -> port 0.3
	XBR0 = 0x25;

	//gondola uses 4 Capture/Compare Modules with CEX outputs on Port pins P0.4, P0.5, P0.6 and P0.7.
	//CCM0 (CEX0) port 0.4 will control the rudder fan <===== THIS FAN WILL NOT BE USED
	//CCM1 (CEX1) port 0.5 will control the thrust angle port 0.5
	//CCM2 (CEX2) port 0.6 will control the left thrust power fan
	//CCM3 (CEX3) port 0.7 will control the right thrust power fan

	// PORT 0
	P0MDOUT |= 0xFF;
	// PORT 1
	P1MDIN &= ~0x04;
	P1MDOUT &= ~0x04;
	P1 |= 0x04;
	// PORT 3
	P3MDOUT &= ~0x80;				//set port 3 slideswitch input bit low
	P3 |= 0x80; 						//set port 3 slideswitch input bit high
}

//-----------------------------------------------------------------------------
// PCA_Init
//-----------------------------------------------------------------------------
// Set up Programmable Counter Array
void PCA_Init() {
	PCA0MD = 0x81;   		//SYSCLK/12, enable CF interrupts, suspend when idle
	PCA0CPM0 = 0xC2; 		//16 bit, enable compare, enable PWM
	PCA0CPM1 = 0xC2;		//16 bit, enable compare, enable PWM
	PCA0CPM2 = 0xC2;		//16 bit, enable compare, enable PWM
	PCA0CPM3 = 0xC2;		//16 bit, enable compare, enable PWM
	PCA0CN = 0x40;  		//enable PCA

	EA = 1;					//enable all interupts
	EIE1 |= 0x08;		//enable PCA0 interupt
}

//-----------------------------------------------------------------------------
// ADC_Init
//-----------------------------------------------------------------------------
//Initialize conversion function
//voltage read by the ADC is 0.236·Vbattery (in other words, the actual battery voltage is 4.235·ADC voltage).
void ADC_Init(void) {
    REF0CN  = 0x03;      //set Vref to use internal reference voltage of 2.4v
    ADC1CF |= 0x01;      //set gain to 1
    ADC1CN  = 0x80;      //enable ADC1
}

//-----------------------------------------------------------------------------
// SMB0_Init
//-----------------------------------------------------------------------------
// Set up the SMBus
void SMB0_Init() {
	SMB0CR  = 0x93;				//set SCL to 100KHz
	SMB0CN |= 0x40;				//enable SMBus
}


//-----------------------------------------------------------------------------
// PCA_ISR
//-----------------------------------------------------------------------------
void PCA_ISR () __interrupt 9 {
  if (CF)  {
    CF = 0;               //clear CF flag
    PCA0 = PCA_start;		  //set PCA0 to PCA_start
    r_count++;
    h_count++;
    print_count++;
    count++;

    //check if its been 80ms since last distance read
    if (r_count > 5) {
      new_distance = 1; //throw new distance flag
      r_count = 0;			//clear the distance counter
    }
    //check if its been 40ms since last heading read
    if (h_count > 2) {
      new_heading = 1;	//throw new heading flag
      h_count = 0;			//clear the heading counter
    }
    //check if it has been 0.5s since last print
	if (print_count >= 12) {
	  print_flag = 1;		//throw new print flag
	  print_count = 0;	//clear the print counter
  	}
  }
  PCA0CN &= 0xC0;			    //handle other PCA interupt sources
}
