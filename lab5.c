/*
Names: Alex Belsten, Kristian Gutenmann, James Naylor
	Section: 03
	Side: B
	Date: 12/10/17
	Description:
        Lab 5: Accelerometer Integration, Slope-directed Steering
		Use a Accelerometer to get the pitch and roll of the car
		and use those values to drive the car up an incline backwords
		and come to a stop at the top.
*/

#include <c8051_SDCC.h>
#include <stdio.h>
#include <stdlib.h>
#include <i2c.h>

//-----------------------------------------------------------------------------
// Function Prototypes
//-----------------------------------------------------------------------------
unsigned char ADC_result(unsigned char pin);
void readAccel(void);
void PCA_ISR(void) __interrupt 9;
void PCA_Init(void);
void Port_Init(void);
void ADC_Init(void);
void SMB0_Init(void);
void setGains(void);
void setServo(void);
void setPWMs(void);

//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------
//define global variables
//sbits
__sbit __at 0xB7 run;				 					//slideswitch associated with port 3 pin 7
__sbit __at 0xB6 reverse;             //output on port 3 pin 6
//ADC pins
unsigned char battery_pin = 6;				//voltage divider at port 1 pin 6
unsigned char gain_pin    = 4;	  		//potentiometer at port 1 pin 4
//pulsewidths
unsigned int PCA_start  = 28672;  		//PCA to overflow every 20ms
//======== motor ========
__xdata unsigned int PW_max     = 3502;		    //maximum pulsewidth of 1.9ms
__xdata unsigned int PW_neutral = 2764;		    //neutral pulsewidth of 1.5ms
__xdata unsigned int PW_min     = 2027;		    //minimum pulsewidth of 1.1ms
unsigned int PW_motor   = 2764;         //current motor pulsewidth
//======== servo ========
unsigned int PW_right  = 3185;				//maximum pulsewidth of 2.1ms
unsigned int PW_center = 2675; 				//neutral pulsewidth of 1.5ms
unsigned int PW_left   = 2065;				//minimum pulsewidth of 0.9ms
unsigned int PW_servo  = 0;						//current servo pulsewidth

//======== flags & counters ========
unsigned int count;
unsigned char back_up;
unsigned char new_accel = 0;          //flag for count of accel timing
unsigned char new_lcd = 0;            //flag for count of LCD timing
unsigned char a_count;                //overflow count for acceleration
unsigned char lcd_count;              //overflow count for LCD updates
//======== gains ========
unsigned char x_gain;
unsigned char steering_gain;
unsigned char y_gain;
//======== accelorometer values ========
signed int gx;
signed int gy;

//-----------------------------------------------------------------------------
// Main Function
//-----------------------------------------------------------------------------
void main() {
	unsigned int Voltage_input;
	//initialize board
	Sys_Init();
	Port_Init();
	PCA_Init();
	ADC_Init();
	SMB0_Init();
	putchar('c');
	Accel_Init_C();

  	//set the motor and servo for at least two seconds
	PCA0CP2 = 0xFFFF - PW_neutral;
	PCA0CP0 = 0xFFFF - PW_center;
	reverse = 1;
	lcd_clear();
	lcd_print("Hello world\n");
	count = 0;
	while (count < 100);
	lcd_clear();

	//get the users desired gains
	setGains();
	setServo();
	printf("y_gain: %d  steering_gain: %d  x_gain: %d\r\n", y_gain, steering_gain, x_gain);

	if (!run){			//to avoid setting the gains twice, wait for the slideswitch to be on before starting while loop
		lcd_clear();
		lcd_print("Flip slideswitch to begin");
		while(!run);
		lcd_clear();
	}
	printf("STARTING\r\n");
	while(1) {
		while (!run) {        //stay in loop until switch is in run position
			setGains();       //function adjusting feedback gains
			while (!run);
		}
		if (new_accel) {		//read from accelerometer and set PWs
			y_gain = ADC_result(gain_pin) / 5;  						//calculate gain
			readAccel();
			setPWMs();
			new_accel = 0;
			a_count = 0;
		}
		if (new_lcd) {			//print info to the lcd and secureCRT
			Voltage_input = ((ADC_result(battery_pin)*2.4)/(256))*1000;
			lcd_clear();
			lcd_print("Y_GAIN: %d\n BATTERY: %dmV", y_gain,Voltage_input);
			printf("gx: %d  gy: %d PW_motor: %d PW_servo: %d\r\n", gx, gy, PW_motor, PW_servo);
			new_lcd = 0;
    	}
  	}
}

//-----------------------------------------------------------------------------
// setPWMs
//-----------------------------------------------------------------------------
//sets the servo to drive toward the desired heading
//set the motor PWM based on the distance from an object based on the gain
void setPWMs() {
	int PW_adj;
	//calculate PW_serevo based on the servo gain and the off x-axis tilt
	PW_servo = PW_center - steering_gain * gx;

	//make sure PW are within the max and mins
	if (PW_servo > PW_right) PW_servo = PW_right;
	if (PW_servo < PW_left) PW_servo = PW_left;
	PCA0CP0 = 0xFFFF - PW_servo;

	//calculate motor PW such that it continues up the incline and slows as y-axis tilt decreases
	PW_adj = y_gain * (gy / 30);
	if (PW_adj > 2000) PW_adj = 2000;
	if (PW_adj < -2000) PW_adj = -2000;

	//adjust the motor PW based on the gx value
	PW_motor = PW_neutral + PW_adj;
	PW_motor -= x_gain * abs(gx);

	//make sure PW is within max and min
	if (PW_motor > PW_max) PW_motor = PW_max;
	if (PW_motor < PW_min) PW_motor = PW_min;
	PCA0CP2 = 0xFFFF - PW_motor;
}

//-----------------------------------------------------------------------------
// readAccel
//-----------------------------------------------------------------------------
void readAccel() {
  unsigned char i;
  unsigned char j = 0;
  signed long avg_gx = 0;
  signed long avg_gy = 0;
  unsigned char Data[4];
  unsigned char addr = 0x3A;
  for (i = 0; i < 8; i++) {
  	//wait for the data to be ready to read
    i2c_read_data(addr, 0x27, Data, 1);
	while ((Data[0] & 0x03) != 0x03) {
		i2c_read_data(addr, 0x27, Data, 1);
		count = 0;
		while (count < 1);
	}
	//read data and conver to an int
    i2c_read_data(addr, 0x28, Data, 4);
    avg_gx += ((Data[1] << 8 | Data[0]) >> 4);
    avg_gy += ((Data[3] << 8) | Data[2] >> 4);
	count = 0;
	while (count < 1);
    j++;

  }
  //average data
  gx = avg_gx / j;
  gy = avg_gy / j;
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
// setGains
//-----------------------------------------------------------------------------
//function allows user to enter a desired gains through the lcd keyboard
void setGains() {
	while (1) {
		//set x gain
		//x gain controls how fast the motor goes based on the magnitude of gx
		lcd_clear();
		lcd_print("Enter desired x_gain\nbetween 1 & 100\n");
		x_gain = kpd_input(1);
		if ((0 < x_gain) && (x_gain <= 100)) {
			break;
		}
		else {
			//print an error statement to the lcd for 2 seconds
			lcd_clear();
			lcd_print("ERROR: not in range.\nTry again.\n");
			count = 0;
			while (count < 100);
		}
	}
	//get the steering gain
  	while (1) {
		lcd_clear();
		lcd_print("Enter desired servo gain\nbetween 1 & 100\n");
		steering_gain = kpd_input(1);
		if ((0 < steering_gain) && (steering_gain <= 100)) {
			break;
		}
		else {
			//print an error statement to the lcd for 2 seconds
			lcd_clear();
			lcd_print("ERROR: not in range.\nTry again.\n");
			count = 0;
			while (count < 100);
		}
	}
	lcd_clear();
}


//-----------------------------------------------------------------------------
// setServo
//-----------------------------------------------------------------------------
//function allows user to set the front wheels so they are straight forward
void setServo() {
	unsigned char input;
  	lcd_clear();
	lcd_print("Center the wheels.");
  	count = 0;
  	while (count < 100);
	while(1) {
    lcd_clear();
    lcd_print("'2' for right\n'5' for left\nPress '8' when done\n");
    while(read_keypad() == 0xFF);
    input = read_keypad();
		if(input == '2') PW_center += 15;
		if(input == '5') PW_center -= 15;
    if(input == '8') break;
		if(PW_center > PW_right) PW_center = PW_right;
		if(PW_center < PW_left) PW_center = PW_left;
    PCA0CP0 = 0xFFFF - PW_center;
    count = 0;
    while (count < 2);
	}
  printf("PW_center: %u\r\n", PW_center);
  lcd_clear();
}

//-----------------------------------------------------------------------------
// ADC_Init
//-----------------------------------------------------------------------------
//Initialize conversion function
void ADC_Init(void) {
    REF0CN  = 0x03;      //set Vref to use internal reference voltage of 2.4v
    ADC1CF |= 0x01;      //set gain to 1
    ADC1CN  = 0x80;      //enable ADC1
}

//-----------------------------------------------------------------------------
// PCA_Init
//-----------------------------------------------------------------------------
// Set up Programmable Counter Array
void PCA_Init() {
	PCA0MD = 0x81;   		//SYSCLK/12, enable CF interrupts, suspend when idle
	PCA0CPM2 = 0xC2; 		//CCM2 16 bit, enable compare, enable PWM
	PCA0CPM0 = 0xC2;		//CCM0 16 bit, enable compare, enable PWM
	PCA0CN |= 0x40;  		//enable PCA

	EA = 1;					//enable all interupts
	EIE1 |= 0x08;		//enable PCA0 interupt
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
	P1MDIN &= ~0x50;				//read analog input from port 1.3 & 1.1
	P1MDOUT |= 0x07; 				//set output pins for CEX0 & CEX2 in push-pull mode
	P1MDOUT &= ~0x50;
	P1 |= 0x50;
	//port 3
	P3MDOUT |= 0x40;        //set port 3 output bits high (LED on 3.6)
	P3MDOUT &= ~0x80;				//set port 3 slideswitch input bit low
	P3 |= 0x80; 						//set port 3 slideswitch input bit high
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
//
// Interrupt Service Routine for Programmable Counter Array Overflow Interrupt
//
void PCA_ISR () __interrupt 9 {
  if (CF) {
    CF = 0;               //clear overflow indicator
	count++;
	back_up++;
    a_count++;
	lcd_count++;
    if(a_count >= 1) {     //update every 20ms
      new_accel=1;
      a_count = 0;
    }
    if (lcd_count > 25) {
      new_lcd = 1;
      lcd_count = 0;
    }
	//toggle the backup led every second
	if (back_up > 50) {
		//if the motor PW is less than 2400, then toggle the
		//led because the car is moving backwords
		if (PW_motor < 2400) reverse = !reverse;
		else reverse = 1;
		back_up = 0;
	}
    PCA0 = PCA_start;
  }
  // handle other PCA interrupt sources
  PCA0CN &= 0xC0;
}
