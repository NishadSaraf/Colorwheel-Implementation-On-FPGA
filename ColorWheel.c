
/**
*
* @file ece544periph_test.c
*
* @author Nishad Saraf(nns4@pdx.edu)
*
* This file implements a program for displaying the colorwheel on PmodOLEDrgb. 
* This also requires PmodENC rotary encoder other than the onboard peripheral on Nexys4DDR.
*
******************************************************************************/
// header files
#include <stdio.h>
#include <stdlib.h>
#include "platform.h"
#include "xparameters.h"
#include "xstatus.h"
#include "nexys4IO.h"
#include "pmodENC.h"
#include "xgpio.h"
#include "xintc.h"
#include "xtmrctr.h"
#include "PmodOLEDrgb.h"

/************************** Constant Definitions ****************************/
// Clock frequencies
#define CPU_CLOCK_FREQ_HZ		XPAR_CPU_CORE_CLOCK_FREQ_HZ
#define AXI_CLOCK_FREQ_HZ		XPAR_CPU_M_AXI_DP_FREQ_HZ

// AXI timer parameters
#define AXI_TIMER_DEVICE_ID		XPAR_AXI_TIMER_0_DEVICE_ID
#define AXI_TIMER_BASEADDR		XPAR_AXI_TIMER_0_BASEADDR
#define AXI_TIMER_HIGHADDR		XPAR_AXI_TIMER_0_HIGHADDR
#define TmrCtrNumber			0

// Definitions for peripheral NEXYS4IO
#define NX4IO_DEVICE_ID		XPAR_NEXYS4IO_0_DEVICE_ID
#define NX4IO_BASEADDR		XPAR_NEXYS4IO_0_S00_AXI_BASEADDR
#define NX4IO_HIGHADDR		XPAR_NEXYS4IO_0_S00_AXI_HIGHADDR

// Definitions for peripheral PMODOLEDRGB
#define RGBDSPLY_DEVICE_ID		XPAR_PMODOLEDRGB_0_DEVICE_ID
#define RGBDSPLY_GPIO_BASEADDR	XPAR_PMODOLEDRGB_0_AXI_LITE_GPIO_BASEADDR
#define RGBDSPLY_GPIO_HIGHADDR	XPAR_PMODOLEDRGB_0_AXI_LITE_GPIO_HIGHADD
#define RGBDSPLY_SPI_BASEADDR	XPAR_PMODOLEDRGB_0_AXI_LITE_SPI_BASEADDR
#define RGBDSPLY_SPI_HIGHADDR	XPAR_PMODOLEDRGB_0_AXI_LITE_SPI_HIGHADDR

// Definitions for peripheral PMODENC
#define PMODENC_DEVICE_ID		XPAR_PMODENC_0_DEVICE_ID
#define PMODENC_BASEADDR		XPAR_PMODENC_0_S00_AXI_BASEADDR
#define PMODENC_HIGHADDR		XPAR_PMODENC_0_S00_AXI_HIGHADDR

// Fixed Interval timer - 100 MHz input clock, 40KHz output clock
// FIT_COUNT_1MSEC = FIT_CLOCK_FREQ_HZ * .001
#define FIT_IN_CLOCK_FREQ_HZ	CPU_CLOCK_FREQ_HZ
#define FIT_CLOCK_FREQ_HZ		40000
#define FIT_COUNT				(FIT_IN_CLOCK_FREQ_HZ / FIT_CLOCK_FREQ_HZ)
#define FIT_COUNT_1MSEC			40

// GPIO parameters
#define GPIO_0_DEVICE_ID			XPAR_AXI_GPIO_0_DEVICE_ID
#define GPIO_0_INPUT_0_CHANNEL		1
#define GPIO_0_OUTPUT_0_CHANNEL		2

#define GPIO_1_DEVICE_ID			XPAR_AXI_GPIO_1_DEVICE_ID
#define GPIO_1_INPUT_1_CHANNEL		1
#define GPIO_1_OUTPUT_1_CHANNEL		2

#define GPIO_2_DEVICE_ID			XPAR_AXI_GPIO_2_DEVICE_ID
#define GPIO_2_INPUT_2_CHANNEL		1
#define GPIO_2_OUTPUT_2_CHANNEL		2

#define GPIO_3_DEVICE_ID			XPAR_AXI_GPIO_3_DEVICE_ID
#define GPIO_3_INPUT_3_CHANNEL		1
#define GPIO_3_OUTPUT_3_CHANNEL		2

// Interrupt Controller parameters
#define INTC_DEVICE_ID			XPAR_INTC_0_DEVICE_ID
#define FIT_INTERRUPT_ID		XPAR_MICROBLAZE_0_AXI_INTC_FIT_TIMER_0_INTERRUPT_INTR

/**************************** Type Definitions ******************************/

/***************** Macros (Inline Functions) Definitions ********************/

/************************** Variable Definitions ****************************/
unsigned long timeStamp = 0;


/************************** Function Prototypes *****************************/
void usleep(u32 usecs);

void PMDIO_itoa(int32_t value, char *string, int32_t radix);
void PMDIO_puthex(PmodOLEDrgb* InstancePtr, uint32_t num);
void PMDIO_putnum(PmodOLEDrgb* InstancePtr, int32_t num, int32_t radix);
void ColorWheel();					// the main function which handles the flow of our project
void FIT_Handler(void);				// subroutine to be executed when the CPU is interrupted

int do_init_nx4io(u32 BaseAddress);
int do_init_pmdio(u32 BaseAddress);
int AXI_Timer_initialize(void);
int do_init();

PmodENC 	pmodENC_inst;				// PmodENC instance ref
PmodOLEDrgb	pmodOLEDrgb_inst;			// PmodOLED instance ref
XGpio		GPIOInst0;					// GPIO instance
XIntc 		IntrptCtlrInst;				// Interrupt Controller instance
XTmrCtr		AXITimerInst;				// PWM timer instance


XGpio		GPIOInst0,GPIOInst1,GPIOInst2,GPIOInst3;		// GPIO instance
uint16_t 	my_rgb;	// to store 565 bit rgb value afteer conversion from hsv scale
uint8_t 	R,G,B;



// The following variables are shared between non-interrupt processing and
// interrupt processing such that they must be global(and declared volatile)
// These variables are controlled by the FIT timer interrupt handler
// "clkfit" toggles each time the FIT interrupt handler is called so its frequency will
// be 1/2 FIT_CLOCK_FREQ_HZ.  timestamp increments every 1msec and is used in delay_msecs()
volatile unsigned int	clkfit;					// clock signal is bit[0] (rightmost) of gpio 0 output port


volatile u32			gpio_in;				// GPIO input port
volatile u32			high_value_sw_red=0;	// Value of high count from software
volatile u32  			low_value_sw_red=0;		// Value of low count from software
volatile u32			high_value_sw_blue=0;	// Value of high count from software
volatile u32  			low_value_sw_blue=0;	// Value of low count from software
volatile u32			high_value_sw_green=0;	// Value of high count from software
volatile u32  			low_value_sw_green=0;	// Value of low count from software

volatile u32 			count_high_red=0;		// Initializing the count_high value to 0
volatile u32			count_low_red=0;		// Initializing the count_low value to 0
volatile u32			old_pwm_red=0;			// Store the old pwm value to enable detection of rising or falling edge
volatile u32 			count_high_blue=0;		// Initializing the count_high value to 0
volatile u32			count_low_blue=0;		// Initializing the count_low value to 0
volatile u32			old_pwm_blue=0;			// Store the old pwm value to enable detection of rising or falling edge
volatile u32 			count_high_green=0;		// Initializing the count_high value to 0
volatile u32			count_low_green=0;		// Initializing the count_low value to 0
volatile u32			old_pwm_green=0;		// Store the old pwm value to enable detection of rising or falling edge

volatile u32            old_hue = 0;
volatile u32            old_Sat = 0;
volatile u32            old_Val = 0;

volatile u32			pwm_red=0;				// Store the current red pwm value
volatile u32			pwm_blue=0;				// Store the current blue pwm value
volatile u32			pwm_green=0;			// Store the current green pwm value
u16 					sw = 0;
u16                     switch1=0;				// stores the value read from the slide switches


//High Low Count
volatile u32            Red_high=0;
volatile u32            Red_low=0;
volatile u32            Green_high=0;
volatile u32            Green_low=0;
volatile u32            Blue_high=0;
volatile u32            Blue_low=0;

//PWM Software
volatile u32			pwm_red1=0;				// Store the current red pwm value
volatile u32			pwm_blue1=0;				// Store the current blue pwm value
volatile u32			pwm_green1=0;			// Store the current green pwm value

//Counted or not
volatile u32            high_R_count=0;
volatile u32            low_R_count=0;
volatile u32            high_G_count=0;
volatile u32            low_G_count=0;
volatile u32            high_B_count=0;
volatile u32            low_B_count=0;


u16  					RotaryCnt;				// stores the value from rotary encoder
uint64_t 				timestamp = 0L;			// used in delay msec


void hsv2rgb(int,int,int,u8*,u8*,u8*);		// function used to convert HSV values into RGB
u32 map(long, long, long, long, long);		// function used to map hue value to 0 to 359 scale

/************************** MAIN PROGRAM ************************************/
int main()
{
	int sts;
	init_platform();

	sts = do_init();		// initialize the peripherals
	if (XST_SUCCESS != sts)
	{
		exit(1);
	}

	microblaze_enable_interrupts();		// enable the interrupts
	// notify the user about the operation of slide switch SW0
	// clear this messaage after a delay of 2 seconds
	OLEDrgb_SetFontColor(&pmodOLEDrgb_inst ,OLEDrgb_BuildRGB(255, 255, 255));	// white color
	OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 0, 0);	//
	OLEDrgb_PutString(&pmodOLEDrgb_inst,"Select a ");
	OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 0, 1);	//
	OLEDrgb_PutString(&pmodOLEDrgb_inst,"method for ");
	OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 0, 2);	//
	OLEDrgb_PutString(&pmodOLEDrgb_inst,"PWM ");
	OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 0, 3);	//
	OLEDrgb_PutString(&pmodOLEDrgb_inst,"detection:");
	OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 0, 5);	//
	OLEDrgb_PutString(&pmodOLEDrgb_inst,"H/W- SW0 ON");
	OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 0, 6);	//
	OLEDrgb_PutString(&pmodOLEDrgb_inst,"S/W- SW0 OFF");
	usleep(2000000); //1 seconds
	OLEDrgb_Clear(&pmodOLEDrgb_inst);
	// call function to start our main opertions
	ColorWheel();

	// cause the loop to terminate
	timeStamp = 0;

	// blank the display digits and turn off the decimal points
	NX410_SSEG_setAllDigits(SSEGLO, CC_BLANK, CC_BLANK, CC_BLANK, CC_BLANK, DP_NONE);
	NX410_SSEG_setAllDigits(SSEGHI, CC_BLANK, CC_BLANK, CC_BLANK, CC_BLANK, DP_NONE);

	// Clear all the display digits and the OLED display at the end of the program
	OLEDrgb_Clear(&pmodOLEDrgb_inst);
	NX4IO_setLEDs(0x00000000);
	// switch the RGB LEDs OFF
	NX4IO_RGBLED_setChnlEn(RGB1, true, true, true);
	NX4IO_RGBLED_setDutyCycle(RGB1, 0, 0, 0);
	NX4IO_RGBLED_setChnlEn(RGB2, true, true, true);
	NX4IO_RGBLED_setDutyCycle(RGB2, 0, 0, 0);
	// hold a BYE BYE message on the seven segment display for 8 seconds
	NX410_SSEG_setAllDigits(SSEGHI, CC_BLANK, CC_B, CC_LCY, CC_E, DP_NONE);
	NX410_SSEG_setAllDigits(SSEGLO, CC_B, CC_LCY, CC_E, CC_BLANK, DP_NONE);
	usleep(800000);
	NX410_SSEG_setAllDigits(SSEGLO, CC_BLANK, CC_BLANK, CC_BLANK, CC_BLANK, DP_NONE);
	NX410_SSEG_setAllDigits(SSEGHI, CC_BLANK, CC_BLANK, CC_BLANK, CC_BLANK, DP_NONE);

	OLEDrgb_end(&pmodOLEDrgb_inst);
	cleanup_platform();
	exit(0);
}


/**
 * Function Name: do_init()
 *
 * Return: XST_FAILURE or XST_SUCCESS
 *
 * Description: Initialize the AXI timer, gpio, interrupt, FIT timer, Encoder,
 * 				OLED display
 */
int do_init()
{
	int sts;
	int status;
	// initialize the Nexys4 driver and (some of)the devices
	status = (uint32_t) NX4IO_initialize(NX4IO_BASEADDR);
	if (sts == XST_FAILURE)
	{
		exit(1);
	}
	// initialize the PMod544IO driver and the PmodENC and PmodCLP
	status = pmodENC_initialize(&pmodENC_inst, PMODENC_BASEADDR);
	if (sts == XST_FAILURE)
	{
		exit(1);
	}

	// initialize the GPIO instances
	status = XGpio_Initialize(&GPIOInst0, GPIO_0_DEVICE_ID);
	status = XGpio_Initialize(&GPIOInst1,GPIO_1_DEVICE_ID);
	status = XGpio_Initialize(&GPIOInst2,GPIO_2_DEVICE_ID);
	status = XGpio_Initialize(&GPIOInst3,GPIO_3_DEVICE_ID);


	if (status != XST_SUCCESS)
	{
		return XST_FAILURE;
	}

	// Initialize the AXI Timer
	status = AXI_Timer_initialize();
	if (status != XST_SUCCESS)
	{
		return XST_FAILURE;
	}

	// Initialize the OLED display
	OLEDrgb_begin(&pmodOLEDrgb_inst, RGBDSPLY_GPIO_BASEADDR, RGBDSPLY_SPI_BASEADDR);

	// initialize the GPIO instances
	status = XGpio_Initialize(&GPIOInst0, GPIO_0_DEVICE_ID);
	if (status != XST_SUCCESS)
	{
		return XST_FAILURE;
	}
	// GPIO0 channel 1 is an 8-bit input port.
	// GPIO0 channel 2 is an 8-bit output port.
	// GPIO 1, 2, and 3 has two 32-bit input port.
	XGpio_SetDataDirection(&GPIOInst0, GPIO_0_INPUT_0_CHANNEL, 0xFF);
	XGpio_SetDataDirection(&GPIOInst0, GPIO_0_OUTPUT_0_CHANNEL, 0x00);

	XGpio_SetDataDirection(&GPIOInst1, GPIO_1_INPUT_1_CHANNEL, 0xFF);
	XGpio_SetDataDirection(&GPIOInst1, GPIO_1_OUTPUT_1_CHANNEL, 0x00);

	XGpio_SetDataDirection(&GPIOInst2, GPIO_2_INPUT_2_CHANNEL, 0xFF);
	XGpio_SetDataDirection(&GPIOInst2, GPIO_2_OUTPUT_2_CHANNEL, 0x00);

	XGpio_SetDataDirection(&GPIOInst3, GPIO_3_INPUT_3_CHANNEL, 0xFF);
	XGpio_SetDataDirection(&GPIOInst3, GPIO_3_OUTPUT_3_CHANNEL, 0x00);


	// initialize the interrupt controller
	status = XIntc_Initialize(&IntrptCtlrInst, INTC_DEVICE_ID);
	if (status != XST_SUCCESS)
	{
		return XST_FAILURE;
	}

	// connect the fixed interval timer (FIT) handler to the interrupt
	status = XIntc_Connect(&IntrptCtlrInst, FIT_INTERRUPT_ID,
			(XInterruptHandler)FIT_Handler,
			(void *)0);
	if (status != XST_SUCCESS)
	{
		return XST_FAILURE;

	}

	// start the interrupt controller such that interrupts are enabled for
	// all devices that cause interrupts.
	status = XIntc_Start(&IntrptCtlrInst, XIN_REAL_MODE);
	if (status != XST_SUCCESS)
	{
		return XST_FAILURE;
	}

	// enable the FIT interrupt
	XIntc_Enable(&IntrptCtlrInst, FIT_INTERRUPT_ID);
	return XST_SUCCESS;
}

/* timer initializes it to generate out a 4Khz signal, Which is given to the Nexys4IO module as clock input.
 * DO NOT MODIFY
 */
int AXI_Timer_initialize(void){

	uint32_t status;				// status from Xilinx Lib calls
	u32		ctlsts;		// control/status register or mask

	status = XTmrCtr_Initialize(&AXITimerInst,AXI_TIMER_DEVICE_ID);
	if (status != XST_SUCCESS) {
		return XST_FAILURE;
	}
	status = XTmrCtr_SelfTest(&AXITimerInst, TmrCtrNumber);
	if (status != XST_SUCCESS) {
		return XST_FAILURE;
	}
	ctlsts = XTC_CSR_AUTO_RELOAD_MASK | XTC_CSR_EXT_GENERATE_MASK | XTC_CSR_LOAD_MASK |XTC_CSR_DOWN_COUNT_MASK ;
	XTmrCtr_SetControlStatusReg(AXI_TIMER_BASEADDR, TmrCtrNumber,ctlsts);

	//Set the value that is loaded into the timer counter and cause it to be loaded into the timer counter
	XTmrCtr_SetLoadReg(AXI_TIMER_BASEADDR, TmrCtrNumber, 24998);
	XTmrCtr_LoadTimerCounterReg(AXI_TIMER_BASEADDR, TmrCtrNumber);
	ctlsts = XTmrCtr_GetControlStatusReg(AXI_TIMER_BASEADDR, TmrCtrNumber);
	ctlsts &= (~XTC_CSR_LOAD_MASK);
	XTmrCtr_SetControlStatusReg(AXI_TIMER_BASEADDR, TmrCtrNumber, ctlsts);

	ctlsts = XTmrCtr_GetControlStatusReg(AXI_TIMER_BASEADDR, TmrCtrNumber);
	ctlsts |= XTC_CSR_ENABLE_TMR_MASK;
	XTmrCtr_SetControlStatusReg(AXI_TIMER_BASEADDR, TmrCtrNumber, ctlsts);

	XTmrCtr_Enable(AXI_TIMER_BASEADDR, TmrCtrNumber);
	return XST_SUCCESS;

}
/*
* This function is reponsible for reading, displaying, and updating the values of H, S, and V on the 
* OLED display. Also the switch SW0 is read in this function itself.
*
*/
void ColorWheel()
{

	int  RotaryIncr;
	int Value=0;
	int Saturation=0;
	u32 Hue = 0;
	volatile u32			pwm_red2=0;				// Store the current red pwm value
	volatile u32			pwm_blue2=0;				// Store the current blue pwm value
	volatile u32			pwm_green2=0;

	bool RotaryNoNeg;
	bool dispSetFlg = false;			// used to reset the display row of the encoder value

	// test the rotary encoder functions
	RotaryIncr = 1;
	RotaryNoNeg = false;	// set false to avoid the value of rotary encoder to go in negative

	// Initialize the rotary encoder
	// clear the counter of the encoder if initialized to garbage value on power on
	pmodENC_init(&pmodENC_inst, RotaryIncr, RotaryNoNeg);
	pmodENC_clear_count(&pmodENC_inst);
	

	while(1)
	{
		// read for the SW0
		// if high implies hardware detection approach is choosen
		// if low then software detection approach is choosen
		switch1 = NX4IO_getSwitches();
		if(switch1){
		NX4IO_setLEDs(0x00000001);	// set the LED on SW0 high to indicate slection is been recorded
		OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 0, 63);
		OLEDrgb_PutString(&pmodOLEDrgb_inst,"H/W");
		}
		else{
		NX4IO_setLEDs(0x00000000);
		OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 0, 63);
		OLEDrgb_PutString(&pmodOLEDrgb_inst,"S/W");
		}
		
		// check if the rotary encoder pushbutton or BTNC is pressed
		// exit the loop if either one is pressed.
		if ( pmodENC_is_button_pressed(&pmodENC_inst)||NX4IO_isPressed(BTNC) )
		{
			break; // breaking the loop will terminate the program
		}
		// check the buttons and perform the appropriate action
		if (NX4IO_isPressed(BTNU))	// clear the rotary count
		{
			if(Value > 98 )
				Value = 0;		// if value tries to go above 99 reset it to zero
			else
				Value= Value + 1;	// orelse keep on incrementing the value by 1
			dispSetFlg = false;			// clear the row of the display which shows the value of Enc in decimal
		}  // end clear the rotary count

		else if (NX4IO_isPressed(BTND))	// clear the rotary count
		{
			if(Value == 0)
				Value = 99;		// if the value goes low below 0 calibrate it to 99 so that the range of value is between 0 to 99	
			else
				Value= Value - 1;	// orelse decreament the value by 1
			dispSetFlg = false;			// clear the row of the display which shows the value of Enc in decimal
		}  
		else if (NX4IO_isPressed(BTNR))		// toggle no-neg flag (may not be reliable)
		{
			if(Saturation > 98)
				Saturation = 0;
			else
				Saturation = Saturation + 1;
			if (RotaryNoNeg)	//No Neg was enabled
			{
				pmodENC_init(&pmodENC_inst, RotaryIncr, false);
				RotaryNoNeg = false;
			}
			else	// No Neg was disabled
			{
				pmodENC_init(&pmodENC_inst, RotaryIncr, true);
				RotaryNoNeg = true;		// enable the no negative mode of the rotary encoder to stop at 0
			}
		}
		else if (NX4IO_isPressed(BTNL))
		{
			if(Saturation == 0)
				Saturation = 99;
			else
				Saturation = Saturation - 1;
		}

		pmodENC_read_count(&pmodENC_inst, &RotaryCnt);

		if((RotaryCnt == 0) && (!dispSetFlg))		// cleared by pressing the UP button
		{// The dispSetFlg is used to avoid the flickering of the display when showing 0 value
			OLEDrgb_PutString(&pmodOLEDrgb_inst, "         ");	// used to clear the row of the display
			dispSetFlg = true;
		
		// as the max value of rotary count is 65535 but the required range is from 0 to 359 mapping should be done
		Hue = map(RotaryCnt, 0, 65535, 0, 359);

		if(RotaryCnt > 359) RotaryCnt = 0;
		if(old_hue != RotaryCnt )
		{
			old_hue = RotaryCnt;
		}
		if(old_Sat!=Saturation)
		{
			old_Sat = Saturation;
		}
		if(old_Val!=Value)
		{
			old_Val = Value;
		}
		// set the font color for displayign H, S, and V values 
		OLEDrgb_SetFontColor(&pmodOLEDrgb_inst,OLEDrgb_BuildRGB(255, 255, 255));
		OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 0, 1);
		OLEDrgb_PutString(&pmodOLEDrgb_inst,"H:");
		// Load H value
		OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 2, 1);		// reset the cursor to the location
		PMDIO_putnum(&pmodOLEDrgb_inst, RotaryCnt, 10);	// show the number in decimal form
		OLEDrgb_PutChar(&pmodOLEDrgb_inst, '*');

		OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 0, 3);
		OLEDrgb_PutString(&pmodOLEDrgb_inst,"S:");
		// Load S value
		OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 2, 3);		// reset the cursor to the location 
		PMDIO_putnum(&pmodOLEDrgb_inst, Saturation, 10);	// show the number in decimal form
		OLEDrgb_PutChar(&pmodOLEDrgb_inst, '%');
		OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 0, 5);
		OLEDrgb_PutString(&pmodOLEDrgb_inst,"V:");
		// Load V value
		OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 2, 5);		// reset the cursor to the location 
		PMDIO_putnum(&pmodOLEDrgb_inst,Value, 10);// show the number in decimal form
		OLEDrgb_PutChar(&pmodOLEDrgb_inst, '%');

		// display the count on the LEDs and seven segment display, too
		NX4IO_setLEDs(RotaryCnt);
		// convert the HSV values to RGB scale
		// conversion is necessary as OLED and RGB LEDs accept values only in the RGB scale
		hsv2rgb(RotaryCnt,Saturation,Value,&R, &G, &B);
		// convert the 8 bit RGB value to 565-bit single RGB value
		my_rgb=OLEDrgb_BuildRGB(R,G,B);
		// draw a rectangle from column #52 to column #95 and row #0 to row #63
		// laso fill this rectangle with my_rgb color
		OLEDrgb_DrawRectangle(&pmodOLEDrgb_inst,52,0,95,63,my_rgb,true,my_rgb);

		// copy the same color as the color inside the OLED rectangle to RGB LEDs
		NX4IO_RGBLED_setChnlEn(RGB1, true, true, true);
		NX4IO_RGBLED_setDutyCycle(RGB1, R, G, B);

		NX4IO_RGBLED_setChnlEn(RGB2, true, true, true);
		NX4IO_RGBLED_setDutyCycle(RGB2, R, G, B);
		
		// high count and low count corresponding to red, green, and blue are read from the GPIO2 and GPIO output ports respectively.
		// this value are obtained from hardware detection
		count_high_red = XGpio_DiscreteRead(&GPIOInst1, GPIO_1_INPUT_1_CHANNEL);
		count_low_red = XGpio_DiscreteRead(&GPIOInst1, GPIO_1_OUTPUT_1_CHANNEL);
		// calculate the duty cycle
		pwm_red=(count_high_red * 100)/(count_high_red + count_low_red);
		// high_value_sw_red and low_value_sw_red are the value obtained from the software detection method
		pwm_red1= (high_value_sw_red * 100) /(high_value_sw_red + low_value_sw_red);

		count_high_green = XGpio_DiscreteRead(&GPIOInst2, GPIO_2_INPUT_2_CHANNEL);
		count_low_green =  XGpio_DiscreteRead(&GPIOInst2, GPIO_2_OUTPUT_2_CHANNEL);
		pwm_green=(count_high_green * 100)/(count_high_green + count_low_green);
		pwm_green1= (high_value_sw_green * 100) /(high_value_sw_green + low_value_sw_green);


		count_high_blue = XGpio_DiscreteRead(&GPIOInst3, GPIO_3_INPUT_3_CHANNEL);
		count_low_blue = XGpio_DiscreteRead(&GPIOInst3, GPIO_3_OUTPUT_3_CHANNEL);
		pwm_blue=(count_high_blue * 100)/(count_high_blue + count_low_blue);
		pwm_blue1= (high_value_sw_blue * 100) /(high_value_sw_blue + low_value_sw_blue);
		// according to the value on SW0 values obtained from different detection method is choosen
		switch1 ? ( pwm_red2 = pwm_red) : (pwm_red2 = pwm_red1);
		switch1 ?( pwm_blue2 = pwm_blue) : (pwm_blue2=pwm_blue1);
		switch1 ?( pwm_green2 = pwm_green) : (pwm_green2=pwm_green1);
		usleep(30000);	 // this delay is useful for stablilizing the seven segment display which minimizes the flickering
		NX4IO_SSEG_setDigit(SSEGHI, DIGIT7, (pwm_red2/10));	// used to extract the units position from the 2 digit number
		NX4IO_SSEG_setDigit(SSEGHI, DIGIT6, (pwm_red2%10));	// used to extract the tens position from the 2 digit number
		NX4IO_SSEG_setDigit(SSEGHI, DIGIT4, (pwm_green2/10));
		NX4IO_SSEG_setDigit(SSEGLO, DIGIT3, (pwm_green2%10));
		NX4IO_SSEG_setDigit(SSEGLO, DIGIT1, (pwm_blue2/10));
		NX4IO_SSEG_setDigit(SSEGLO, DIGIT0, (pwm_blue2%10));
		NX4IO_SSEG_setDigit(SSEGHI, DIGIT5, CC_BLANK);	// disable digit 5 and digit 2 onnthe seven segment display to provide better visual effect
		NX4IO_SSEG_setDigit(SSEGLO, DIGIT2, CC_BLANK);
		
		gpio_in = XGpio_DiscreteRead(&GPIOInst0, GPIO_0_INPUT_0_CHANNEL);
	} // rotary button has been pressed - exit the loop

	// Write one final string
	OLEDrgb_Clear(&pmodOLEDrgb_inst);

	return;
}


/****************************************************************************/
/**
 * insert delay (in microseconds) between instructions.
 *
 * This function should be in libc but it seems to be missing.  This emulation implements
 * a delay loop with (really) approximate timing; not perfect but it gets the job done.
 *
 * @param	usec is the requested delay in microseconds
 *
 * @return	*NONE*
 *
 * @note
 * This emulation assumes that the microblaze is running @ 100MHz and takes 15 clocks
 * per iteration - this is probably totally bogus but it's a start.
 *
 *****************************************************************************/

static const u32	DELAY_1US_CONSTANT	= 15;	// constant for 1 microsecond delay

void usleep(u32 usec)
{
	volatile u32 i, j;

	for (i = 0; i < usec; i++)
	{
		for (j = 0; j < DELAY_1US_CONSTANT; j++);
	}
	return;
}


/****************************************************************************/
/**
 * initialize the Nexys4 LEDs and seven segment display digits
 *
 * Initializes the NX4IO driver, turns off all of the LEDs and blanks the seven segment display
 *
 * @param	BaseAddress is the memory mapped address of the start of the Nexys4 registers
 *
 * @return	XST_SUCCESS if initialization succeeds.  XST_FAILURE otherwise
 *
 * @note
 * The NX4IO_initialize() function calls the NX4IO self-test.  This could
 * cause the program to hang if the hardware was not configured properly
 *
 *****************************************************************************/
int do_init_nx4io(u32 BaseAddress)
{
	int sts;

	// initialize the NX4IO driver
	sts = NX4IO_initialize(BaseAddress);
	if (sts == XST_FAILURE)
		return XST_FAILURE;

	// turn all of the LEDs off using the "raw" set functions
	// functions should mask out the unused bits..something to check w/
	// the debugger when we bring the drivers up for the first time
	NX4IO_setLEDs(0xFFF0000);
	NX4IO_RGBLED_setRGB_DATA(RGB1, 0xFF000000);
	NX4IO_RGBLED_setRGB_DATA(RGB2, 0xFF000000);
	NX4IO_RGBLED_setRGB_CNTRL(RGB1, 0xFFFFFFF0);
	NX4IO_RGBLED_setRGB_CNTRL(RGB2, 0xFFFFFFFC);

	// set all of the display digits to blanks and turn off
	// the decimal points using the "raw" set functions.
	// These registers are formatted according to the spec
	// and should remain unchanged when written to Nexys4IO...
	// something else to check w/ the debugger when we bring the
	// drivers up for the first time
	NX4IO_SSEG_setSSEG_DATA(SSEGHI, 0x0058E30E);
	NX4IO_SSEG_setSSEG_DATA(SSEGLO, 0x00144116);

	return XST_SUCCESS;

}

/*********************** DISPLAY-RELATED FUNCTIONS ***********************************/

/****************************************************************************/
/**
 * Converts an integer to ASCII characters
 *
 * algorithm borrowed from ReactOS system libraries
 *
 * Converts an integer to ASCII in the specified base.  Assumes string[] is
 * long enough to hold the result plus the terminating null
 *
 * @param 	value is the integer to convert
 * @param 	*string is a pointer to a buffer large enough to hold the converted number plus
 *  			the terminating null
 * @param	radix is the base to use in conversion,
 *
 * @return  *NONE*
 *
 * @note
 * No size check is done on the return string size.  Make sure you leave room
 * for the full string plus the terminating null in string
 *****************************************************************************/
void PMDIO_itoa(int32_t value, char *string, int32_t radix)
{
	char tmp[33];
	char *tp = tmp;
	int32_t i;
	uint32_t v;
	int32_t  sign;
	char *sp;

	if (radix > 36 || radix <= 1)
	{
		return;
	}

	sign = ((10 == radix) && (value < 0));
	if (sign)
	{
		v = -value;
	}
	else
	{
		v = (uint32_t) value;
	}

	while (v || tp == tmp)
	{
		i = v % radix;
		v = v / radix;
		if (i < 10)
		{
			*tp++ = i+'0';
		}
		else
		{
			*tp++ = i + 'a' - 10;
		}
	}
	sp = string;

	if (sign)
		*sp++ = '-';

	while (tp > tmp)
		*sp++ = *--tp;
	*sp = 0;

	return;
}


/****************************************************************************/
/**
 * Write a 32-bit unsigned hex number to PmodOLEDrgb in Hex
 *
 * Writes  32-bit unsigned number to the pmodOLEDrgb display starting at the current
 * cursor position.
 *
 * @param num is the number to display as a hex value
 *
 * @return  *NONE*
 *
 * @note
 * No size checking is done to make sure the string will fit into a single line,
 * or the entire display, for that matter.  Watch your string sizes.
 *****************************************************************************/
void PMDIO_puthex(PmodOLEDrgb* InstancePtr, uint32_t num)
{
	char  buf[9];

	int32_t   cnt;
	char  *ptr;
	int32_t  digit;

	ptr = buf;
	for (cnt = 7; cnt >= 0; cnt--) {
		digit = (num >> (cnt * 4)) & 0xF;

		if (digit <= 9)
		{
			*ptr++ = (char) ('0' + digit);
		}
		else
		{
			*ptr++ = (char) ('a' - 10 + digit);
		}
	}

	*ptr = (char) 0;
	OLEDrgb_PutString(InstancePtr,buf);

	return;
}


/****************************************************************************/
/**
 * Write a 32-bit number in Radix "radix" to LCD display
 *
 * Writes a 32-bit number to the LCD display starting at the current
 * cursor position. "radix" is the base to output the number in.
 *
 * @param num is the number to display
 *
 * @param radix is the radix to display number in
 *
 * @return *NONE*
 *
 * @note
 * No size checking is done to make sure the string will fit into a single line,
 * or the entire display, for that matter.  Watch your string sizes.
 *****************************************************************************/
void PMDIO_putnum(PmodOLEDrgb* InstancePtr, int32_t num, int32_t radix)
{
	char  buf[16];

	PMDIO_itoa(num, buf, radix);
	OLEDrgb_PutString(InstancePtr,buf);

	return;
}

/**************************** INTERRUPT HANDLER ******************************/

/****************************************************************************/
/**
* Fixed interval timer interrupt handler
*
* Reads the GPIO port which reads back the hardware generated PWM wave for the RGB Leds
* This function is invoked when the CPU is interrupted.
* Save the final count value in the variables high_value_sw_<color> and low_value_sw_<color> defined globally.
*
 *****************************************************************************/
void FIT_Handler(void)
{
	// Read the GPIO port to read back the generated PWM signal for RGB led's
	gpio_in = XGpio_DiscreteRead(&GPIOInst0, GPIO_0_INPUT_0_CHANNEL);
	
	// mask everything other than the second bit of gpio_in to read count corresponding to red color 
	// <Color>_high and Color_low are the temporary counters to store the high count and low count ]
	// high_<C>_count and low_<C>_count are the flags used to know a whether new input cycle has begun or not
	if ((gpio_in & 0x0004) == 0x0004)
	{
		// when the next cycle starts output the count value by assigning the temporary value to global variables counters
		if(high_R_count == 1 && low_R_count == 1)
		{
			high_value_sw_red= Red_high;	
			low_value_sw_red= Red_low;

			Red_high = 1;	// as one count is wasted to mark the start of new cycle initialize the counter from 1 instead of 0
			Red_low = 0;

			high_R_count = 0;
			low_R_count = 0;
		}
		else
		{
			high_R_count = 1;
			Red_high= Red_high + 1;
		}
	}
	// when signal goes low
	else
	{
		if(high_R_count == 1)
		{
			low_R_count = 1;
			Red_low = Red_low + 1;
		}
		else
		{
			low_R_count = 0;
		}
	}
	// mask other bits to read green color component
	if((gpio_in & 1) == 1)
	{
		if(high_G_count == 1 && low_G_count == 1)
		{
			high_value_sw_green= Green_high;
			low_value_sw_green= Green_low;

			Green_high = 1;
			Green_low = 0;

			high_G_count = 0;
			low_G_count = 0;
		}

		else
		{
			high_G_count = 1;
			Green_high= Green_high + 1;
		}
	}
	else
	{
		if(high_G_count == 1)
		{
			low_G_count = 1;
			Green_low = Green_low + 1;
		}
		else
		{
			low_G_count = 0;
		}
	}
	// mask other bits to read blue color component 
	if((gpio_in & 2) == 2)
	{
		if(high_B_count == 1 && low_B_count == 1)
		{
			high_value_sw_blue= Blue_high;
			low_value_sw_blue= Blue_low;

			Blue_high = 1;
			Blue_low = 0;

			high_B_count = 0;
			low_B_count = 0;
		}
		else
		{
			high_B_count = 1;
			Blue_high = Blue_high + 1;
		}
	}
	else
	{
		if(high_B_count == 1)
		{
			low_B_count = 1;
			Blue_low = Blue_low + 1;
		}
		else
		{
			low_B_count = 0;
		}
	}
}

/***
**	
**	Parameters:
**		hue		- Hue of color
**		sat		- Saturation of color
**		val		- Value of color
**
**	Return Value:
**		none
**	Errors:
**		none
**
**	Description:
**	Function to convert HSV values to RGB scale,
**	Along with H, S and V accepts pointers to store the converted R, G abd B value.
**
**
*/
void hsv2rgb(int hue,int sat,int value,u8 *r,u8 *g,u8 *b)
{
	double       p, q, t, ff;
	double R,G,B;
	long        i;
	double hh;
	double h,s,v;
	h=hue;
	s=sat/100.0;
	v=value/100.0;

	if(s <= 0.0) {
		*r = v;
		*g = v;
		*b = v;
	}
	hh = h;
	if(hh >= 360.0) hh = 0.0;
	hh /= 60.0;
	i = (long)hh;
	ff = hh - i;
	p = v * (1.0 - s);
	q = v * (1.0 - (s * ff));
	t = v * (1.0 - (s * (1.0 - ff)));

	switch(i) {
	case 0:
		R = v;
		G = t;
		B = p;
		break;
	case 1:
		R = q;
		G = v;
		B = p;
		break;
	case 2:
		R = p;
		G = v;
		B = t;
		break;

	case 3:
		R = p;
		G = q;
		B = v;
		break;
	case 4:
		R = t;
		G = p;
		B = v;
		break;
	case 5:
	default:
		R = v;
		G = p;
		B = q;
		break;
	}
	*r=(R)*255;
	*b=(B)*255;
	*g=(G)*255;
}

/*
* This is a function specifically to map hue value.
* x input value to be mapped
* in_min least possible input value
* in_max maximum possible input value
* out_min least possible output value
* out_max maximum possible output value	
* return 32 - bit mapped value
*/
u32 map(long x, long in_min, long in_max, long out_min, long out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
