/************************************************************************/
/*																		*/
/*	main.c	--	Main program module for project							*/
/*																		*/
/************************************************************************/
/*	Author: 	Dion Moses												*/
/*	Copyright 2009, Digilent Inc.										*/
/************************************************************************/
/*  Module Description: 												*/
/*																		*/
/*	This program is a reference design for the Digilent	Basic			*/
/*	Robotic Development Kit (RDK-Basic) with the Cerebot 32MX4 			*/
/*	Microcontroller board.  It uses two timers to drive two motors 		*/
/*	with output compare modules.										*/
/*																		*/
/*																		*/
/************************************************************************/
/*  Revision History:													*/
/*																		*/
/*	 12/09/09(DionM): created											*/
/*   12/29/09(LeviB): altered to add movement functions and PmodBtn and */
/*					  PmodSwt functionality								*/
/*	 12/08/10(AaronO): renamed to RDK_Basic								*/	
/************************************************************************/

/* ------------------------------------------------------------ */
/*				Include File Definitions						*/
/* ------------------------------------------------------------ */

#include <plib.h>
#include "stdtypes.h"
#include "config.h"
#include "MtrCtrl.h"
#include "spi.h"
#include "util.h"

/* ------------------------------------------------------------ */
/*				Local Type Definitions							*/
/* ------------------------------------------------------------ */



#define		TCKPS22 			6
#define 	TCKPS21				5
#define 	TCKPS20				4

#define		TCKPS32 			6
#define 	TCKPS31				5
#define 	TCKPS30				4

#define     spdFac              75000
/* ------------------------------------------------------------ */
/*				Global Variables								*/
/* ------------------------------------------------------------ */
#ifndef OVERRIDE_CONFIG_BITS

#pragma config ICESEL   = ICS_PGx2		// ICE/ICD Comm Channel Select
#pragma config BWP      = OFF			// Boot Flash Write Protect
#pragma config CP       = OFF			// Code Protect
#pragma config FNOSC    = PRIPLL		// Oscillator Selection
#pragma config FSOSCEN  = OFF			// Secondary Oscillator Enable
#pragma config IESO     = OFF			// Internal/External Switch-over
#pragma config POSCMOD  = HS			// Primary Oscillator
#pragma config OSCIOFNC = OFF			// CLKO Enable
#pragma config FPBDIV   = DIV_8			// Peripheral Clock divisor
#pragma config FCKSM    = CSDCMD		// Clock Switching & Fail Safe Clock Monitor
#pragma config WDTPS    = PS1			// Watchdog Timer Postscale
#pragma config FWDTEN   = OFF			// Watchdog Timer 
#pragma config FPLLIDIV = DIV_2			// PLL Input Divider
#pragma config FPLLMUL  = MUL_16		// PLL Multiplier
#pragma config UPLLIDIV = DIV_2			// USB PLL Input Divider
#pragma config UPLLEN   = OFF			// USB PLL Enabled
#pragma config FPLLODIV = DIV_1			// PLL Output Divider
#pragma config PWP      = OFF			// Program Flash Write Protect
#pragma config DEBUG    = OFF			// Debugger Enable/Disable
    
#endif

/* ------------------------------------------------------------ */
/*				Local Variables									*/
/* ------------------------------------------------------------ */

#define	stPressed	1
#define	stReleased	0

#define	cstMaxCnt	10 // number of consecutive reads required for
					   // the state of a button to be updated

float Kp = 250.0;
float Ki = 35.0;
float Kd = 3.50;
float DeasVal = 8;

int IC2count = 0;
int IC3count = 0;
int Timer5count =0;
int timeArray_IC2[20];
int timeArray_IC3[20];
int diffArray_IC2[20];
int diffArray_IC3[20];
float speedArray_IC3[20];
float speedArray_IC2[20];
float avgSpeed_IC2 = 0.00;
float avgSpeed_IC3 = 0.00;
float alpha = 0.95;
float avgSpeed_IC2_array[500];
float deasVal_array[500] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,
12.5,12.5,12.5,12.5,12.5,12.5,12.5,12.5,12.5,12.5,12.5,12.5,12.5,12.5,12.5,12.5,12.5,12.5,12.5,12.5,12.5,12.5,12.5,12.5,12.5,12.5,12.5,12.5,12.5,12.5,12.5,12.5,12.5,12.5,12.5,12.5,12.5,12.5,12.5,12.5,12.5,12.5,12.5,12.5,12.5,12.5,12.5,12.5,12.5,12.5,
17.5,17.5,17.5,17.5,17.5,17.5,17.5,17.5,17.5,17.5,17.5,17.5,17.5,17.5,17.5,17.5,17.5,17.5,17.5,17.5,17.5,17.5,17.5,17.5,17.5,17.5,17.5,17.5,17.5,17.5,17.5,17.5,17.5,17.5,17.5,17.5,17.5,17.5,17.5,17.5,17.5,17.5,17.5,17.5,17.5,17.5,17.5,17.5,17.5,17.5,
20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,
20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,
17.5,17.5,17.5,17.5,17.5,17.5,17.5,17.5,17.5,17.5,17.5,17.5,17.5,17.5,17.5,17.5,17.5,17.5,17.5,17.5,17.5,17.5,17.5,17.5,17.5,17.5,17.5,17.5,17.5,17.5,17.5,17.5,17.5,17.5,17.5,17.5,17.5,17.5,17.5,17.5,17.5,17.5,17.5,17.5,17.5,17.5,17.5,17.5,17.5,17.5,
12.5,12.5,12.5,12.5,12.5,12.5,12.5,12.5,12.5,12.5,12.5,12.5,12.5,12.5,12.5,12.5,12.5,12.5,12.5,12.5,12.5,12.5,12.5,12.5,12.5,12.5,12.5,12.5,12.5,12.5,12.5,12.5,12.5,12.5,12.5,12.5,12.5,12.5,12.5,12.5,12.5,12.5,12.5,12.5,12.5,12.5,12.5,12.5,12.5,12.5,
10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
int arrayCount = 0;


float ei_2 = 0;
float ei_3 = 0;
float prev_error_2 = 0;
float prev_error_3 = 0;
float error_2 = 0;
float error_3 = 0;
float ei_max = 400;
float ei_min = -400;
int controlSig = 0;
int controlval = 0;


struct btn {
	BYTE	stBtn;	// status of the button (pressed or released)
	BYTE	stCur;  // current read state of the button
	BYTE	stPrev; // previous read state of the button
	BYTE	cst;	// number of consecutive reads of the same button 
					// state
};

//PmodCLS instructions
static	char szClearScreen[] = { 0x1B, '[', 'j', 0};

static	char szCursorOff[] = { 0x1B, '[', '0', 'c', 0 };
static	char szBacklightOn[]     = { 0x1B, '[', '3', 'e', 0 };

static	char szScrollLeft[] = {0x1B, '[', '1', '@', 0}; 
static	char szScrollRight[] = {0x1B, '[', '1', 'A', 0}; 
static	char szWrapMode[] = {0x1B, '[', '0', 'h', 0}; 

static	char szCursorPos00[] = {0x1B, '[', '0', ';', '0', 'H', 0};
static	char szCursorPos10[] = {0x1B, '[', '1', ';', '0', 'H', 0}; 
/* ------------------------------------------------------------ */
/*				Global Variables				                */
/* ------------------------------------------------------------ */

volatile	struct btn	btnBtn1;
volatile	struct btn	btnBtn2;

volatile	struct btn	PmodBtn1;
volatile	struct btn	PmodBtn2;
volatile	struct btn	PmodBtn3;
volatile	struct btn	PmodBtn4;

volatile	struct btn	PmodSwt1;
volatile	struct btn	PmodSwt2;
volatile	struct btn	PmodSwt3;
volatile	struct btn	PmodSwt4;

/* ------------------------------------------------------------ */
/*				Forward Declarations							*/
/* ------------------------------------------------------------ */

void	DeviceInit(void);
void	AppInit(void);
void	Wait_ms(WORD ms);

/* ------------------------------------------------------------ */
/*				Interrupt Service Routines						*/
/* ------------------------------------------------------------ */
/***	Timer5Handler
**
**	Parameters:
**		none
**
**	Return Value:
**		none
**
**	Errors:
**		none
**
**	Description:
**		Interrupt service routine for Timer 5 interrupt. Timer 5
**		is used to perform software debouncing of the on-board
**		buttons. It is also used as a time base for updating
**		the on-board LEDs and the Pmod8LD LEDs at a regular interval.
*/

void __ISR(_INPUT_CAPTURE_2_VECTOR, ipl5) _IC2_IntHandler(void) //JD3
{
    static uint16_t i = 0;
    static uint16_t j = 0;
    
    mIC2ClearIntFlag(); // clear interrupt flag for Input Capture 2
    //int bp = 100;
        while(1<<3 & IC2CON){
           timeArray_IC2[i] = 0x0000FFFF & IC2BUF;
           if(i == 0){
                diffArray_IC2[i] = timeArray_IC2[i] - timeArray_IC2[19];
            }
            else{
                diffArray_IC2[i] = timeArray_IC2[i] - timeArray_IC2[i-1];
            }
            if(diffArray_IC2[i] < 0){
                diffArray_IC2[i] = diffArray_IC2[i] + 65000;
            }
            if(diffArray_IC2[i] < 2000){
                diffArray_IC2[i] = 2000;
            }
            speedArray_IC2[i] = 75000 / diffArray_IC2[i];
            avgSpeed_IC2 = alpha*avgSpeed_IC2 + speedArray_IC2[i]*(1-alpha);
        }
    IC2count++; // increment counter
   /* if(IC2count == 3200){
        OC2R = 0;
        OC2RS = 0;
    }
   */
    if (i++ > 19){
        i=0;
    }
}

void __ISR(_INPUT_CAPTURE_3_VECTOR, ipl5) _IC3_IntHandler(void) //JD9
{
    static uint16_t i = 0;
    static uint16_t j = 0;
   
    mIC3ClearIntFlag(); // clear interrupt flag for Input Capture 3
        while(1<<3 & IC3CON){
            timeArray_IC3[i] = 0x0000FFFF & IC3BUF;
            if(i == 0){
                diffArray_IC3[i] = timeArray_IC3[i] - timeArray_IC3[19];
            }
            else{
                diffArray_IC3[i] = timeArray_IC3[i] - timeArray_IC3[i-1];
            }
            if(diffArray_IC3[i] < 0){
                diffArray_IC3[i] = diffArray_IC3[i] + 65000;
            }
            if(diffArray_IC3[i] < 2000){
                diffArray_IC3[i] = 2000;
            }
            speedArray_IC3[i] = 75000 / diffArray_IC3[i];
            avgSpeed_IC3 = alpha*avgSpeed_IC3 + speedArray_IC3[i]*(1-alpha);
        }
    IC3count++; // increment counter
    
    
       /* if(IC3count == 3200){
        OC3R = 0;
        OC3RS = 0;
    }
     */
// increment counter
     if (i++ > 19){
        i=0;
    }
}

void __ISR(_TIMER_5_VECTOR, ipl7) Timer5Handler(void)
{
	static	WORD tusLeds = 0;
    static  uint16_t j;
    static int p_localcount2 = 0;
    static int p_localcount3 = 0;
	
	mT5ClearIntFlag();
	
	// Read the raw state of the button pins.
	btnBtn1.stCur = ( prtBtn1 & ( 1 << bnBtn1 ) ) ? stPressed : stReleased;
	btnBtn2.stCur = ( prtBtn2 & ( 1 << bnBtn2 ) ) ? stPressed : stReleased;
	
	//Read the raw state of the PmodBTN pins
	PmodBtn1.stCur = ( prtJE1 & ( 1 << bnJE1 ) ) ? stPressed : stReleased;
	PmodBtn2.stCur = ( prtJE2 & ( 1 << bnJE2 ) ) ? stPressed : stReleased;
	PmodBtn3.stCur = ( prtJE3 & ( 1 << bnJE3 ) ) ? stPressed : stReleased;
	PmodBtn4.stCur = ( prtJE4 & ( 1 << bnJE4 ) ) ? stPressed : stReleased;

	//Read the raw state of the PmodSWT pins
	PmodSwt1.stCur = ( prtJA1 & ( 1 << swtJA1 ) ) ? stPressed : stReleased;
	PmodSwt2.stCur = ( prtJA2 & ( 1 << swtJA2 ) ) ? stPressed : stReleased;
	PmodSwt3.stCur = ( prtJA3 & ( 1 << swtJA3 ) ) ? stPressed : stReleased;
	PmodSwt4.stCur = ( prtJA4 & ( 1 << swtJA4 ) ) ? stPressed : stReleased;

	// Update state counts.
	btnBtn1.cst = ( btnBtn1.stCur == btnBtn1.stPrev ) ? btnBtn1.cst + 1 : 0;
	btnBtn2.cst = ( btnBtn2.stCur == btnBtn2.stPrev ) ? btnBtn2.cst + 1 : 0;

	//Update state counts for PmodBTN
	PmodBtn1.cst = (PmodBtn1.stCur == PmodBtn1.stPrev) ? PmodBtn1.cst +1 : 0;
	PmodBtn2.cst = (PmodBtn2.stCur == PmodBtn2.stPrev) ? PmodBtn2.cst +1 : 0;
	PmodBtn3.cst = (PmodBtn3.stCur == PmodBtn3.stPrev) ? PmodBtn3.cst +1 : 0;
	PmodBtn4.cst = (PmodBtn4.stCur == PmodBtn4.stPrev) ? PmodBtn4.cst +1 : 0;

	//Update state counts for PmodSWT
	PmodSwt1.cst = (PmodSwt1.stCur == PmodSwt1.stPrev) ? PmodSwt1.cst +1 : 0;
	PmodSwt2.cst = (PmodSwt2.stCur == PmodSwt2.stPrev) ? PmodSwt2.cst +1 : 0;
	PmodSwt3.cst = (PmodSwt3.stCur == PmodSwt3.stPrev) ? PmodSwt3.cst +1 : 0;
	PmodSwt4.cst = (PmodSwt4.stCur == PmodSwt4.stPrev) ? PmodSwt4.cst +1 : 0;
	
	// Save the current state.
	btnBtn1.stPrev = btnBtn1.stCur;
	btnBtn2.stPrev = btnBtn2.stCur;

	// Save the current state for PmodBTN
	PmodBtn1.stPrev = PmodBtn1.stCur;
	PmodBtn2.stPrev = PmodBtn2.stCur;
	PmodBtn3.stPrev = PmodBtn3.stCur;
	PmodBtn4.stPrev = PmodBtn4.stCur;

	// Save the current state for PmodSWT
	PmodSwt1.stPrev = PmodSwt1.stCur;
	PmodSwt2.stPrev = PmodSwt2.stCur;
	PmodSwt3.stPrev = PmodSwt3.stCur;
	PmodSwt4.stPrev = PmodSwt4.stCur;
	
	// Update the state of button 1 if necessary.
	if ( cstMaxCnt == btnBtn1.cst ) {
		btnBtn1.stBtn = btnBtn1.stCur;
		btnBtn1.cst = 0;
	}
	
	// Update the state of button 2 if necessary.
	if ( cstMaxCnt == btnBtn2.cst ) {
		btnBtn2.stBtn = btnBtn2.stCur;
		btnBtn2.cst = 0;
	}

	//if statements for buttons

	// Update the state of PmodBTN1 if necessary.
	if ( cstMaxCnt == PmodBtn1.cst ) {
		PmodBtn1.stBtn = PmodBtn1.stCur;
		PmodBtn1.cst = 0;
	}
	
	// Update the state of PmodBTN2 if necessary.
	if ( cstMaxCnt == PmodBtn2.cst ) {
		PmodBtn2.stBtn = PmodBtn2.stCur;
		PmodBtn2.cst = 0;
	}

	// Update the state of PmodBTN3 if necessary.
	if ( cstMaxCnt == PmodBtn3.cst ) {
		PmodBtn3.stBtn = PmodBtn3.stCur;
		PmodBtn3.cst = 0;
	}

	// Update the state of PmodBTN4 if necessary.
	if ( cstMaxCnt == PmodBtn4.cst ) {
		PmodBtn4.stBtn = PmodBtn4.stCur;
		PmodBtn4.cst = 0;
	}

	//if statements for switches

	// Update the state of PmodSWT1 if necessary.
	if ( cstMaxCnt == PmodSwt1.cst ) {
		PmodSwt1.stBtn = PmodSwt1.stCur;
		PmodSwt1.cst = 0;
	}
	
	// Update the state of PmodSWT2 if necessary.
	if ( cstMaxCnt == PmodSwt2.cst ) {
		PmodSwt2.stBtn = PmodSwt2.stCur;
		PmodSwt2.cst = 0;
	}

	// Update the state of PmodSWT3 if necessary.
	if ( cstMaxCnt == PmodSwt3.cst ) {
		PmodSwt3.stBtn = PmodSwt3.stCur;
		PmodSwt3.cst = 0;
	}

	// Update the state of PmodSWT4 if necessary.
	if ( cstMaxCnt == PmodSwt4.cst ) {
		PmodSwt4.stBtn = PmodSwt4.stCur;
		PmodSwt4.cst = 0;
	}
    Timer5count++;
    
    if(j++ > 499){
        j = 0;
        DeasVal = deasVal_array[arrayCount];
        avgSpeed_IC2_array[arrayCount] = avgSpeed_IC2;
        
        int localcount_2 = IC2count;
        if(DeasVal == 0){
            OC2R = 0;
            OC2RS = 0;
            ei_2=0;
            avgSpeed_IC2 = 0;
        }
        else if (p_localcount2 < localcount_2){
            error_2 = DeasVal - avgSpeed_IC2;
            ei_2 = ei_2 + error_2;
            if(ei_2 > ei_max){
                ei_2 = ei_max;
            }
            if(ei_2 < ei_min){
                ei_2 = ei_min;
            }
    
            controlSig = ((Kp*error_2)+(Ki*ei_2)+(Kd*(error_2-prev_error_2)));
    
            if(controlSig>10000){
                controlSig = 10000;
            }
            if(controlSig<0){
                controlSig = 0;
            }
    
            prev_error_2 = error_2;
            //controlval = PID_Controller(avgSpeed_IC3, DeasVal);
            OC2R = controlSig;
            OC2RS = controlSig;
        }
        else{
            OC2R = 5000;
            OC2RS = 5000;
        }
        
        int localcount_3 = IC3count;
        if(DeasVal == 0){
            OC3R = 0;
            OC3RS = 0;
            ei_3=0;
            avgSpeed_IC3 = 0;
        }
        else if (p_localcount3 < localcount_3){
            error_3 = DeasVal - avgSpeed_IC3;
            ei_3 = ei_3 + error_3;
            if(ei_3 > ei_max){
                ei_3 = ei_max;
            }
            if(ei_3 < ei_min){
                ei_3 = ei_min;
            }
    
            controlSig = ((Kp*error_3)+(Ki*ei_3)+(Kd*(error_3-prev_error_3)));
    
            if(controlSig>10000){
                controlSig = 10000;
            }
            if(controlSig<0){
                controlSig = 0;
            }
    
            prev_error_3 = error_3;
            //controlval = PID_Controller(avgSpeed_IC2, DeasVal);
            OC3R = controlSig;
            OC3RS = controlSig;
        }
        else{
            OC3R = 5000;
            OC3RS = 5000;
        }
        arrayCount++;
        if(arrayCount == 500){
            arrayCount = 0;
        }
    }
}

/*int PID_Controller(int MeasVal, int DesVal){
    int error = DesVal - MeasVal;
    
    ei = ei + error;
    
    if(ei > ei_max){
        ei = ei_max;
    }
    if(ei < ei_min){
        ei = ei_min;
    }
    
    controlSig = ((Kp*error)+(Ki*ei)+(Kd*(error-prev_error)));
    
    if(controlSig>10000){
        controlSig = 10000;
    }
    if(controlSig<1500){
        controlSig = 1500;
    }
    
    prev_error = error;
    
    return controlSig;
}*/

/* ------------------------------------------------------------ */
/*				Procedure Definitions							*/
/* ------------------------------------------------------------ */
/***	main
**
**	Synopsis:
**		st = main()
**
**	Parameters:
**		none
**
**	Return Values:
**		does not return
**
**	Errors:
**		none
**
**	Description:
**		Main program module. Performs basic board initialization
**		and then enters the main program loop.
*/

int main(void) {
    int i = 0;
	BYTE	stBtn1;
	BYTE	stBtn2;

	BYTE	stPmodBtn1;
	BYTE	stPmodBtn2;
	BYTE	stPmodBtn3;
	BYTE	stPmodBtn4;

	BYTE	stPmodSwt1;
	BYTE	stPmodSwt2;
	BYTE	stPmodSwt3;
	BYTE	stPmodSwt4;
    
    uint8_t n;
    //n = fprintf buffer IC2count 


	DeviceInit();
	AppInit();

	INTDisableInterrupts();
	DelayMs(500);
	

	//write to PmodCLS
	SpiEnable();
	SpiPutBuff(szClearScreen, 3);
	DelayMs(4);
	SpiPutBuff(szBacklightOn, 4);
	DelayMs(4);
	SpiPutBuff(szCursorOff, 4);
	DelayMs(4);
	SpiPutBuff("Hello from", 10);
	DelayMs(4);
	SpiPutBuff(szCursorPos10, 6);
	DelayMs(4);
	SpiPutBuff("Digilent!", 9);
	DelayMs(2000);
    SpiPutBuff(szClearScreen, 3);
	SpiDisable();

    /*for(i=0; i<500; i++){
        if(i<50){
            deasVal_array[i] = 0;
        }
        else if(i<100){
            deasVal_array[i] = 10;
        }
        else if(i<150){
            deasVal_array[i] = 12.5;
        }
        else if(i<200){
            deasVal_array[i] = 17.5;
        }        
        else if(i<300){
            deasVal_array[i] = 20;
        }
        else if(i<350){
            deasVal_array[i] = 17.5;
        }        
        else if(i<400){
            deasVal_array[i] = 12.5;
        }
        else if(i < 450){
            deasVal_array[i] = 10;
        }
        else{
            deasVal_array[i] = 0;
        }
    }*/
    
    
	prtLed1Set	= ( 1 << bnLed1 );
    
	INTEnableInterrupts();
    
    //OC2R = 10000;
    //OC2RS = 10000;
    //OC3R = 10000;
    //OC3RS = 10000;
    
	while (fTrue)
	{		
		INTDisableInterrupts();
	
		//get data here
		stBtn1 = btnBtn1.stBtn;
		stBtn2 = btnBtn2.stBtn;

		stPmodBtn1 = PmodBtn1.stBtn;
		stPmodBtn2 = PmodBtn2.stBtn;
		stPmodBtn3 = PmodBtn3.stBtn;
		stPmodBtn4 = PmodBtn4.stBtn;

		stPmodSwt1 = PmodSwt1.stBtn;
		stPmodSwt2 = PmodSwt2.stBtn;
		stPmodSwt3 = PmodSwt3.stBtn;
		stPmodSwt4 = PmodSwt4.stBtn;

		INTEnableInterrupts();
		//configure OCR to go forward

		/*if(stPressed == stPmodBtn1){
			//start motor if button 2 pressed

			MtrCtrlStop();  //stop before sending new data to avoid possible short circuit
			UpdateMotors(); 
			Wait_ms(0x0A00);
			MtrCtrlFwd();
			UpdateMotors();
			Wait_ms(0x0800);
			MtrCtrlStop();
			UpdateMotors();
			
		}else if(stPressed == stPmodBtn2){
			//start left turn

			MtrCtrlStop();  //stop before sending new data to avoid possible short circuit
			UpdateMotors(); 
			Wait_ms(0x0A00);
			MtrCtrlBwd();
			UpdateMotors();
			Wait_ms(0x0800);
			MtrCtrlStop();
			UpdateMotors();
			
		}else if(stPressed == stPmodBtn3){
			//start right turn

			MtrCtrlStop();  //stop before sending new data to avoid possible short circuit
			UpdateMotors(); 
			Wait_ms(0x0A00);
			MtrCtrlRight();
			UpdateMotors();
			Wait_ms(0x0200);
			MtrCtrlStop();
			UpdateMotors();

		} else if(stPressed == stPmodBtn4){
			//start move backward

			MtrCtrlStop();  //stop before sending new data to avoid possible short circuit
			UpdateMotors(); 
			Wait_ms(0x0A00);
			MtrCtrlLeft();
			UpdateMotors();
			Wait_ms(0x0200);
			MtrCtrlStop();
			UpdateMotors();

		} else if(stPressed == stPmodSwt1){
			//make square to right

			MtrCtrlStop();  //stop before sending new data to avoid possible short circuit
			UpdateMotors(); 
			Wait_ms(0x0A00);  //gives delay to toggle switch
			MtrCtrlFwd();
			UpdateMotors();
			Wait_ms(0x0800);
			MtrCtrlStop();
			UpdateMotors();
			Wait_ms(0x0500);
			MtrCtrlRight();
			UpdateMotors();		// first turn
			Wait_ms(0x0180);
			MtrCtrlStop();
			UpdateMotors();
			Wait_ms(0x0500);
			MtrCtrlFwd();
			UpdateMotors();
			Wait_ms(0x0800);
			MtrCtrlStop();
			UpdateMotors();
			Wait_ms(0x0500);
			MtrCtrlRight();     // second turn
			UpdateMotors();
			Wait_ms(0x0180);
			MtrCtrlStop();
			UpdateMotors();
			Wait_ms(0x0500);
			MtrCtrlFwd();
			UpdateMotors();
			Wait_ms(0x0800);
			MtrCtrlStop();
			UpdateMotors();
			Wait_ms(0x0500);
			MtrCtrlRight();		// third turn
			UpdateMotors();
			Wait_ms(0x0180);
			MtrCtrlStop();
			UpdateMotors();
			Wait_ms(0x0500);
			MtrCtrlFwd();
			UpdateMotors();
			Wait_ms(0x0800);
			MtrCtrlStop();
			UpdateMotors();
			Wait_ms(0x0500);
			MtrCtrlRight();
			UpdateMotors();
			Wait_ms(0x0180);
			MtrCtrlStop();
			UpdateMotors();

		} else if(stPressed == stPmodSwt2){
			//make triangle to left

			MtrCtrlStop();  //stop before sending new data to avoid possible short circuit
			UpdateMotors(); 
			Wait_ms(0x0A00); //gives delay to toggle switch
			MtrCtrlFwd();
			UpdateMotors();
			Wait_ms(0x0800);
			MtrCtrlStop();
			UpdateMotors();
			Wait_ms(0x0500);
			MtrCtrlLeft();  	//first turn
			UpdateMotors();
			Wait_ms(0x0280);
			MtrCtrlStop();
			UpdateMotors();
			Wait_ms(0x0500);
			MtrCtrlFwd();
			UpdateMotors();
			Wait_ms(0x0800);
			MtrCtrlStop();
			UpdateMotors();
			Wait_ms(0x0500);
			MtrCtrlLeft();		//second turn
			UpdateMotors();
			Wait_ms(0x0280);
			MtrCtrlStop();
			UpdateMotors();
			Wait_ms(0x0500);
			MtrCtrlFwd();
			UpdateMotors();
			Wait_ms(0x0800);
			MtrCtrlStop();
			UpdateMotors();
			Wait_ms(0x0500);
			MtrCtrlLeft();		//third turn
			UpdateMotors();
			Wait_ms(0x0280);
			MtrCtrlStop();
			UpdateMotors();
		
		}else if(stPressed == stPmodSwt3){
			// Three point turn around

			MtrCtrlStop();  //stop before sending new data to avoid possible short circuit
			UpdateMotors(); 
			Wait_ms(0x0A00);  //gives delay to toggle switch
			MtrCtrlFwdRight();
			UpdateMotors();
			Wait_ms(0x0800);
			MtrCtrlStop();
			UpdateMotors();
			Wait_ms(0x0500);
			MtrCtrlBwdLeft();
			UpdateMotors();
			Wait_ms(0x0800);
			MtrCtrlStop();
			UpdateMotors();
			Wait_ms(0x0500);
			MtrCtrlFwd();
			UpdateMotors();
			Wait_ms(0x0800);
			MtrCtrlStop();
			UpdateMotors();

		}else if(stPressed == stPmodSwt4){
			// dance
			
			MtrCtrlStop();  //stop before sending new data to avoid possible short circuit
			UpdateMotors(); 
			Wait_ms(0x0A00);  //gives delay to toggle switch
			MtrCtrlFwdLeft(); // step left
			UpdateMotors();
			Wait_ms(0x0300);
			MtrCtrlStop();
			UpdateMotors();
			Wait_ms(0x0200);
			MtrCtrlFwdRight(); // step right
			UpdateMotors();		
			Wait_ms(0x0300);
			MtrCtrlStop();
			UpdateMotors();
			Wait_ms(0x0200);
			MtrCtrlFwdLeft();  // step left
			UpdateMotors();
			Wait_ms(0x0300);
			MtrCtrlStop();
			UpdateMotors();
			Wait_ms(0x0200);
			MtrCtrlFwdRight(); // step right
			UpdateMotors();
			Wait_ms(0x0200);
			MtrCtrlStop();
			UpdateMotors();
			Wait_ms(0x0200);
			MtrCtrlLeft();     // spin
			UpdateMotors();
			Wait_ms(0x0800);
			MtrCtrlStop();
			UpdateMotors();
		}  //end if
*/
        int breakpoint;
        breakpoint = 100;
        
        /*char buffer[30];
        n = sprintf(buffer, "IC2: %d",IC2count);
        SpiEnable();
        SpiPutBuff(szCursorPos00, 6);
        DelayMs(4);
        SpiPutBuff(szCursorOff, 4);
        DelayMs(4);
        SpiPutBuff(buffer, n);
        DelayMs(4);
        n = sprintf(buffer, "IC3: %d",IC3count);
        SpiEnable();
        SpiPutBuff(szCursorPos10, 6);
        DelayMs(4);
        SpiPutBuff(szCursorOff, 4);
        DelayMs(4);
        SpiPutBuff(buffer, n);
        DelayMs(4);
        SpiDisable();*/
        
        char buffer[30];
        n = sprintf(buffer, "Left:  %4.2f i/s",avgSpeed_IC2);
        SpiEnable();
        SpiPutBuff(szCursorPos00, 6);
        DelayMs(4);
        SpiPutBuff(szCursorOff, 4);
        DelayMs(4);
        SpiPutBuff(buffer, n);
        DelayMs(4);
        n = sprintf(buffer, "Right: %4.2f i/s",avgSpeed_IC3);
        SpiEnable();
        SpiPutBuff(szCursorPos10, 6);
        DelayMs(4);
        SpiPutBuff(szCursorOff, 4);
        DelayMs(4);
        SpiPutBuff(buffer, n);
        DelayMs(4);
        SpiDisable();
        
       // OC2R = 2000;
       // OC2RS = 2000;
       // OC3R = 2000;
       // OC3RS = 2000;
        breakpoint = 200;
    }  //end while
}  //end main

/* ------------------------------------------------------------ */
/***	DeviceInit
**
**	Synopsis:
**		DeviceInit()
**
**	Parameters:
**		none
**
**	Return Values:
**		none
**
**	Errors:
**		none
**
**	Description:
**		Initializes on chip peripheral devices to the default
**		state.
*/

void DeviceInit() {

	// Configure left motor direction pin and set default direction.
	trisMtrLeftDirClr	= ( 1 << bnMtrLeftDir );
	prtMtrLeftDirClr	= ( 1 << bnMtrLeftDir );	// forward
	
	// Configure right motor direction pin and set default direction.
	trisMtrRightDirClr	= ( 1 << bnMtrRightDir );	
	prtMtrRightDirSet	= ( 1 << bnMtrRightDir );	// forward

	// Configure Output Compare 2 to drive the left motor.
	OC2CON	= ( 1 << 3 ) | ( 1 << 2 ) | ( 1 << 1 );	// pwm set up PWM Mode, fault pin disabled, using timer 3
	OC2R	= dtcMtrStopped;
	OC2RS	= dtcMtrStopped;

	// Configure Output Compare 3.
	OC3CON = ( 1 << 3 ) | ( 1 << 2 ) | ( 1 << 1 );	// pwm set up PWM Mode, fault pin disabled, using timer 3
	OC3R	= dtcMtrStopped;
	OC3RS	= dtcMtrStopped;

	// Configure Timer 2.  //used for IC2 and IC3 for speed control timing
	TMR2	= 0;									// clear timer 2 count
	PR2		= 64999;  // 65ms timer

	// Configure Timer 3. //used for OC2 and OC3 for PWM
	TMR3	= 0;
	PR3		= 9999;   // 10ms timer

	// Start timers and output compare units.
	T2CON		= ( 1 << 15 ) | ( 1 << TCKPS20 )|(1 << TCKPS21);		// timer 2 prescale = 8
	OC2CONSET	= ( 1 << 15 );	// enable output compare module 2
	OC3CONSET	= ( 1 << 15 );	// enable output compare module 3
	T3CON		= ( 1 << 15 ) | ( 1 << TCKPS31 ) | ( 1 << TCKPS30); 	// timer 3 prescale = 8

	// Configure Timer 5.
	TMR5	= 0;
	PR5		= 99; // period match every 100 us
	IPC5SET	= ( 1 << 4 ) | ( 1 << 3 ) | ( 1 << 2 ) | ( 1 << 1 ) | ( 1 << 0 ); // interrupt priority level 7, sub 3
	IFS0CLR = ( 1 << 20);
	IEC0SET	= ( 1 << 20);
    
    // Configure IC2.
    TRISDSET = (1 << 9);    //setup pin as an input for IC2
    IC2CONSET = (1 << 7) | (1 << 1) | (1 << 0);   //using timer2 event on every rising edge 
    IPC2SET = (1 << 10) | (1 << 12) | (1 << 9) | (1 << 8); // Interrupt priority level 5, sub 3
    IFS0CLR = (1 << 9);     //clears IC2 interrupt status flag
    IEC0SET = (1 << 9);     //Enables IC2 interrupt
    IC2CONSET = (1 << 15);  //turns on input capture function
 
    // Configure IC3.
    TRISDSET = (1 << 10);   //setup pin as an input for IC3
    IC3CONSET = (1 << 7) | (1 << 1) | (1 << 0); //using timer2 event on every rising edge 
    IPC3SET = (1 << 10) | (1 << 12) | (1 << 9) | (1 << 8); // Interrupt priority level 5, sub 3
    IFS0CLR = ( 1 << 13);   //clears IC3 interrupt status flag
    IEC0SET = (1 << 13);    //Enables IC3 interrupt
    IC3CONSET = (1 << 15);  //turns on input capture function


	
	// Start timers.
	T5CON = ( 1 << 15 ) | ( 1 << 5 ) | ( 1 << 4 ); // fTimer5 = fPb / 8
    
	//enable SPI
	SpiInit();

	// Enable multi-vector interrupts.
	INTEnableSystemMultiVectoredInt();
    
}

/* ------------------------------------------------------------ */
/***	AppInit
**
**	Synopsis:
**		AppInit()
**
**	Parameters:
**		none
**
**	Return Values:
**		none
**
**	Errors:
**		none
**
**	Description:
**		Performs application specific initialization. Sets devices
**		and global variables for application.
*/

void AppInit() {



}


/* ------------------------------------------------------------ */
/***	Wait_ms
**
**	Synopsis:
**		Wait_ms(WORD)
**
**	Parameters:
**		WORD (range from 0 to 65535)
**
**	Return Values:
**		none
**
**	Errors:
**		none
**
**	Description:
**		Will wait for specified number of milliseconds.  Using a 
**		word variable allows for delays up to 65.535 seconds.  The value
**		in the for statement may need to be altered depending on how your
**		compiler translates this code into AVR assembly.  In assembly, it is
**		possible to generate exact delay values by counting the number of clock
**		cycles your loop takes to execute.  When writing code in C, the compiler
**		interprets the code, and translates it into assembly.  This process is 
**		notoriously inefficient and may vary between different versions of AVR Studio
**		and WinAVR GCC.  A handy method of calibrating the delay loop is to write a 
**		short program that toggles an LED on and off once per second using this 
**		function and using a watch to time how long it is actually taking to
**		complete. 
**
*/

void Wait_ms(WORD delay) {

	WORD i;

	while(delay > 0){

		for( i = 0; i < 375; i ++){
			;;
		}
		delay -= 1;
	}
}

/************************************************************************/
