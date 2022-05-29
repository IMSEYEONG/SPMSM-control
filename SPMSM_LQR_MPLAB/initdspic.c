/**********************************************************************
* © 2012 Microchip Technology Inc.
*
* SOFTWARE LICENSE AGREEMENT:
* Microchip Technology Incorporated ("Microchip") retains all ownership and 
* intellectual property rights in the code accompanying this message and in all 
* derivatives hereto.  You may use this code, and any derivatives created by 
* any person or entity by or on your behalf, exclusively with Microchip's
* proprietary products.  Your acceptance and/or use of this code constitutes 
* agreement to the terms and conditions of this notice.
*
* CODE ACCOMPANYING THIS MESSAGE IS SUPPLIED BY MICROCHIP "AS IS".  NO 
* WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED 
* TO, IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY AND FITNESS FOR A 
* PARTICULAR PURPOSE APPLY TO THIS CODE, ITS INTERACTION WITH MICROCHIP'S 
* PRODUCTS, COMBINATION WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION. 
*
* YOU ACKNOWLEDGE AND AGREE THAT, IN NO EVENT, SHALL MICROCHIP BE LIABLE, WHETHER 
* IN CONTRACT, WARRANTY, TORT (INCLUDING NEGLIGENCE OR BREACH OF STATUTORY DUTY), 
* STRICT LIABILITY, INDEMNITY, CONTRIBUTION, OR OTHERWISE, FOR ANY INDIRECT, SPECIAL, 
* PUNITIVE, EXEMPLARY, INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, FOR COST OR EXPENSE OF 
* ANY KIND WHATSOEVER RELATED TO THE CODE, HOWSOEVER CAUSED, EVEN IF MICROCHIP HAS BEEN 
* ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE FULLEST EXTENT 
* ALLOWABLE BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY RELATED TO 
* THIS CODE, SHALL NOT EXCEED THE PRICE YOU PAID DIRECTLY TO MICROCHIP SPECIFICALLY TO 
* HAVE THIS CODE DEVELOPED.
*
* You agree that you are solely responsible for testing the code and 
* determining its suitability.  Microchip has no obligation to modify, test, 
* certify, or support the code.
*
******************************************************************************/
/***********************************************************************
 *      Code Description                                               *
 *                                                                     *
 *  Initialization for dsPIC's ports                                   *
 *                                                                     *
 **********************************************************************/

#include "general.h"

void SetupPorts( void )
{

	// ============= Port A ==============
	LATA  = 0;
	TRISA = 0x1F93;			// Reset value (all inputs)
    ANSELA = 0X0003;
	// ============= Port B ==============
    LATB  = 0x0000;
    TRISB = 0x03FF;			// RB10~15 outputs for PWM, rest are inputs
    ANSELB = 0;
	// ============= Port C ==============
	LATC  = 0x0000;
	TRISC = 0xBFFF;			// Reset value (all inputs)
    ANSELC = 0;
    // ============= Port D ==============
	LATD  = 0x0000;
	TRISD = 0x0100;	            // 0000 0000 0000 0000; RD8 - Test Point I/P ,RD6 -  LED1 , RD5 - LED2
    // ============= Port E ==============
    LATE  = 0x0000;
    TRISE = 0x2000;             // 0010 0000 0000 0000  ;RE13 - POT ; 
    ANSELE = 0X2000;            // ANSE13 - POT 
    // ============= Port F ==============
    LATF  = 0x0000;
    TRISF = 0x0000;             // 0000 0000 0000 0000  RF1 - TX  
	// ============= Port G ==============
    LATG  = 0x0000;
    TRISG = 0x03C0;			// Reset value (all inputs)

	/************** Code section for the low pin count devices ******/
	/*Assigning the TX and RX and FAULT pins to ports RP97 , RP53 and RP32 to the dsPIC*/
	__builtin_write_OSCCONL(OSCCON & (~(1<<6))); // clear bit 6 

    RPINR19bits.U2RXR = 53;		// Make Pin RP53 U2RX
    RPOR7bits.RP97R = 3;	    // Make Pin RP97 U2TX
	RPINR12bits.FLT1R = 32;		// Make Pin RP32 FLT I/P

	__builtin_write_OSCCONL(OSCCON | (1<<6)); 	 // Set bit 6 	
	/****************************************************************/



	return;
}

