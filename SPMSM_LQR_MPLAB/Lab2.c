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
/************** GLOBAL DEFINITIONS ***********/

#define INITIALIZE
#include "general.h"
#include "Parms.h"
#include "SVGen.h"
#include "ReadADC.h"
#include "MeasCurr.h"
#include "Control.h"
#include "PI.h"
#include "Park.h"
#include "UserParms.h"
#include "smcpos.h"
#include "FdWeak.h"
#ifdef RTDM
    #include "RTDM.h"
#endif
#ifdef DMCI_DEMO
    #include "RTDM.h"
#endif
#include <libq.h>

SFRAC16 atan2CORDIC(SFRAC16, SFRAC16);


/******************************************************************************/
/* Configuration bits                                                         */
/******************************************************************************/
_FPOR(ALTI2C1_OFF & ALTI2C2_OFF);
_FWDT(PLLKEN_ON & FWDTEN_OFF);
_FOSCSEL(FNOSC_FRC & IESO_OFF & PWMLOCK_OFF);
_FGS(GWRP_OFF & GCP_OFF);

_FICD(ICS_PGD2 & JTAGEN_OFF);	// PGD3 for 28pin 	PGD2 for 44pin
_FOSC(FCKSM_CSECMD & POSCMD_XT);		//XT W/PLL
//_FOSC(FCKSM_CSECMD & OSCIOFNC_OFF & POSCMD_NONE); // FRC W/PLL	

/************** END OF GLOBAL DEFINITIONS ***********/


/********************* Variables to display data using DMCI *********************************/

#ifdef RTDM

int RecorderBuffer1[DATA_BUFFER_SIZE]; //Buffer to store the data samples for the DMCI data viewer Graph1
int RecorderBuffer2[DATA_BUFFER_SIZE];	//Buffer to store the data samples for the DMCI data viewer Graph2
int RecorderBuffer3[DATA_BUFFER_SIZE];	//Buffer to store the data samples for the DMCI data viewer Graph3
int RecorderBuffer4[DATA_BUFFER_SIZE];	//Buffer to store the data samples for the DMCI data viewer Graph4

int * PtrRecBuffer1 = &RecorderBuffer1[0];	//Tail pointer for the DMCI Graph1
int * PtrRecBuffer2 = &RecorderBuffer2[0];	//Tail pointer for the DMCI Graph2
int * PtrRecBuffer3 = &RecorderBuffer3[0];	//Tail pointer for the DMCI Graph3
int * PtrRecBuffer4 = &RecorderBuffer4[0];	//Tail pointer for the DMCI Graph4
int * RecBuffUpperLimit = RecorderBuffer4 + DATA_BUFFER_SIZE -1;	//Buffer Recorder Upper Limit
typedef struct DMCIFlags{
		    unsigned Recorder : 1;	// Flag needs to be set to start buffering data
			unsigned StartStop : 1;
			unsigned unused : 14;  
} DMCIFLAGS;
DMCIFLAGS DMCIFlags;
int	SnapCount = 0;

int SnapShotDelayCnt = 0;
int SnapShotDelay = SNAPDELAY;
int SpeedReference = 0;

#endif // End of #ifdef RTDM

int count = 0; // delay for ramping the reference velocity 
int VelReq = 0; 

SMC smc1 = SMC_DEFAULTS;

unsigned long Startup_Ramp = 0;	/* Start up ramp in open loop. This variable
								is incremented in CalculateParkAngle()
								subroutine, and it is assigned to 
								ParkParm.qAngle as follows:
								ParkParm.qAngle += (int)(Startup_Ramp >> 16);*/

unsigned int Startup_Lock = 0;	/* This is a counter that is incremented in
								CalculateParkAngle() every time it is called. 
								Once this counter has a value of LOCK_TIME, 
								then theta will start increasing moving the 
								motor in open loop. */
unsigned int  trans_counter = 0;
union   {
        struct
            {
            unsigned OpenLoop:1;	// Indicates if motor is running in open or closed loop
            unsigned RunMotor:1;	// If motor is running, or stopped.
			unsigned EnTorqueMod:1;	// This bit enables Torque mode when running closed loop
			unsigned EnVoltRipCo:1;	// Bit that enables Voltage Ripple Compensation
            unsigned Btn1Pressed:1;	// Button 1 has been pressed.
            unsigned Btn2Pressed:1;	// Button 2 has been pressed.
            unsigned ChangeMode:1;	// This flag indicates that a transition from open to closed
									// loop, or closed to open loop has happened. This
									// causes DoControl subroutine to initialize some variables
									// before executing open or closed loop for the first time
            unsigned ChangeSpeed:1;	// This flag indicates a step command in speed reference.
									// This is mainly used to analyze step response
            unsigned ADCH0_PotOrVdc:1; //This bit Indicates ADC Channel O Sample A or Sample B is converted

            unsigned    :7;
            }bit;
        WORD Word;
        } uGF;

tPIParm     PIParmD;	// Structure definition for Flux component of current, or Id
tPIParm     PIParmQ;	// Structure definition for Torque component of current, or Iq
tPIParm     PIParmW;	// Structure definition for Speed, or Omega

tReadADCParm ReadADCParm;	// Struct used to read ADC values.

// Speed Calculation Variables

WORD iADCisrCnt = 0;	// This Counter is used as a timeout for polling the push buttons
						// in main() subroutine. It will be reset to zero when it matches
						// dButPolLoopCnt defined in UserParms.h
SFRAC16 PrevTheta = 0;	// Previous theta which is then substracted from Theta to get
						// delta theta. This delta will be accumulated in AccumTheta, and
						// after a number of accumulations Omega is calculated.
SFRAC16 AccumTheta = 0;	// Accumulates delta theta over a number of times
WORD AccumThetaCnt = 0;	// Counter used to calculate motor speed. Is incremented
						// in SMC_Position_Estimation() subroutine, and accumulates
						// delta Theta. After N number of accumulations, Omega is 
						// calculated. This N is diIrpPerCalc which is defined in
						// UserParms.h.

// Vd and Vq vector limitation variables

SFRAC16 qVdSquared = 0;	// This variable is used to know what is left from the VqVd vector
						// in order to have maximum output PWM without saturation. This is
						// done before executing Iq control loop at the end of DoControl()

SFRAC16 DCbus = 0;		// DC Bus measured continuously and stored in this variable
						// while motor is running. Will be compared with TargetDCbus
						// and Vd and Vq will be compensated depending on difference
						// between DCbus and TargetDCbus

SFRAC16 TargetDCbus = 0;// DC Bus is measured before running motor and stored in this
						// variable. Any variation on DC bus will be compared to this value
						// and compensated linearly.	

SFRAC16 Theta_error = 0;// This value is used to transition from open loop to closed looop. 
						// At the end of open loop ramp, there is a difference between 
						// forced angle and estimated angle. This difference is stored in 
						// Theta_error, and added to estimated theta (smc1.Theta) so the 
						// effective angle used for commutating the motor is the same at 
						// the end of open loop, and at the begining of closed loop. 
						// This Theta_error is then substracted from estimated theta 
						// gradually in increments of 0.05 degrees until the error is less
						// than 0.05 degrees.

/************* START OF MAIN FUNCTION ***************/
int main ( void )
{
  //The settings below set up the oscillator and PLL for x MIPS as
    //follows:
    //                  Fin  * M
    // Fosc       =     ---------
    //                   N2 * N1   
    //
    // Fin   	  = 7.37 MHz (Internal Oscillator) or 8MHZ ( Crystal Oscillator)
    // Fosc       = x MHz
    // Fcy        = Fosc/2 = 

	/****************** Clock definitions *********************************/
/*
    // 70 MIPS (70.015 Mhz) (i.e 7.37 * (76/4))  Configuration for Internal Oscillator 

    PLLFBD =  74; 			    // M = 76    74
    CLKDIVbits.PLLPOST = 0; 	// N2 = (2 x (PLLPOST + 1)) = 2   0
    CLKDIVbits.PLLPRE = 0;		// N1 = (PLLPRE + 2) = 2        0
*/
    // 70 MIPS (70.00 Mhz) (i.e 8 * (70/4)) Clock Configuration using External Oscillator
	PLLFBD = 68; 			    // M = 70
	CLKDIVbits.PLLPOST = 0;		// N1=2
	CLKDIVbits.PLLPRE = 0;		// N2=2

    
// Initiate Clock Switch to Primary Oscillator with PLL (NOSC=0b011)

    __builtin_write_OSCCONH(0x03);
    __builtin_write_OSCCONL(0x01);
   
    while(OSCCONbits.COSC != 0x03);
    
    while(OSCCONbits.LOCK != 1);	
    
    // Initiate Clock Switch to FRC with PLL (NOSC=0b001)

/*    __builtin_write_OSCCONH(0x01);
    __builtin_write_OSCCONL(0x01);

    while(OSCCONbits.COSC != 0b001);
    
    while(OSCCONbits.LOCK != 1);
 */


	#ifdef RTDM
    RTDM_Start();  // Configure the UART module used by RTDM
				   // it also initializes the RTDM variables
	#endif

	SMCInit(&smc1);
    SetupPorts();
   	SetupControlParameters(); 
	FWInit();
    uGF.Word = 0;                   // clear flags

	#ifdef TORQUEMODE
    uGF.bit.EnTorqueMod = 1;
	#endif

	#ifdef ENVOLTRIPPLE
    uGF.bit.EnVoltRipCo = 1;
    uGF.bit.ADCH0_PotOrVdc = 0;        // Selects POT as CHO Input
	#endif


	#ifdef RTDM
	DMCIFlags.StartStop = 0;
	DMCIFlags.Recorder = 0;
	#endif

    while(1)
    {
        uGF.bit.ChangeSpeed = 0;
        // init Mode
        uGF.bit.OpenLoop = 1;           // start in openloop
        trans_counter = 0;                 //initialize transition counter		
        IEC0bits.AD1IE = 0;				// Make sure ADC does not generate
										// interrupts while parameters
										// are being initialized
        
        // init user specified parms and stop on error
        if( SetupParm() )
        {
            // Error
            uGF.bit.RunMotor=0;
            return 0;
        }
        
        // zero out i sums 
        PIParmD.qdSum = 0;
        PIParmQ.qdSum = 0;
        PIParmW.qdSum = 0;
     
        // Enable ADC interrupt and begin main loop timing
        IFS0bits.AD1IF = 0; 
        IEC0bits.AD1IE = 1;
		
        if(!uGF.bit.RunMotor)
        {	            
			#ifdef DMCI_DEMO
	            // Initialize current offset compensation
	            while(!DMCIFlags.StartStop)	// wait here until user starts motor 
	            {							// with DMCI
					#ifdef RTDM
					RTDM_ProcessMsgs();
					#endif
	            }
			#else
            // Initialize current offset compensation
            while(!pinButton1)                  //wait here until button 1 is pressed 
                {
					#ifdef RTDM
					RTDM_ProcessMsgs();
					#endif
                }
            while(pinButton1);                  //when button 1 is released 
			#endif

			SetupParm();
            uGF.bit.RunMotor = 1;               //then start motor
        }

        // Run the motor
        uGF.bit.ChangeMode = 1;	// Ensure variable initialization when open loop is
								// executed for the first time
		
		//Run Motor loop
        while(1)
        {
			#ifdef RTDM
	        RTDM_ProcessMsgs();			//RTDM process incoming and outgoing messages
			#endif                          

            // The code that polls the buttons executes every 100 msec.
            if(iADCisrCnt >= BUTPOLLOOPCNT)
            {

				if (uGF.bit.RunMotor == 0)
					break;

				#ifdef DMCI_DEMO
					if(!DMCIFlags.StartStop)
					{	
		            	uGF.bit.RunMotor = 0;
		            	        trans_counter = 0;                 //initialize transition counter

		            	break;
					}
				#else

      			// Button 1 starts or stops the motor
				if(pinButton1)  
	            {
					DebounceDelay();
                    if(pinButton1)  
					{
						if( !uGF.bit.Btn1Pressed )
                        	uGF.bit.Btn1Pressed  = 1;
                    }
                	else
                    {
                    	if( uGF.bit.Btn1Pressed )
                        {
	                        // Button just released
	                        uGF.bit.Btn1Pressed  = 0;
	                        // begin stop sequence
	                        uGF.bit.RunMotor = 0;
		            	        trans_counter = 0;                 //initialize transition counter

	                        break;
                        }
                    }
				}
				#endif

				//while running button 2 will double/half the speed
                if(pinButton2)
                {
					DebounceDelay();
					if(pinButton2)
					{
	                    if( !uGF.bit.Btn2Pressed )
	                        uGF.bit.Btn2Pressed  = 1; 
	                }
                	else
                    {
                    	if( uGF.bit.Btn2Pressed )
                        {
                        	// Button just released
                        	uGF.bit.Btn2Pressed  = 0;
							uGF.bit.ChangeSpeed = !uGF.bit.ChangeSpeed;
                        }
                    }
				}
            }  // end of button polling code              
        }   // End of Run Motor loop
    } // End of Main loop
    // should never get here
    while(1){}
}

//---------------------------------------------------------------------
// Executes one PI itteration for each of the three loops Id,Iq,Speed,

inline void DoControl( void )
{

	#ifndef	DMCI_DEMO
        #ifndef ENVOLTRIPPLE
        	ReadSignedADC0( &ReadADCParm );
        #endif
  	#endif

    #ifdef ENVOLTRIPPLE
        if(uGF.bit.ADCH0_PotOrVdc == 0)
        {
	        ReadSignedADC0( &ReadADCParm );
            AD1CHS0bits.CH0SA = 10;         // Selects DCBUS as CHO Input for next conversion
            uGF.bit.ADCH0_PotOrVdc = 1;
        }
        else if(uGF.bit.ADCH0_PotOrVdc == 1)
        {
	        ReadADC0_VDC();
            AD1CHS0bits.CH0SA = 13;         // Selects POT as CHO Input for next conversion
            uGF.bit.ADCH0_PotOrVdc = 0;
        }
    #endif


    if( uGF.bit.OpenLoop )
        {
        // OPENLOOP:	force rotating angle, and control Iq and Id
		//				Also limits Vs vector to ensure maximum PWM duty
		//				cycle and no saturation

		// This If statement is executed only the first time we enter open loop,
		// everytime we run the motor
        if( uGF.bit.ChangeMode )
        {
            // just changed to openloop
            uGF.bit.ChangeMode = 0;
            // synchronize angles

            // VqRef & VdRef not used
            CtrlParm.qVqRef = 0;
            CtrlParm.qVdRef = 0;
			CtrlParm.qVelRef = 0;
			Startup_Lock = 0;
			Startup_Ramp = 0;
			// Initialize SMC
			smc1.Valpha = 0;
			smc1.Ealpha = 0;
			smc1.EalphaFinal = 0;
			smc1.Zalpha = 0;
			smc1.EstIalpha = 0;
			smc1.Vbeta = 0;
			smc1.Ebeta = 0;
			smc1.EbetaFinal = 0;
			smc1.Zbeta = 0;
			smc1.EstIbeta = 0;
			smc1.Ialpha = 0;
			smc1.IalphaError = 0;
			smc1.Ibeta = 0;
			smc1.IbetaError = 0;
			smc1.Theta = 0;
			smc1.Omega = 0;
        }

		// Enter initial torque demand in Amps using REFINAMPS() macro.
		// Maximum Value for reference is defined by shunt resistor value and 
		// differential amplifier gain. Use this equation to calculate 
		// maximum torque in Amperes:
		// 
		// Max REFINAMPS = (VDD/2)/(RSHUNT*DIFFAMPGAIN)
		//
		// For example:
		//
		// RSHUNT = 0.005
		// VDD = 3.3
		// DIFFAMPGAIN = 75
		//
		// Maximum torque reference in Amps is:
		//
		// (3.3/2)/(.005*75) = 4.4 Amperes, or REFINAMPS(4.4)
		//
		// If motor requires more torque than Maximum torque to startup, user
		// needs to change either shunt resistors installed on the board,
		// or differential amplifier gain.

		CtrlParm.qVqRef = REFINAMPS(INITIALTORQUE);

        if(AccumThetaCnt == 0)
	    {
            PIParmW.qInMeas = smc1.Omega;
		}

        // PI control for D
        PIParmD.qInMeas = ParkParm.qId;
        PIParmD.qInRef  = CtrlParm.qVdRef;
        CalcPI(&PIParmD);
        ParkParm.qVd    = PIParmD.qOut;

		// Vector limitation
		// Vd is not limited
		// Vq is limited so the vector Vs is less than a maximum of 95%.
		// The 5% left is needed to be able to measure current through
		// shunt resistors.
		// Vs = SQRT(Vd^2 + Vq^2) < 0.95
		// Vq = SQRT(0.95^2 - Vd^2)
		qVdSquared = FracMpy(PIParmD.qOut, PIParmD.qOut);
       	PIParmQ.qOutMax = Q15SQRT(Q15(0.95*0.95) - qVdSquared);
		PIParmQ.qOutMin = -PIParmQ.qOutMax;

        // PI control for Q
        PIParmQ.qInMeas = ParkParm.qIq;
        PIParmQ.qInRef  = CtrlParm.qVqRef;
        CalcPI(&PIParmQ);
        ParkParm.qVq    = PIParmQ.qOut;
		
    }

    else
    // Closed Loop Vector Control
    {
		// Pressing one of the push buttons, speed reference (or torque reference
		// if enabled) will be doubled. This is done to test transient response
		// of the controllers
		if( ++count == SPEEDDELAY ) 
	    {
			#ifdef DMCI_DEMO
		    	VelReq = FracMpy(SpeedReference, DQK) + Q15((OMEGA10 + OMEGA1)/2.0);
			#else
		   		VelReq = ReadADCParm.qADValue + Q15((OMEGA10 + OMEGA1)/2.0);
			#endif

		
			if((uGF.bit.ChangeSpeed) && (CtrlParm.qVelRef <= VelReq)) // 2x speed
			{
			     CtrlParm.qVelRef += SPEEDDELAY; 
			}
			else if (CtrlParm.qVelRef <= VelReq/2) //normal speed
			{
			     CtrlParm.qVelRef += SPEEDDELAY; 
			}
			else CtrlParm.qVelRef -= SPEEDDELAY ; 
	 	count = 0;
		}
		
		// When it first transition from open to closed loop, this If statement is
		// executed
        if( uGF.bit.ChangeMode )
        {
            // just changed from openloop
            uGF.bit.ChangeMode = 0;
			// An initial value is set for the speed controller accumulation.
			//
			// The first time the speed controller is executed, we want the output
			// to be the same as it was the last time open loop was executed. So,
			// last time open loop was executed, torque refefernce was constant,
			// and set to CtrlParm.qVqRef.
			//
			// First time in closed loop, CtrlParm.qVqRef = PIParmW.qdSum >> 16
			// assuming the error is zero at time zero. This is why we set 
			// PIParmW.qdSum = (long)CtrlParm.qVqRef << 16.
			PIParmW.qdSum = (long)CtrlParm.qVqRef << 16;
			Startup_Lock = 0;
			Startup_Ramp = 0;
				//velocity reference ramp begins at minimum speed
			CtrlParm.qVelRef = Q15(OMEGA0);
		
	    }  

        // Check to see if new velocity information is available by comparing
        // the number of interrupts per velocity calculation against the
        // number of velocity count samples taken.  If new velocity info
        // is available, calculate the new velocity value and execute
        // the speed control loop.

        if(AccumThetaCnt == 0)
        {
        	// Execute the velocity control loop
			PIParmW.qInMeas = smc1.Omega;
        	PIParmW.qInRef  = CtrlParm.qVelRef;
        	CalcPI(&PIParmW);
        	CtrlParm.qVqRef = PIParmW.qOut;
        }
         
        // If the application is running in torque mode, the velocity
        // control loop is bypassed.  The velocity reference value, read
        // from the potentiometer, is used directly as the torque 
        // reference, VqRef. This feature is enabled automatically only if
		// #define TORQUEMODE is defined in UserParms.h. If this is not
		// defined, uGF.bit.EnTorqueMod bit can be set in debug mode to enable
		// torque mode as well.

		if (uGF.bit.EnTorqueMod)
			CtrlParm.qVqRef = CtrlParm.qVelRef;

		// Get Id reference from Field Weakening table. If Field weakening
		// is not needed or user does not want to enable this feature, 
		// let NOMINALSPEEDINRPM be equal to FIELDWEAKSPEEDRPM in
		// UserParms.h
		CtrlParm.qVdRef = FieldWeakening(_Q15abs(CtrlParm.qVelRef));

        // PI control for D
        PIParmD.qInMeas = ParkParm.qId;
        PIParmD.qInRef  = CtrlParm.qVdRef;
        CalcPI(&PIParmD);

		// If voltage ripple compensation flag is set, adjust the output
		// of the D controller depending on measured DC Bus voltage. This 
		// feature is enabled automatically only if #define ENVOLTRIPPLE is 
		// defined in UserParms.h. If this is not defined, uGF.bit.EnVoltRipCo
		// bit can be set in debug mode to enable voltage ripple compensation.
		//
		// NOTE:
		//
		// If Input power supply has switching frequency noise, for example if a
		// switch mode power supply is used, Voltage Ripple Compensation is not
		// recommended, since it will generate spikes on Vd and Vq, which can
		// potentially make the controllers unstable.
		if(uGF.bit.EnVoltRipCo)
        	ParkParm.qVd = VoltRippleComp(PIParmD.qOut);
		else
			ParkParm.qVd = PIParmD.qOut;

		// Vector limitation
		// Vd is not limited
		// Vq is limited so the vector Vs is less than a maximum of 95%. 
		// Vs = SQRT(Vd^2 + Vq^2) < 0.95
		// Vq = SQRT(0.95^2 - Vd^2)
		qVdSquared = FracMpy(ParkParm.qVd, ParkParm.qVd);
       	PIParmQ.qOutMax = Q15SQRT(Q15(0.95*0.95) - qVdSquared);
		PIParmQ.qOutMin = -PIParmQ.qOutMax;

        // PI control for Q
        PIParmQ.qInMeas = ParkParm.qIq;
        PIParmQ.qInRef  = CtrlParm.qVqRef;
        CalcPI(&PIParmQ);

		// If voltage ripple compensation flag is set, adjust the output
		// of the Q controller depending on measured DC Bus voltage
		if(uGF.bit.EnVoltRipCo)
        	ParkParm.qVq = VoltRippleComp(PIParmQ.qOut);
		else
        	ParkParm.qVq = PIParmQ.qOut;

		// Limit, if motor is stalled, stop motor commutation
		if (smc1.OmegaFltred < 0)
		{
			uGF.bit.RunMotor = 0;
        }
	}
}


inline void SMC_Position_Estimation_Inline (SMC *s)
{
	PUSHCORCON();
	CORCONbits.SATA = 1;
	CORCONbits.SATB = 1;
	CORCONbits.ACCSAT = 1;

	CalcEstI();

	CalcIError();

	// Sliding control calculator

	if (_Q15abs(s->IalphaError) < s->MaxSMCError)
	{
		// s->Zalpha = (s->Kslide * s->IalphaError) / s->MaxSMCError
		// If we are in the linear range of the slide mode controller,
		// then correction factor Zalpha will be proportional to the
		// error (Ialpha - EstIalpha) and slide mode gain, Kslide.
		CalcZalpha();
	}
	else if (s->IalphaError > 0)
		s->Zalpha = s->Kslide;
	else
		s->Zalpha = -s->Kslide;

	if (_Q15abs(s->IbetaError) < s->MaxSMCError)
	{
		// s->Zbeta = (s->Kslide * s->IbetaError) / s->MaxSMCError
		// If we are in the linear range of the slide mode controller,
		// then correction factor Zbeta will be proportional to the
		// error (Ibeta - EstIbeta) and slide mode gain, Kslide.
		CalcZbeta();
	}
	else if (s->IbetaError > 0)
		s->Zbeta = s->Kslide;
	else
		s->Zbeta = -s->Kslide;

	// Sliding control filter -> back EMF calculator
	// s->Ealpha = s->Ealpha + s->Kslf * s->Zalpha -
	//						   s->Kslf * s->Ealpha
	// s->Ebeta = s->Ebeta + s->Kslf * s->Zbeta -
	//						 s->Kslf * s->Ebeta
	// s->EalphaFinal = s->EalphaFinal + s->KslfFinal * s->Ealpha
	//								   - s->KslfFinal * s->EalphaFinal
	// s->EbetaFinal = s->EbetaFinal + s->KslfFinal * s->Ebeta
	//								 - s->KslfFinal * s->EbetaFinal
	CalcBEMF();

	// Rotor angle calculator -> Theta = atan(-EalphaFinal,EbetaFinal)

	s->Theta = atan2CORDIC(-s->EalphaFinal,s->EbetaFinal);

	AccumTheta += s->Theta - PrevTheta;
	PrevTheta = s->Theta;
	
	AccumThetaCnt++;
	if (AccumThetaCnt == IRP_PERCALC)
	{
		s->Omega = AccumTheta;
		AccumThetaCnt = 0;
		AccumTheta = 0;
	}

	/* increment global for open-closed loop transition */
        trans_counter++;
        if( trans_counter == TRANSITION_STEPS) trans_counter = 0;
    //                    Q15(Omega) * 60
    // Speed RPMs = -----------------------------
    //               SpeedLoopTime * Motor Poles
    // For example:
    //    Omega = 0.5
    //    SpeedLoopTime = 0.001
    //    Motor Poles (pole pairs * 2) = 10
    // Then:
    //    Speed in RPMs is 3,000 RPMs

	// s->OmegaFltred = s->OmegaFltred + FilterCoeff * s->Omega
	//								   - FilterCoeff * s->OmegaFltred

	CalcOmegaFltred();

	// Adaptive filter coefficients calculation
	// Cutoff frequency is defined as 2*_PI*electrical RPS
	//
	// 		Wc = 2*_PI*Fc.
	// 		Kslf = Tpwm*2*_PI*Fc
	//
	// Fc is the cutoff frequency of our filter. We want the cutoff frequency
	// be the frequency of the drive currents and voltages of the motor, which
	// is the electrical revolutions per second, or eRPS.
	//
	// 		Fc = eRPS = RPM * Pole Pairs / 60
	//
	// Kslf is then calculated based on user parameters as follows:
	// First of all, we have the following equation for RPMS:
	//
	// 		RPM = (Q15(Omega) * 60) / (SpeedLoopTime * Motor Poles)
	//		Let us use: Motor Poles = Pole Pairs * 2
	//		eRPS = RPM * Pole Pairs / 60), or
	//		eRPS = (Q15(Omega) * 60 * Pole Pairs) / (SpeedLoopTime * Pole Pairs * 2 * 60)
	//	Simplifying eRPS
	//		eRPS = Q15(Omega) / (SpeedLoopTime * 2)
	//	Using this equation to calculate Kslf
	//		Kslf = Tpwm*2*_PI*Q15(Omega) / (SpeedLoopTime * 2)
	//	Using diIrpPerCalc = SpeedLoopTime / Tpwm
	//		Kslf = Tpwm*2*Q15(Omega)*_PI / (diIrpPerCalc * Tpwm * 2)
	//	Simplifying:
	//		Kslf = Q15(Omega)*_PI/diIrpPerCalc
	//
	// We use a second filter to get a cleaner signal, with the same coefficient
	//
	// 		Kslf = KslfFinal = Q15(Omega)*_PI/diIrpPerCalc
	//
	// What this allows us at the end is a fixed phase delay for theta compensation
	// in all speed range, since cutoff frequency is changing as the motor speeds up.
	// 
	// Phase delay: Since cutoff frequency is the same as the input frequency, we can
	// define phase delay as being constant of -45 DEG per filter. This is because
	// the equation to calculate phase shift of this low pass filter is 
	// arctan(Fin/Fc), and Fin/Fc = 1 since they are equal, hence arctan(1) = 45 DEG.
	// A total of -90 DEG after the two filters implemented (Kslf and KslfFinal).
	
	s->Kslf = s->KslfFinal = FracMpy(s->OmegaFltred,Q15(_PI / IRP_PERCALC));

	// Since filter coefficients are dynamic, we need to make sure we have a minimum
	// so we define the lowest operation speed as the lowest filter coefficient

	if (s->Kslf < Q15(OMEGA0 * _PI / IRP_PERCALC))
	{
		s->Kslf = Q15(OMEGA0 * _PI / IRP_PERCALC);
		s->KslfFinal = Q15(OMEGA0 * _PI / IRP_PERCALC);
	}
	s->ThetaOffset = CONSTANT_PHASE_SHIFT;
	s->Theta = s->Theta + s->ThetaOffset;

	POPCORCON();

	return;
}

inline void CalculateParkAngle(void)
{
 	smc1.Ialpha = ParkParm.qIalpha;
  	smc1.Ibeta = ParkParm.qIbeta;
    smc1.Valpha = ParkParm.qValpha;
    smc1.Vbeta = ParkParm.qVbeta;

	SMC_Position_Estimation_Inline(&smc1);

	if(uGF.bit.OpenLoop)	
	{
		if (Startup_Lock < MotorParm.LockTime)
			Startup_Lock += 1;	// This variable is incremented until
								// lock time expires, them the open loop
								// ramp begins
		else if (Startup_Ramp < MotorParm.EndSpeed)
			// Ramp starts, and increases linearly until EndSpeed is reached.
			// After ramp, estimated theta is used to commutate motor.
			Startup_Ramp += DELTA_STARTUP_RAMP;
		else
		{
			// This section enables closed loop, right after open loop ramp.
            uGF.bit.ChangeMode = 1;
            uGF.bit.OpenLoop = 0;
			// Difference between force angle and estimated theta is saved,
			// so a soft transition is made when entering closed loop.
			Theta_error = ParkParm.qAngle - smc1.Theta;
		}
		ParkParm.qAngle += (int)(Startup_Ramp >> 16);
	}
	else
	{
		// This value is used to transition from open loop to closed looop. 
		// At the end of open loop ramp, there is a difference between 
		// forced angle and estimated angle. This difference is stored in 
		// Theta_error, and added to estimated theta (smc1.Theta) so the 
		// effective angle used for commutating the motor is the same at 
		// the end of open loop, and at the begining of closed loop. 
		// This Theta_error is then substracted from estimated theta 
		// gradually in increments of 0.05 degrees until the error is less
		// than 0.05 degrees.
		ParkParm.qAngle = smc1.Theta + Theta_error;
		if( (_Q15abs(Theta_error) > _0_05DEG)&&(trans_counter == 0))
		{
			if (Theta_error < 0)
				Theta_error += _0_05DEG;
			else
				Theta_error -= _0_05DEG;
		}
	}
	return;
}



//---------------------------------------------------------------------
// The ADC ISR does speed calculation and executes the vector update loop.
// The ADC sample and conversion is triggered by the PWM period.
// The speed calculation assumes a fixed time interval between calculations.
//---------------------------------------------------------------------

void __attribute__((interrupt, no_auto_psv)) _AD1Interrupt(void)
{    

    IFS0bits.AD1IF = 0;        

    // Increment count variable that controls execution
    // of display and button functions.
    iADCisrCnt++;
 
    if( uGF.bit.RunMotor )
    {
         
            // Calculate qIa,qIb
            MeasCompCurr();
            
            // Calculate commutation angle using estimator
            CalculateParkAngle();

            // Calculate qId,qIq from qSin,qCos,qIa,qIb
            ClarkePark();
                           
            // Calculate control values
            DoControl();
			
            // Calculate qSin,qCos from qAngle
            SinCos();

            // Calculate qValpha, qVbeta from qSin,qCos,qVd,qVq
            InvPark();    

            // Calculate Vr1,Vr2,Vr3 from qValpha, qVbeta 
            CalcRefVec();

            // Calculate and set PWM duty cycles from Vr1,Vr2,Vr3
            CalcSVGen();
            
    }    

	#ifdef RTDM
    /********************* DMCI Dynamic Data Views  ***************************/
	/********************** RECORDING MOTOR PHASE VALUES ***************/
	if(DMCIFlags.Recorder)
	{
		SnapShotDelayCnt++;
		if(SnapShotDelayCnt == SnapShotDelay)
		{
			SnapShotDelayCnt = 0;
			*PtrRecBuffer1++ 	= SNAP1;
			*PtrRecBuffer2++	= SNAP2;
			*PtrRecBuffer3++	= SNAP3;
			*PtrRecBuffer4++	= SNAP4;
			
			if(PtrRecBuffer4 > RecBuffUpperLimit)
			{
				PtrRecBuffer1 = RecorderBuffer1;
				PtrRecBuffer2 = RecorderBuffer2;
		        PtrRecBuffer3 = RecorderBuffer3;
		        PtrRecBuffer4 = RecorderBuffer4;
		        DMCIFlags.Recorder = 0;
		    }   
		}
	}
	#endif	
	return;
}

//---------------------------------------------------------------------
bool SetupParm(void)
{
    // Turn saturation on to insure that overflows will be handled smoothly.
    CORCONbits.SATA  = 0;

    // Setup required parameters
 
// ============= Open Loop ======================
	// Motor End Speed Calculation
	// MotorParm.EndSpeed = ENDSPEEDOPENLOOP * POLEPAIRS * LOOPTIMEINSEC * 65536 * 65536 / 60.0;
	// Then, * 65536 which is a right shift done in "void CalculateParkAngle(void)"
	// ParkParm.qAngle += (int)(Startup_Ramp >> 16);
	MotorParm.EndSpeed = ENDSPEEDOPENLOOP * POLEPAIRS * LOOPTIMEINSEC * 65536 * 65536 / 60.0;
	MotorParm.LockTime = LOCKTIME;

// ============= ADC - Measure Current & Pot ======================

    // Scaling constants: Determined by calibration or hardware design.
    ReadADCParm.qK      = DQK;    

    MeasCurrParm.qKa    = DQKA;    
    MeasCurrParm.qKb    = DQKB;   

// ============= SVGen ===============
    // Set PWM period to Loop Time 
    SVGenParm.iPWMPeriod = LOOPINTCY;      

   // ============= Motor PWM ======================

    // Center aligned PWM.
    // Note: The PWM period is set to dLoopInTcy/2 but since it counts up and 
    // and then down => the interrupt flag is set to 1 at zero => actual 
    // interrupt period is dLoopInTcy

	PHASE1 = LOOPINTCY;
	PHASE2 = LOOPINTCY;
	PHASE3 = LOOPINTCY;
	PTPER = 2*LOOPINTCY+1;		//one trigger per PWM period

	PWMCON1 = 0x0204;	// Enable PWM output pins and configure them as 
	PWMCON2 = 0x0204;	// complementary mode
	PWMCON3 = 0x0204;

	//I/O pins controlled by PWM
	IOCON1 = 0xC000;
	IOCON2 = 0xC000;
	IOCON3 = 0xC000;

	DTR1 = 0x0000;
	DTR2 = 0x0000;
	DTR3 = 0x0000;

	ALTDTR1 = DDEADTIME;	// 700ns of dead time
	ALTDTR2 = DDEADTIME;	// 700ns of dead time
	ALTDTR3 = DDEADTIME;	// 700ns of dead time

	//fault disabled	
	FCLCON1 = 0x0003;
	FCLCON2 = 0x0003;
	FCLCON3 = 0x0003;
	
	PTCON2 = 0x0000;	// Divide by 1 to generate PWM

    /* low side turn on errate workaraund */
    PDC1 = MIN_DUTY;   // PDC cannot be init with 0, please check errata
    PDC2 = MIN_DUTY;
    PDC3 = MIN_DUTY;
    
    IPC23bits.PWM1IP = 4;	// PWM Interrupt Priority 4
	IPC23bits.PWM2IP = 4;	// PWM Interrupt Priority 4
	IPC24bits.PWM3IP = 4;	// PWM Interrupt Priority 4

	IFS5bits.PWM1IF = 0;	//clear PWM interrupt flag
	IEC5bits.PWM1IE = 0;	// Disable PWM interrupts

    PTCON = 0x8000;         // Enable PWM for center aligned operation

    // SEVTCMP: Special Event Compare Count Register 
    // Phase of ADC capture set relative to PWM cycle: 0 offset and counting up
	SEVTCMP = 0;

	// ============= ADC - Measure Current & Pot ======================
    // ADC setup for simultanous sampling on 
    //      CH0=AN13, CH1=AN0, CH2=AN1, CH3=AN2. 
    // Sampling triggered by Timer3 and stored in signed fractional form.
    // Signed fractional (DOUT = sddd dddd dd00 0000)
    AD1CON1bits.FORM = 3;    
	// Timer3 overflow ends sampling and starts conversion
	AD1CON1bits.SSRC = 3;
	AD1CON1bits.SSRCG = 0;
    // Simultaneous Sample Select bit (only applicable when CHPS = 01 or 1x)
    // Samples CH0, CH1, CH2, CH3 simultaneously (when CHPS = 1x)
    // Samples CH0 and CH1 simultaneously (when CHPS = 01)
    AD1CON1bits.SIMSAM = 1;  
    // Sampling begins immediately after last conversion completes. 
    // SAMP bit is auto set.
    AD1CON1bits.ASAM = 1;  


    AD1CON2 = 0;
    // Samples CH0, CH1, CH2, CH3 simultaneously (when CHPS = 1x)
    AD1CON2bits.CHPS = 2;  


    AD1CON3 = 0;
    // A/D Conversion Clock Select bits = 6 * Tcy
    AD1CON3bits.ADCS = 6;  


     /* ADCHS: ADC Input Channel Select Register */
    AD1CHS0 = 0;
    // CH0 is AN13 for POT
    AD1CHS0bits.CH0SA = 13;

   // CH1 positive input is AN0, CH2 positive input is AN1, CH3 positive input is AN2
    AD1CHS123bits.CH123SA = 0;
  
   /* ADCSSL: ADC Input Scan Select Register */
    AD1CSSL = 0;

    // Turn on A/D module
    AD1CON1bits.ADON = 1;

    #ifdef ENVOLTRIPPLE

        // CH0 is AN10 for VDC
        AD1CHS0bits.CH0SA = 10;
    	// Wait until first conversion takes place to measure offsets.
    	DebounceDelay();
        // Target DC Bus, without sign.
        TargetDCbus = ((SFRAC16)ADC1BUF0 >> 1) + Q15(0.5);

        // CH0 is AN13 for POT
        AD1CHS0bits.CH0SA = 13;
        uGF.bit.ADCH0_PotOrVdc = 0;

    #endif

	// Wait until first conversion takes place to measure offsets.
	DebounceDelay();

    //fault enabled	
    FCLCON1 = 0x00FD; //Fault enabled Fault SRC - Comparator 4 O/P
	FCLCON2 = 0x00FD; //Fault enabled Fault SRC - Comparator 4 O/P
	FCLCON3 = 0x00FD; //Fault enabled Fault SRC - Comparator 4 O

    // Initial Current offsets
	InitMeasCompCurr( ADC1BUF2, ADC1BUF1 );	 

    return False;
}

void SetupControlParameters(void)
{

// ============= PI D Term ===============      
    PIParmD.qKp = DKP;       
    PIParmD.qKi = DKI;              
    PIParmD.qKc = DKC;       
    PIParmD.qOutMax = DOUTMAX;
    PIParmD.qOutMin = -PIParmD.qOutMax;

    InitPI(&PIParmD);

// ============= PI Q Term ===============
    PIParmQ.qKp = QKP;    
    PIParmQ.qKi = QKI;
    PIParmQ.qKc = QKC;
    PIParmQ.qOutMax = QOUTMAX;
    PIParmQ.qOutMin = -PIParmQ.qOutMax;

    InitPI(&PIParmQ);

// ============= PI W Term ===============
    PIParmW.qKp = WKP;       
    PIParmW.qKi = WKI;       
    PIParmW.qKc = WKC;       
    PIParmW.qOutMax = WOUTMAX;   
    PIParmW.qOutMin = -PIParmW.qOutMax;

    InitPI(&PIParmW);
	return;
}

void DebounceDelay(void)
{
	long i;
	for (i = 0;i < 100000;i++)
		;
	return;
}

// NOTE:
//
// If Input power supply has switching frequency noise, for example if a
// switch mode power supply is used, Voltage Ripple Compensation is not
// recommended, since it will generate spikes on Vd and Vq, which can
// potentially make the controllers unstable.

SFRAC16 VoltRippleComp(SFRAC16 Vdq)
{
	SFRAC16 CompVdq;
	// DCbus is already updated with new DC Bus measurement
	// in ReadSignedADC0 subroutine.
	//
	// If target DC Bus is greater than what we measured last sample, adjust
	// output as follows:
	//
	//                  TargetDCbus - DCbus
	// CompVdq = Vdq + --------------------- * Vdq
	//                         DCbus
	//
	// If Measured DCbus is greater than target, then the following compensation
	// is implemented:
	//
	//            TargetDCbus 
	// CompVdq = ------------- * Vdq
	//               DCbus
	//
	// If target and measured are equal, no operation is made.
	//
	if (TargetDCbus > DCbus)
		CompVdq = Vdq + FracMpy(FracDiv(TargetDCbus - DCbus, DCbus), Vdq);
	else if (DCbus > TargetDCbus)
		CompVdq = FracMpy(FracDiv(TargetDCbus, DCbus), Vdq);
	else
		CompVdq = Vdq;

	return CompVdq;
}
