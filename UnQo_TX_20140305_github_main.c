//******************************************************************************
//  MSP430G2xx3 Demo - Timer_A, Ultra-Low Pwr UART 9600 Echo, 32kHz ACLK
//
//  Description: Use Timer_A CCR0 hardware output modes and SCCI data latch
//  to implement UART function @ 9600 baud. Software does not directly read and
//  write to RX and TX pins, instead proper use of output modes and SCCI data
//  latch are demonstrated. Use of these hardware features eliminates ISR
//  latency effects as hardware insures that output and input bit latching and
//  timing are perfectly synchronised with Timer_A regardless of other
//  software activity. In the Mainloop the UART function readies the UART to
//  receive one character and waits in LPM3 with all activity interrupt driven.
//  After a character has been received, the UART receive function forces exit
//  from LPM3 in the Mainloop which configures the port pins (P1 & P2) based
//  on the value of the received byte (i.e., if BIT0 is set, turn on P1.0).

//  ACLK = TACLK = LFXT1 = 32768Hz, MCLK = SMCLK = default DCO
//  //* An external watch crystal is required on XIN XOUT for ACLK *//
//
//               MSP430G2xx3
//            -----------------
//        /|¥|              XIN|-
//         | |                 | 32kHz
//         --|RST          XOUT|-
//           |                 |
//           |   CCI0B/TXD/P1.2|-------->
//           |                 | 9600 8N1
//           |   CCI0A/RXD/P1.1|<--------
//
//
//  D. Dang
//  Texas Instruments Inc.
//  December 2010
//   Built with CCS Version 4.2.0 and IAR Embedded Workbench Version: 5.10
//******************************************************************************

//******************************************************************************
//P1.0 LED RED
//P1.1TXD
//P1.2RXD
//P1.3 mode change switch
//P1.4 Power supply for ANT RF
//P1.5 pulse ticket capture with COMPARATOR
//P1.6 LED green
//P1.7 COMPARATOR BIAS
//P2.0 cadence capture
//***don't use***P2.2 pulse ticket capture
//P2.5 RESET for ANT RF
//you need ANT NETWORK_ID at ANTAP1_AssignNetwork()
//visit this is transient com and make free acount! 
//goloveski
//******************************************************************************


//#include "msp430g2452.h"
#include <stdint.h>
#include "msp430g2553.h"

//------------------------------------------------------------------------------
// Hardware-related definitions
//------------------------------------------------------------------------------
#define UART_TXD   0x02                     // TXD on P1.1 (Timer0_A.OUT0)
#define UART_RXD   0x04                     // RXD on P1.2 (Timer0_A.CCI1A)


//------------------------------------------------------------------------------
// Conditions for 9600 -> 4800 Baud SW UART, SMCLK = 1MHz
//------------------------------------------------------------------------------
//#define UART_TBIT_DIV_2     (1000000 / (9600 * 2))
//#define UART_TBIT           (1000000 / 9600)

#define UART_TBIT_DIV_2     (1000000 / (4800 * 2))
#define UART_TBIT           (1000000 / 4800)

#define kPeriod      0x800 // 32768*8=2048Hz -> capture torque tickets
#define kPeriod1      0x8000 // 32768/32768=1Hz -> capture torque tickets
//------------------------------------------------------------------------------
// Global variables used for full-duplex UART communication
//------------------------------------------------------------------------------
unsigned int txData;                        // UART internal variable for TX
unsigned char rxBuffer;                     // Received UART character
const char string1[] = { "Hello World¥r¥n" };
unsigned int i;

//------------------------------------------------------------------------------
// Function prototypes
//------------------------------------------------------------------------------
void TimerA_UART_init(void);
void TimerA_UART_tx(unsigned char byte);
void TimerA_UART_print(char *string);

void Timer1_A_period_CAL_init(void);
void Timer1_A_period_init(void);
void Sleep_setup(void);
//------------------------------------------------------------------------------
// ANT Data
//------------------------------------------------------------------------------
#define MESG_NETWORK_KEY_ID      0x46
#define MESG_NETWORK_KEY_SIZE       9
#define ANT_CH_ID    0x00
#define ANT_CH_TYPE  0x10     //Master (0x10)
#define ANT_NET_ID   0x00
#define ANT_DEV_ID1  0x29     //41
#define ANT_DEV_ID2  0x00
#define ANT_DEV_TYPE 0x0B     // Device Type, HRM=0x78, Power=0x0B(11)
#define ANT_TX_TYPE  0x05     //ANT+ devices follow the transmission type definition as outlined in the ANT protocol.
#define ANT_CH_FREQ  0x0039   //   2457MHz
#define ANT_CH_PER   0x1FF6  //0x1FF6 32768/8182=4.004888780Hz // 0x1FA6 32768/8102=4.044Hz 32768/8192=4Hz

typedef uint8_t uchar;
uchar	txBuffer[32];
uint8_t	txBufferSize;
uint8_t	txBufferPos;


uint16_t new_cad_time_stamp=0;
uint16_t old_cad_time_stamp=0;

unsigned int PulseTicket=0;
uint16_t cad_timer_cap=0;

uint16_t ctf_time_stamp1;
uint16_t ctf_torque_ticks1;
uint8_t Rotation_event_counter;
uint16_t cad_period_counter;


enum{
	CTMMODE = 1,
	OFFSETMODE
};
int unqomode = CTMMODE;         // for mode changer 1=CTM mode 2=OFFSET mode


enum{
	ACTIVE = 1,
	SLEEP
};
int activemode = ACTIVE;         // for mode changer 1=Active mode 2=Sleep mode

uint16_t active_timer_counter;



//------------------------------------------------------------------------------
//  TX: sync+data+sum+CR+LF
//------------------------------------------------------------------------------

void txMessage(uchar*	message, uint8_t	messageSize)
{
  uint8_t i;
  _BIC_SR(GIE);  // disable interrupt

    txBufferPos     = 0;                               // set position to 0
    txBufferSize    = messageSize + 3;   // message plus syc, size and checksum
    txBuffer[0]     = 0xa4;                          // sync byte
    txBuffer[1]     = (uchar) messageSize - 1;      // message size - command size (1)

  for(i=0; i<messageSize; i++)
    txBuffer[2+i] = message[i];

    // calculate the checksum
       txBuffer[txBufferSize - 1] = 0; //add
    for(i=0; i<txBufferSize - 1; ++i)
        txBuffer[txBufferSize - 1] = txBuffer[txBufferSize - 1] ^ txBuffer[i];

   _BIS_SR(GIE);   // enable interrupt

  // now send via UART
  for(i=0; i<txBufferSize; i++)
      TimerA_UART_tx(txBuffer[i]);
}

//------------------------------------------------------------------------------
//  ANT Data
//------------------------------------------------------------------------------

// Resets module
void reset()
{
    uchar setup[2];
    setup[0] = 0x4a; // ID Byte
    setup[1] = 0x00; // Data Byte N (N=LENGTH)
    txMessage(setup, 2);
}

void deep_sleep()
{
    uchar setup[2];
    setup[0] = 0xc5; // ID Byte
    setup[1] = 0x00; // Data Byte N (N=LENGTH)
    txMessage(setup, 2);
}



void ANTAP1_AssignNetwork()
{
    uchar setup[10];

    setup[0] = MESG_NETWORK_KEY_ID;
    setup[1] = ANT_CH_ID; // chan
    setup[2] = 0xXX; //NETWORK_KEY_ID
    setup[3] = 0xXX;
    setup[4] = 0xXX;
    setup[5] = 0xXX;
    setup[6] = 0xXX;
    setup[7] = 0xXX;
    setup[8] = 0xXX;
    setup[9] = 0xXX;
    txMessage(setup, 10);
}


// Assigns CH=0, CH Type=10(TX), Net#=0
void assignch()
{
    uchar setup[4];
    setup[0] = 0x42;
    setup[1] = ANT_CH_ID;    // Channel ID
    setup[2] = ANT_CH_TYPE;  // CH Type
    setup[3] = ANT_NET_ID;   // Network ID
    txMessage(setup, 4);
}

// set RF frequency
void setrf()
{
    uchar setup[3];
    setup[0] = 0x45;
    setup[1] = ANT_CH_ID;    // Channel ID
    setup[2] = 0x39;    // RF Frequency
    txMessage(setup, 3);
}

// set channel period
void setchperiod()
{
    uchar setup[4];
    setup[0] = 0x43;
    setup[1] = ANT_CH_ID;
    setup[2] = (ANT_CH_PER & 0xFF);  //Channel Period LSB
    setup[3] = ((ANT_CH_PER & 0xFF00) >> 8);   // Channel Period MSB
    txMessage(setup, 4);
}

// Assigns CH#, Device#=0000, Device Type ID=00, Trans Type=00
void setchid()
{
    uchar setup[6];
    setup[0] = 0x51;
    setup[1] = ANT_CH_ID;      // Channel Number, 0x00 for HRM
    setup[2] = ANT_DEV_ID1;    // Device Number LSB
    setup[3] = ANT_DEV_ID2;    // Device Number MSB
    setup[4] = ANT_DEV_TYPE;   // Device Type, 0x78 for HRM
    setup[5] = ANT_TX_TYPE;
    txMessage(setup, 6);
}

/////////////////////////////////////////////////////////////////////////
// Priority:
//
// ucPower_:   0 = TX Power -20dBM
//             1 = TX Power -10dBM
//             2 = TX Power -5dBM
//             3 = TX Power 0dBM
//
/////////////////////////////////////////////////////////////////////////
/*
void ChannelPower()
{
    uchar setup[3];
    setup[0] = 0x47;
    setup[1] = 0x00;
    setup[2] = 0x03;


    txMessage(setup, 3);
}
*/

// Opens CH 0
void opench()
{
    uchar setup[2];
    setup[0] = 0x4b;
    setup[1] = ANT_CH_ID;
    txMessage(setup, 2);
}



// Sends sendPower_CTF1
void sendPower_CTF1()
{
    uchar setup[10];
    setup[0] = 0x4e;     //broadcast data
    setup[1] = ANT_CH_ID;     // 0x41;     //?
    setup[2] = 0x20;     // 0x20 Data Page Number Crank Torque Frequency
    setup[3] = Rotation_event_counter;      //Rotation event counter increments with each completed pedal revolution.
    setup[4] = 0x01;     // Slope MSB 1/10 Nm/Hz
    setup[5] = 0x31;     // Slope LSB 1/10 Nm/Hz
    setup[6] = ((0xFF00 & (ctf_time_stamp1)) >>8);      //Accumulated Time Stamp MSB 1/2000s
    setup[7] = (0xFF & (ctf_time_stamp1));     //Accumulated Time Stamp LSB 1/2000s
    setup[8] = ((0xFF00 & ctf_torque_ticks1) >>8);     //Accumulated Torque Ticks Stamp MSB
    setup[9] = (0xFF & ctf_torque_ticks1);        //Accumulated Torque Ticks Stamp LSB
        txMessage(setup, 10);
}

// Sends sendPower_CTF1_Calibration
void sendPower_CTF1_CAL()
{
    uchar setup[10];
    setup[0] = 0x4e;      //broadcast data
    setup[1] = ANT_CH_ID;     // 0x41;     //?
    setup[2] = 0x01;      // Data page Number : Calibration massage
    setup[3] = 0x10;      // Calibration ID : CTF defined massage
    setup[4] = 0x01;      // CTF Defined ID : Zero offset
    setup[5] = 0xFF;      // Reserved :
    setup[6] = 0xFF;      // Reserved :
    setup[7] = 0xFF;      // Reserved :
    setup[8] = ((0xFF00 & ctf_torque_ticks1) >>8);     //Offset MSB
    setup[9] = (0xFF & ctf_torque_ticks1);        //Offset LSB
    txMessage(setup, 10);
}


//------------------------------------------------------------------------------
// main()
//------------------------------------------------------------------------------
void main(void)

{
 WDTCTL = WDTPW + WDTHOLD;               // Stop watchdog timer

    DCOCTL = 0x00;                          // Set DCOCLK to 1MHz
    BCSCTL1 = CALBC1_1MHZ;
    DCOCTL = CALDCO_1MHZ;

    BCSCTL1 |= DIVA_3;                      // ACLK/8   32768/8=4096 0X1000 DIVA_3 DIVA_0
    BCSCTL3 |= (XCAP0 + XCAP1);				// Set ACLK Capacity to 12.5 pF


// P1.X  setup
//    P1OUT = 0x00;                           // Initialize all GPIO
//    P1REN = 0x00;                           // pull up disable
    P1OUT |= BIT3;                          // P1.3 SW is VDD pulled up
    P1SEL = 0x00;
//    P1DIR = 0xFF;

    P1SEL = UART_TXD + UART_RXD;            // Timer function for TXD/RXD pins
    P1DIR &= ~UART_RXD;               // Set all pins but RXD to output
    P1DIR &= ~BIT1; //TXD HIZ
    P1DIR &= ~BIT2; //RXD HIZ
    P2DIR &= ~BIT5; //ANT RESET HIZ

// P2.X  setup
//    P2OUT = 0x00;                           // Initialize all GPIO
//    P2REN = 0x00;                           // pull up disable
//    P2SEL = 0x00;
//    P2DIR = 0xFF;

// P2.5 ANT chip RESET
    //P2DIR |=  BIT5;
    //P2OUT &= ~BIT5;                          //ANT chip RESET P2.5
//P1.4 ANT chip VDD OFF
    P1OUT &= ~BIT4;                          //ANT chip VDD OFF P1.4
    P1DIR |=  BIT4;

// P1.7 COMPARATOR BIAS PULL UP OFF
    P1OUT &= ~BIT7;                          //COMPARATOR BIAS PULL UP OFF P1.7
    P1DIR |=  BIT7;

//CONFIG ALART
    P1DIR |=  BIT6;
    for (i=0; i<5; i++)                  // Delay
     {
       P1OUT |=  BIT6;                          // P1.6 = toggle
       __delay_cycles(5000);                    // Delay between comm cycles  5msec
       P1OUT &= ~BIT6;                          // P1.6 = toggle
       __delay_cycles(100000);                   // Delay between comm cycles 100msec
     }
     __delay_cycles(20000);                  // Delay between comm cycles 20msec
     P1IFG &= ~BIT3;   // clear flag

     __delay_cycles(10000);                  // Delay between comm cycles 10msec

// P1.6 LED setup
    P1DIR |= BIT6;

// P2.1 LED setup PWM OUTPUT
     //P2SEL |= BIT1;
     //P2DIR |= BIT1;


// 1.0 LED setup
    P1DIR |=  BIT0;
    //P1OUT |=  BIT0;                          //LED ON P1.0
// P1.4 ANT chip VDD ON
     P1OUT |=  BIT4;                          //ANT chip VDD ON P1.4
     P1DIR |=  BIT4;
// P2.5 ANT chip RESET pull up
     //P2OUT |=  BIT5;                          //ANT chip RESET release P2.5
     //P2DIR |=  BIT5;
     //P2IFG &= ~BIT5;

// P1.7 COMPARATOR BIAS PULL UP ON
     P1DIR |=  BIT7;
     P1OUT |=  BIT7;                          //COMPARATOR BIAS PULL UP ON P1.7
     __delay_cycles(1000);                  // Delay between comm cycles 1msec

// P2.5 ANT chip RESET
     P2OUT &= ~BIT5;                          //ANT chip RESET release P2.5
     P2DIR |=  BIT5;
     P2IFG &= ~BIT5;
     __delay_cycles(100);                    // Delay between comm cycles 100usec
     P2OUT |=  BIT5;                          //ANT chip RESET release P2.5
     P2IFG &= ~BIT5;
     __delay_cycles(100);                    // Delay between comm cycles 100usec
     __delay_cycles(10000);                  // Delay between comm cycles 10msec

// Start Timer_A UART

     __enable_interrupt();
     TimerA_UART_init();                     // Start Timer_A UART
     __delay_cycles(1000);                  // Delay between comm cycles 1msec
     P1DIR |= UART_TXD;
// ANT chip configuration
//    P1OUT |= BIT0;                         //LED ON P1.0
    __delay_cycles(5000);                  // Delay between comm cycles 5msec

    reset();
    __delay_cycles(5000);                  // Delay between comm cycles 5msec


    ANTAP1_AssignNetwork();
     __delay_cycles(5000);                  // Delay between comm cycles 5msec

    assignch();
    __delay_cycles(5000);                  // Delay between comm cycles 5msec

    setrf();
    __delay_cycles(5000);                  // Delay between comm cycles 5msec


    setchperiod();
    __delay_cycles(5000);                  // Delay between comm cycles 5msec

    setchid();
    __delay_cycles(5000);                  // Delay between comm cycles 5msec

    opench();
    __delay_cycles(5000);                  // Delay between comm cycles 5msec

    __delay_cycles(100000);        // Delay between comm cycles 100msec
    P1OUT &= ~BIT0;               //LED OFF P1.0

// interrupt setup
// P1.3 for mode change switch
//   P1OUT |=  BIT6;              //LED ON P1.6
    P1DIR &= ~BIT3;               // INPUT mode
    P1REN |= BIT3;                // pull up enable
    P1OUT |= BIT3;                //
    P1IES |= BIT3;                // H -> L edge
    P1IFG &= ~BIT3;               // clear flag
    P1IE  |= BIT3;                // enable interrupt
//    _BIS_SR(GIE);                 // enable interrupt


// 2.2 pin for pulse ticket capture
   /*
    P1OUT |=  BIT0;               //LED ON P1.6
    P2DIR &= ~BIT2;               // INPUT mode
    P2REN &= ~BIT2;                // pull up enable->disable@140117
    P2OUT |= BIT2;                // pull VDD
    P2IES |= BIT2;                // H -> L edge
//    __delay_cycles(250000);       // Delay between comm cycles
    P1OUT &= ~BIT0;               //LED OFF P1.6
    P2IE  |= BIT2;                // enable interrupt
    */

// 1.5 pin for pulse ticket capture with COMPARATOR
    CACTL1 = CAEX  + CAREF_2 + CAON + CAIE; // 0.5 Vcc = -comp, on, CARSEL  + CAIES
    CACTL2 = P2CA3 + P2CA1 + CAF;                           // P1.5/CA5,FILTER on  + CAF
    CAPD = CAPD5;


// P2.0 set to TimerA_A3.CCI0A : CADENCE capture
//    P2REN &= ~ BIT0;               // pull up disable@140117
    P2REN |= BIT0;                   // pull up enable
    P2OUT |= BIT0;                   // pull VDD
    P2DIR &= ~ BIT0;                 // INPUT mode
    P2SEL |= BIT0;                   // Timer1_A3.CCI0A
    P2IFG &= ~BIT0;
    //__delay_cycles(500000);        // Delay between comm cycles


    Timer1_A_period_init();
    sendPower_CTF1();              //send dummy data


    P1OUT &= ~BIT6;               //LED OFF P1.6
    P1OUT &= ~BIT0;               //LED OFF P1.0



    _BIS_SR(LPM0_bits + GIE);                 // Enter LPM3 w/ interrupt
}

//------------------------------------------------------------------------------
// PORT1 mode change capture P1.3
//------------------------------------------------------------------------------
#pragma vector=PORT1_VECTOR
__interrupt void Port_1(void)
{
	 P1IFG &= ~BIT3;   // clear flag

    if(unqomode >= OFFSETMODE)
   		unqomode = CTMMODE;
       else
           unqomode = OFFSETMODE;

       if(unqomode == CTMMODE)
         {
            Timer1_A_period_init();             // Timer set CTM mode
                 _NOP();                                // SET BREAKPOINT HERE
         }
       else
       {
    	   Timer1_A_period_CAL_init();            // Timer set CAL mode
                 _NOP();                                // SET BREAKPOINT HERE

        }
    _NOP();                                // SET BREAKPOINT HERE
}

//------------------------------------------------------------------------------
// PORT1 ticket capture P1.5 with COMPARATOR
//------------------------------------------------------------------------------
#pragma vector=COMPARATORA_VECTOR
__interrupt void COMPARATORA(void)
{
    PulseTicket++;    //Counter
//    P1OUT |= BIT0;                        // if CAOUT set, set P1.0
    active_timer_counter = 0x00;            //reset sleep counter
    __delay_cycles(40);                      // Delay between comm cycles
//	CACTL2 &= ~CAOUT; // clear flag
//	P1OUT &= ~BIT0;                         //reset P1.0
//    __delay_cycles(2);                      // Delay between comm cycles

if (PulseTicket >= 0xFFFF) //over flow 65535=FFFF
//if (PulseTicket >= 256) //for check
    {
     PulseTicket = 0;
    }
 _NOP();
}

//------------------------------------------------------------------------------
// Timer1_A0 cadence period capture P2.0
//------------------------------------------------------------------------------
#pragma vector=TIMER1_A0_VECTOR
 __interrupt void TIMER1_A0(void)
{
     P1OUT |= BIT6;
	if(Rotation_event_counter >= 0xFF)
      {
         Rotation_event_counter = 0x00;
      }

   Rotation_event_counter++;
   ctf_torque_ticks1 = PulseTicket;
   cad_timer_cap = TA1CCR0;
   new_cad_time_stamp = cad_timer_cap; // 2048 pulse/s
   old_cad_time_stamp =  new_cad_time_stamp;

   if(unqomode == CTMMODE)
     {
              ctf_time_stamp1 = new_cad_time_stamp;
              sendPower_CTF1();
              _NOP();                                 // SET BREAKPOINT HERE
     }
   else
    {
             sendPower_CTF1_CAL();                   // Capture by 1 second with kPeriod
             PulseTicket = 0;                        // added for reset
              _NOP();                                // SET BREAKPOINT HERE
    }
   P1OUT &= ~BIT6;
_NOP();                                // SET BREAKPOINT HERE

}

//------------------------------------------------------------------------------
// Function configures Timer1_A for CTM period capture
//------------------------------------------------------------------------------
void Timer1_A_period_init(void)
{

   TA1CCTL0 = CM_1 + SCS + CCIS_0 + CAP + CCIE; // Rising edge + Timer1_A3.CCI0A (P2.0) //
                                                // + Capture Mode + Interrupt
   TA1CTL = TASSEL_1 + ID_1  + MC_2 + TAIE;         // ACLK + 4096/2, Continuous Mode interrupt
}
//------------------------------------------------------------------------------
// Function configures Timer1_A for cal period capture
//------------------------------------------------------------------------------
void Timer1_A_period_CAL_init(void)
{
   TA1CCTL0 = CM_0 + SCS + CCIS_0 +  CCIE; // no capture + Timer1_A3.CCI0A (P2.0)
   TA1CCR0 = kPeriod;
   TA1CTL = TASSEL_1 + ID_1  + MC_1; // + TAIE;       // ACLK, 4096/2, UP mode //interrupt
}

//------------------------------------------------------------------------------
// Timer1_A1 sleep control
//------------------------------------------------------------------------------
#pragma vector=TIMER1_A1_VECTOR
 __interrupt void TIMER1_A1(void)
 {
	  switch( TA1IV )
	   {
	   case  2:
			    _NOP();                                // SET BREAKPOINT HERE
		        break;                                     // CCR1 not used
	   case  4:
		        _NOP();                                // SET BREAKPOINT HERE
		        break;                                     // CCR2 not used
	   case 10:
	        	   active_timer_counter++ ;                   // Timer1_A1 overflow

                  if (active_timer_counter >= 0x0002)          //65536/2048=32sec 32sec縲\x 2 =64sec
	     	         {
                	  //CONFIG ALART
                	 for (i=0; i<5; i++)                  // Delay
                	 {
                	     P1OUT |=  BIT0;                          // P1.0 = toggle
                	 	 _delay_cycles(5000);                  // Delay between comm cycles 5msec
                	 	 P1OUT &= ~BIT0;                          // P1.0 = toggle
                	 	 __delay_cycles(100000);                  // Delay between comm cycles 100msec
                	  }
                	 	 __delay_cycles(200000);                 // Delay between comm cycles  200msec


                	      active_timer_counter = 0x00;
            	          activemode = SLEEP;
            	          Sleep_setup();
   	                   _bis_SR_register(LPM4_bits + GIE);       // Enter LPM4 w/interrupt; //sleep!!
                          WDTCTL = WDTPW+WDTCNTCL+WDTSSEL+WDTIS0; //for reset:watchdog timer set 250msec
	     	          }
               break;
	   }

}
 //------------------------------------------------------------------------------
 // Function configures IC sleep setup
 //------------------------------------------------------------------------------
 void Sleep_setup(void)
 {
   P1OUT |=  BIT0;                          // P1.0 = toggle
// ANT chip configuration
   reset();
   __delay_cycles(5000);                  // Delay between comm cycles 5msec
   deep_sleep();

   TA1CTL |= TACLR; // disable Timer1 interrupt
   TA0CTL |= TACLR; // disable Timer0 interrupt
   TA1CTL &= ~TAIE; // disable Timer1 interrupt
   TA0CTL &= ~TAIE; // disable Timer0 interrupt
   CACTL1 = 0x00;   // disable COMPARATOR
   CACTL2 = 0x00;   // disable COMPARATOR
   CAPD = 0x00;     // disable COMPARATOR

   P1SEL &= ~BIT1;  // UART TX OFF
   P1SEL &= ~BIT2;  // UART RX OFF
   P1DIR &= ~BIT1;  // HIZ
   P1DIR &= ~BIT2;  // HIZ
   P1OUT &= ~BIT1;  // UART TX OFF
   P1OUT &= ~BIT2;  // UART RX OFF
   P2OUT &= ~BIT5;  // ANT reset
   P1OUT &= ~BIT4;  // ANT Power supply OFF
   P1OUT &= ~BIT5;  // COMPARATOR OFF
   P1OUT &= ~BIT6;  // LED6 OFF

   P1OUT &= ~BIT7;  // COMPARATOR BIAS OFF


   // P2.0 set to wake up interrupt
   P2REN |= BIT0;               // pull up enable@140218
   P2OUT |= BIT0;               // pull VDD
   P2DIR &= ~ BIT0;             // INPUT mode
   P2SEL &= ~ BIT0;             // disable Timer1_A3.CCI0A enable I/O
   P2IES &= ~ BIT0;             // L -> H edge
   P2IFG &= ~BIT0;              // clear flag
   P2IE  |= BIT0;               // enable interrupt
  //   P1SEL = 0x00;
  //   P1DIR = 0xFF;
   __delay_cycles(1000000);                  // Delay between comm cycles 1000msec
   P1OUT &= ~BIT0;  // LED1 OFF
   P1OUT &= ~BIT0;  // LED1 OFF
}
//------------------------------------------------------------------------------
// PORT2 wake up interrupt capture P2.0
//------------------------------------------------------------------------------
#pragma vector=PORT2_VECTOR
__interrupt void Port_2(void)
{
  P2IFG &= ~BIT0;   // clear flag
  _bic_SR_register_on_exit(LPM4_bits);   /* Exit Low Power Mode 4 */
  _NOP();
}


//------------------------------------------------------------------------------
// Function configures Timer_A for full-duplex UART operation
//------------------------------------------------------------------------------
void TimerA_UART_init(void)
{
    TA0CCTL0 = OUT;                          // Set TXD Idle as Mark = '1'
    TA0CCTL1 = SCS + CM1 + CAP + CCIE;       // Sync, Neg Edge, Capture, Int
    TA0CTL = TASSEL_2 + MC_2;                // SMCLK, start in continuous mode
}
//------------------------------------------------------------------------------
// Outputs one byte using the Timer_A UART
//------------------------------------------------------------------------------
void TimerA_UART_tx(unsigned char byte)
{
   while (TACCTL0 & CCIE);                 // Ensure last char got TX'd
    TA0CCR0 = TAR;                           // Current state of TA counter
    TA0CCR0 += UART_TBIT;                    // One bit time till first bit
    TA0CCTL0 = OUTMOD0 + CCIE;               // Set TXD on EQU0, Int
    txData = byte;                          // Load global variable
    txData |= 0x100;                        // Add mark stop bit to TXData
    txData <<= 1;                           // Add space start bit

}
//------------------------------------------------------------------------------
// Prints a string over using the Timer_A UART
//------------------------------------------------------------------------------
void TimerA_UART_print(char *string)
{
    while (*string) {
        TimerA_UART_tx(*string++);
    }
}
//------------------------------------------------------------------------------
// Timer0_A0 UART - Transmit Interrupt Handler
//------------------------------------------------------------------------------
#pragma vector = TIMER0_A0_VECTOR
__interrupt void Timer_A0_ISR(void)
{
    static unsigned char txBitCnt = 10;
    TA0CCR0 += UART_TBIT;                    // Add Offset to CCRx
    if (txBitCnt == 0) {                    // All bits TXed
        TA0CCTL0 &= ~CCIE;                   // All bits TXed, disable interrupt
        txBitCnt = 10;                      // Re-load bit counter

    }
    else {
    	 if (txData & 0x01) {
          TA0CCTL0 &= ~OUTMOD2;              // TX Mark '1'
        }
        else {
          TA0CCTL0 |= OUTMOD2;               // TX Space '0'
        }
        txData >>= 1;
        txBitCnt--;
        }
}
//------------------------------------------------------------------------------
// Timer0_A1 UART - Receive Interrupt Handler
//------------------------------------------------------------------------------
#pragma vector = TIMER0_A1_VECTOR
__interrupt void Timer_A1_ISR(void)
{
    static unsigned char rxBitCnt = 8;
    static unsigned char rxData = 0;
    //P1OUT ^= BIT0;
    switch (__even_in_range(TA0IV, TA0IV_TAIFG)) { // Use calculated branching
        case TA0IV_TACCR1:                        // TA0CCR1 CCIFG - UART RX
            TA0CCR1 += UART_TBIT;                 // Add Offset to CCRx
            if (TA0CCTL1 & CAP) {                 // Capture mode = start bit edge
//            	     P1OUT |= BIT0;
                TA0CCTL1 &= ~CAP;                 // Switch capture to compare mode
                TA0CCR1 += UART_TBIT_DIV_2;       // Point CCRx to middle of D0
            }
            else {
                rxData >>= 1;
                if (TA0CCTL1 & SCCI) {            // Get bit waiting in receive latch
                    rxData |= 0x80;
                }
                rxBitCnt--;
                if (rxBitCnt == 0) {             // All bits RXed
                    rxBuffer = rxData;           // Store in global variable
                    rxBitCnt = 8;                // Re-load bit counter
                    TA0CCTL1 |= CAP;              // Switch compare to capture mode
//                       P1OUT &= ~BIT0;
                    __bic_SR_register_on_exit(LPM0_bits);  // Clear LPM0 bits from 0(SR)
                }
            }
            break;
    }
}




