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
//        /|\|              XIN|-
//         | |                 | 32kHz
//         --|RST          XOUT|-
//           |                 |
//           |   CCI0B/TXD/P1.1|-------->
//           |                 | 4800 8N1    //9600 8N1
//           |   CCI0A/RXD/P1.2|<--------
//
//
//  D. Dang
//  Texas Instruments Inc.
//  December 2010
//   Built with CCS Version 4.2.0 and IAR Embedded Workbench Version: 5.10
//******************************************************************************

//******************************************************************************
//P1.1TXD
//P1.2RXD
//P1.3=mode change switch
//
//P2.0=cadence capture
//P2.2=torque capture
//
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

#define kPeriod      0x1000 // 32768*8=4096Hz -> capture torque tickets
#define kPeriod1      0x8000 // 32768/32768=1Hz -> capture torque tickets
//------------------------------------------------------------------------------
// Global variables used for full-duplex UART communication
//------------------------------------------------------------------------------
unsigned int txData;                        // UART internal variable for TX
unsigned char rxBuffer;                     // Received UART character
const char string1[] = { "Hello World\r\n" };
unsigned int i;

//------------------------------------------------------------------------------
// Function prototypes
//------------------------------------------------------------------------------
void TimerA_UART_init(void);
void TimerA_UART_tx(unsigned char byte);
void TimerA_UART_print(char *string);

void Timer1_A_period_CAL_init(void);
void Timer1_A_period_init(void);

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
	CAD_1 = 1,
	CAD_2
};

int CADMODE = CAD_1;         // for mode changer cad period 0-7FFF 8000-FFFF



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

    BCSCTL1 |= DIVA_3;                      // ACLK/8   32768/8=4096 0X1000

    // P1.X  setup
    P1OUT = 0x00;                           // Initialize all GPIO
    P1OUT |= BIT3;                          // P1.3 SW is VDD pulled up
    P1SEL = 0x00;
    P1DIR = 0xFF;

    P1SEL = UART_TXD + UART_RXD;            // Timer function for TXD/RXD pins
    P1DIR = 0xFF & ~UART_RXD;               // Set all pins but RXD to output

// P1.6 LED setup
    P1DIR |=  BIT6;
    P1OUT |=  BIT6;                          //LED ON P1.6

// 1.0 LED setup
    P1DIR |=  BIT0;

    __enable_interrupt();
    TimerA_UART_init();                     // Start Timer_A UART

// ANT chip configuration
    P1OUT |= BIT0;                         //LED ON P1.0

    reset();
    //P1OUT |= BIT0;
    __delay_cycles(5000);                  // Delay between comm cycles
    //P1OUT &= ~BIT0;
    //__delay_cycles(5000);

    ANTAP1_AssignNetwork();
    //P1OUT |= BIT0;
    __delay_cycles(5000);                  // Delay between comm cycles
    //P1OUT &= ~BIT0;
    //__delay_cycles(5000);

    assignch();
    //P1OUT |= BIT0;
    __delay_cycles(5000);                  // Delay between comm cycles
    //P1OUT &= ~BIT0;
    //__delay_cycles(5000);

    setrf();
    //P1OUT |= BIT0;
    __delay_cycles(5000);                  // Delay between comm cycles
    //P1OUT &= ~BIT0;
    //__delay_cycles(5000);

    setchperiod();
    //P1OUT |= BIT0;
    __delay_cycles(5000);                  // Delay between comm cycles
    //P1OUT &= ~BIT0;
    //__delay_cycles(5000);

    setchid();
    //P1OUT |= BIT0;
    __delay_cycles(5000);                  // Delay between comm cycles
    //P1OUT &= ~BIT0;
    //__delay_cycles(5000);

//    ChannelPower();
//    P1OUT |= BIT0;
//     __delay_cycles(5000);               // Delay between comm cycles
//     P1OUT &= ~BIT0;
//     __delay_cycles(5000);

    opench();
    //P1OUT |= BIT0;
    __delay_cycles(5000);                  // Delay between comm cycles
    //P1OUT &= ~BIT0;
    //__delay_cycles(5000);

    __delay_cycles(250000);        // Delay between comm cycles
    P1OUT &= ~BIT0;               //LED OFF P1.0
//    __delay_cycles(250000);        // Delay between comm cycles

// interrupt setup
// P1.3 for mode change switch
    P1OUT |=  BIT6;               //LED ON P1.6

    P1DIR &= ~BIT3;               // INPUT mode
    P1REN |= BIT3;                // pull up enable
    P1OUT |= BIT3;                //
    P1IES |= BIT3;                // H -> L edge
//    __delay_cycles(250000);        // Delay between comm cycles
    P1OUT &= ~BIT6;               //LED OFF P1.6
    P1IE  |= BIT3;                // enable interrupt

// 2.2 pin for pulse ticket capture
    P1OUT |=  BIT0;               //LED ON P1.6
    P2DIR &= ~BIT2;               // INPUT mode
    P2REN |= BIT2;                // pull up enable
    P2OUT |= BIT2;                // pull VDD
    P2IES |= BIT2;                // H -> L edge
//    __delay_cycles(250000);       // Delay between comm cycles
    P1OUT &= ~BIT0;               //LED OFF P1.6
    P2IE  |= BIT2;                // enable interrupt

// P2.0 set to TimerA_A3.CCI0A : CADENCE capture
    P1OUT |=  BIT6;               //LED ON P1.0
    P2REN |=  BIT0;               // pull up enable
    P2OUT |=  BIT0;               // pull VDD
    P2DIR &= ~ BIT0;              // INPUT mode
    P2SEL |= BIT0;                // Timer1_A3.CCI0A
    //__delay_cycles(500000);        // Delay between comm cycles


    Timer1_A_period_init();

    P1OUT &= ~BIT6;               //LED OFF P1.6



//    _BIS_SR(LPM3_bits + GIE);                 // Enter LPM3 w/ interrupt
}

//------------------------------------------------------------------------------
// PORT1 mode change capture P1.3
//------------------------------------------------------------------------------

#pragma vector=PORT1_VECTOR
__interrupt void Port_1(void)
{
    P1OUT ^= BIT6;                // LED Toggle P1.6 using exclusive-OR
//    __delay_cycles(100000);       // Delay between comm cycles
    P1OUT &= ~BIT6;               //LED OFF P1.6
//    __delay_cycles(100000);       // Delay between comm cycles
//  P1OUT ^= BIT0;   // LED Toggle P1.0 using exclusive-OR

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
// PORT2 ticket capture P2.2
//------------------------------------------------------------------------------
#pragma vector=PORT2_VECTOR
__interrupt void Port_2(void)
{
P2IFG &= ~BIT2;   // clear flag

PulseTicket++;    //Counter

if (PulseTicket >= 0xFFFF) //over flow 65535=FFFF
//if (PulseTicket >= 256) //for check
{
    PulseTicket = 0;
}
 _NOP();

}
//------------------------------------------------------------------------------
// Timer_A cadence capture P2.0
//------------------------------------------------------------------------------
#pragma vector=TIMER1_A0_VECTOR
 __interrupt void TIMER1_A0(void)
 {
	   if (Rotation_event_counter >= 0xFF)
	   {
	    Rotation_event_counter = 0x00;
	   }
      Rotation_event_counter++;
      ctf_torque_ticks1 = PulseTicket;
      cad_timer_cap = TA1CCR0;
      new_cad_time_stamp = cad_timer_cap  / 2 ;     // 4096 pulse/s change to 2000/s

      if(CADMODE == CAD_1) // 0000-7FFF
    	   {
    		   if(new_cad_time_stamp <= old_cad_time_stamp)
    		     {
    			  CADMODE = CAD_2;
    		     }
    	   }
      else                 // +0x8000
    	   {
    	      if(new_cad_time_stamp + 0x8000 <= old_cad_time_stamp)
    	    	{
    	    	 CADMODE = CAD_1;
    	    	}
    	   }

      if(CADMODE == CAD_1) // 0000-7FFF
    	   {
	         //new_cad_time_stamp = new_cad_time_stamp;  //
    	   }
      else                 // +0x8000
    	   {
    	       new_cad_time_stamp = new_cad_time_stamp + 0x8000;
    	   }

	   old_cad_time_stamp =  new_cad_time_stamp;

    if(unqomode == CTMMODE)
          {
                     P1OUT &= ~BIT0;                         // LED OFF P1.0
                     P1OUT ^= BIT6;                          // Toggle P1.6 using exclusive-OR
                     ctf_time_stamp1 = new_cad_time_stamp;
                     sendPower_CTF1();
                     _NOP();                                 // SET BREAKPOINT HERE
          }
    else
    {
                    P1OUT &= ~BIT6;                         // LED OFF P1.6
                    P1OUT ^= BIT0;                          // LED Toggle P1.0 using exclusive-OR
                    sendPower_CTF1_CAL();                   // Capture by 1 second with kPeriod
                    PulseTicket = 0;                        // added for reset
                     _NOP();                                // SET BREAKPOINT HERE
    }
    _NOP();                                // SET BREAKPOINT HERE
}
 //------------------------------------------------------------------------------
 // Function configures Timer1_A for CTM period capture
 //------------------------------------------------------------------------------
 void Timer1_A_period_init(void)
 {

   TA1CCTL0 = CM_1 + SCS + CCIS_0 + CAP + CCIE; // Rising edge + Timer1_A3.CCI0A (P2.0) //
                                                // + Capture Mode + Interrupt
   TA1CTL = TASSEL_1 + MC_2; // + TAIE;         // ACLK + Continuous Mode   //interrupt
 }
 //------------------------------------------------------------------------------
 // Function configures Timer1_A for cal period capture
 //------------------------------------------------------------------------------
 void Timer1_A_period_CAL_init(void)
 {

   TA1CCTL0 = CM_0 + SCS + CCIS_0 +  CCIE; // no capture + Timer1_A3.CCI0A (P2.0)
   //TA1CCTL0 |= CCIE;                          // enable interrupt
   //TA1CCTL0 |= CM_0; // no capture
   //TA1CCTL0 |= CCIE; // enable interrupt
   TA1CCR0 = kPeriod;
   TA1CTL = TASSEL_1 + MC_1; // + TAIE;       // ACLK, UP mode //interrupt
 }
//------------------------------------------------------------------------------
// Function configures Timer_A for full-duplex UART operation
//------------------------------------------------------------------------------
void TimerA_UART_init(void)
{
    TACCTL0 = OUT;                          // Set TXD Idle as Mark = '1'
    TACCTL1 = SCS + CM1 + CAP + CCIE;       // Sync, Neg Edge, Capture, Int
    TACTL = TASSEL_2 + MC_2;                // SMCLK, start in continuous mode
}
//------------------------------------------------------------------------------
// Outputs one byte using the Timer_A UART
//------------------------------------------------------------------------------
void TimerA_UART_tx(unsigned char byte)
{
    while (TACCTL0 & CCIE);                 // Ensure last char got TX'd
    TACCR0 = TAR;                           // Current state of TA counter
    TACCR0 += UART_TBIT;                    // One bit time till first bit
    TACCTL0 = OUTMOD0 + CCIE;               // Set TXD on EQU0, Int
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
// Timer_A UART - Transmit Interrupt Handler
//------------------------------------------------------------------------------
#pragma vector = TIMER0_A0_VECTOR
__interrupt void Timer_A0_ISR(void)
{
    static unsigned char txBitCnt = 10;

    TACCR0 += UART_TBIT;                    // Add Offset to CCRx
    if (txBitCnt == 0) {                    // All bits TXed
        TACCTL0 &= ~CCIE;                   // All bits TXed, disable interrupt
        txBitCnt = 10;                      // Re-load bit counter
    }
    else {
        if (txData & 0x01) {
          TACCTL0 &= ~OUTMOD2;              // TX Mark '1'
        }
        else {
          TACCTL0 |= OUTMOD2;               // TX Space '0'
        }
        txData >>= 1;
        txBitCnt--;
    }
}
//------------------------------------------------------------------------------
// Timer_A UART - Receive Interrupt Handler
//------------------------------------------------------------------------------
#pragma vector = TIMER0_A1_VECTOR
__interrupt void Timer_A1_ISR(void)
{
    static unsigned char rxBitCnt = 8;
    static unsigned char rxData = 0;

    switch (__even_in_range(TA0IV, TA0IV_TAIFG)) { // Use calculated branching
        case TA0IV_TACCR1:                        // TACCR1 CCIFG - UART RX
            TACCR1 += UART_TBIT;                 // Add Offset to CCRx
            if (TACCTL1 & CAP) {                 // Capture mode = start bit edge
                TACCTL1 &= ~CAP;                 // Switch capture to compare mode
                TACCR1 += UART_TBIT_DIV_2;       // Point CCRx to middle of D0
            }
            else {
                rxData >>= 1;
                if (TACCTL1 & SCCI) {            // Get bit waiting in receive latch
                    rxData |= 0x80;
                }
                rxBitCnt--;
                if (rxBitCnt == 0) {             // All bits RXed
                    rxBuffer = rxData;           // Store in global variable
                    rxBitCnt = 8;                // Re-load bit counter
                    TACCTL1 |= CAP;              // Switch compare to capture mode
                    __bic_SR_register_on_exit(LPM0_bits);  // Clear LPM0 bits from 0(SR)
                }
            }
            break;
    }
}



