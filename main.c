#include <msp430.h>
#include <stdio.h>

//------------------------------------------------------------------------------
//	Ports
//------------------------------------------------------------------------------
#define BUZZER BIT0 // Buzzer on P
#define BUTTON BIT0 // Button on P
#define RXD	BIT2	//Rx on P1.2
#define TXD BIT1	//Tx on P1.1

//------------------------------------------------------------------------------
//	Constants
//------------------------------------------------------------------------------
#define FALSE 0
#define TRUE  1
#define T 0x34
#define F 0x26

//------------------------------------------------------------------------------
//	Flags
//------------------------------------------------------------------------------

unsigned int fallDet; // Fall detected - True = 1, False = 0

//------------------------------------------------------------------------------
//	Variables
//------------------------------------------------------------------------------

char cmd;		//Used for the UART
int countPress = 0; // User to count the number of press
char data[] = {'$'};
unsigned char letter = 'E';
unsigned char *string;
int i=0;

char receivedData[20];

//------------------------------------------------------------------------------
// Declarations
//------------------------------------------------------------------------------
void initUART();
void configureUART(unsigned int br0, unsigned int brs);
void configurePorts();
void sendUART(unsigned char *str, unsigned int len);
void setupBT();


//------------------------------------------------------------------------------
// main()
//------------------------------------------------------------------------------
int main(void){

    WDTCTL = WDTPW + WDTHOLD;               // Stop watchdog timer

    //----------------------------- Configure the clock-------------------------

    if(CALBC1_1MHZ==0xFF){			//If calibration constant erased
    	while(1);					//trap cpu
    }
    // Basic Clock Module (set to 1MHz)
    DCOCTL = 0;
    BCSCTL1 = CALBC1_1MHZ;
    DCOCTL = CALDCO_1MHZ;

    configurePorts();

    initUART();
//    sendUART("HOLA",6);
//
//    sendUART("KENNETH", 9);
//
//    sendUART("H",2);


    sendUART("Changed to 9600", 18);
    sendUART("E\n", 4);


    //UCA0TXBUF = 'A';                  // Transmit a byte

    //__enable_interrupt();
    //_BIS_SR(GIE);

    _BIS_SR(LPM0_bits + GIE);         // Going to LPM0
//    while(1){
//    	__no_operation();
//    }
}

void configurePorts(){

	P1DIR |= BIT0 + BIT6;	//P1.0 and P1.6 as output
	P1OUT &= ~BIT0 + BIT6;	//P1.0 and P1.6 = 0

//	P1DIR |= BIT1 + BIT2;	//P1.1 and P1.2 as output
//	P1OUT &= ~BIT1 + ~BIT2;	//P1.1 and P1.2 = 0

	P1DIR &= ~BIT3; //P1.3 i/p
	P1REN |=  BIT3; //P1.3 enable pullup resistor
	P1IES |=  BIT3; //P1.3 high to low transition
	P1IFG &= ~BIT3; //P1.3 clear interrupt flag
	P1IE  |=  BIT3; //enable P1.3 interrupt

	//Set the UART function for P1.1 and P1.2
	P1SEL |= BIT1 + BIT2;
	P1SEL2 |= BIT1 + BIT2;
}

void initUART(){
	//For a baudrate 115200
    configureUART(8, 0x0C);
    //string  = "SSS";
    string  = "$$$";

    //Change to the command mode
    sendUART(string,5);

    //Change the baudrate in the Bluetooth to 9600
    sendUART("U,9600,N\n",11);

}
void configureUART(unsigned int br0, unsigned int brs){
	//Configure the UART(USCI_A0)
	UCA0CTL1 |= UCSSEL_2 + UCSWRST;	// Set USCI clock to SMCLK and put state machine in reset

	//UCA0BR0 = 8;					// From the datasheet, for a 9600 baud rate if SMCLK=1MHz
	UCA0BR0 = br0;

	UCA0BR1 = 0;					// Baudrate = 1MHz / (256 * UCA0BR1 + UCA0BR0)

	//UCA0MCTL |= UCBRS_6;			// Modulation UCBRSx=1
	UCA0MCTL |= brs;

	//UCA0STAT |=  UCLISTEN;          // loop back mode enabled
	UCA0CTL1 &= ~UCSWRST;			// Initialize USCI state machine
	IE2 |= UCA0RXIE; 				// Enable USCI_A0 RX Interrupt
//	IE2 |= UCA0TXIE;				// Enable USCI_A0 TX Interrupt
}

void sendUART(unsigned char *value, unsigned int len){

	while(len--){
		while(!(IFG2 & UCA0TXIFG));
		UCA0TXBUF = *value;
//		if(*value != '\n'){
//			UCA0TXBUF = '\r';
//		}
		value++;
	}

}

#pragma vector = USCIAB0RX_VECTOR
__interrupt void ReceiveInterrupt(void){

	P1OUT ^= BIT6;	// Toggle P1.6 led on Rx
	receivedData[i] = UCA0RXBUF;
	i++;
	//printf("cmd: %d\n", cmd);

//	switch(UCA0RXBUF){
//		case T:
//			fallDet = TRUE;
//		case F:
//			fallDet = FALSE;
//	}
	if(i == 15){
		i = 0;
	}
	IFG2 &= ~UCA0RXIFG;	// Clear RX Flag
	P1OUT ^= BIT6;	// Toggle P1.6 led on Rx

}

//#pragma vector = USCIAB0TX_VECTOR
//__interrupt void TransmitInterrupt(void){
//	P1OUT ^= BIT0;	// Toggle P1.6 led on Rx
//	__delay_cycles(1000);
//	IFG2 &= ~UCA0TXIFG;	// Clear TX Flag
//	P1OUT ^= BIT0;	// Toggle P1.6 led on Rx
//}

#pragma vector = PORT1_VECTOR
__interrupt void Port_1(void){

  P1IFG &= ~BIT3; //clear P1IFG for P1.3
  P1OUT ^= BIT0;  //toggle LED at P1.0
  sendUART("E", 2);

}

//------------------------------------------------------------------------------
