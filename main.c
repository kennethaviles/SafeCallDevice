#include <msp430.h>
#include <stdio.h>
#include <time.h>

#include  "msp430g2553.h"
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
#define T 0x54
#define F 0x46
#define E 0x45
#define D 0x44

//------------------------------------------------------------------------------
//	Flags
//------------------------------------------------------------------------------

unsigned int fallDet; // Fall detected - True = 1, False = 0

//------------------------------------------------------------------------------
//	Variables
//------------------------------------------------------------------------------

unsigned char letter = 'E';
unsigned char *string;
int i=0;

char receivedData[20];

unsigned int ADC_value=0;
int fall=0;
int alarm=0;
int change=0;
int BON=0;
int sec=0, sw1=0, sw2=0;
int count=0;


//------------------------------------------------------------------------------
// Declarations
//------------------------------------------------------------------------------
void initUART();
void configureUART(unsigned int br0, unsigned int brs);
void configurePorts();
void sendUART(unsigned char *str, unsigned int len);
void setupBT();
void delay(int milliseconds);

void ConfigureAdc(void);
void BuzzerON(void);
void BuzzerOFF(void);


//------------------------------------------------------------------------------
// main()
//------------------------------------------------------------------------------
int main(void){

	WDTCTL = WDTPW + WDTHOLD + WDTNMI;		// Stop WDT and deactivate reset

    //----------------------------- Configure the clock-----------------------------------------

    if(CALBC1_1MHZ==0xFF){					//If calibration constant erased
    	while(1);							//trap cpu
    }
    // Basic Clock Module (set to 1MHz)
    DCOCTL = 0;
    BCSCTL1 = CALBC1_1MHZ;
    DCOCTL = CALDCO_1MHZ;

    //------------------------------ Call methods to configure ports, uart and adc---------------

    configurePorts();

    initUART();

	ConfigureAdc();							// ADC set-up function call

//    _BIS_SR(LPM0_bits + GIE);         	// Going to LPM0

	__enable_interrupt();					// Enable interrupts.

	__delay_cycles(1000);					// Wait for ADC Ref to settle

	//----------------------------- Start of ADC Conversions and fall detection-------------------

	ADC10CTL0 |= ENC + ADC10SC;
	while(1)
	{
		__delay_cycles(1000);
		//			__bis_SR_register(CPUOFF + GIE);	// Low Power Mode 0 with interrupts enabled
		ADC_value = ADC10MEM;				// Assigns the value held in ADC10MEM to the integer called ADC_value
		if(ADC_value<750) 					//Check if sw2 has a fall
		{
			if(fall<200){					//Delay to detect fall
				fall++;
			}
			else{							//If fall is detected sw2=1
				sw2=1;
				fall=0;						//Reset variable fall
			}

		}
		else{
			//BuzzerOFF();
		}
		if(sw1==1 && sw2==2){
			alarm=1;						//If both triggered alarm is set
		}
		if(BON==1 | alarm==1){				//Start Alarm if Button pressed or if fall is detected
			BuzzerON();
		}
		else{								//If no alarm is triggered turn off buzzer
			BuzzerOFF();
			ADC10CTL0 |= ENC + ADC10SC;		// Sampling and conversion start
		}
	}


}
//------------------------------------------------------------------------------
//								Methods
//------------------------------------------------------------------------------
void configurePorts(){

	P1DIR |= BIT0 + BIT6;					//P1.0 and P1.6 as output
	P1OUT &= ~BIT0 + BIT6;					//P1.0 and P1.6 = 0

//	P1DIR |= BIT1 + BIT2;					//P1.1 and P1.2 as output
//	P1OUT &= ~BIT1 + ~BIT2;					//P1.1 and P1.2 = 0

	P1DIR &= ~BIT3 + ~BIT4; 				//P1.3 i/p
	P1REN |=  BIT3 + BIT4; 					//P1.3 enable pullup resistor
	P1IES |=  BIT3 + BIT4; 					//P1.3 high to low transition
	P1IFG &= ~BIT3 + ~BIT4; 				//P1.3 clear interrupt flag
	P1IE  |=  BIT3 + BIT4; 					//enable P1.3 interrupt

	//Set the UART function for P1.1 and P1.2
	P1SEL |= BIT1 + BIT2;
	P1SEL2 |= BIT1 + BIT2;

	P1DIR |= BIT5;
	P2DIR |= BIT1+BIT0;           			// P2.1 to output
	P2SEL |= BIT1;           				// P2.1 to TA0.1
	P1IE |= 0x09;			 				// Interupt in P1.0 & P1.3
	P1IES |= 0x01;		     				// Hi/lo in P1.0 & Lo/Hi P1.3
	P1IFG &= 0x09;			 				// Clear Interupt Flags
	P1OUT=BIT5;
	P2OUT=BIT0;
	P1SEL |= BIT4;							// ADC input pin P1.4
}

void initUART(){
	//For a baudrate 115200
    configureUART(8, 0x0C);

//    string  = "$$$";

    //Change to the command mode
//    sendUART(string,5);

    //Change the baudrate in the Bluetooth to 9600
//    sendUART("U,9600,N\n",11);

    //For a baudrate 9600
//	configureUART(104, 0x02);

//    sendUART("GK\n",6);
}
void configureUART(unsigned int br0, unsigned int brs){

	UCA0CTL1 |= UCSSEL_2 + UCSWRST;			// Set USCI clock to SMCLK and put state machine in reset
	UCA0BR0 = br0;
	UCA0BR1 = 0;							// Baudrate = 1MHz / (256 * UCA0BR1 + UCA0BR0)
	UCA0MCTL |= brs;
	UCA0CTL1 &= ~UCSWRST;					// Initialize USCI state machine
	IE2 |= UCA0RXIE; 						// Enable USCI_A0 RX Interrupt
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

void delay(int milliseconds){

    long pause;
    clock_t now,then;

    pause = milliseconds*(CLOCKS_PER_SEC/1000);
    now = then = clock();
    while( (now-then) < pause )
        now = clock();
}

void BuzzerON(void){

	if(change <100){
		TA1CCR0 = 170;             			// PWM Period
		TA1CCTL1 = OUTMOD_7;         		// CCR1 reset/set
		TA1CCR1 = 70;                		// CCR1 PWM duty cycle
		TA1CTL = TASSEL_2 + MC_1 + ID_1;    // SMCLK, up mode
		change++;

	}
	else{
		TA1CCR0 = 120;             			// PWM Period
		TA1CCTL1 = OUTMOD_7;          		// CCR1 reset/set
		TA1CCR1 = 55;						// CCR1 PWM duty cycle
		change++;
		TA1CTL = TASSEL_2 + MC_1 + ID_1;    // SMCLK, up mode
		if (change == 200){
			change = 0;
		}
	}

}
void BuzzerOFF(void){

	TA1CCR0 = 0;             				// PWM Period
	TA1CCTL1 = OUTMOD_7;          			// CCR1 reset/set
	TA1CCR1 = 0;							// CCR1 PWM duty cycle
	TA1CTL = TASSEL_2 + MC_1 + ID_1;        // SMCLK, up mode

}

//------------------------------------------------------------------------------
// 							Interrupts
//------------------------------------------------------------------------------
#pragma vector = USCIAB0RX_VECTOR
__interrupt void ReceiveInterrupt(void){

//	P1OUT ^= BIT6;	// Toggle P1.6 led on Rx
//	receivedData[i] = UCA0RXBUF;
//	i++;
	//printf("cmd: %d\n", cmd);

	switch(UCA0RXBUF){
		case E:{
			fallDet = TRUE;
			P1OUT |= BIT6;					// Toggle P1.6 led on Rx
		}
		break;
		case D:{
			fallDet = FALSE;
			P1OUT &= ~BIT6;					// Toggle P1.6 led on Rx
		}
		break;
	}
//	if(i == 15){
//		i = 0;
//	}
	IFG2 &= ~UCA0RXIFG;						// Clear RX Flag

}

#pragma vector = PORT1_VECTOR
__interrupt void Port_1(void){

	P1OUT ^= BIT0;  						//toggle LED at P1.0
	if(P1IFG & BIT3){
	  sendUART("E", 2);
	  P1IFG &= ~BIT3; 					//clear P1IFG for P1.3
	}
	else if(P1IFG & BIT4){
	  sendUART("F", 2);
	  P1IFG &= ~BIT4; 					//clear P1IFG for P1.3

	}

}

// ADC10 interrupt service routine
#pragma vector=ADC10_VECTOR
__interrupt void ADC10_ISR (void){

}

// Function containing ADC set-up
void ConfigureAdc(void){

	ADC10CTL1 = INCH_4 + ADC10DIV_3 ;         				// Channel 4, ADC10CLK/3
	ADC10CTL0 = SREF_0 + ADC10SHT_3 + ADC10ON + ADC10IE;  	// Vcc & Vss as reference, Sample and hold for 64 Clock cycles, ADC on, ADC interrupt enable
	ADC10AE0 |= BIT4;                        				// ADC input enable P1.4
}


#pragma vector=PORT1_VECTOR //ISR
__interrupt void Port_1 (void){
	sec=P1IN&BIT3;

	delay_cycles(250000);
	if(BON==0){								//If button pressed when buzzer was Off, turn On Buzzer
		BON=1;

	}

	else if (P1IN&BIT0){
		_delay_cycles(25000);				//Read button to see if pressed to deactivate buzzer
		BuzzerOFF();
		sec=0;
		BON=0;
		alarm=0;
		sw1=0;
		sw2=0;

	}
	else if (P1IN&BIT0){

		_delay_cycles(25000);
		BuzzerOFF();
		sec=0;
		BON=0;
		alarm=0;
		sw1=0;
		sw2=0;
	}
	else{
		if(P1IN&BIT3){						//If sw1 activates an interrupt
			if(count<10){
				count++;
			}
			else{							//If sw1 detects fall
				sw1=1;
				count=0;
			}
		}
	}
	P1IFG &= ~0x09;
}

//------------------------------------------------------------------------------
