/* heavily modified from https://github.com/Ladvien/ATtiny1634_AVR_Code thanks! */

#ifndef	UART_1634
#define UART_1634

#include <avr/interrupt.h>  //Add the interrupt library; int. used for TX RX.



/* UART Buffer Defines */
#define UART_RX_BUFFER_SIZE 128     /* 2,4,8,16,32,64,128 or 256 bytes */
#define UART_TX_BUFFER_SIZE 256


#define UART_RX_BUFFER_MASK ( UART_RX_BUFFER_SIZE - 1 )
#if ( UART_RX_BUFFER_SIZE & UART_RX_BUFFER_MASK )
#error RX buffer size is not a power of 2
#endif

#define UART_TX_BUFFER_MASK ( UART_TX_BUFFER_SIZE - 1 )
#if ( UART_TX_BUFFER_SIZE & UART_TX_BUFFER_MASK )
#error TX buffer size is not a power of 2
#endif


/* Static Variables -- Tx & Rx Ring Buffers */
static unsigned char UART_RxBuf[UART_RX_BUFFER_SIZE];
static volatile unsigned char UART_RxHead;
static volatile unsigned char UART_RxTail;
static unsigned char UART_TxBuf[UART_TX_BUFFER_SIZE];
static volatile unsigned char UART_TxHead;
static volatile unsigned char UART_TxTail;


//Buffers for UART0 and UART1
//USART0
//char ReceivedData0[32];	//Character array for Rx data.
//int ReceivedDataIndex0;	//Character array index.
//int rxFlag0;			//Boolean flag to show character has be retrieved from RX.


// Function prototypes, old
//void USART_init0(long int BUADRATE);
//void USART_Transmit0( unsigned char data);
//void Serial_Print0(char *StringOfCharacters);
//EOT characters.
//void LF0();
//void CR0();


// function prototypes, new:
unsigned char get_USART_buf_byte( void );
unsigned char USART_data_present( void );
void USART_send_byte(unsigned char);

// XMIT-enable: PA2
#define XMIT_EN PA2

void DISABLE_TX() {PORTA &= ~(1<<XMIT_EN);}
void ENABLE_TX() {PORTA |= (1<<XMIT_EN);}

// Function to initialize UART0
void USART_init0(long int Desired_Baudrate)
{
	long int the_baud;
	//Only set baud rate once.  If baud is changed serial data is corrupted.
	#ifndef UBBR
		//Set the baud rate dynamically, based on current microcontroller
		//speed and the desired baud rate given by the user.
		#define UBBR ((F_CPU)/(Desired_Baudrate*16UL)-1)
		the_baud = ((F_CPU)/(Desired_Baudrate*16UL)-1);
	#endif
	
	//Set baud rate.
	//UBRR0H = (unsigned char)(UBBR>>8);
	//UBRR0L = (unsigned char)UBBR;

	UBRR0H = (unsigned char)(the_baud >>8);
	UBRR0L = (unsigned char)(the_baud );
	
	//UBRR0H = (unsigned char)0;
	//UBRR0L = (unsigned char)5;
	
	
	//Enables the RX interrupt and TX interrupt.
	//NOTE: The RX data buffer must be clear or this will continue
	//to generate interrupts. Pg 157.
	UCSR0B |= (1<<RXCIE0) | (1<<TXCIE0);
//	UCSR0B |= (1<<RXCIE0);
	
	//Enable receiver and transmitter
	UCSR0B |= (1<<RXEN0)|(1<<TXEN0);
	//UCSR0B |= (1<<RXEN0) | (1<<UDRIE0);

	
	//Set frame format: 8data, 1 stop bit
	UCSR0C |= (1<<UCSZ00)|(1<<UCSZ01);   // 8bit data format

	// init buffer pointers  */
	UART_RxTail = 0;
	UART_RxHead = 0;
	UART_TxTail = 0;
	UART_TxHead = 0;

	
	//Enables global interrupts.
	sei();
}



ISR(USART0_RX_vect){
	//RX0 interrupt

/*	
	//Show we have received a character.
	rxFlag0 = 1;
	
	//Load the character into the poor man's buffer.
	//The buffer works based on a end-of-transmission character (EOTC)
	//sent a the end of a string.  The buffer stops at 63 instead of 64
	//to always give room for this EOTC.  In our case, "."
	if (ReceivedDataIndex0 < 63){
		//Actually pull the character from the RX register.
		ReceivedData0[ReceivedDataIndex0] = UDR0;
		//Increment RX buffer index.
		ReceivedDataIndex0++;
	}
	else {
		//If the buffer is greater than 63, reset the buffer.
		ReceivedDataIndex0=0;
		clearBuffer0();
	}
*/

  unsigned char data;
  unsigned char tmphead;

  data = UDR0;                 /* Read the received data */
  /* Calculate buffer index */
  tmphead = ( UART_RxHead + 1 ) & UART_RX_BUFFER_MASK;
  UART_RxHead = tmphead;      /* Store new index */
  
  if ( tmphead == UART_RxTail )
  {
	  /* ERROR! Receive buffer overflow */
  }
  
  UART_RxBuf[tmphead] = data; /* Store received data in buffer */

}


unsigned char USART_data_present( void )
{
	return ( UART_RxHead != UART_RxTail ); /* Return 0 (FALSE) if the receive buffer is empty */
}


// -------------------------------------------------------------------------------
// Pull 1 byte from Ring Buffer of bytes received from USART
unsigned char get_USART_buf_byte( void )
{
	unsigned char tmptail;
	
	while ( UART_RxHead == UART_RxTail )  /* Wait for incoming data */
	;
	tmptail = ( UART_RxTail + 1 ) & UART_RX_BUFFER_MASK;/* Calculate buffer index */
	
	UART_RxTail = tmptail;	/* Store new index */
	return UART_RxBuf[tmptail]; /* Return data */
}

/*  transmit  ********************************************************************************************************/

void USART_Transmit0( unsigned char data )
{
	//We have to disable RX interrupts.  If we have
	//an interrupt firing at the same time we are
	//trying to transmit we'll lose some data.
	UCSR0B ^= ((1<<RXEN0)|(1<<RXCIE0));

	// enable TX complete interrupt

	//Wait for empty transmit buffer
	while ( !( UCSR0A & (1<<UDRE0)) );

	// Enable RS-485 transmit
	PORTA |= (1<<XMIT_EN);
	//Put data into buffer, sends the data
	UDR0 = data;
	

	//Re-enable RX interrupts.
	UCSR0B ^= ((1<<RXEN0)|(1<<RXCIE0));

}

/// new transmit stuff here
void USART_send_byte( unsigned char data )
{
	unsigned char tmphead;
	/* Calculate buffer index */
	tmphead = ( UART_TxHead + 1 ) & UART_TX_BUFFER_MASK;

	/* Wait for free space in buffer */
	while ( tmphead == UART_TxTail );

	UART_TxBuf[tmphead] = data; /* Store data in buffer */
	UART_TxHead = tmphead;	    /* Store new index */

	UCSR0B |= (1<<UDRIE0);	/* Enable UDRE interrupt */
}

ISR(USART0_UDRE_vect) 
{
	unsigned char tmptail;

	//We have to disable RX interrupts.  If we have
	//an interrupt firing at the same time we are
	//trying to transmit we'll lose some data.
	UCSR0B ^= ((1<<RXEN0)|(1<<RXCIE0));

	
	/* Check if all data is transmitted */
	if ( UART_TxHead != UART_TxTail ) {
		/* Calculate buffer index */
		tmptail = ( UART_TxTail + 1 ) & UART_TX_BUFFER_MASK;
		UART_TxTail = tmptail;      /* Store new index */

		// Enable RS-485 transmit
		PORTA |= (1<<XMIT_EN);
		
		UDR0 = UART_TxBuf[tmptail];  /* Start transmission */
	}
	else {
		UCSR0B &= ~(1<<UDRIE0);   /* Disable UDRE interrupt or we'll re-trigger on exit */
	}

	//Re-enable RX interrupts.
	UCSR0B ^= ((1<<RXEN0)|(1<<RXCIE0));


}



// new stuff for rs-485
ISR(USART0_TX_vect) {

// Transmission complete, turn off transmit enable
PORTA &= ~(1<<XMIT_EN);	

}



#endif
