/*
 * omma_attiny1634.c
 *
 * Created: 8/12/2015 5:23:07 PM
 *  Author: foote
 */ 

#include <avr/io.h>

#define F_CPU 12000000 // 12 MHz

// addressing: top 10  a-bcd-efg-hij
//           bottom 9    klm-nop-qrs

#define ADDRESS 'e'


/*


a: .		k: --.
b: ..		l: --..
c: ...		m: --...
d: ....		n: --....
e: -		o: ---
f: -.		p: ---.
g: -..		q: ---..
h: -...		r: ---...
i: -....	s: ---....
j: --		(t: ----)
*/

/* FUSES
BODACT = BOD_DISABLED
SELFPRGEN = [ ]
RSTDISBL = [ ]
DWEN = [ ]
SPIEN = [X]
WDTON = [ ]
EESAVE = [ ]
BODLEVEL = 4V3
CKDIV8 = [ ]
CKOUT = [ ]
SUT_CKSEL = EXTXOSC_8MHZ_XX_16KCK_16CK_16MS

EXTENDED = 0xFF (valid)
HIGH = 0xDC (valid)
LOW = 0xEF (valid)
*/


// This is the address of the AVR on the RS-485 bus. Every one should be different!
// Address is one byte: a a-j for 20-unit sphere
uint8_t addr = ADDRESS;  //this is a char, not a number!
// number of times to blink, 1 for 'a', 2 for 'b', etc. 
uint8_t blink_address = ADDRESS - 'a' + 1;

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay.h>
#include "setup.c"
#include "1634_UART.h"

// ATtiny1634 Pin map
//
//              +-\/-+
//   TXD  PB0  1|o   |20  PB1 MOSI
//   RXD  PA7  2|    |19  PB2 MISO
//        PA6  3|    |18  PB3
//        PA5  4|    |17  PC0
//        PA4  5|    |16  PC1 SCK
//        PA3  6|    |15  PC2
//XMIT_EN PA2  7|    |14  PC3 RESET
//        PA1  8|    |13  PC4
//        PA0  9|    |12  PC5
//        GND 10|    |11  VCC


// indicator LED on PC0
#define LED_C0 PC0

// Turn off all IR leds. To avoid overcurrent, ONLY ONE IR SHOULD BE ON AT A TIME

// IRLED 1: LED2 PB3 (center)
#define IRLED_B3 PB3
// IRLED 2: PA0
#define IRLED_A0 PA0
// IRLED 3: PC2
#define IRLED_C2 PC2
// IRLED 4: PA1
#define IRLED_A1 PA1


/// ADC stuff. ADCs are
// IR sensor 0: PA3-ADC0
// IR sensor 1: PA4-ADC1
// IR sensor 2: PA5-ADC2
// IR sensor 3: PA6-ADC3


// define buffers for averaging. 
// max length, Actual length is dynamic
#define MAX_BUFFER 64

// length of on and off buffers
volatile uint8_t off_buff = 4;
volatile uint8_t on_buff = 4;

// raw ADC response when IR led is unlit
volatile uint8_t sense_off[4*MAX_BUFFER];
// index into off buffer
volatile uint8_t offptr = 0;

// raw ADC response when IR led is unlit
volatile uint8_t sense_on[4*MAX_BUFFER];
// index into off buffer
volatile uint8_t onptr = 0;

// when true, trigger IR sensing in ADC interrupt. 
volatile uint8_t autosense = 0; 

// Raw ADC response when IR led is lit
volatile uint8_t  lit = -1;
volatile uint8_t  adc_ptr = 0;


#define TXD0 PB0


#define MAX_COMMAND_LENGTH 24
unsigned char command_str[MAX_COMMAND_LENGTH];
unsigned char command_len = 0;


void put_hex(uint8_t number) {
	unsigned char c;
	c = ((number >> 4) & 0x0F);
	if (c > 9)
	    USART_send_byte(c -10 + 'A');
	else
	    USART_send_byte(c + '0');
	c = number & 0x0F;
	if (c > 9)
	   USART_send_byte(c - 10 + 'A');
	else
	   USART_send_byte(c + '0');
}


void init_ADC(void){
	adc_ptr = 0;

	ADMUX = 0;		//Select 5V internal reference and channel 0

	// not free running, use conversion complete interrupt, 128x clock division
	ADCSRA = (1<<ADEN) | (1<<ADIE) | (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0);
	ADCSRA = (1<<ADEN) | (1<<ADIE) | (1<<ADPS2) |  (1<<ADPS1);

	//ADLAR: right-shft so MSB values in ADCH (ignore ADCL)
	ADCSRB = (1<<ADLAR)  ;

	// Disable digital inputs on ADC0-ADC3
	DIDR0 = (1<<ADC3D) |(1<<ADC2D) |(1<<ADC1D) |(1<<ADC0D);

	ADCSRA |= (1<<ADSC); //init thing and start Conversion
}

void reset_buffers(){
	// clear all buffers
	uint8_t i;
	onptr = 0;
	offptr = 0;
	for (i=0; i<MAX_BUFFER; i++){
		sense_off[i] = 0;
		sense_on[i] = 0;
	}
}


// Setup the ATtiny
void setup(void) {
	
	// Set input/outputs
	DDRC |= (1<<LED_C0) | (1<<IRLED_C2);
	DDRB |= (1<<IRLED_B3) | (1<<TXD0);
	DDRA |=	(1<<IRLED_A0) | (1<<IRLED_A1) | (1<<XMIT_EN);

	// set initial values
	// All IRLEDs OFF, XMIT_EN LOW
	PORTC &= ~((1<<LED_C0) | (1<<IRLED_C2));
	PORTB &= ~(1<<IRLED_B3);
	PORTA &= ~((1<<IRLED_A0) | (1<<IRLED_A1) | (1<<XMIT_EN));

   // initialize serial port on USART0:
   	USART_init0(250000L);
	reset_buffers();

	// Init ADC
	init_ADC();
	
	sei(); // Turn on interrupts
}




/* serial protocol is ASCII hex bytes delimited by '<' and '>'
Address is first byte, for readability 0-9 and a-j
second byte is opcode:
x -- turn off all LEDs
y -- turn on red LED
0,1,2,3, -- turn on IR led 0,1,2,3

f -- return value of 4 off sensors
o -- return value of 4 sensors with LED on
q - read all 4 IR sensors, normalized by subtracting average of other 3 non-illuminated (returns quad ASCII byte)
r -- read all 4 IR sensors normalized by subtracting average of self non-illuminated
s -- set autosense
w -- write address (volatile)

third and following bytes are data if any

*/

void send_sensor_response_r(void){
	// Send the sensor response back to host over the serial line
	int i, j, temp;
	int sum_off = 0;
	int sum_on = 0;
	USART_send_byte('<');

	// send upper-case address in response so we don't address something else on the bus!
	//	put_hex(sense[0]);
	//	put_hex(sense[1]);
	//	put_hex(sense[2]);
	//	put_hex(sense[3]);
	// send upper-case address in response so we don't address something else on the bus!
	USART_send_byte(ADDRESS - 'a' + 'A');

	for (i=0; i<4; i++){
		sum_on = 0;
		sum_off = 0;
		for (j=0; j<off_buff; j++){
			sum_off += sense_off[i*off_buff + j];
		}
		for (j=0; j<on_buff; j++){
			sum_on += sense_on[i*on_buff + j];
		}
		temp = (sum_off/off_buff) - (sum_on/on_buff);
		if (temp < 0)
			temp = 0;
		if (temp > 255)
			temp = 255;
		put_hex((uint8_t)temp);
	}
	USART_send_byte('>');
	//USART_send_byte('\n');

}

void send_sensor_response_q(void){
	// Send the sensor response back to host over the serial line
	// in this version, subtract the off sensor for other channels from from the on sensor value for this channel
	int i, j, temp;
	//int sum_off[] = {0, 0, 0, 0};
	int sum_off = 0;
	int sum_on = 0;
	USART_send_byte('<');

	// send upper-case address in response so we don't address something else on the bus!
	USART_send_byte(ADDRESS - 'a' + 'A');

	sum_off = 0;
	for (i=0; i<4; i++){
		for (j=0; j<off_buff; j++){
			sum_off += sense_off[i*off_buff + j];
		}
	}	

	sum_off = sum_off / (4*off_buff);
	//sum_off = 255;
	
	for (i=0; i<4; i++){
		sum_on = 0;
		for (j=0; j<on_buff; j++){
			sum_on += sense_on[i*on_buff + j];
		}
		temp = sum_off - (sum_on/on_buff);
		if (temp < 0)
		 temp = 0;
		if (temp > 255)
		temp = 255;
		put_hex((uint8_t)temp);
	}
	USART_send_byte('>');
	//USART_send_byte('\n');

}

uint8_t a2int8(unsigned char c[]){
	// el cheapo 8-bit atoi, string
	return (((c[0] - '0')*10) + c[1] - '0');
}

void send_on_response(void){
	// Send the averaged on response from each sensor
	int i, j;
	int sum_on = 0;
	USART_send_byte('<');

	USART_send_byte(ADDRESS - 'a' + 'A');
//	USART_send_byte('o');

	for (i=0; i<4; i++){
		sum_on = 0;
		for (j=0; j<on_buff; j++){
			sum_on += sense_on[i*on_buff + j];
		}
		put_hex((uint8_t)(sum_on/on_buff));
	}
	USART_send_byte('>');
	//USART_send_byte('\n');

}
void send_off_response(void){
	// Send the averaged on response from each sensor
	int i, j;
	int sum_off = 0;

	USART_send_byte('<');
	USART_send_byte(ADDRESS - 'a' + 'A');

//	USART_send_byte('f');

	for (i=0; i<4; i++){
		sum_off = 0;
		for (j=0; j<on_buff; j++){
			sum_off += sense_off[i*off_buff + j];
		}
		put_hex((uint8_t)(sum_off/off_buff));
	}
	USART_send_byte('>');
}


void set_off_buffer_length(uint8_t new_off){
	// set the buffer length
	// first clear all the buffers
	if (new_off > MAX_BUFFER){
		new_off = MAX_BUFFER;
	}
	off_buff = new_off;
	reset_buffers();
}

void set_on_buffer_length(uint8_t new_on){
	// set the buffer length
	// first clear all the buffers
	if (new_on > MAX_BUFFER){
		new_on = MAX_BUFFER;
	}
	on_buff = new_on;
	reset_buffers();
}

void all_leds_off(void){
	// Turn off all IR leds. To avoid overcurrent, ONLY ONE IR SHOULD BE ON AT A TIME
	PORTC &= ~(1<<IRLED_C2);
	PORTB &= ~(1<<IRLED_B3);
	PORTA &= ~((1<<IRLED_A0) | (1<<IRLED_A1));
}

void parse_command(void){
	// if we are here there is a valid serial command in command_str
	// with length in command_len
	// zeroth char will be a '<'
	if (command_len >=3) {
		// first char is the address. If it's not ours, ignore
		if (command_str[1] != ADDRESS) 
			return;

		// Turn on LED
		PORTC &= ~(1<<LED_C0);
		switch(command_str[2]){

		case  'y':
			// turn on red led
			PORTC &= ~(1<<LED_C0);
			//USART_send_byte('x');
			break;

		case  'x':
			// turn off all LEDs
			PORTC |= (1<<LED_C0);
			all_leds_off();
			lit = 0;
			break;

		case  'q':
		send_sensor_response_q();
		break;

		case  'r':
		send_sensor_response_r();
		break;

		case 'O':
		// set off buffer length
		set_off_buffer_length(a2int8(command_str + 3));
		break;

		case 'F':
		// set off buffer length
		set_on_buffer_length(a2int8(command_str + 3));
		break;
		
		case 'b':
		// return buffer status for debugging
		put_hex(on_buff);
		put_hex(off_buff);
		break;

		case  's':
		// start conversion
		send_sensor_response_q();
		offptr = 0;
		onptr = 0;
		adc_ptr = 0;
		PORTC &= ~(1<<LED_C0);
		break;
		
		case  'c':
		// start conversion and send response from last conversion
		send_sensor_response_r();
		offptr = 0;
		onptr = 0;
		adc_ptr = 0;
		PORTC &= ~(1<<LED_C0);
		break;

		case  'o':
		send_on_response();
		break;

		case  'f':
		send_off_response();
		break;
		}

		// Turn off LED
		PORTC |= (1<<LED_C0);

	}
}




uint8_t accumulate_command_string(uint8_t c){ // ... and add to command string
// --------------------------------------------------------------------------
// process incoming chars - commands start with '<' and end with '>'
// return 1 if command string is complete - else return zero

	/* catch beginning of this string */
	if (c == '<') { // this will catch re-starts and stalls as well as valid commands.
		command_len = 0;
		command_str[command_len++] = c;
		return 0;
	}
	
	if (command_len != 0){	 // string in progress, accumulate next char
		
		if (command_len < MAX_COMMAND_LENGTH)
		command_str[command_len++] = c;
		return (c == '>');
	}
	return 0;
}


#ifdef FOO
// Quasi-free-running ADC, step through 4 ADC channels sequentially
ISR(ADC_READY_vect)
{

	if (lit == adc_ptr) // If there's an LED lit, store the value here
	sense_on[adc_ptr] = ADCH;	// Read ADC
	else // otherwise store the value here
	sense_off[adc_ptr] = ADCH;	// Read ADC


	if (++adc_ptr >= 4) adc_ptr = 0;
	// select next channel
	ADMUX = adc_ptr;
	ADCSRA |= (1<<ADSC);
	
	// for debug: turn on LED so we know we're here
	//PORTC &= ~(1<<LED);

}

// Quasi-free-running ADC, step through 4 ADC channels sequentially
ISR(ADC_READY_vect)
{

	if (lit == adc_ptr){ // If there's an LED lit, store the value here
		sense_on[adc_ptr*ON_BUFF + onptr++] = ADCH;	// Read ADC
		if (onptr >= ON_BUFF) onptr = 0;
	}
	else { // otherwise store the value here
		sense_off[adc_ptr*OFF_BUFF + offptr++] = ADCH; //Read ADC
		if (offptr >= OFF_BUFF) offptr = 0;
	}
	
	if (++adc_ptr >= 4) adc_ptr = 0;
	// select next channel
	ADMUX = adc_ptr;
	ADCSRA |= (1<<ADSC);
	
	// for debug: turn on LED so we know we're here
	//PORTC &= ~(1<<LED);

}

#endif

// Quasi-free-running ADC, step through 4 ADC channels sequentially
ISR(ADC_READY_vect)
{

	// adc pointer auto-increments, if it's less than 4 turn on LED
	if (adc_ptr < 4 ){ 
		sense_on[adc_ptr*on_buff + onptr++] = ADCH;	// Read ADC
		if (onptr >= on_buff){ 
			// keep sampling with IR on for ON_BUFF samples
			adc_ptr++;
			onptr = 0;
		}
	}
	else if (adc_ptr < 8) { // otherwise store the value here
		sense_off[(adc_ptr-4)*off_buff + offptr++] = ADCH; //Read ADC
		if (offptr >= off_buff){ 
			// keep sampling with IR off for OFF_BUFF samples
			adc_ptr++;
			offptr = 0;
		}
	}
	
	// uncomment to autorun, else trigger capture by adc_ptr =0
	// select next channel
	//if (adc_ptr >= 8){
	//	adc_ptr = 0;
	//}

	if(1){
		switch(adc_ptr){
			case 0:
			// turn on corresponding IR
			PORTB |= (1<<IRLED_B3);
			break;

			case  1:
			// turn on LED 2 -- East
			PORTA |= (1<<IRLED_A0);
			PORTB &= ~(1<<IRLED_B3);
			break;
			
			case  2:
			// turn on LED 3 -- West
			PORTC |= (1<<IRLED_C2);
			PORTA &= ~(1<<IRLED_A0);
			break;

			case  3: // turn on North (serial) LED
			PORTA |= (1<<IRLED_A1);
			PORTC &= ~(1<<IRLED_C2);
			break;

			default:
			PORTC &= ~(1<<IRLED_C2);
			PORTB &= ~(1<<IRLED_B3);
			PORTA &= ~((1<<IRLED_A0) | (1<<IRLED_A1));
			

		}
	}

	ADMUX = adc_ptr & 0x03;
	ADCSRA |= (1<<ADSC);

	// for debug: turn on LED so we know we're here
	//PORTC &= ~(1<<LED);
}


#define BLINK_MS 100

void wait_for_data(){
	// wait for data to show up in buffer. Blink address until it does
	uint8_t acount = blink_address;
	while(!USART_data_present()){
		acount = blink_address;		
		// long blinks for multiples of five, thus 16 = - - - .
		while (acount > 5) {
			PORTC &= ~(1<<LED_C0); // turn on
			_delay_ms(BLINK_MS);
			if (USART_data_present()) return;
			_delay_ms(BLINK_MS);
			if (USART_data_present()) return;
			_delay_ms(BLINK_MS);
			PORTC |= (1<<LED_C0); // turn off
			_delay_ms(BLINK_MS);
			if (USART_data_present()) return;
			acount -= 5;
			
		}
		while (acount > 0) {
			PORTC &= ~(1<<LED_C0); // turn on
			_delay_ms(BLINK_MS);
			if (USART_data_present()) return;
			PORTC |= (1<<LED_C0); // turn off
			_delay_ms(BLINK_MS);
			if (USART_data_present()) return;
			acount -= 1;
		}
		acount = 10;
		while (acount > 0) {
			PORTC |= (1<<LED_C0); // turn off
			_delay_ms(BLINK_MS);
			if (USART_data_present()) return;
			acount -= 1;
		}
	}	
	
}

/*****************************************************************************************************************************************/
int main(void) {
	setup();

	uint8_t cData;



	// turn off LED
	PORTC |= (1<<LED_C0);

	wait_for_data();
	while(1) {

        // Main parser loop starts here.
		if (USART_data_present()) { // check for waiting UART data from SPU
			cData = get_USART_buf_byte(); // get next char from ring buffer...
			if(0) USART_send_byte(cData);   /* echo char to serial out for debug does not work rs 485*/
			if(accumulate_command_string(cData)){ // ... and add to command string
				// if we are here we have a complete command; parse it
				parse_command(); // parse and execute commands
			}
			//_delay_ms(1);
		}

		if(0) {
				// blink LED for testing
		_delay_ms(250);
		PORTC &= ~(1<<LED_C0);
		_delay_ms(250);
		PORTC |= (1<<LED_C0);
	}
    
/*


		USART_Transmit0('A');
		//watchdog_sleep(T500MS);

		USART_Transmit0('b');

		_delay_ms(250);

*/	
	
	}
}

ISR(WDT_vect) { }
