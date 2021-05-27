#define F_CPU 16000000UL //16 MHz
#include <avr/io.h>
#include <avr/delay.h>
#include <avr/interrupt.h>
#include <stdlib.h>

#define TOLERANCE 110


uint16_t pot8;
uint16_t pot7;
uint16_t pot6;
uint16_t pot5;
uint16_t pot4;
uint16_t pot3;
uint16_t pot2;
uint16_t pot1;


void adc_init(){
	ADMUX |= (1<<REFS1)|(1<<REFS0)|(1<<ADLAR); //2.56V ref, left adjusted

	// enable adc, clock prescaler of 128
	ADCSRA |= (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);
	ADCSRB = 0;
}

void sen_8() {
	ADMUX = 0b11100000;
	ADCSRB = 0b00100000;		// enable adc 8 -> 100000
	ADCSRA |= (1<<6);			// start conversion
	while(ADCSRA&(1<<ADSC)){} 	// wait for conversion to complete
	pot8 = ADCH;
}

void sen_7() {
	ADMUX = 0b11100001; // sensor 7 -> adc9
	ADCSRB = 0b00100000;
	ADCSRA |= (1<<6);
	while(ADCSRA&(1<<ADSC)){}
	pot7 = ADCH;
}

void sen_6() {
	ADMUX = 0b11100010; // sensor 6 -> adc10
	ADCSRB = 0b00100000;
	ADCSRA |= (1<<6);
	while(ADCSRA&(1<<ADSC)){}
	pot6 = ADCH;
}

void sen_5() {
	ADMUX = 0b11100011; // sensor 5 -> adc11
	ADCSRB = 0b00100000;
	ADCSRA |= (1<<6);
	while(ADCSRA&(1<<ADSC)){}
	pot5 = ADCH;
}

void sen_4() { 
	ADMUX = 0b11100111; // sensor 4 -> adc7
	ADCSRB = 0b00000000;
	ADCSRA |= (1<<6);
	while(ADCSRA&(1<<ADSC)){}
	pot4 = ADCH;
}

void sen_3() {  // sensor 3 -> adc6
	ADMUX = 0b11100110;
	ADCSRB = 0b00000000;
	ADCSRA |= (1<<6);
	while(ADCSRA&(1<<ADSC)){}
	pot3 = ADCH;
}

void sen_2() { // sensor 2 -> adc5
	ADMUX = 0b11100101;
	ADCSRB = 0b00000000;
	ADCSRA |= (1<<6);
	while(ADCSRA&(1<<ADSC)){}
	pot2 = ADCH;
}

void sen_1() {  // sensor 1 -> adc4
	ADMUX = 0b11100100;
	ADCSRB = 0b00000000;
	ADCSRA |= (1<<6);
	while(ADCSRA&(1<<ADSC)){}
	pot1 = ADCH;
}



int main(){

	adc_init();

	DDRE |= (1<<6);
	DDRB |= (1<<7)|(1<<6)|(1<<5)|(1<<2)|(1<<1)|(1<<0);
	DDRD |= (1<<0);

	while(1) {

		sen_8(); 
		sen_7(); 
		sen_6(); 
		sen_5(); 
		sen_4(); 
		sen_3(); 
		sen_2(); 
		sen_1(); 


		if(pot1 < TOLERANCE)
			{PORTE |= (1<<6);}
 		else if(pot2 < TOLERANCE)
			{PORTB |= (1<<0);}
		else if(pot3 < TOLERANCE)
			{PORTB |= (1<<1);}
		else if(pot4 < TOLERANCE)
			{PORTB |= (1<<2);}
		else if(pot5 < TOLERANCE)
			{PORTB |= (1<<7);}
		else if(pot6 < TOLERANCE)
			{PORTD |= (1<<0);}
		else if(pot7 < TOLERANCE)
			{PORTB |= (1<<6);}
		else if(pot8 < TOLERANCE)
			{PORTB |= (1<<5);}
		else{
			PORTE &= ~(1<<6);
			PORTB &= ~((1<<7)|(1<<6)|(1<<5)|(1<<2)|(1<<1)|(1<<0));
			PORTD &= ~(1<<0);
		}
	}
}
