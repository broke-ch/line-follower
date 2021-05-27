#define F_CPU 16000000UL //16 MHz
#include <avr/io.h>
#include <avr/delay.h>
#include <avr/interrupt.h>
#include <stdlib.h>

#define BASE 	100	
#define SLIGHT 	97
#define MID		90 	
#define TOLERANCE 125


uint16_t pot8;
uint16_t pot7;
uint16_t pot6;
uint16_t pot5;
uint16_t pot4;
uint16_t pot3;
uint16_t pot2;
uint16_t pot1;

void pwm_init(){
	TCCR0A |= (1<<WGM00);
	TCCR0A |= (1<<WGM01);
	TCCR0A &= ~(1<<WGM02);

	TCCR0B |= (1<<CS02);
	TCCR0B &= ~(1<<CS01);
	TCCR0B &= ~(1<<CS00);

	TCCR0A |= (1<<COM0B1);
	TCCR0A &= ~(1<<COM0B0);

	TCCR0A |= (1<<COM0A1);
	TCCR0A &= ~(1<<COM0A0);
	// motor A uses OCR0A
	// motor A uses OCR0B

	//Motor 2
	DDRB |= (1<<0);
	DDRB |= (1<<7);
	//Motor 1
	DDRE |= (1<<6);
	DDRD |= (1<<0);

	PORTB |= (1<<0); 
	PORTE |= (1<<6);
}

void adc_init(){
  ADMUX |= (1<<REFS1)|(1<<REFS0)|(1<<ADLAR); //2.56V ref, left adjusted

  // enable adc, clock prescaler of 128
  ADCSRA |= (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);
  ADCSRB = 0;
}

void sen_8() {
  ADMUX = 0b11100000;
  ADCSRB = 0b00100000;    // enable adc 8 -> 100000
  ADCSRA |= (1<<6);     // start conversion
  while(ADCSRA&(1<<ADSC)){}   // wait for conversion to complete
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
	pwm_init();

	while(1) {
		ADCSRA |= (1<<6);				// start conversion
		sen_8();
		sen_7();
		sen_6();
		sen_5();
		sen_4();
		sen_3();
		sen_2();
		sen_1();

		if (pot4 < TOLERANCE && pot5 < TOLERANCE) { 	// if 4 & 5 see line, go straight
			OCR0A = BASE;
			OCR0B = BASE;
		}		
		else if(pot4 >= TOLERANCE && pot5 < TOLERANCE){	// if 5 sees the line and 4 does not SLIGHT LEFT
			OCR0A = SLIGHT;
			OCR0B = BASE;
		}
		else if(pot4 < TOLERANCE && pot5 >= TOLERANCE){	// if 4 sees line & 5 does not SLIGHT RIGHT
			OCR0A = BASE;
			OCR0B = SLIGHT;
		}
		else if(pot6 < TOLERANCE && pot3 < TOLERANCE){	// if 3 & 6 see line go straight (intersection)
			OCR0A = BASE;
			OCR0B = BASE;
		}
		else if(pot3 < TOLERANCE && pot6 >= TOLERANCE){	// if 3 sees line & 6 does not, MID RIGHT
			OCR0A = BASE;
			OCR0B = 0;
		}
		else if(pot6 < TOLERANCE && pot3 >= TOLERANCE){	// if 6 sees line & 3 does not, MID LEFT
			OCR0A = 0;
			OCR0B = BASE;
		}
//		else if(pot7 < TOLERANCE && pot2 >= TOLERANCE){	// if 7 sees line & 2 does not, HARD LEFT
//			OCR0A = 0;
//			OCR0B = BASE;
//		}
//		else if(pot2 < TOLERANCE && pot7 >= TOLERANCE){	// if 2 sees line & 7 does not, HARD RIGHT
//			OCR0A = BASE;
//			OCR0B = 0;
//		}
//		else if(pot1 < TOLERANCE && pot8 >= TOLERANCE){	// if 1 sees line & 8 does not, HARD LEFT
//			OCR0A = 0;
//			OCR0B = BASE+10;
//		}
//		else if(pot1 < TOLERANCE && pot8 >= TOLERANCE){	// if 8 sees line & 1 does not, HARD RIGHT
//			OCR0A = BASE+10;
//			OCR0B = 0;
//		}
		else{ 
			OCR0A = 0;
			OCR0B = 0;
		}
		ADCSRA &= ~(1<<6);			// stop conversion
	}
}
