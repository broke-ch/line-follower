#define F_CPU 16000000UL //16 MHz
#include <avr/io.h>
#include <avr/delay.h>
#include <avr/interrupt.h>
#include <stdlib.h>

#define TOLERANCE 100
#define BB_BASE 73
#define BB_SLIGHT 68
#define PD_BASE 45
#define KP 7.75
#define KD 1.45


uint16_t pot8;
uint16_t pot7;
uint16_t pot6;
uint16_t pot5;
uint16_t pot4;
uint16_t pot3;
uint16_t pot2;
uint16_t pot1;

int Base;
int Type;
int error;
int position = 0;
int *last_error = 0;

double Kp = KP;
double Kd = KD;

void pwm_init(){
	TCCR0A |= (1<<WGM00);
	TCCR0A |= (1<<WGM01);
	TCCR0B &= ~(1<<WGM02);
	
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
	ADMUX |= (1<<REFS1)|(1<<REFS0)|(1<<ADLAR); //5V ref, left adjusted

	// enable adc, clock prescaler of 128
	ADCSRA |= (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1);
	ADCSRB = 0;
	DDRC &= ~((1<<7)|(1<<6));
}

void sen_8() {
	ADMUX = 0b11100000;			// ADC0
	ADCSRB = 0b00000000;
	ADCSRA |= (1<<6);			// start conversion
	while(ADCSRA&(1<<ADSC)){}   // wait for conversion to complete
	pot8 = ADCH;
}

void sen_7() {
	ADMUX = 0b11100001;			// ADC1
	ADCSRB = 0b00000000;
	ADCSRA |= (1<<6);
	while(ADCSRA&(1<<ADSC)){}
	pot7 = ADCH;
}

void sen_6() {
	ADMUX = 0b11100100;			// ADC4
	ADCSRB = 0b00000000;
	ADCSRA |= (1<<6);
	while(ADCSRA&(1<<ADSC)){}
	pot6 = ADCH;
}

void sen_5() {
	ADMUX = 0b11100101; 		// ADC5
	ADCSRB = 0b00000000;
	ADCSRA |= (1<<6);
	while(ADCSRA&(1<<ADSC)){}
	pot5 = ADCH;
}

void sen_4() { 
	ADMUX = 0b11100001;			// ADC9
	ADCSRB = 0b00100000;
	ADCSRA |= (1<<6);
	while(ADCSRA&(1<<ADSC)){}
	pot4 = ADCH;
}

void sen_3() {
	ADMUX = 0b11100000;			// ADC8
	ADCSRB = 0b00100000;
	ADCSRA |= (1<<6);
	while(ADCSRA&(1<<ADSC)){}
	pot3 = ADCH;
}

void sen_2() {
	ADMUX = 0b11100011;			// ADC11
	ADCSRB = 0b00100000;
	ADCSRA |= (1<<6);
	while(ADCSRA&(1<<ADSC)){}
	pot2 = ADCH;
}

void sen_1() {
	ADMUX = 0b11100010;			// ADC10
	ADCSRB = 0b00100000;
	ADCSRA |= (1<<6);
	while(ADCSRA&(1<<ADSC)){}
	pot1 = ADCH;
}

void read_sensors(){
	sen_7();
	sen_6();
	sen_5();
	sen_4();
	sen_3();
	sen_2();
}

uint8_t LHS_W= 0;
uint8_t RHS_W = 0;

uint8_t threshold = 10; 	// tolerance
uint8_t seen = 0;

int lap = 0;

void marker_detect(){
	sen_1();
	sen_2();

	if(pot8 < TOLERANCE){ // IF SENSOR 8 SEES LINE INCREMENT
		 LHS_W++;
	}
	else if(LHS_W > 0){
		LHS_W--;
	}

	if(pot1 < TOLERANCE){ // IF SENSOR 8 SEES LINE INCREMENT
		 RHS_W++;
	}
	else if(RHS_W > 0){
		RHS_W--;
	}
}

int RHS_marker = 0;
int LHS_marker = 0;

void lap_counter(){

	if(RHS_W > threshold){
		RHS_marker++;
	}

	if(RHS_marker  > 6){
		setMotorSpeeds(0, 0);
		PORTB &= ~((1<<6)|(1<<5)|(1<<2)|(1<<1));
	}

	if((RHS_marker % 2) == 0){
		_delay_ms(2000);
	}
}

void setMotorSpeeds(double motorA, double motorB) {
	OCR0A = 255 * motorA/100; 
	OCR0B = 255 * motorB/100;
}

int current_position(){
	if(pot2 < TOLERANCE && pot3 < TOLERANCE && pot4 < TOLERANCE
	&& pot5 < TOLERANCE && pot6 < TOLERANCE && pot7 < TOLERANCE)
		{position = 0;} // Straight line case
	if(pot4 < TOLERANCE && pot5 < TOLERANCE)
		{position = 0;}// Straight line case

	if(pot2 < TOLERANCE && pot3 >= TOLERANCE && pot4 >= TOLERANCE && pot5 >= TOLERANCE && pot6 >= TOLERANCE && pot7 >= TOLERANCE)
		{position =-5;}
	if(pot2 < TOLERANCE && pot3 <  TOLERANCE && pot4 >= TOLERANCE && pot5 >= TOLERANCE && pot6 >= TOLERANCE && pot7 >= TOLERANCE)
		{position =-4;}
	if(pot3 < TOLERANCE && pot2 >= TOLERANCE && pot4 >= TOLERANCE && pot5 >= TOLERANCE && pot6 >= TOLERANCE && pot7 >= TOLERANCE)
		{position =-3;}
	if(pot3 < TOLERANCE && pot4 <  TOLERANCE && pot2 >= TOLERANCE && pot5 >= TOLERANCE && pot6 >= TOLERANCE && pot7 >= TOLERANCE)
		{position =-2;}
	if(pot4 < TOLERANCE && pot2 >= TOLERANCE && pot3 >= TOLERANCE && pot5 >= TOLERANCE && pot6 >= TOLERANCE && pot7 >= TOLERANCE)
		{position =-1;}
	if(pot5 < TOLERANCE && pot2 >= TOLERANCE && pot3 >= TOLERANCE && pot4 >= TOLERANCE && pot6 >= TOLERANCE && pot7 >= TOLERANCE)
		{position = 1;}
	if(pot5 < TOLERANCE && pot6 <  TOLERANCE && pot2 >= TOLERANCE && pot3 >= TOLERANCE && pot4 >= TOLERANCE && pot7 >= TOLERANCE)
		{position = 2;}
	if(pot6 < TOLERANCE && pot2 >= TOLERANCE && pot3 >= TOLERANCE && pot4 >= TOLERANCE && pot5 >= TOLERANCE && pot7 >= TOLERANCE)
		{position = 3;}
	if(pot6 < TOLERANCE && pot7 <  TOLERANCE && pot2 >= TOLERANCE && pot3 >= TOLERANCE && pot4 >= TOLERANCE && pot5 >= TOLERANCE)
		{position = 4;}
	if(pot7 < TOLERANCE && pot2 >= TOLERANCE && pot3 >= TOLERANCE && pot4 >= TOLERANCE && pot5 >= TOLERANCE && pot6 >= TOLERANCE)
		{position = 5;}

  return position;
}

void control(double kp, double kd, int *last_error, int base, int type){
  int current_pos = current_position();
	if(type == 1){
		if(pot4 < TOLERANCE && pot5 < TOLERANCE) { 	// if 4 & 5 see line, go straight
			setMotorSpeeds(BB_BASE, BB_BASE);
		}		
		else if(pot4 >= TOLERANCE && pot5 < TOLERANCE){	// if 5 sees the line and 4 does not SLIGHT LEFT
			setMotorSpeeds(BB_BASE, BB_SLIGHT);
		}
		else if(pot4 < TOLERANCE && pot5 >= TOLERANCE){	// if 4 sees line & 5 does not SLIGHT RIGHT
			setMotorSpeeds(BB_SLIGHT, BB_BASE);
		}
	}
	else{
		if(
			pot7 > TOLERANCE
			&& pot6 > TOLERANCE 
			&& pot5 > TOLERANCE
			&& pot4 > TOLERANCE 
			&& pot3 > TOLERANCE
			&& pot2 > TOLERANCE 
			){
//			error = *last_error;
			setMotorSpeeds(0, 0);
			delay(1000);
		}

		else{
			error = 0 - current_pos;
		}
		int derivative = error - *last_error;
		int control = (kp * error) + (kd * derivative);
		setMotorSpeeds(base + control, base - control);
		*last_error = error;
	}
}

void led_init(){
	DDRB |= (1<<1)|(1<<2)|(1<<6)|(1<<5);
}

int main(){

	led_init();
	adc_init();
	pwm_init();

	while(1) {
		marker_detect();
		lap_counter();

		read_sensors();

		if(position == -1 || position == 0 || position == 1){
			Type = 1;
			// TURN ON ALL LEDS
			PORTB |= (1<<6)|(1<<5)|(1<<2)|(1<<1);
		}
		else{
			Type = 0;
			Base = PD_BASE;
			Kp = KP;
			// TURN OFF ALL LEDS
			PORTB &= ~((1<<6)|(1<<5)|(1<<2)|(1<<1));
		}
		control(Kp, Kd, *last_error, Base, Type);
	}
}
