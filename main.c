/*
 * main.c
 *
 *  Created on: 16.08.2018
 *      Author: Anton Nechaev
 *      Yet another automatic irrigation system
 * DDRxn=0 PORTxn=0 – HI-Z
 * DDRxn=0 PORTxn=1 – PullUp
 * DDRxn=1 PORTxn=0 – OutLow
 * DDRxn=1 PORTxn=1 – OutHi
 *
 */

#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <util/twi.h>
#include <stdlib.h>

#ifndef F_CPU
	#define F_PCU 16000000UL
#endif
/*
 * Set FUSE on external clock source from quartz 16 MHz
 */
#define DEBUG true //UART and/or i2c debug output
#ifdef DEBUG
	#include "i2cmaster.h" //Fleury i2c lib
	#include "lcd.h" //Fleury lcd lib
	extern void lcd_backlight(char on);    //not in lcd.h
	#include "uart.h"
#endif


#define SENSOR_PORT PORTC
#define SENSOR_REGISTER DDRC
#define SENSOR_PIN0 PC0
#define SENSOR_PIN1 PC1
#define ATX_PIN PC2
#define BUTTON_PORT PORTD
#define BUTTON_REGISTER DDRD
#define BUTTON_PIN PD2
#define MOTOR_REGISTER DDRB
#define MOTOR_PORT PORTB
#define HERATBEAT_PIN PB3
#define MOTOR_PIN PB5


volatile uint32_t timer1;
volatile struct {
	uint8_t forward;
	uint8_t reverse;
	uint8_t calculated;
}humidity;
volatile signed char inc = 1;//heartbeat increment
volatile const uint32_t half_day = 10676; // ~12 hours

void port_config(void)
{
	SENSOR_REGISTER |= (1<<ATX_PIN); //set ATX_PIN to out
	//SENSOR_PORT |= (1<<SENSOR_PIN0)|(1<<SENSOR_PIN1);
	BUTTON_PORT |= (1<<BUTTON_PIN); // enable PulUP
	EIMSK |= (1<<INT0); // Interrupt for INT0

	TCCR1B |= (1<<CS12)|(1<<CS10);//set clk/1024 prescaler, period 64us timer1
	TIMSK1 |= (1<<ICIE1)|(1<<TOIE1);// enable timer1 interrupt

	ADMUX |= (1<<ADLAR)
			|(1<<REFS0);// set left adjusted result 10-bit ADC // set ref as AVCC
	ADCSRA |= (1<<ADEN)|(1<<ADSC) // enable ADC
		|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0); // Prescaler /128

	MOTOR_REGISTER |=(1<<MOTOR_PIN)|(1<<HERATBEAT_PIN);// set ports as out
	// set Fast PWM
	TCCR2A |= (1<<COM2A1)|(1<<COM2A0)
			|(1<<WGM21)|(1<<WGM20);
	//COM2A[1..0] b11 - Set OC2A on Compare Match, clear OC2A at BOTTOM,    (inverting mode)
	//WGM2[2..0] b011 - mode 3 Fast PWM top - 0xFF
	TCCR2B |= (1<<CS22)|(1<<CS21);//|(1<<CS20);
	// b111 - /1024 prescaler
	TIMSK2 |= (1<<TOIE2);



//	Config TWI from gnu-avr example
	  /* initialize TWI clock: 100 kHz clock, TWPS = 0 => prescaler = 1 */
	#if defined(TWPS0)
	  /*has prescaler (mega128 & newer)*/
	  TWSR = 0;
	#endif
	TWBR = (F_CPU / 100000UL - 16) / 2;
	TWCR |= (1<<TWEN)|(1<<TWIE); //Enable TWI
}


uint8_t check_humidity(uint8_t direction) // Check humidity in both direction to prevent electric oxidation
{
	if (direction == 0)
	{
		SENSOR_PORT |= (1<<SENSOR_PIN1);
	} else {
		SENSOR_PORT |= (1<<SENSOR_PIN0);
		ADMUX |= (1<<MUX0); //connect PIN1 to ADC
	}
	ADCSRA &= ~(1<<ADIF); //Start ADC
	ADCSRA |= (1<<ADSC);
	while(ADCSRA & (1<<ADSC)); // wait ADC
	SENSOR_PORT &= ~((1<<SENSOR_PIN0)|(1<<SENSOR_PIN1));
	ADMUX &= ~(1<<MUX0);
	return ADCH; // left adjusted, need only highest bits
}

void irrigate(int ms_time) // enable water pump in ms
{
	MOTOR_PORT |= (1<<MOTOR_PIN);
	int i;
	for (i = 0; i < (ms_time / 10); i++) {
		_delay_ms(10);
	}
	MOTOR_PORT &= ~(1<<MOTOR_PIN);
}

ISR(TIMER1_OVF_vect)
{
	timer1++;
	#ifdef DEBUG
		printf("Timer1: %lu\n", timer1);
	#endif
}

ISR(TIMER2_OVF_vect)
{
	OCR2A = OCR2A + inc;
	if (OCR2A == 254 || OCR2A == 0)
	{
		inc = inc * -1;
	}
}

ISR(INT0_vect)
{
	timer1 = half_day - 1;
	#ifdef DEBUG
		printf("Button pressed!\n");
		printf("Timer1: %lu\n", timer1);
	#endif

}

int main(void)
{
	port_config();
	uart_config();
    stdout = &uart_output;
    stdin  = &uart_input;
	sei();

	#ifdef DEBUG
		printf("Auto irrigation system is started\n");

		// Init LCD1602
		lcd_init(LCD_ON_DISPLAY);
		lcd_backlight(1);
		_delay_ms(500);
		lcd_backlight(0);
		_delay_ms(500);
		lcd_backlight(1);
		_delay_ms(500);

		lcd_clrscr();
		lcd_gotoxy(0, 0);
		lcd_puts_P("Auto irrigation");
		lcd_gotoxy(0, 1);
		lcd_puts("Hm:");
		_delay_ms(1000);
	#endif

	char hmdty_string[16];
	while(1)
	{
		if (timer1 >= half_day)
		{
			timer1 = 0;
			humidity.forward = check_humidity(0);
			_delay_ms(100); //0.1 sec delay for discharge
			humidity.reverse = check_humidity(1); //checking with reversed polarity for prevent electrodes degradation
			humidity.calculated = (humidity.forward + humidity.reverse) / 2;
			#ifdef DEBUG
				itoa(humidity.forward, hmdty_string, 10);
				lcd_gotoxy(4, 1);
				lcd_puts(hmdty_string);
				itoa(humidity.reverse, hmdty_string, 10);
				lcd_gotoxy(8, 1);
				lcd_puts(hmdty_string);
				itoa(humidity.calculated, hmdty_string, 10);
				lcd_gotoxy(13, 1);
				lcd_puts(hmdty_string);
				printf("Humidity:%d %d %d\n", humidity.forward, humidity.reverse, humidity.calculated);
			#endif
			if (humidity.calculated < 128)
			{
				#ifdef DEBUG
					printf("Start ATX(1 sec)\n");
				#endif
				SENSOR_PORT |= ~(1<<ATX_PIN);
				_delay_ms(1000); //delay for 1 second during ATX start
				#ifdef DEBUG
					printf("irrigation 10 sec\n");
				#endif
				irrigate(10000);
				timer1 = half_day - 1;
			} else {
				SENSOR_PORT &= ~(1<<ATX_PIN);
			}
		}
	}
}
