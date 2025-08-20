/*
 * project_stopwatch.c
 *  Created on: Sep 14, 2024
 *      Author: Tweety
 */

#include <avr/io.h>
#include<avr/interrupt.h>
#include<util/delay.h>
/* Initialization */
unsigned char count_down_mode = 0;
/* Initialization using help increment and decrement from flag_PB0-> flag_PB7 */

unsigned char flag_PB0=0;
unsigned char flag_PB1=0;
unsigned char flag_PB3=0;
unsigned char flag_PB4=0;
unsigned char flag_PB5=0;
unsigned char flag_PB6=0;
unsigned char flag_PB7=0;


/* variable second  and mintues and hours*/

unsigned char secound = 0 ;
unsigned char mintues = 0 ;
unsigned char hours = 0 ;

           /*Function decleration*/
void INT0_Init(void);
void INT1_Init(void);
void INT2_Init(void);
void configurations_pins(void);
void timer1_CTC_init(void);
void SevenSegment_Display(void);
void I_bits(void);
void inrement_decrement(void);
void count_down(void);

int main ()
{
	/* Enable Global Interrupts*/
	I_bits();

	/*Function call*/
	timer1_CTC_init();
	INT0_Init();
	INT1_Init();
	INT2_Init();
	configurations_pins();





	while(1)
	{


		SevenSegment_Display();
		inrement_decrement();
		count_down();
	}
}


void INT0_Init(void)
{
	MCUCR|=(1<<ISC01);
	GICR|=(1<<INT0);

	DDRD &=~(1<<PD2) ;//BUTTON IS INPUT
	PORTD |=(1<<PD2) ; // Enable internal pull up resistor


}
/* External INT1 enable and configuration function */
	void INT1_Init(void)
	{
		// Trigger INT1 with the falling edge
		DDRD &=~(1<<PD3) ;
		MCUCR |= (1<<ISC11);
		MCUCR &= ~(1<<ISC10);
		GICR  |= (1<<INT1);    // Enable external interrupt pin INT1
	}
	void INT2_Init(void)
	{
		DDRB &=~(1<<PB2) ;
	 	PORTB |=(1<<PB2) ;
		MCUCSR &=~ (1<<ISC2);     // Trigger INT2 with the falling edge
		GICR   |= (1<<INT2);	 // Enable external interrupt pin INT2



	}


	void configurations_pins(void)
	{


		DDRC |=0X0F ;/* Define PC0,PC1,PC2,PC3 as output pins for decoder*/
		PORTC &=0XF0 ;	/* Initially Display 0 on seven segment*/

		DDRA |=0X3F ;  /* Define PA0:PA5 as output pins for Enable control Lines*/
		PORTA &=0XC0 ;/* Enable the 6 seven segments*/

		DDRB &=0X84 ;
		PORTB|= 0X7B ;
	                   /* internal pull up*/
		DDRB &=~ (1 << PB7);         // Configure pin 0 in PORTB as input pin
		PORTB |= (1 << PB7);         // Activate the internal pull up resistor at PB0

		 DDRD |= (1 << PD4);         // Configure pin 4 in PORTB as output pin
		 DDRD|=(1<<PD5);             // Configure pin 5 in PORTB as output pin

		 PORTD &= ~(1 << PD4);  // Set pin  in PORTD with value 1 at the beginning(LED OFF)
		 PORTD &= ~(1 << PD5);  // Set pin 5 in PORTD with value 1 at the beginning(LED OFF)

		 PORTD&=~(1<<0);			//Turn Off Buzzer at the beginning
		 DDRD |=(1<<0);
	}

	void timer1_CTC_init(void)
	{

		/* Configure timer control register TCCR1A
			 * 1. Disconnect OC1A and OC1B  COM1A1=0 COM1A0=0 COM1B0=0 COM1B1=0
			 * 2. FOC1A=1 FOC1B=0
			 * 3. CTC Mode WGM10=0 WGM11=0 (Mode Number 4)
			 */
		TCCR1A=(1<<FOC1A);

		/* Configure timer control register TCCR1B
			 * 1. CTC Mode WGM12=1 WGM13=0 (Mode Number 4)
			 * 2. Prescaler = F_CPU/1024 CS10=1 CS11=0 CS12=1
			 */
		TCCR1B=(1<<WGM12)|(1<<CS10)|(1<<CS12);

		TCNT1 = 0;		/* Set timer1 initial count to zero */


		/* Description:
		 * For System Clock = 16Mhz and prescaler F_CPU/1024.
		 * Timer frequency will be around 16Khz, Ttimer = 62.5us
		 * For compare value equals to 250 the timer will generate compare match interrupt every 15.625ms.
		 * Compare interrupt will be generated every 15625
		 */
		OCR1A  = 15625; //compare value

		TIMSK |= (1<<OCIE1A); /* Enable Timer1 Compare A Interrupt */



	}
	void SevenSegment_Display(void)
	{
		    PORTA = (PORTA & 0XC0) | (1 <<PA5) ;
			PORTC = (PORTC & 0XF0) | (secound%10 & 0X0F) ;
			_delay_ms(3) ;																	/*  delay to show the number (multiplexing method) */
			PORTA = (PORTA & 0XC0) |(1<<PA4) ;
			PORTC = (PORTC & 0XF0) | (secound/10 & 0X0F) ;
			_delay_ms(3) ;																	/*  delay to show the number (multiplexing method) */
			PORTA = (PORTA & 0XC0) |(1<<PA3) ;
			PORTC = (PORTC & 0XF0) | (mintues%10 & 0X0F) ;
			_delay_ms(3) ;																	/*  delay to show the number (multiplexing method) */
			PORTA = (PORTA & 0XC0) |(1<<PA2) ;
			PORTC = (PORTC & 0XF0) | (mintues/10 & 0X0F) ;
			_delay_ms(3) ;																	/*  delay to show the number (multiplexing method) */
			PORTA = (PORTA & 0XC0) |(1<<PA1) ;
			PORTC = (PORTC & 0XF0) | (hours%10 & 0X0F) ;
			_delay_ms(3) ;																	/*  delay to show the number (multiplexing method) */
			PORTA = (PORTA & 0XC0) |( 1<<PA0) ;
			PORTC = (PORTC & 0XF0) | (hours/10 & 0X0F) ;
			_delay_ms(3);																	/*  delay to show the number (multiplexing method) */
	}
	void I_bits(void)
	{
		SREG |=(1<<7) ;
	}
	void inrement_decrement(void)
	{

	/*to decrement hours*/
	if (!(PINB & (1 << PB0)))
	{
		if (flag_PB0 == 0)
		{

			if (hours > 0)
			{
				hours--;
			}
		}

		flag_PB0 = 1;
	}

	else

	{
		flag_PB0 = 0;
	}

	/*to increment hours*/

	if (!(PINB & (1 << PB1)))
	{
		if (flag_PB1 == 0)
		{
			if (hours < 99)
			{
				hours++;
			}

			flag_PB1 = 1;
		}

	}

	else {
		flag_PB1 = 0;
	}

	/*to decrement mintues*/

	if (!(PINB & (1 << PB3)))
	{
		if (flag_PB3 == 0)
		{

			if (mintues > 0)
			{
				mintues--;
			}
		}

		flag_PB3 = 1;
	}

	else

	{
		flag_PB3 = 0;
	}

	/*to increment mintues*/

	if (!(PINB & (1 << PB4)))
	{

		if (flag_PB4 == 0)
		{

			if (mintues < 60)
			{
				mintues++;
			}
		}

		flag_PB4 = 1;
	}

	else

	{
		flag_PB4 = 0;
	}

	/*to decrement secound*/

	if (!(PINB & (1 << PB5)))
	{

		if (flag_PB5 == 0)
		{
			if (secound > 0)

			{
				secound--;
			}

			flag_PB5 = 1;
		}
	} else {
		flag_PB5 = 0;
	}

	/*to increment secound*/

	if (!(PINB & (1 << PB6)))
	{
		if (flag_PB6 == 0)
		{
			if (secound < 60)
			{
				secound++;
			}
		}
		flag_PB6 = 1;
	}

	else

	{
		flag_PB6 = 0;
	}

	}
	void count_down(void)
	{


		/*count_downmode*/

		    if (!(PINB & (1 << PB7)))
		    {
		        if (flag_PB7 == 0)
		        {
		            PORTD = PORTD | (1 << PD5);
		            PORTD = PORTD & ~(1 << PD4);
		            flag_PB7= 1;
		            count_down_mode=1 ;

		        }
		    }

		  else
		  {

		   flag_PB7= 0 ;
		   count_down_mode=0 ;
		  }
	}




 ISR(INT0_vect)
 {
	 secound = 0 ;
	 mintues = 0 ;
	 hours   = 0 ;


 }

ISR(INT1_vect)
	{
		 TCCR1B &= ~(1<<CS10) & ~(1<<CS11) & ~(1<<CS12) ;
	}


	  ISR(INT2_vect)
	  {
		  TCCR1B|= (1<<CS10) |(1<<CS12);
	  }


	  ISR(TIMER1_COMPA_vect)
	  {

	  	    PORTD&=~(1<<0);			          //Turn Off Buzzer
	  		if(count_down_mode==1)			/*If Count down  mode is chosen */
	  		{
	  			if(secound>0)
	  			{
	  				secound--;
	  			}
	  			else if(secound==0 && mintues>0)
	  			{
	  				mintues--;
	  				secound=60;
	  			}
	  			else if(secound==0 && mintues==0 && hours>0)
	  			{
	  				hours--;
	  				mintues=60;
	  			}
	  			else
	  			{
	  				PORTD|=(1<<0);  //Turn Off Buzzer
	  			}
	  		}
	  		else
	  		{

	  	PORTD = PORTD |(1 << PD4);
	  	PORTD = PORTD & ~(1 << PD5);
	  	secound++ ;
	  	if(secound==60)
	  	{
	  		secound = 0 ;
	  		mintues++ ;
	  	}

	  	if( mintues==60)
	  	{
	  	   mintues=0 ;
	  	   hours++ ;
	  	}

	  	 if(hours==99)
	  	 {
	  		    hours=0 ;
	  		  secound = 0 ;
	  		    mintues = 0 ;
	  	  }
	  		}
	  }







