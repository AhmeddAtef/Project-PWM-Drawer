/*
 * Final PWM Project.c
 *
 * Created: 20/02/2022 01:51:48 م
 * Author : Ahmed Atef Tawfik Mohamed
 */
#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
                                       //LCD//
#define LCD_DATA PORTB
#define	en PORTE1					   
#define rw PORTE2
#define rs PORTE3

void init_LCD(void);
void LCD_cmd(unsigned char cmd);
void LCD_write(unsigned char data);
void Cursor_pos(unsigned char x_pos, unsigned char y_pos);
                                       //ISR//
ISR(PCINT3_vect)
{
	while ((PIND & 0b10000000))
	{
		PIND |= (1 << PIND0);
		_delay_ms(200);
	}
}						
                                       //ADC//
void init_ADC(void)
{
	ADCSRA |= ((1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0));
	ADMUX  |= (1<<REFS0);								   			   
    ADCSRA |= (1<<ADEN);
	ADCSRA |= (1<<ADSC);
}

                                    //ADC read//
uint16_t ADC_read(uint16_t channel)
{
	ADMUX &= 0xF0;
	ADMUX |= channel;
	ADCSRA |= (1<<ADSC);
	while(ADCSRA & (1<<ADSC));
	return ADC;
}
                                   //PWM init//
void PWM_Init(void)
{
	TCCR1A |= (1 << COM1B1) | (1 << WGM11) | (1 << WGM10);  // This is the fast width modulation mode
	TCCR1B |= (1 << CS10) | (1 << WGM12); // CS10 = 1 it means no prescaler N = 1
	// WGM12 = 1 it means clear timer on compare match (CTC) mode is enabled
	
}	

							   
int main(void)								   
{
	
	DDRB=0xFF;
	DDRE=0xFF;
	
	
	DDRD = 0b00111111;
	PORTD = 0xFF;
	
	PCMSK3 |= (1 << PCINT9);
	PCICR  |= (1 << PCIE3);
	
	sei();
	
	PWM_Init();
	init_ADC();
	init_LCD();
	
	OCR1A = 800;
	OCR1B = 0;
	
	_delay_ms(100);
	
	LCD_cmd(0x0C);
	_delay_ms(100);
	
	LCD_cmd(0x01);
 
 // --------------------------------------------------------------//
 
    LCD_write('A');
	_delay_ms(1);
    LCD_write('D');
    _delay_ms(1);
    LCD_write('C');
    _delay_ms(0);
    LCD_write('0');
    _delay_ms(1);
	LCD_write(' ');
	_delay_ms(1);
	LCD_write('=');
	
	LCD_cmd(0xC0);
	_delay_ms(1);
	LCD_cmd(0x0C);
	_delay_ms(1);
	
	LCD_write(' ');
	_delay_ms(1);
	LCD_write(' ');
	_delay_ms(1);
	LCD_write('H');
	_delay_ms(1);
	LCD_write('a');
	_delay_ms(1);
	LCD_write('s');
	_delay_ms(1);
	LCD_write('h');
	_delay_ms(1);
	LCD_write('i');
	_delay_ms(1);
	LCD_write('=');
	
	_delay_ms(1);
	
	unsigned char ch[4] = {' '};
	uint16_t Data_final; // 3l4an final display
	
	
	while (1)
	{
		// writing data on the first row // 
		Data_final = ADC_read(0);
		OCR1B = ADC_read(0);
		// 3l4an aclear character array//
		
		for (int j = 0; j<4; j++)
		{
			ch[j] = ' ';
		}
		// writing the data onn the first row//
		
		Cursor_pos(0, 8);
		itoa(Data_final,ch, 10);
		for (int j = 0; j<4;j++)
		{
			if (ch[j] <'0' || ch[j] > '9')
			LCD_write(' ');
			else
			LCD_write(ch[j]);
		}
		_delay_ms(200);
	
	}
	
	
}
								   
                                   //LCD init//								   
void init_LCD(void)								   
{
	LCD_cmd(0x38);								   
	_delay_ms(1);							   
	LCD_cmd(0x01);							   
	_delay_ms(1);
	LCD_cmd(0x02);
	_delay_ms(1);
	LCD_cmd(0x06);
	_delay_ms(1);
	LCD_cmd(0x80);
	_delay_ms(1);
}
void LCD_cmd(unsigned char cmd)		
{
	LCD_DATA = cmd;
	PORTE &= ~(1<<rs);
	PORTE &= ~(1<<rw);
	PORTE |= (1<<en);
	_delay_ms(2);
	PORTE &= ~(1<<en);
	
}		

void LCD_write(unsigned char data)
{
	LCD_DATA = data;
	PORTE |= (1<<rs);
	PORTE &= ~(1<<rw);
	PORTE |= (1<<en);
	_delay_ms(2);
	PORTE &= ~(1<<en);
	
}

                     //Set LCD cursor position//
void Cursor_pos(unsigned char x_pos, unsigned char y_pos)
{
	uint16_t The_Address =0;
	if ( x_pos == 0 )
	The_Address = 0x80;
	else if ( x_pos == 1 )
	The_Address = 0xC0;
	if( y_pos < 16 )
	The_Address += y_pos;
	LCD_cmd (The_Address);
	
}






		
								   