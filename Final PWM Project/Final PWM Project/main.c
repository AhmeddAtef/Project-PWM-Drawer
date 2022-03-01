/*
 * Final PWM Project.c
 *
 * Created: 20/02/2022 01:51:48 م
 * Author : Ahmed Atef Tawfik Mohamed
 */
//
// include for libraries//
#define F_CPU 16000000UL    //define cpu 16 MHz
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>   // for interrupt purpose

                                       //LCD//
									   
#define LCD_DATA PORTB                //port B is selected as LCD data port
#define	en PORTE1		                 // enable signal is connected to port E pin 1			   
#define rw PORTE2                         // read  w  write signal is connected to port E pin 2
#define rs PORTE3                           // register select signal  connected to port E pin 3

void init_LCD(void);
void LCD_cmd(unsigned char cmd);
void LCD_write(unsigned char data);
void Cursor_pos(unsigned char x_pos, unsigned char y_pos);            // x_pos:0~1 , y:0~15 cursor position in rows and columns
                                       //ISR//
ISR(PCINT3_vect)
{
	while ((PIND & 0b10000000))
	{
		PIND |= (1 << PIND0);            // 3l4an lw galy interrupt signal ON pin number 0 fe port D w OFF every 200 milisecond
		_delay_ms(200);                   // delay every 100 milisecond 
	}
}						
                                      //ADC for initialization//
void init_ADC(void)
{
	ADCSRA |= ((1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0));
	ADMUX  |= (1<<REFS0);		                    // go to registers to enable some pins						   			   
    ADCSRA |= (1<<ADEN);
	// do an initial conversion because this one is the slowest and to ensure that everything is up and running
	ADCSRA |= (1<<ADSC);
}

                                    //ADC for reading al channel// 
uint16_t ADC_read(uint16_t channel)
{
	ADMUX &= 0xF0;                    // to clear the older channel that was read
	ADMUX |= channel;                   //  3l4an ydefine the new ADC channel to be read
	ADCSRA |= (1<<ADSC);                 // to start a new conversion ( signal conversion )
	while(ADCSRA & (1<<ADSC));            // wait until the conversion is done
	return ADC;                            // returns the ADC value of the chossen channel
}
                                   //PWM init//
void PWM_Init(void)
{
	TCCR1A |= (1 << COM1B1) | (1 << WGM11) | (1 << WGM10);  // This is the fast width modulation mode
	TCCR1B |= (1 << CS10) | (1 << WGM12); // CS10 = 1 it means no prescaler N = 1
	// WGM12 = 1 it means clear timer on compare match (CTC) mode is enabled
	// 3l4an a7ded anhy pin 3awz ast5dmha ka pulse width modulation
	
}	

							   
int main(void)								   
{
	
	DDRB=0xFF;                      // set LCD data port as output
	DDRE=0xFF;                      // set LCD signals ( RS , RW , E ) as output
	
	
	DDRD = 0b00111111;              // set port D as input
	PORTD = 0xFF;                   // enable pull up
	// PORTD , PIND7
	PCMSK3 |= (1 << PCINT9);
	PCICR  |= (1 << PCIE3);
	
	sei();                // initialize for global interrupt
	
	PWM_Init();          // initialize PWM
	init_ADC();          // init       ADC
	init_LCD();          // init       LCD
	
	// Freq = F_CPU / ( 2 * N * ( 1 * OCR1A )
	// OCR1A = ( F_CPU / ( 2 * N * Freq ) - 1
	
	OCR1A = 800;          // ( pwm frequancy ) to control the freq 3la asas al equation aly fo2
	OCR1B = 0;           // al resolution bta3t al duty cycle 10 bit pwm  3l4an ycontrol al duty cycle 0 to 1023
	
	_delay_ms(100);
	
	LCD_cmd(0x0C);
	_delay_ms(100);
	
	LCD_cmd(0x01);          // Clear LCD display screen
 
 // --------------------------------------------------------------// // ADC channel 0 //
 
    LCD_write('A');          // call a function to display "A" on LCD
	_delay_ms(1);
    LCD_write('D');         // call a function to display "D" on LCD
    _delay_ms(1);
    LCD_write('C');          //   //   //        //      "C"    //
    _delay_ms(0);
    LCD_write('0');          //   //    //     //    //  "0"  //
    _delay_ms(1);
	LCD_write(' ');          //        //     //      //   " "  //
	_delay_ms(1);
	LCD_write('=');            //     //       //     //   "="   /
	
	LCD_cmd(0xC0);             // move cursor to the start of 2nd line
	_delay_ms(1);
	LCD_cmd(0x0C);             // display one , cursor off
	_delay_ms(1);
	
	LCD_write(' ');              // call a function to display " " on LCD
	_delay_ms(1);
	LCD_write(' ');              // call a function to display " " on LCD
	_delay_ms(1);
	LCD_write('A');             // call a function to display "A" on LCD
	_delay_ms(1);
	LCD_write('h');             // call a function to display "h" on LCD
	_delay_ms(1);
	LCD_write('m');             // call a function to display "m" on LCD
	_delay_ms(1);
	LCD_write('e');              // call a function to display "e" on LCD
	_delay_ms(1);
	LCD_write('e');               // call a function to display "e" on LCD
	_delay_ms(1);
	LCD_write('d');                // call a function to display "d" on LCD
	
	_delay_ms(1);
	
	unsigned char ch[4] = {' '};
	uint16_t Data_final; // 3l4an final display
	
	
	while (1)
	{
		// writing data on the first row // 
		Data_final = ADC_read(0);        // store the ADC value into a variable
		OCR1B = ADC_read(0);              // write ADC to the PWM channel
		// 3l4an aclear character array//
		
		for (int j = 0; j<4; j++)
		{
			ch[j] = ' ';
		}
		// writing the data onn the first row//
		
		Cursor_pos(0, 8);                     // Start from row first row 8 column
		itoa(Data_final,ch, 10);              // integer to ASCII     -     10 is for decimal ,  replace by 16 for hexadecimal
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
	LCD_cmd(0x38);			          // init of 16 by 2 LCD in 8 bit  mode ( 16x2 )					   
	_delay_ms(1);							   
	LCD_cmd(0x01);			           // For clearing LCD display screen				   
	_delay_ms(1);
	LCD_cmd(0x02);                     // return home
	_delay_ms(1);
	LCD_cmd(0x06);                     // make increment in cursor
	_delay_ms(1);
	LCD_cmd(0x80);                     // go to the first line and 8th position   ( l2en al 3ad fe lcd bybd2 mn al 0 wana 3ndy 16 bit mn 0 l 15 )
	_delay_ms(1);
}
// al function de 3l4an ab3t al command//

// sending command on LCD ( En = enable , Rs = Reset , Rw = write //
void LCD_cmd(unsigned char cmd)		
{
	LCD_DATA = cmd;                 // Data lines are set to send command - PORTB = LCD_DATA
	PORTE &= ~(1<<rs);                  // Rs sets 0
	PORTE &= ~(1<<rw);                  // Rw sets 0
	PORTE |= (1<<en);                   // make enable high
	_delay_ms(2);
	PORTE &= ~(1<<en);                 // make enable from high to low
	
}		

    // write data on LCD //
void LCD_write(unsigned char data)
{
	LCD_DATA = data;
	PORTE |= (1<<rs);                // Rs sets 1
	PORTE &= ~(1<<rw);               // Rw sets = 0
	PORTE |= (1<<en);                 // make enable high
	_delay_ms(2);
	PORTE &= ~(1<<en);                // make enable from high to low
	
}

                     //Set LCD cursor position//
void Cursor_pos(unsigned char x_pos, unsigned char y_pos)        // x_pos:0~1 , y:0~15 cursor position in rows and columns
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






		
								   