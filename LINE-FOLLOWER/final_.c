#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define RS 0
#define RW 1
#define EN 2
#define lcd_port PORTC
#define sbit(reg,bit)    reg |= (1<<bit)
#define cbit(reg,bit)    reg &= ~(1<<bit)


void motion_pin_config (void)
{
 DDRB = DDRB | 0x0F;   //set direction of the PORTB3 to PORTB0 pins as output
 PORTB = PORTB & 0xF0; // set initial value of the PORTB3 to PORTB0 pins to logic 0
 DDRD = DDRD | 0x30;   //Setting PD4 and PD5 pins as output for PWM generation
 PORTD = PORTD | 0x30; //PD4 and PD5 pins are for velocity control using PWM
}

unsigned char min(int a, int b)
{
	if( a> b)
	{
		return (unsigned char)b;
	}

	return (unsigned char)a;
}

int abs(int a)
{
	if( a > 0)
		return a;
	else 
		return -a;
}

//Function to initialize ports
void port_init()
{
 motion_pin_config();
 adc_pin_config();
 lcd_port_config();
}

//Function used for setting motor's direction
void motion_set (unsigned char Direction)
{
 unsigned char PortBRestore = 0;

 Direction &= 0x0F; 			// removing upper nibbel as it is not needed
 PortBRestore = PORTB; 			// reading the PORTB's original status
 PortBRestore &= 0xF0; 			// setting lower direction nibbel to 0
 PortBRestore |= Direction; 	// adding lower nibbel for direction command and restoring the PORTB status
 PORTB = PortBRestore; 			// setting the command to the port
}

void forward (void)         //both wheels forward
{
  motion_set(0x06);
}

void back (void)            //both wheels backward
{
  motion_set(0x09);
}

void left (void)            //Left wheel backward, Right wheel forward
{
  motion_set(0x05);
}

void right (void)           //Left wheel forward, Right wheel backward
{   
  motion_set(0x0A);
}

void soft_left (void)       //Left wheel stationary, Right wheel forward
{
 motion_set(0x04);
}

void soft_right (void)      //Left wheel forward, Right wheel is stationary
{ 
 motion_set(0x02);
}

void soft_left_2 (void)     //Left wheel backward, right wheel stationary
{
 motion_set(0x01);
}

void soft_right_2 (void)    //Left wheel stationary, Right wheel backward
{
 motion_set(0x08);
}

void hard_stop (void)       //hard stop(stop suddenly)
{
  motion_set(0x00);
}

void soft_stop (void)       //soft stop(stops solowly)
{
  motion_set(0x0F);
}

void init_ports();
void lcd_set_4bit();
void lcd_init();
void lcd_wr_command(unsigned char);
void lcd_wr_char(char);
void lcd_home();
void lcd_cursor(char, char);
void lcd_print(char, char, unsigned int, int);
void lcd_string(char*);

void init_devices (void)
{
 cli(); //Clears the global interrupts
 port_init();
 adc_init(); 
 lcd_init();
 lcd_set_4bit();
 
// timer1_init();
 sei(); //Enables the global interrupts
}
void lcd_port_config (void) 
{ 
 DDRC = DDRC | 0xF7; //all the LCD pin's direction set as output 
 PORTC = PORTC & 0x80; // all the LCD pins are set to logic 0 except PORTC 7 
} 

void lcd_set_4bit() 
{ 
 _delay_ms(1); 
 
 cbit(lcd_port,RS); //RS=0 --- Command Input 
 cbit(lcd_port,RW); //RW=0 --- Writing to LCD 
 lcd_port = 0x30; //Sending 3 in the upper nibble 
 sbit(lcd_port,EN); //Set Enable Pin 
 _delay_ms(5); //delay 
 cbit(lcd_port,EN); //Clear Enable Pin 
_delay_ms(1); 
 
 cbit(lcd_port,RS); //RS=0 --- Command Input 
 cbit(lcd_port,RW); //RW=0 --- Writing to LCD 
 lcd_port = 0x30; //Sending 3 in the upper nibble 
 sbit(lcd_port,EN); //Set Enable Pin 
 _delay_ms(5); //delay 
 cbit(lcd_port,EN); //Clear Enable Pin 
 
 _delay_ms(1); 
 
 cbit(lcd_port,RS); //RS=0 --- Command Input 
 cbit(lcd_port,RW); //RW=0 --- Writing to LCD 
 lcd_port = 0x30; //Sending 3 in the upper nibble 
 sbit(lcd_port,EN); //Set Enable Pin 
 _delay_ms(5); //delay 
 cbit(lcd_port,EN); //Clear Enable Pin 
 
 _delay_ms(1); 
 
 cbit(lcd_port,RS); //RS=0 --- Command Input 
 cbit(lcd_port,RW); //RW=0 --- Writing to LCD 
 lcd_port = 0x20; //Sending 2 in the upper nibble to initialize LCD 4-bit mode 
 sbit(lcd_port,EN); //Set Enable Pin 
 _delay_ms(5); //delay 
 cbit(lcd_port,EN); //Clear Enable Pin 
} 

//Function to Initialize LCD 
void lcd_init() 
{ 
 _delay_ms(1); 
 lcd_wr_command(0x28); //4-bit mode and 5x8 dot character font 
 lcd_wr_command(0x01); //Clear LCD display 
 lcd_wr_command(0x06); //Auto increment cursor position 
 lcd_wr_command(0x0E); //Turn on LCD and cursor 
 lcd_wr_command(0x80); //Set cursor position 
}

//Function to write command on LCD 
void lcd_wr_command(unsigned char cmd) 
{ 
 unsigned char temp; 
 temp = cmd; 
 temp = temp & 0xF0; 
 lcd_port &= 0x0F; 
 lcd_port |= temp; 
 cbit(lcd_port,RS); 
 cbit(lcd_port,RW); 
 sbit(lcd_port,EN); 
 _delay_ms(5);
cbit(lcd_port,EN); 
 
 cmd = cmd & 0x0F; 
 cmd = cmd<<4; 
 lcd_port &= 0x0F; 
 lcd_port |= cmd; 
 cbit(lcd_port,RS); 
 cbit(lcd_port,RW); 
 sbit(lcd_port,EN); 
 _delay_ms(5); 
 cbit(lcd_port,EN); 
}

//Function to write data on LCD 
void lcd_wr_char(char letter) 
{ 
 char temp; 
 
 temp = letter; 
 temp = (temp & 0xF0); 
 lcd_port &= 0x0F; 
 lcd_port |= temp; 
 sbit(lcd_port,RS); 
 cbit(lcd_port,RW); 
 sbit(lcd_port,EN); 
 _delay_ms(5); 
cbit(lcd_port,EN); 
 
 letter = letter & 0x0F; 
 letter = letter<<4; 
 lcd_port &= 0x0F; 
 lcd_port |= letter; 
 sbit(lcd_port,RS); 
 cbit(lcd_port,RW); 
 sbit(lcd_port,EN); 
 _delay_ms(5); 
 cbit(lcd_port,EN); 
} 

void lcd_home() 
{ 
 lcd_wr_command(0x80); 
}

void lcd_string(char *str) 
{ 
 while(*str != '\0') 
 { 
 lcd_wr_char(*str); 
 str++; 
 } 
} 

//Position the LCD cursor at "row", "column" 
void lcd_cursor (char row, char column) 
{ 
 switch (row) { 
 case 1: lcd_wr_command (0x80 + column - 1); break; 
 case 2: lcd_wr_command (0xc0 + column - 1); break; 
 case 3: lcd_wr_command (0x94 + column - 1); break; 
 case 4: lcd_wr_command (0xd4 + column - 1); break; 
 default: break; 
 } 
} 
unsigned int temp;
unsigned int unit;
unsigned int tens;
unsigned int hundred;
unsigned int thousand;
unsigned int million;
// Function to print any input value up to the desired digit on LCD 
void lcd_print (char row, char coloumn, unsigned int value, int digits) 
{ 
 unsigned char flag=0; 
 if(row==0||coloumn==0) 
 {
lcd_home(); 
 } 
 else 
 { 
 lcd_cursor(row,coloumn); 
 } 
 if(digits==5 || flag==1) 
 {
million=value/10000+48; 
 lcd_wr_char(million); 
 flag=1; 
 } 
 if(digits==4 || flag==1) 
 { 
 temp = value/1000; 
 thousand = temp%10 + 48; 
 lcd_wr_char(thousand); 
 flag=1; 
 } 
 if(digits==3 || flag==1) 
 { 
 temp = value/100; 
 hundred = temp%10 + 48; 
 lcd_wr_char(hundred); 
 flag=1; 
 }
if(digits==2 || flag==1) 
 { 
 temp = value/10; 
 tens = temp%10 + 48; 
 lcd_wr_char(tens); 
 flag=1; 
 } 
 if(digits==1 || flag==1) 
 { 
 unit = value%10 + 48; 
 lcd_wr_char(unit); 
 } 
 if(digits>5) 
 { 
 lcd_wr_char('E'); 
 } 
} 

//ADC pin configuration 
void adc_pin_config (void) 
{

 DDRA = 0x00; //set PORTA direction as input 
 PORTA = 0x00; //set PORTA pins floating 
}

//Function to Initialize ADC 
void adc_init() 
{ 
 ADCSRA = 0x00; 
 ADMUX = 0x20; //Vref=5V external --- ADLAR=1 --- MUX4:0 = 0000 
 ACSR = 0x80; 
 ADCSRA = 0x86; //ADEN=1 --- ADIE=1 --- ADPS2:0 = 1 1 0 
} 


//This Function accepts the Channel Number and returns the corresponding Analog Value 
unsigned char ADC_Conversion(unsigned char Ch) 
{ 
 unsigned char a; 
 Ch = Ch & 0x07; 
 ADMUX= 0x20| Ch; 
 ADCSRA = ADCSRA | 0x40; //Set start conversion bit 
 while((ADCSRA&0x10)==0); //Wait for ADC conversion to complete 
 a=ADCH; 
 ADCSRA = ADCSRA|0x10; //clear ADIF (ADC Interrupt Flag) by writing 1 to it 
// ADCSRB = 0x00; 
 return a; 
} 

void print_sensor(char row, char coloumn,unsigned char channel)
{
 int ADC_Value;
 ADC_Value = ADC_Conversion(channel);
 lcd_print(row, coloumn, ADC_Value, 3);
}

void velocity (unsigned char left_motor, unsigned char right_motor)
{
 OCR1AH = 0x00;
 OCR1AL = min(left_motor,255); 
 OCR1BH = 0x00;
 OCR1BL = min(right_motor,255);
}

//Main Function
int main()
{
	init_devices();f

	lcd_set_4bit();
	lcd_init();

	unsigned char lch=0;
	unsigned char rch=0;
	unsigned char cch=0;
	int ler=0;
	int rer=0;
	float thd=10;
	int per=0;
	int thdh=80;
	int thdl=20;
	float kp=10;
	int kd=50;
	int rl=0,old_rl=0;

	unsigned char old_ler=0;
	unsigned char old_rer=0;
	unsigned char old_per=100;	

	while(1){
		
		lch=ADC_Conversion(3);
		rch=ADC_Conversion(5);
		cch=ADC_Conversion(4);

		ler=lch-cch;
		rer=rch-cch;
		per=lch-rch;


		/*if(lch>120 || cch>120 || rch>120)
		forward();
		else if(lch>100 && cch<90)
		soft_left();
		else if(rch>100 && cch<50)
		soft_right();
		else
		back(); */
		
		print_sensor(1,1,3);		//Prints value of White Line Sensor Left
		print_sensor(1,5,4);		//Prints value of White Line Sensor Center
		print_sensor(1,9,5);		//Prints value of White Line Sensor Right
		

		rl=rch-lch;

		unsigned int lch_disp = lch;		
		unsigned int rch_disp = rch;
		unsigned int cch_disp = cch;		

	/*	lcd_print(1,1,lch_disp,3);
		lcd_print(1,6,rch_disp,3);
		lcd_print(2,1,cch_disp,3);	
		
	*/	if(rl>0)
		thd=120 - (kp*rl+kd*(rl-old_rl));
		else
		thd= -120 - (kp*rl+kd*(rl-old_rl));
			
		
		//lcd_print (2,1,abs(rl),3);
		//lcd_print (2,5,(unsigned char)abs(thd),3);
		  



	/*	if(lch>60 && rch>60)
		forward();
		else if(per>thdh)
		left();
		else if(per>thd)
		soft_left();
		else if(per<-thdh)
		right();
		else if(per<-thd)
		soft_right();
		else 
		forward();

	*/ //Godmaxx traction

	/*	if(old_per < thdl && old_per > -thdl)
		forward();
		else if(lch>60 && rch>60)
		forward();
		else if(per > -thd && per < thd)
		forward();
		else if(per > thd && per < thdh)
		soft_left();
		else if(per > thdh)
		left();
		else if(per < -thd && per > -thdh)
		soft_right();
		else if(per < -thdh)
		right();
		else 
		forward();

		old_per=per;

*/		//nearly final code

	/*	if(ler>10)	
		left();
		else if(rer>10)
		right();
		else forward();

		old_ler=ler;
		old_rer=rer;
*/
	/*	if(ler<thd && (-1*rer)>thd)
		{
			right();
		}
	 	if(rer<thdr && (-1*ler)>thdr)
		{
			left();
		}
		else 
		{
			forward();
		}*/
		
		//_delay_ms(1);
		}
}
