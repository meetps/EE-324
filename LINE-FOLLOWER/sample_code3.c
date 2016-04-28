#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "lcd.c"
	
#define		THRESHOLD		160       // set the pots such that all three sensor 
                                      // calibrated to show its min value on LCD. 
                                      // i.e on LCD Sensor values are betwn 168 to 172
									  // on black line  
#define		VELOCITY_MAX	50
#define		VELOCITY_MIN	30
#define 	VELOCITY_LOW	0

unsigned char ADC_Conversion(unsigned char);
unsigned char ADC_Value;
unsigned char Left_white_line = 0;
unsigned char Center_white_line = 0;
unsigned char Right_white_line = 0;

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

//Function to configure LCD port
void lcd_port_config (void)
{
 DDRC = DDRC | 0xF7;    //all the LCD pin's direction set as output
 PORTC = PORTC & 0x80;  // all the LCD pins are set to logic 0 except PORTC 7
}

//ADC pin configuration
void adc_pin_config (void)
{
 DDRA = 0x00;   //set PORTF direction as input
 PORTA = 0x00;  //set PORTF pins floating
}

//Function to configure ports to enable robot's motion
void motion_pin_config (void)
{
 DDRB = DDRB | 0x0F;    //set direction of the PORTB3 to PORTB0 pins as output
 PORTB = PORTB & 0xF0;  //set initial value of the PORTB3 to PORTB0 pins to logic 0
 DDRD = DDRD | 0x30;    //Setting PD5 and PD4 pins as output for PWM generation
 PORTD = PORTD | 0x30;  //PD5 and PD4 pins are for velocity control using PWM
}

//Function to Initialize PORTS
void port_init()
{
 lcd_port_config();
 adc_pin_config();		
 motion_pin_config();
}

//TIMER1 initialize - prescale:64
// WGM: 5) PWM 8bit fast, TOP=0x00FF
// desired value: 450Hz
// actual value: 450.000Hz (0.0%)
void timer1_init(void)
{
 TCCR1B = 0x00; //stop
 TCNT1H = 0xFF; //setup
 TCNT1L = 0x01;
 OCR1AH = 0x00;
 OCR1AL = 0xFF;
 OCR1BH = 0x00;
 OCR1BL = 0xFF;
 ICR1H  = 0x00;
 ICR1L  = 0xFF;
 TCCR1A = 0xA1;
 TCCR1B = 0x0D; //start Timer
}


//Function to Initialize ADC
void adc_init()
{
 ADCSRA = 0x00;
 ADMUX = 0x20;		//Vref=5V external --- ADLAR=1 --- MUX4:0 = 0000
 ACSR = 0x80;
 ADCSRA = 0x86;		//ADEN=1 --- ADIE=1 --- ADPS2:0 = 1 1 0
}

//This Function accepts the Channel Number and returns the corresponding Analog Value 
unsigned char ADC_Conversion(unsigned char Ch)
{
 unsigned char a;
 Ch = Ch & 0x07;  			
 ADMUX= 0x20| Ch;	   		
 ADCSRA = ADCSRA | 0x40;	//Set start conversion bit
 while((ADCSRA&0x10)==0);	//Wait for ADC conversion to complete
 a=ADCH;
 ADCSRA = ADCSRA|0x10;      //clear ADIF (ADC Interrupt Flag) by writing 1 to it
 return a;
}

// This Function prints the Analog Value Of Corresponding Channel No. at required Row
// and Coloumn Location. 
void print_sensor(char row, char coloumn,unsigned char channel)
{
 ADC_Value = ADC_Conversion(channel);
 lcd_print(row, coloumn, ADC_Value, 3);
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


void stop (void) //hard stop
{
  motion_set(0x00);
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

//Function for velocity control
void myVelocity (unsigned char left_motor, unsigned char right_motor)
{
 OCR1AH = 0x00;
 OCR1AL = left_motor; 
 OCR1BH = 0x00;
 OCR1BL = right_motor;
}


//Function for velocity control
void velocity (unsigned char left_motor, unsigned char right_motor)
{
 OCR1AH = 0x00;
 OCR1AL = min(left_motor,255); 
 OCR1BH = 0x00;
 OCR1BL = min(right_motor,255);
}

void init_devices (void)
{
 cli();          //Clears the global interrupts
 port_init();
 timer1_init();
 adc_init();
 sei();          //Enables the global interrupts
}

//Main Function
int main(void)
{
 unsigned char flag ;

 init_devices();

 lcd_set_4bit();
 lcd_init();

 velocity(VELOCITY_MAX,VELOCITY_MAX);    // Set the speed to max velocity
 forward();                              // start to move froward

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

	int PWM_ratio_1 = 10;
	int PWM_ratio_2 = 25;

	unsigned char old_ler=0;
	unsigned char old_rer=0;
	unsigned char old_per=100;	

 while(1)
 {	
		
  		lch=ADC_Conversion(3);
	 	rch=ADC_Conversion(5);
		cch=ADC_Conversion(4);

		ler=lch-cch;
		rer=rch-cch;
		per=lch-rch;

		rl=rch-lch;

		print_sensor(1,1,3);		//Prints value of White Line Sensor Left
		print_sensor(1,5,4);		//Prints value of White Line Sensor Center
		print_sensor(1,9,5);		//Prints value of White Line Sensor Right

	
		if(rl>0)
		{
			thd = 120 - (kp*rl+kd*(rl-old_rl));
		}
		else
		{
			thd = -120 - (kp*rl+kd*(rl-old_rl));
		}	
		

		lcd_print (2,1,abs(rl),3);
		lcd_print (2,5,(unsigned char)abs(thd),3);

		
		if(rl>(-thd) && rl<(thd))
		{
			forward();
			//velocity(255,255);
		}
		
		else
		{
			if(rl>0)
			{
				if(rl<thd+30)
					 soft_right();
					//velocity(min(255,rl*PWM_ratio_1),0);
				else
					right();
					//velocity(min(255,rl*PWM_ratio_2),0);
			}
			else
			{
				if(rl>-thd-30)
					soft_left();
					//velocity(0,min(255,-rl*PWM_ratio_1));
				else
					left();
					//velocity(0,min(255,-rl*PWM_ratio_2));
			}
		}

		old_rl=rl;
		}
}
