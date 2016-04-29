
/* 
  ReadAnalogVoltage
  Reads an analog input on pin 0, converts it to voltage, and prints the result to the serial monitor.
  Attach the center pin of a potentiometer to pin A0, and the outside pins to +5V and ground.

 This example code is in the public domain.
 */
 
int potin = 1;
int mot1 = 3;
int mot2 = 5;
int sensorValue = 0;
int Error = 0;
int dt=5;
//set user input:    
int userin=550;
int kP=120;
int kI=5;
int kD=1;
float pwmRatio = 2;
float Previous_error=0;
  int initsensorValue = 1;
  int flag=1;
  
  float time;
  //float Drive;
// the setup routine runs once when you press reset:

void setup()

{
  
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  //setting of pin:
  pinMode(potin,INPUT);
  pinMode(mot1,OUTPUT);
  pinMode(mot2,OUTPUT);
   flag=1; 
   initsensorValue = analogRead(1);
   initsensorValue = analogRead(1);
   initsensorValue = analogRead(1);
   initsensorValue = analogRead(1);
   initsensorValue = analogRead(1);initsensorValue = analogRead(1);
}


// the loop routine runs over and over again forever:
void loop() 
{
//  if(flag==1)
//    { delay(10);
//      initsensorValue = analogRead(1);
//      
//    flag=0;}
  // read the input on analog pin 0:
  int sensorValue = analogRead(1);
  // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):
  //float voltage = sensorValue * (5.0 / 1023.0);
  // print out the value you read:
 // Serial.println(voltage);

  
  //Serial.println(sensorValue);
  
  //Serial.println(pwm);



  
  if(initsensorValue<512){ 
  float Error=((sensorValue-initsensorValue)-userin) * (5.0 / 1023.0);
  //pwm calculation:
  

   
 
    float Integral = Integral + (Error*dt);
    float Derivative = (Error - Previous_error)/dt; 
    float Drive = (Error*kP) + (Integral*kI) + (Derivative*kD); 
    float Previous_error = Error;
    
//Serial.println(Drive);

   if (Drive<0)
    {
analogWrite(mot1,min(255,-int(Drive*pwmRatio)));
    analogWrite(mot2,0);
    }

   if (Drive>0)
    {
analogWrite(mot2,min(255,int(Drive*pwmRatio)));
    analogWrite(mot1,0);
    }
  }
else { float Error=((initsensorValue-sensorValue)-userin) * (5.0 / 1023.0);
  //pwm calculation:
  

   
 
    float Integral = Integral + (Error*dt);
    float Derivative = (Error - Previous_error)/dt; 
    float Drive = (Error*kP) + (Integral*kI) + (Derivative*kD); 
    float Previous_error = Error;
    

Serial.println(Drive);

   if (Drive<0)
    {
analogWrite(mot2,min(255,-int(Drive*pwmRatio)));
    analogWrite(mot1,0);
    }

   if (Drive>0)
    {
analogWrite(mot1,min(255,int(Drive*pwmRatio)));
    analogWrite(mot2,0);
    } } 
     
//analogWrite(mot1,255);
  //  analogWrite(mot2,0);
   // Serial.println(mot1);
  //  Serial.println(mot2);
   time=millis();
//   Serial.print(time);
//   Serial.print(" ");
//   Serial.println(sensorValue);
    delay(dt);
    
  }
