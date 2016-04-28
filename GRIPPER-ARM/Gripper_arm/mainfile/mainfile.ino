#include<Servo.h>

Servo S1;
Servo S2;
Servo S3;
Servo S5;
int S1_pin=9;
int S2_pin=3;
int S3_pin=7;
int S5_pin=2;
int cnt=0;

int S1_init=0;
int S2_init=120;
int S3_init=90;
int S5_init=0;

boolean done=1;
int angle1,angle2,angle3;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  S1.attach(S1_pin);
  S2.attach(S2_pin);
  S3.attach(S3_pin);
  S5.attach(S5_pin);
  
    
  servo_move(1,0);
  servo_move(2,120);
  servo_move(3,90);
  servo_move(5,0);
//  
  
}
  
void loop() {
  // put your main code here, to run repeatedly:
  if(Serial.available()){
  angle1 = Serial.read(); Serial.println(angle1); 
  delay(2);
  angle2 = 180-Serial.read();Serial.println(angle2);
  delay(2);
  angle3 = Serial.read();Serial.println(angle3);
  done = 0;
  }

if(done==0){
  
  Serial.print(angle1);
  Serial.print("  ");
  Serial.print(angle2);
  Serial.print("  ");
  Serial.println(angle3);
  servo_move(1,angle1);
  servo_move(2,angle2);
  servo_move(5,120);
  servo_move(3,angle3);
  
  servo_move(5,40);  //holding the object
  
  //moving the object
  servo_move(3,90);
  servo_move(2,120);
  servo_move(1,0);
  servo_move(2,angle2);
  servo_move(3,angle3);
  
  servo_move(5,120); //leaving the object
  
  servo_move(1,0);
  servo_move(2,120);
  servo_move(3,90);
  servo_move(5,0);
  done=1;
  }

}


void servo_move(int servo_no, int final_angle) {
  
    if(servo_no==1){
      if(final_angle>=S1_init){
          for(int i=S1_init;i<=final_angle; i++){
            S1.write(i); delay(10); }}
      else {
          for(int i=S1_init;i>=final_angle; i--){
            S1.write(i);delay(10);}}
     S1_init=final_angle;}
     
       
    if(servo_no==2){
      if(final_angle>S2_init){
          for(int i=S2_init;i<=final_angle; i++){
            S2.write(i);delay(10);}}
      else {
          for(int i=S2_init;i>=final_angle; i--){
            S2.write(i);delay(10);}}
     S2_init=final_angle;}
     
       
    if(servo_no==3){
      if(final_angle>S3_init){
          for(int i=S3_init;i<=final_angle; i++){
            S3.write(i);delay(10);}}
      else {
          for(int i=S3_init;i>=final_angle; i--){
            S3.write(i);delay(10);}}
     S3_init=final_angle;}
     
       
    if(servo_no==5){
      if(final_angle>S5_init){
          for(int i=S5_init;i<=final_angle; i++){
            S5.write(i);delay(10);}}
      else {
          for(int i=S5_init;i>=final_angle; i--){
            S5.write(i);delay(10);}}
     S5_init=final_angle;}
    
  
  delay(200);
}
  
