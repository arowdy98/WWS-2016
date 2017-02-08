#include<Wire.h>
#include <avr/io.h>
#define F_CPU 16000000UL
#include <util/delay.h>
#include <avr/interrupt.h>
#define SONARLIMIT 11000
volatile int timercount = 0;
volatile int echo = 0; 
volatile double counter;
#define LED 5

const int MPU_addr = 0x68;  // I2C address of the MPU-6050
int16_t GyZ;

/*
 * PB4,6,7 PG1,0,5 --> output - motor driver
 * adc PORT F --> analog input - accelero
 * PA1-->out, PD3-->in - sonar
 * PB5 --> output - servo
*/
void sonar_trigger()           
{
  PORTA = (1<<PA1);      
  _delay_us(10);
  PORTA = 0;
}
void Result_Sonar()
{
  digitalWrite(LED,LOW);
  delay(5);
}
void avr_init()
{
  //*************************Sonar*****************
  EICRA = (1<<ISC30);
  EIMSK = (1<<INT3);
  TCCR3A = 0;
  TCCR3B = 0;
  TCCR3B |= (1<<CS10);
  TCNT3=0;
  TIMSK3 = (1<<TOIE3);
  DDRA = 255;
  DDRD = 0;
  PORTD = 255;
  //****************************Sonar*******************
  //****************************servo*************************
  DDRB=255;
  TCCR1A=(1<<COM1A1)|(1<<WGM11);
  TCCR1B=(1<<WGM12)|(1<<WGM13)|(1<<CS10)|(1<<CS11);
  ICR1=4999;  
  OCR1A=417;
  //*****************servo************************
}
//************************motordriver**************************
int input(int p)
{
  ADMUX|=(1<<REFS0);
  ADMUX|=(ADMUX&11100000)|p;
  ADCSRA|=(1<<ADSC);
  while(ADCSRA&(1<<ADSC));      
    return (ADC);
}
char giv_direction(int y, int x)
{
  if(y<-20)
 
  return 'b';

  if(y>20)
  return 'f';


  if(x>20)
  return 'r';

  if(x<-20)
  return 'l';

  else return 's';
} 
int pwm(int y)
 {
  if(y<-20)
  return 255;
  
  if(y<20 && y>-20)
  return 0;
  
  if(y>=20 && y<=35)
  return 100;

  if(y>35 && y<=50)
  return 200;

  if(y>50)
  return 255;

 }
 void m1(char dir,int pwm_val){
  OCR0A=pwm_val;
 // PORTB|=(1<<7); //Enable pin
  if (dir=='C'){
    PORTB=(1<<4)|(0<<6);
  }
  if (dir=='A'){
    PORTB=(0<<4)|(1<<6);
  }
  if (dir=='B'){
    PORTB=(0<<4)|(0<<6);
  }
  
}

void m2(char dir,int pwm_val){
  OCR0B=pwm_val;
  //PORTG|=(1<<5); //Enable pin
  if (dir=='C'){
    PORTH=(1<<5)|(0<<6);
  }
  if (dir=='A'){
    PORTH=(0<<5)|(1<<6);
  }
  if (dir=='B'){
    PORTH=(0<<5)|(0<<6);
  }
}
//************************motordriver**************************
void MotorDriver_init()
{
   ADCSRA|=(1<<ADEN);
   DDRH=(1<<5)|(1<<6);
  DDRG=(1<<5);
  DDRB=(1<<4)|(1<<6)|(1<<7);
  TCCR0A|=(1<<COM0A1)|(1<<WGM01)|(1<<COM0B1)|(1<<WGM00);
  TCCR0B|=(1<<CS00);
}
void MPU_init()
{
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
}
void bot_run()
{
  sonar_trigger();
  MotorDriver_loop();
  MPU_loop();
}
int servocall(char i)
{
    if (i=='r')
    OCR1A=200;  //-90deg input   right
    if (i=='m')
    OCR1A=417;  //0deg input  center
    if (i=='l')
    OCR1A=625;  //90deg input   left
}
void MPU_loop()
{
    Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
  GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  Serial.print(" | GyZ = "); Serial.println(GyZ);
  //delay(333);
  if(GyZ>5000)
  {
    servocall('r');
  }
  if(GyZ<100)
  {
    servocall('l');
  }
  if(GyZ>100&&GyZ<5000)
  {
    servocall('m');
  }
}
void MotorDriver_loop()
{
    char dir;
  int x,y,pwm_val;

  x=input(1)-335;
  y=input(0)-330;
  
  dir=giv_direction(y,x);

 pwm_val=pwm(y);
  
  if (dir=='l'){
    m1('B',0);m2('C',255);
  }
  if (dir=='r'){
    m1('C',255);m2('B',0);
  }
  if (dir=='f'){
    m1('C',pwm_val);m2('C',pwm_val);
  }
  if (dir=='b'){
    m1('A',255);m2('A',255);
  }
  if (dir=='s'){
    m1('B',0);m2('B',0);
  }
}

void setup(){
  sei();
  avr_init();
  MPU_init();
  MotorDriver_init();
  pinMode(LED,OUTPUT);
  Serial.begin(9600);
}
void loop()
{
  bot_run();
}
//****************************Interrupt SONAR Timer3*************************************
ISR(INT3_vect)
{
  if(echo==0)
  {
                echo=1;   
    TCNT3 = 0;                    
                timercount = 0;         
  }
  else if(echo==1)
  {
    echo =0;
                counter=TCNT3;     
                //Serial.print(D);        
    if(counter<SONARLIMIT)
    {
      Result_Sonar();
    }
    else
    {
     digitalWrite( LED ,HIGH);
    }
                Serial.print(timercount);
                Serial.print('\t');
                Serial.print(counter);
                Serial.print('\t');
          //      Serial.println(distance);
  } 
}
ISR(TIMER3_OVF_vect)
{
  timercount++;
}
//********************************SONAR*******************************************************
