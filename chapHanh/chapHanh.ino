#include <Servo.h>
//#include <SerialCommand.h>
//SerialCommand sCmd;
int stepPin_0 = 4; 
int dirPin_0 = 5;
int stepPin_1 = 51;
int dirPin_1 = 50; 
#define SERVO_PIN_L 3 // trên 
#define SERVO_PIN_R 2 // dưới
Servo gServo_Top ;
Servo gServo_Bot ;
#define HANH_TRINH_VAO 8
#define HANH_TRINH_RA 9
#define HANH_TRINH_LEN 10
#define HANH_TRINH_XUONG 11
#define RELAY1 22
#define RELAY2 23
#define RELAY3 25
#define RELAY4 27
unsigned long preTime =0;
unsigned long currentMillis=0 , HieuTime=0;
int stepCount_0=0;
int stepCount_1=0;
//bool pin_connected;
//bool endStage_0 =false;
//bool endStage_1 =false;
bool x=0;
bool sTop=0, stOp_1=0;
void TINHTIENNGANG_VAO(int distance=8000)
{
  digitalWrite(RELAY2,LOW);
  delay(200);
  digitalWrite(dirPin_0,LOW);
  for (int j=1;j<=distance;j++){
  if(digitalRead(HANH_TRINH_VAO))break;
  digitalWrite(stepPin_0,HIGH);
  delayMicroseconds(1100);
  if(digitalRead(HANH_TRINH_VAO))break;
  digitalWrite(stepPin_0,LOW);
  delayMicroseconds(1100);
  if(digitalRead(HANH_TRINH_VAO))break;
  }
  
  digitalWrite(RELAY2,HIGH);
}
void TINHTIENNGANG_RA(int distance=8000 )
{
  digitalWrite(RELAY2,LOW);
  delay(200);
  digitalWrite(dirPin_0,HIGH);
  for (int j=1;j<distance;j++){
    if(digitalRead(HANH_TRINH_RA))break;
    digitalWrite(stepPin_0,HIGH);
    delayMicroseconds(1100);
    if(digitalRead(HANH_TRINH_RA))break;
    digitalWrite(stepPin_0,LOW);
    delayMicroseconds(1100);
    if(digitalRead(HANH_TRINH_RA))break;
  }
  digitalWrite(RELAY2,HIGH);
  return;
}
void TINHTIENDOC_LEN(int dst)
{
  digitalWrite(RELAY3,LOW);
  delay(2000);
  digitalWrite(dirPin_1,HIGH);
  for (int i=0;i<=dst;i++){
    if(digitalRead(HANH_TRINH_LEN))break;
    digitalWrite(stepPin_1,HIGH);
    delayMicroseconds(500);
    if(digitalRead(HANH_TRINH_LEN))break;
    digitalWrite(stepPin_1,LOW);
    delayMicroseconds(500);
    }
    
    digitalWrite(RELAY3,HIGH);
}
void TINHTIENDOC_XUONG(int dst)
{
  digitalWrite(RELAY3,LOW);
  delay(200);
  digitalWrite(dirPin_1,LOW);
  for (int i=0;i<=dst;i++){
    if(digitalRead(HANH_TRINH_XUONG))break;
    digitalWrite(stepPin_1,HIGH);
    delayMicroseconds(500);
    if(digitalRead(HANH_TRINH_XUONG))break;
    digitalWrite(stepPin_1,LOW);
    delayMicroseconds(500);
    }
    
    digitalWrite(RELAY3,HIGH);
}
void THA_1_BONG()
{
  
    digitalWrite(RELAY4,HIGH);
    delay(1000);
    gServo_Bot.write(0);
    delay(1000);
    gServo_Bot.write(0);
    delay(1000);
    gServo_Bot.write(40);
    delay(1000);
    gServo_Bot.write(40);
    delay(1000);
    digitalWrite(RELAY4,LOW);
}
void THA_QUA_2()
{
  digitalWrite(RELAY4,HIGH);
    delay(1000);
   gServo_Top.write(130);
   delay(1000);
   gServo_Top.write(130);
   delay(1000);
   gServo_Top.write(0);
   delay(1000);
   gServo_Top.write(0);
   digitalWrite(RELAY4,LOW);
  
}
void GAP()
{
   delay(2000);
    gServo_Top.write(90);
    delay(2000);
    gServo_Bot.write(40);
    delay(2000);
    gServo_Top.write(180);
    delay(0);
    gServo_Bot.write(120);
    return ;
}
//================================================================
void setup() {
  // Sets the two pins as Outputs
  Serial.begin(9600);
 
  pinMode(RELAY1,OUTPUT);digitalWrite(RELAY1,HIGH);
  pinMode(RELAY2,OUTPUT);digitalWrite(RELAY2,HIGH);
  pinMode(RELAY3,OUTPUT);digitalWrite(RELAY3,HIGH);
  pinMode(RELAY4,OUTPUT);digitalWrite(RELAY4,LOW);
  pinMode(HANH_TRINH_VAO,INPUT_PULLUP);
  pinMode(HANH_TRINH_RA,INPUT_PULLUP);
  pinMode(HANH_TRINH_LEN,INPUT_PULLUP);
  pinMode(HANH_TRINH_XUONG,INPUT_PULLUP);
  pinMode(stepPin_1,OUTPUT); 
  pinMode(dirPin_1,OUTPUT);
  pinMode(stepPin_0,OUTPUT); 
  pinMode(dirPin_0,OUTPUT);
  pinMode(13,OUTPUT);
  digitalWrite(dirPin_0,HIGH);
  digitalWrite(stepPin_0,LOW);
  digitalWrite(dirPin_1,LOW);
  digitalWrite(stepPin_1,LOW);
  
  gServo_Top.attach(SERVO_PIN_L);//,600,2300);
  gServo_Bot.attach(SERVO_PIN_R);//,600,2300); 
  
  Serial2.begin(9600);
  Serial2.setTimeout(5);
/*  sCmd.addCommand("gap",GAP);
  sCmd.addCommand("tha_1_bong",THA_1_BONG);
  sCmd.addCommand("tha_qua_2",THA_QUA_2);
  sCmd.addCommand("tinhtienngangvao",TINHTIENNGANG_VAO);
  sCmd.addCommand("tinhtiendoclen",TINHTIENDOC_LEN);
  sCmd.addCommand("tinhtienngangra",TINHTIENNGANG_RA);
  sCmd.addCommand("tinhtiendocxuong",TINHTIENDOC_XUONG);*/
  gServo_Bot.write(40);
  gServo_Top.write(0);
  TINHTIENNGANG_VAO(9000);
  //TINHTIENNGANG_VAO();

  //SUT();
  //THA_1_BONG();
  //THA_QUA_2();

}
void SUT()
{
  digitalWrite(RELAY1,LOW);
  delay(300);
  digitalWrite(RELAY1,HIGH);
 }
 int waitForCommand = 2;
void loop() 
{   
// TINHTIENNGANG_RA(2800);
  //THA_1_BONG();
//  THA_QUA_2();   
  if(Serial2.available()>0)
  {
    int command = Serial2.read();
    if(command==waitForCommand)
    {
      if(command==2)
      {
        TINHTIENNGANG_RA(2800);
        digitalWrite(13,HIGH);
        THA_1_BONG();
        digitalWrite(13,LOW);
        TINHTIENNGANG_VAO(8000);
        Serial2.write(2);
        Serial2.write(2);
        delay(200);
        Serial2.write(2);
        Serial2.write(2);
        waitForCommand = 3;

        delay(20000);
        TINHTIENNGANG_RA(8000);
        THA_QUA_2();
        THA_1_BONG();
         delay(15000);
         SUT();
                  delay(150000);
      }
      else if (command==3)
      {
        TINHTIENNGANG_RA(2800);
        digitalWrite(13,HIGH);
        THA_1_BONG();
        digitalWrite(13,LOW);
        //TINHTIENNGANG_VAO(8000);
        Serial2.write(2);
        Serial2.write(2);
        delay(200);
        Serial2.write(2);
        Serial2.write(2);
        waitForCommand = 3;

        delay(20000);
        //TINHTIENNGANG_RA(8000);
        THA_QUA_2();
        THA_1_BONG();
         delay(15000);
         SUT();
                  delay(150000);
      }
      else if(command==4)
      {
        
        THA_QUA_2();
        THA_1_BONG();
        Serial2.write(3);
        waitForCommand = 4;
        }
      else if (command==4)
      {
        SUT();
        Serial2.write(4);
        waitForCommand=5;
      }
    }
  }
}
