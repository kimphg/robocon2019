#include <Servo.h>
//#include <SerialCommand.h>
//SerialCommand sCmd;
int stepPin_0 = 4; 
int dirPin_0 = 5;
int stepPin_1 = 51;
int dirPin_1 = 50; 
#define SERVO_PIN_L 7 // trên 
#define SERVO_PIN_R 6 // dưới
Servo gServo_L ;
Servo gServo_R ;
#define HANH_TRINH_VAO 8
#define HANH_TRINH_RA 9
#define RELAY1 22
#define RELAY2 23
#define RELAY3 24
#define RELAY4 25
unsigned long preTime =0;
unsigned long currentMillis=0 , HieuTime=0;
int stepCount_0=0;
int stepCount_1=0;
//bool pin_connected;
//bool endStage_0 =false;
//bool endStage_1 =false;
bool x=0;
bool sTop=0, stOp_1=0;
void TINHTIENNGANG_VAO(int distance = 9000)
{
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
}
void TINHTIENNGANG_RA(int distance = 9000)
{
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
}
void TINHTIENDOC_LEN()
{
  digitalWrite(dirPin_1,HIGH);
  for (int i=0;i<=200;i++){
  digitalWrite(stepPin_1,HIGH);
  delayMicroseconds(800);
  digitalWrite(stepPin_1,LOW);
  delayMicroseconds(800);}
}
void TINHTIENDOC_XUONG()
{
  digitalWrite(dirPin_1,LOW);
  for (int j=1;j<=35;j++)
  {
  for (int i=0;i<=200;i++)
  {
  digitalWrite(stepPin_1,HIGH);
  delayMicroseconds(800);
  digitalWrite(stepPin_1,LOW);
  delayMicroseconds(800);
  }
  }
}
void THA_1_BONG()
{
    //delay(2000);
    //gServo_L.write(180);
    delay(2000);
    gServo_R.write(120);
    delay(2000);
    //gServo_L.write(90);
    //delay(2000);
    gServo_R.write(40);
    return ;
}
void THA_QUA_2()
{
   delay(2000);
   gServo_L.write(180);
   delay(2000);
   gServo_L.write(90);
   delay(2000);
  
}
void GAP()
{
    delay(2000);
    gServo_L.write(90);
    delay(2000);
    gServo_R.write(40);
    delay(2000);
    gServo_L.write(180);
    delay(2000);
    gServo_R.write(120);
    return ;
}
//================================================================
void setup() {
  // Sets the two pins as Outputs
  Serial.begin(115200);
 
  pinMode(RELAY1,OUTPUT);digitalWrite(RELAY1,HIGH);
  pinMode(RELAY2,OUTPUT);digitalWrite(RELAY2,HIGH);
  pinMode(RELAY3,OUTPUT);digitalWrite(RELAY3,HIGH);
  pinMode(RELAY4,OUTPUT);digitalWrite(RELAY4,HIGH);
  pinMode(HANH_TRINH_VAO,INPUT_PULLUP);
  pinMode(HANH_TRINH_RA,INPUT_PULLUP);
  pinMode(stepPin_1,OUTPUT); 
  pinMode(dirPin_1,OUTPUT);
  pinMode(stepPin_0,OUTPUT); 
  pinMode(dirPin_0,OUTPUT);
  pinMode(13,OUTPUT);
  digitalWrite(dirPin_0,HIGH);
  digitalWrite(stepPin_0,LOW);
  digitalWrite(dirPin_1,LOW);
  digitalWrite(stepPin_1,LOW);
  
  gServo_L.attach(SERVO_PIN_L,600,2300);
  gServo_R.attach(SERVO_PIN_R,600,2300); 
  
  Serial2.begin(9600);
  Serial2.setTimeout(5);
/*  sCmd.addCommand("gap",GAP);
  sCmd.addCommand("tha_1_bong",THA_1_BONG);
  sCmd.addCommand("tha_qua_2",THA_QUA_2);
  sCmd.addCommand("tinhtienngangvao",TINHTIENNGANG_VAO);
  sCmd.addCommand("tinhtiendoclen",TINHTIENDOC_LEN);
  sCmd.addCommand("tinhtienngangra",TINHTIENNGANG_RA);
  sCmd.addCommand("tinhtiendocxuong",TINHTIENDOC_XUONG);*/
  gServo_R.write(40);
  gServo_L.write(90);
  TINHTIENNGANG_VAO();
  SUT();
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
 digitalWrite(13,LOW);
  if(Serial2.available()>0)
  {
    int command = Serial2.read();
    if(command==waitForCommand)
    {
      if(command==2)
      {
      
      digitalWrite(13,HIGH);
      TINHTIENNGANG_RA(2800);
      THA_1_BONG();
      //TINHTIENNGANG_VAO();
      Serial2.write(2);
      Serial2.write(2);
      digitalWrite(13,LOW);
      waitForCommand = 3;
      }
      else if(command==3)
      {
        
        THA_QUA_2();
        THA_1_BONG();
        SUT();
        Serial2.write(3);
        waitForCommand = 4;
        }
    }
  }
       
  
 
}
