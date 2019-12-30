// PINOUT
// MOTOR RIGHT
// L_EN -> 11
// R_EN -> 12
// L_PWM -> 10
// R_PWM -> 9

// MOTOR LEFT
// L_EN -> 7
// R_EN -> 8
// L_PWM -> 6
// R_PWM -> 5
//#define PRINT_LINE
#define PRINT_ROBOT_STAT
//#define PRINT_ECODER
#include "BTS7960.h"
//#include "MotorDcController.h"
double Kp = 2.7, Ki = 0.1, Kd = 0.05 ;
#define ENCODER_0_A 47
#define ENCODER_0_B 49
#define ENCODER_1_A 51
#define ENCODER_1_B 53
const uint8_t LEFT_L_EN = 8;
const uint8_t LEFT_R_EN = 9;
const uint8_t LEFT_L_PWM = 10;
const uint8_t LEFT_R_PWM = 11;

const uint8_t RIGHT_L_EN = 6;
const uint8_t RIGHT_R_EN = 7;
const uint8_t RIGHT_L_PWM = 12;
const uint8_t RIGHT_R_PWM = 13;

const uint8_t sensor_pin[8] = {22, 23, 24, 25, 26, 27, 28, 29};

BTS7960 motorControllerRight(RIGHT_L_EN, RIGHT_R_EN, RIGHT_L_PWM, RIGHT_R_PWM);
BTS7960 motorControllerLeft(LEFT_L_EN, LEFT_R_EN, LEFT_L_PWM, LEFT_R_PWM);

int dem_line = 0, temp_count = 0;
// DC Motor 's Speed
double BASE_SPEED = 70;

//khai bao bien theta de giam xung tuyen tinh
int theta = 2;
int chay = 0; // ban dau robot dung yen
int esp = 15;  // esp la he so van toc
#define LEFT_LINE_1 A9
#define LEFT_LINE_2 A11
#define RIGHT_LINE_1 A10
#define RIGHT_LINE_2 A12


int tr_ = 0;
int tl_ = 0;
int lineLeft = 0;
int lineRight = 0;
int lineL = 0, lineR = 0;
int oldTime = 0;
void demNga3()
{
  
    int value = analogRead(RIGHT_LINE_1) + analogRead(RIGHT_LINE_2);

    tr_ += (value - tr_) / 5;
    Serial.println(tr_);
    if (value - tr_ > 10)
      lineR++;
    else
    {
      lineR = 0;
    }
    if (lineR == 5)
    {
      if (oldTime > 200)
        lineRight++;
    }
    value = analogRead(LEFT_LINE_1) + analogRead(LEFT_LINE_2);
    tl_ += (value - tl_) / 5;
    if (value - tl_ > 10)
      lineL++;
    else
    {
      lineL = 0;
    }
    if (lineL == 5)
    {
      if (oldTime > 200)
        lineLeft++;
    }
  
}
// Các biến dưới đây dùng để làm việc với các sensor và điều khiển hành vi của robot
int sensor[8] = {0, 0, 0, 0, 0, 0, 0, 0};
//float s[8] = {1.5,1.2,1.2,1.30,1.2,1.3,1.2,1.2};
float s[8] = {1, 1, 1, 1, 1, 1, 1, 1};
int count = 0, rate = 750, n = 0, back = 0;
int Action = -1;
float sample_time = 0.005;

// Các biến dưới đây dùng để tính PID
double last_8_Error[8] = {0, 0, 0, 0, 0, 0, 0, 0}, Error = 0, last_Error = 0, PID_value = 0;


void print_line_sensor()
{
    for (int i = 0; i < 8; i++)
    {
        Serial.print("Line:");
        Serial.print(sensor[i]);
        
    }
    
    
}

// robot 's movement function
void Move(double PWM_L1, double PWM_L2, double PWM_R1, double PWM_R2)
{
    analogWrite(LEFT_L_PWM, PWM_L1);
    analogWrite(LEFT_R_PWM, PWM_L2);
    analogWrite(RIGHT_R_PWM, PWM_R1);
    analogWrite(RIGHT_L_PWM, PWM_R2);
}

void Action_Forward()
{
    while (count >=3)
    {
        Move(BASE_SPEED, 0, 0, BASE_SPEED);
        calculate_Error();
    }
}

void rotate_90_left()
{
    motorControllerLeft.Enable();
    motorControllerRight.Enable();
    BASE_SPEED=60;
    Move(0,0,0,0);
    // while (sensor[3] == 0 || sensor[4] == 0 || sensor[5] == 0)
    unsigned long times=millis();
    while((millis()-times)<=1200){//lui
        //Serial.print("\naaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa\n");
        Move(0, 60, 55, 0);
    }
    times=millis();
    while ((millis()-times)<=2400)
    {
        //Serial.print("\nbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb\n");
        Move(10, 0, 0, 80);
        //calculate_Error();
    }
    Move(0,0,0,0);
    //  delay(100000);
    //  while (sensor[3] == 0)
    //  {
    //    Serial.print("\ncccccccccccccccccccccccccccccccccccccccccccccccccccccccccc\n");
    //    Move(0, 20, 0, 30);
    //    calculate_Error();
    //  }
    Move(20, 0, 0, 40);
    delay(300);
    //Move(0, 0, 0, 0);
    //delay(300);
}

void calculate_Error()
{
  
    //demNga3();
    //oldTime=ntime;
    double temp1 = 0, temp2 = 0;
    sensor[0] = digitalRead(sensor_pin[0]);
    sensor[1] = digitalRead(sensor_pin[1]);
    sensor[2] = digitalRead(sensor_pin[2]);
    sensor[3] = digitalRead(sensor_pin[3]);
    sensor[4] = digitalRead(sensor_pin[4]);
    sensor[5] = digitalRead(sensor_pin[5]);
    sensor[6] = digitalRead(sensor_pin[6]);
    sensor[7] = digitalRead(sensor_pin[7]);

    for (int i = 0; i < 8; i++)
    {
        if (sensor[i] == 1)
            sensor[i] = 0;
        else
            sensor[i] = 1;
        temp1 += i * sensor[i];
        temp2 += sensor[i];
    }

    count = temp2; //Tổng số led bắt được đường line
    if (count == 0)
    {
        if (Error <= -3.5)
        {
            Error = -4;
        }
        else if (Error >= 3.5)
        {
            Error = 4;
        }
        else
            Error = 0;
    }
    else
        Error = double((temp1 / temp2) - 3.5) * esp; // Tính sai số

    if (count>=4)
    {
        if(millis()-temp_count>500)
        {
            dem_line++;
            temp_count = millis();
        }
    }


  
  
}

void PID_compute()
{
    double sum_last_8_error = 0;

    if (n > 7)
        n = 0;
    last_8_Error[n] = Error;
    n += 1;

    for (int i = 0; i < 8; i++)
    {
        sum_last_8_error += last_8_Error[i];
    }
    sum_last_8_error = sum_last_8_error / 8;

    PID_value = Kp * Error + Ki * sum_last_8_error * sample_time + Kd * (Error - last_Error) / sample_time;
    //Serial.print("PID_value:");

    //Serial.print("\n");
    last_Error = Error;
}
#define MAX_POWER 200
void Motor_control()
{
    int L, R;
    if (Error < 0)
    {
        L = BASE_SPEED + PID_value;
        R = BASE_SPEED-PID_value;
        /*Serial.print("L :");
        Serial.print(L);
        Serial.print("\n");
        Serial.print("R :");
        Serial.print(R);
        Serial.print("\n");*/
        if (L > MAX_POWER)
            L = MAX_POWER;
        if (L < 0)
            L = 0;
        if (R > MAX_POWER)
            R = MAX_POWER;
        if (R < 0)
            R = 0;
        Move(L, 0, 0, R);
    }
    else if (Error > 0)
    {
        L = BASE_SPEED+PID_value;
        R = BASE_SPEED - PID_value;
        /* Serial.print("L :");
        Serial.print(L);
        Serial.print("\n");
        Serial.print("R :");
        Serial.print(R);
        Serial.print("\n");*/
        if (L > MAX_POWER)
            L = MAX_POWER;
        if (L < 0)
            L = 0;
        if (R > MAX_POWER)
            R = MAX_POWER;
        if (R < 0)
            R = 0;
        Move(L, 0, 0, R);
    }
    else
    {
        Move(BASE_SPEED, 0, 0, BASE_SPEED);
    }
}
long long int distanceOld = 0;

long long int countEncoder0 = 0, countEncoder1 = 0;

void resetDistanceCount()
{
    distanceOld = (countEncoder0 + countEncoder1) / 2;
}
int getDistanceCount()
{
    return (countEncoder0 + countEncoder1) / 2 - distanceOld;
}
bool n0, n1;
bool n0_old, n1_old;
void readEncoder0()
{

    n0 = digitalRead(ENCODER_0_A);
    //Serial.println (n0);
    if ((n0_old == LOW) && (n0 == HIGH))
    {
        if (digitalRead(ENCODER_0_B) == LOW)
        {
            countEncoder0--;
        }
        else
        {
            countEncoder0++;
        }
        //Serial.print ("encoder0Pos:");
        //Serial.println (encoder0Pos);
    }
    n0_old = n0;
}
void readEncoder1()
{
    n1 = digitalRead(ENCODER_1_A);
    //Serial.println (n1);
    if ((n1_old == LOW) && (n1 == HIGH))
    {
        if (digitalRead(ENCODER_1_B) == LOW)
        {
            countEncoder1--;
        }
        else
        {
            countEncoder1++;
        }
        //Serial.print ("encoder1Pos:");
        //Serial.println (encoder1Pos);
    }
    n1_old = n1;
}
void setup()
{

    cli();//stop interrupts

    //set timer4 interrupt at 1Hz
    TCCR4A = 0;// set entire TCCR1A register to 0
    TCCR4B = 0;// same for TCCR1B
    TCNT4  = 0;//initialize counter value to 0
    // set compare match register for 1hz increments
    OCR4A = 15624/500;// = (16*10^6) / (1*1024) - 1 (must be <65536)
    // turn on CTC mode
    TCCR4B |= (1 << WGM12);
    // Set CS12 and CS10 bits for 1024 prescaler
    TCCR4B |= (1 << CS12) | (1 << CS10);
    // enable timer compare interrupt
    TIMSK4 |= (1 << OCIE4A);

    sei();//allow interrupts
    motorControllerLeft.Enable();
    motorControllerRight.Enable();
    pinMode(13, OUTPUT);
    pinMode(ENCODER_0_A, INPUT);
    pinMode(ENCODER_0_B, INPUT);
    pinMode(ENCODER_1_A, INPUT);
    pinMode(ENCODER_1_B, INPUT);

    pinMode(LEFT_R_PWM, OUTPUT);
    pinMode(LEFT_L_PWM, OUTPUT);
    pinMode(RIGHT_L_PWM, OUTPUT);
    pinMode(RIGHT_L_PWM, OUTPUT);

    pinMode(sensor_pin[0], INPUT);
    pinMode(sensor_pin[1], INPUT);
    pinMode(sensor_pin[2], INPUT);
    pinMode(sensor_pin[3], INPUT);
    pinMode(sensor_pin[4], INPUT);
    pinMode(sensor_pin[5], INPUT);
    pinMode(sensor_pin[6], INPUT);
    pinMode(sensor_pin[7], INPUT);

    Serial.begin(9600); // Enable Serial Communications
    Serial.setTimeout(5);
    Serial2.begin(9600);
}

ISR(TIMER4_COMPA_vect){
    readEncoder0();
    readEncoder1();
#ifdef PRINT_ENCODER
    Serial.print("\nEncoder0: ");
    Serial.print(" Encoder1: ");

#endif

}
int stage = 0;
void Robot_RUN()
{
    //while ((count <= 2) || ((count == 3) && (sensor[0] == 0) && (sensor[7] == 0) && (sensor[1] == 0) && (sensor[6] == 0)))
    //{

    calculate_Error();
    PID_compute();
    Motor_control();
#ifdef PRINT_ROBOT_STAT
    Serial.print("\ndst:");
    Serial.print(getDistanceCount());
    Serial.print(" stage:");
    Serial.print(stage);
    Serial.print(" dem_line:");
    Serial.print(dem_line);
    Serial.print(" count:");
    Serial.print(count);
    Serial.print(" control:");
    Serial.println(PID_value);
#endif
#ifdef PRINT_LINE
    print_line_sensor();
  #endif
    //}
}
void rotate_90_right()
{
  motorControllerLeft.Enable();
    motorControllerRight.Enable();
    BASE_SPEED=60;
    Move(0,0,0,0);
    // while (sensor[3] == 0 || sensor[4] == 0 || sensor[5] == 0)
    unsigned long times=millis();
    while((millis()-times)<=1200){//lui
        //Serial.print("\naaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa\n");
        Move(0, 55, 60, 0);
    }
    times=millis();
    while ((millis()-times)<=2400)
    {
        //Serial.print("\nbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb\n");
        Move(80, 0, 0, 10);
        //calculate_Error();
    }
    Move(0,0,0,0);
    //  delay(100000);
    //  while (sensor[3] == 0)
    //  {
    //    Serial.print("\ncccccccccccccccccccccccccccccccccccccccccccccccccccccccccc\n");
    //    Move(0, 20, 0, 30);
    //    calculate_Error();
    //  }
    Move(40, 0, 0, 20);
    delay(300);
    //Move(0, 0, 0, 0);
    //delay(300);
  }
void gotoStage(int newstage)
{
    BASE_SPEED=70;
    lineLeft = 0;
    lineRight = 0;
    //dem_line=0;
    resetDistanceCount();
    stage=newstage;
}
void UpdateRobot(){
    if(stage==0){

        
        if(dem_line==1){

            BASE_SPEED=50;
        }
        //Serial.print(BASE_SPEED);

        //Serial.print("\n");
        if (dem_line == 2)
        {

            Serial.print("\n---------------------------------------------------------\n");
            motorControllerRight.Disable();
            motorControllerLeft.Disable();
            delay(50);
            rotate_90_right();
            gotoStage(2);
        }
        else{

            calculate_Error();
            Robot_RUN();
            Action_Forward();
        }
    }
    else if(stage==2){
        
        if(dem_line==6){
            Move(0,0,0,0);
            delay(500);
            rotate_90_right();
            
            gotoStage(3);
            //dem_line=0;
            
            /*
            //Serial2.write(7);
            while(Serial2.available()<=0)
               {
                    //int command = Serial2.read();
                    //if(command==2)
                    {
                        gotoStage(3);
                        digitalWrite(13,HIGH);
                        break;
                    }
               }*/
               
        }
        calculate_Error();
        Robot_RUN();
        Action_Forward();
    }
    else if(stage==3)
    {
      if(dem_line==8){
    
        BASE_SPEED=50;
        rotate_90_left();
        gotoStage(4);
      }
        calculate_Error();
        Robot_RUN();
        Action_Forward();
    }
    else if(stage==4)
    {
      if(dem_line==10){
        Move(0,0,0,0);
            delay(500);
            
            Serial2.write(3);
            Serial2.write(3);
            delay(15000);
            gotoStage(5);
      }
      calculate_Error();
        Robot_RUN();
        Action_Forward();
    }else if(stage==5)
    {
      if(dem_line==11){
        Move(0,0,0,0);
            delay(500);
            Serial2.write(3);
            Serial2.write(3);
            delay(200000);
            gotoStage(5);
      }
      calculate_Error();
        Robot_RUN();
        Action_Forward();
    }
    else{
        calculate_Error();
        Robot_RUN();
        Action_Forward();
    }
}
void loop()
{
    //Serial.println(dem_line);
    UpdateRobot();
}
