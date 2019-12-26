#define LEFT_LINE_1 A9
#define LEFT_LINE_2 A11
#define RIGHT_LINE_1 A10
#define RIGHT_LINE_2 A12

#define UPDATE_TIME 50.0 // time to update control in milisec
#define MAX_POWER_TIEN 90.0
#define MAX_POWER_LUI 90.0
#define SIDE_COMPENSATION 1.0
bool keep_center_line = true ;
float leftErrOld = 0, rightErrOld = 0;
#define MOVE_P_TIEN 1000
#define MOVE_I_TIEN 0
#define MOVE_D_TIEN (0)
// di thang (p:6,d:141)
#define MOVE_P_LUI MOVE_P_TIEN
#define MOVE_I_LUI MOVE_I_TIEN
#define MOVE_D_LUI MOVE_D_TIEN

#define MOVE_P_PHAI 5
#define MOVE_I_PHAI 0
#define MOVE_D_PHAI (500)

#define MOVE_P_TRAI 5
#define MOVE_I_TRAI 0
#define MOVE_D_TRAI (500)
const uint8_t LEFT_L_EN = 8;
const uint8_t LEFT_R_EN = 9;
const uint8_t LEFT_L_PWM = 10;
const uint8_t LEFT_R_PWM = 11;

const uint8_t RIGHT_L_EN = 6;
const uint8_t RIGHT_R_EN = 7;
const uint8_t RIGHT_L_PWM = 12;
const uint8_t RIGHT_R_PWM = 13;
int tr_ = 0;
int tl_ = 0;
int lineLeft = 0;
int lineRight = 0;
int lineL = 0, lineR = 0;
int oldTime = 0;
void DemNga3()
{
    long long time = millis();

    if (time - oldTime > 10)
    {
        // Serial.print(lineLeft);
        // Serial.print(" ");

        // Serial.print(lineRight);
        // Serial.print("\n");
        oldTime = time;
        int value = analogRead(RIGHT_LINE_1) + analogRead(RIGHT_LINE_2);

        tr_ += (value - tr_) / 5;
        //Serial.println(tr_);
        if (value - tr_ > 10)
            lineR++;
        else
        {
            lineR = 0;
        }
        if (lineR == 5)
        {
            if (time > 200)
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
            if (time > 200)
                lineLeft++;
        }
    }
}
float p_value,i_value, d_value, max_power = 90.0;
const uint8_t sensor_pin[8] = {22, 23, 24, 25, 26, 27, 28, 29};
int stage = 0;
//BTS7960 motorControllerRight(RIGHT_L_EN, RIGHT_R_EN, RIGHT_L_PWM, RIGHT_R_PWM);
//BTS7960 motorControllerLeft(LEFT_L_EN, LEFT_R_EN, LEFT_L_PWM, LEFT_R_PWM);

int dem_line = 0, temp_count = 0;
// DC Motor 's Speed
const double BASE_SPEED = 70;

//khai bao bien theta de giam xung tuyen tinh
int theta = 2;
int chay = 0; // ban dau robot dung yen
int esp = 7;  // esp la he so van toc

// Các biến dưới đây dùng để làm việc với các sensor và điều khiển hành vi của robot
int sensor[8] = {0, 0, 0, 0, 0, 0, 0, 0};
//float s[8] = {1.5,1.2,1.2,1.30,1.2,1.3,1.2,1.2};
float s[8] = {1, 1, 1, 1, 1, 1, 1, 1};
int count = 0, rate = 750, n = 0, back = 0;
int Action = -1;
float sample_time = 0.005;

// Các biến dưới đây dùng để tính PID
double last_8_Error[8] = {0, 0, 0, 0, 0, 0, 0, 0}, errorLineCenter = 0, last_Error = 0, PID_value = 0;
double Kp = 1.2, Ki = 0.1, Kd = 0.03;

void print_line_sensor()
{
    for (int i = 0; i < 8; i++)
    {
        Serial.print(sensor[i]);
        Serial.print("  ");
    }

    Serial.print("\n");
    Serial.print("Dem line ");
    Serial.print(dem_line);
    Serial.println();
}

// robot 's movement function
float speedLeftEncoder, speedRightEncoder;
// float leftPower = 0, rightPower = 0;


int dst_value, angle_value;

void SetMotion(float sp, float dir)
{

    SetSpeeed(sp + dir, sp - dir);
}
void setAbsPosition(int left,int right)
{

}
int bufferIndex=0;
int leftErrArray[] = {0,0,0,0,0,0,0,0};
int rightErrArray[] = {0,0,0,0,0,0,0,0};
void SetSpeeed(double leftControl, double rightControl)
{
    
    if(bufferIndex<7)bufferIndex++;else bufferIndex=0;
    //
    int leftErr = (leftControl - speedLeftEncoder);
    leftErrArray[bufferIndex] = leftErr;
    float leftPower=  leftErr* p_value
            + (leftErrArray[0]
            + leftErrArray[1]
            + leftErrArray[2]
            + leftErrArray[3]
            + leftErrArray[4]
            + leftErrArray[5]
            + leftErrArray[6]
            + leftErrArray[7])*i_value
            +(leftErr - leftErrOld)/UPDATE_TIME * d_value;
    if (abs(leftPower) > max_power)
        leftPower /= ((abs(leftPower)) / max_power);
    leftErrOld = leftErr;
    //
    float rightErr = (rightControl - speedRightEncoder);
    rightErrArray[bufferIndex] = leftErr;
    float rightPower = rightErr* p_value
            + (rightErrArray[0]
            + rightErrArray[1]
            + rightErrArray[2]
            + rightErrArray[3]
            + rightErrArray[4]
            + rightErrArray[5]
            + rightErrArray[6]
            + rightErrArray[7])*i_value
            +(rightErr - rightErrOld)/UPDATE_TIME * d_value;
    rightPower*= SIDE_COMPENSATION;
    if (abs(rightPower) > max_power)
        rightPower /= ((abs(rightPower)) / max_power);
    rightErrOld = rightErr;
    //
    Serial.print("\nspeed:");
    Serial.print(leftPower);
    Serial.print("  ");
    Serial.print(rightPower);
    //

    int power = abs(leftPower);
    analogWrite(LEFT_L_EN, power);
    analogWrite(LEFT_R_EN, power);
    if (leftPower > 0)
    {

        digitalWrite(LEFT_L_PWM, 1);
        digitalWrite(LEFT_R_PWM, 0);
    }
    else
    {
        digitalWrite(LEFT_L_PWM, 0);
        digitalWrite(LEFT_R_PWM, 1);
    }
    power = abs(rightPower);
    analogWrite(RIGHT_L_EN, power);
    analogWrite(RIGHT_R_EN, power);
    if (rightPower > 0)
    {
        digitalWrite(RIGHT_L_PWM, 1);
        digitalWrite(RIGHT_R_PWM, 0);
    }
    else
    {
        digitalWrite(RIGHT_L_PWM, 0);
        digitalWrite(RIGHT_R_PWM, 1);
    }
}


void calculate_ErrorLineCenter()
{
    double temp1 = 0, temp2 = 0, temp =0;
    sensor[0] = digitalRead(sensor_pin[0]);
    sensor[1] = digitalRead(sensor_pin[1]);
    sensor[2] = digitalRead(sensor_pin[2]);
    sensor[3] = digitalRead(sensor_pin[3]);
    sensor[4] = digitalRead(sensor_pin[4]);
    sensor[5] = digitalRead(sensor_pin[5]);
    sensor[6] = digitalRead(sensor_pin[6]);
    sensor[7] = digitalRead(sensor_pin[7]);

    int val[8] = {3,2,1,0,0,-1,-2,-3};

    for (int i = 0; i < 8; i++)
    {
        if (sensor[i] == 1)
            sensor[i] = 0;
        else
            sensor[i] = 1;
        temp1 += val[i] * sensor[i];
        temp2 += sensor[i];
    }

    // count = temp2; //Tổng số led bắt được đường line
    // if (count == 0)
    // {
    //   if (Error <= -3.5)
    //   {
    //     Error = -4;
    //   }
    //   else if (Error >= 3.5)
    //   {
    //     Error = 4;
    //   }
    //   else
    //     Error = 0;
    // }
    // else
    //   Error = double((temp1 / temp2) - 3.5) * esp; // Tính sai số
    if(temp2 == 0) errorLineCenter = last_Error;
    else if( temp1 < 0 )
        errorLineCenter = - atan ( abs(temp1 / temp2) / 7) / 3.14 * 180;
    else if( temp1 >=0 )
        errorLineCenter = atan ( abs(temp1 / temp2) / 7) / 3.14 * 180;
    // if (count >= 4 && temp_count < 4)
    // {
    // dem_line++;
    // temp_count = count;
    // }
    // else
    // {
    // temp_count = count;
    // }

    last_Error = errorLineCenter;

}



#define ENCODER_0_A 47
#define ENCODER_0_B 49
#define ENCODER_1_A 51
#define ENCODER_1_B 53
void setup()
{
    //  motorControllerLeft.Enable();
    //  motorControllerRight.Enable();
    pinMode(ENCODER_0_A, INPUT);
    pinMode(ENCODER_0_B, INPUT);
    pinMode(ENCODER_1_A, INPUT);
    pinMode(ENCODER_1_B, INPUT);
    pinMode(LEFT_LINE_1, INPUT);
    pinMode(LEFT_LINE_2, INPUT);
    pinMode(RIGHT_LINE_1, INPUT);
    pinMode(RIGHT_LINE_2, INPUT);
    pinMode(LEFT_L_EN, OUTPUT);
    pinMode(LEFT_R_EN, OUTPUT);
    pinMode(LEFT_R_PWM, OUTPUT);
    pinMode(LEFT_L_PWM, OUTPUT);
    pinMode(RIGHT_L_EN, OUTPUT);
    pinMode(RIGHT_R_EN, OUTPUT);
    pinMode(RIGHT_L_PWM, OUTPUT);
    pinMode(RIGHT_R_PWM, OUTPUT);

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
}
bool n0, n1;
bool n0_old, n1_old;
long long int countEncoder0 = 0, countEncoder1 = 0;

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
long long int distanceOld = 0;
int angleOld = 0;
void resetDistanceCount()
{
    distanceOld = (countEncoder0 + countEncoder1) / 2;
}
long long int getDistanceCount()
{
    return (countEncoder0 + countEncoder1) / 2 - distanceOld;
}
void resetAngle()
{
    angleOld = (countEncoder1 - countEncoder0);
}
int getAngle()
{
    return (countEncoder0 - countEncoder1) - angleOld;
}

unsigned long oldMillis = 0;

long long int oldEncoder0Pos = 0, oldEncoder1Pos = 0;

void gotoStage(int st)
{
    stage = st;
    SetMotion(0, 0);
    resetDistanceCount();
    resetAngle();
}
int dstLeft = 0;
int dstLeftOld = 0, angleLeftOld = 0;
//int speedMax =50;

bool goStraight(int dst)
{
    keep_center_line = true;
    if(dst>0)
    {
        p_value = MOVE_P_TIEN;
        i_value = MOVE_I_TIEN;
        d_value = MOVE_D_TIEN;
        //max_power = MAX_POWER_TIEN;
    }
    else
    {
        p_value = MOVE_P_LUI;
        i_value = MOVE_I_LUI;
        d_value = MOVE_D_LUI;
        // = MAX_POWER_LUI;
    }

    int angle = getAngle();
    int distanceCount = getDistanceCount();
    //Serial.println(distanceCount);
    dstLeft = dst - distanceCount;
    //angleLeft = dstAngle - angle;
    float sp = dstLeft / 1000.0;
    //float angleSp = angleLeft / 2000.0;

    if(keep_center_line)SetMotion(sp, - errorLineCenter / 5.0 );
    else SetMotion(sp,0);

    if ((abs(dstLeft - dstLeftOld) < 10) && (abs(dstLeft) < 50))
    {

    }
    // Serial.println(angleLeft);
    //Serial.print(angleLeft);
    dstLeftOld = dstLeft;
    return false;
}
bool turnRight90()
{
    p_value = MOVE_P_PHAI;
    d_value = MOVE_D_PHAI;
    int errorAngle = getAngle()-500;
    SetMotion(0,errorAngle);
    if(abs(errorAngle)<10&&speedLeftEncoder==0&&speedRightEncoder==0)return true;
    return false;
}
bool turnLeft90()
{
    p_value = MOVE_P_TRAI;
    d_value = MOVE_D_TRAI;
    int errorAngle = getAngle()+500;
    SetMotion(0,errorAngle);
    if(abs(errorAngle)<10&&speedLeftEncoder==0&&speedRightEncoder==0)return true;
    return false;
}
void updateRobot()
{
    readEncoder0();
    readEncoder1();
    DemNga3();
    calculate_ErrorLineCenter();
    unsigned long currentMillis = millis();
    //
    if (currentMillis - oldMillis > UPDATE_TIME)
    {
        oldMillis = currentMillis;
        //control scenario

        if (stage == 0)
        {
            // quay phai 90 do: 585
            if(goStraight(400))gotoStage(10);
            if(lineLeft>1)max_power=20;
            if(lineLeft>2){max_power=10;gotoStage(1);}
            Serial.println(lineLeft);
            //if (dem_line == 3){
            //dem_line = 0;
            //gotoStage(1);
            //}
            // (gotoDst(0, 00));
        }
        else if (stage == 1)
        {
            if(goStraight(-100))gotoStage(2);
            // if (turnLeft90(0)){
            //  speedMax = 50;
            // gotoStage(20);
            // }
        }
        else if (stage == 2)
        {
            if (turnLeft90())gotoStage(100);
        }
        else if (stage == 3)
        {
            goStraight(300);
            if (dem_line == 3){
                max_power = 70;
                gotoStage(4);
            }
        }
        else if(stage == 4){
            if(goStraight(300) && dem_line == 5)
                gotoStage(5);
        }
        else {
            SetMotion(0,0);
            Serial.println("stop");
        }
        // encoder speed
        speedLeftEncoder = (countEncoder0 - oldEncoder0Pos) / UPDATE_TIME;
        speedRightEncoder = (countEncoder1 - oldEncoder1Pos) / UPDATE_TIME;
        oldEncoder0Pos = countEncoder0;
        oldEncoder1Pos = countEncoder1;
        //Serial.print("\nSpeed:");
        //Serial.print(speedLeftEncoder);
        //Serial.print(":");
        //Serial.print(speedRightEncoder);
        //Serial.print("\n");
        //if(millis()<2000)Move(1,1);
    }
}
//-------------------------------------------------------------------------------------------------------------------------------------

void loop()
{

    updateRobot();
}
