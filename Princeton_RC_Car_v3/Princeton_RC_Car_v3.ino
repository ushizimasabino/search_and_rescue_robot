#include <Servo.h>
#include <Pixy.h>
//-----------------------------------------------------------------

// Define Servo Variables
  Servo R_Servo;  // Servo DC Motor Driver (Designed for RC cars)
  Servo L_Servo;  // Servo DC Motor Driver (Designed for RC cars)
  Servo ArmR;
  Servo ArmL;
  Servo ArmM;
  int Rwheel;               // Variable to hold R wheel speed
  int Lwheel;               // Variable to hold L wheel speed
  int LeftMaxIn;        //Variable to hold Max Data In
  int RightMaxIn;      //Variable to hold Max Data In
  int CalcHold;        //Variable to remp hold calculations for steering stick corections
  int rSpeed, lSpeed;       // Variables to hold autonomous speed changes for each wheel
  float rcScale;
  int neutralRC = 1500;
  float rWheel = 1500;
  float lWheel = 1500;
  int hit = 0;
// Define RC Variables
  int Ch1,Ch2,Ch3,Ch4,Ch5,Ch6;
  const int LED = 13;       // Onboard LED location

// Define Pixy Variables
  Pixy pixy;
  int signature, x, y, width, height;
  int cont = 0;
  float cx, cy, area, dxIR;
  float maxArea = 315;
  uint16_t blocks;

// Prox Sensor Pins
  const int proxFrontPin = A5;  
  const int proxLeftPin = A3;  
  const int proxRightPin = A4;  

// Prox Sensor Values
  int proxFront;
  float proxFrontF;
  float proxLeftF;
  float proxRightF;
  int proxLeft;
  int proxRight;
  int proxDiff;
  int proxMax = 0.9;

// CALIBRATE
float deadZone = .1;
int fast = 150, slow = 80, neutral = 1500, doublefast = 200;
int diff = -1; // Ratio of Lspeeds to Rspeeds

//**************************************************************
//*****************  Setup  ************************************
//**************************************************************
void setup() {
  // Set the pins that the transmitter will be connected to all to input
  pinMode(A5, INPUT); //I connected this to Chan1 of the Receiver
  pinMode(A4, INPUT); //I connected this to Chan2 of the Receiver
  pinMode(A3, INPUT); //I connected this to Chan3 of the Receiver
//  pinMode(A2, INPUT); //I connected this to Chan4 of the Receiver
//  pinMode(A1, INPUT); //I connected this to Chan5 of the Receiver
//  pinMode(A0, INPUT); //I connected this to Chan6 of the Receiver
  pinMode(LED, OUTPUT);//Onboard LED to output for diagnostics

//  pinMode(8, OUTPUT);
//  pinMode(9, INPUT);
//  pinMode(11, INPUT);
  
  pinMode(proxFrontPin,INPUT);
  pinMode(proxLeftPin,INPUT);
  pinMode(proxRightPin,INPUT);
 
  // Attach Speed controller that acts like a servo to the board
  R_Servo.attach(10);
  L_Servo.attach(9);
  ArmR.attach(A0);
  ArmL.attach(A1);
  ArmM.attach(A2);
  rSpeed = neutral + diff*slow;
  lSpeed = neutral + slow;

//  ArmR.attach(A9);
//  ArmL.attach(A8);
 
  //Flash the LED on and Off 10x before entering main loop
  for (int i = 0; i < 10; i++) {
    digitalWrite(13, HIGH);
    delay(200);
    digitalWrite(13, LOW);
    delay(200);
  }
  //Flash the LED on and Off 10x End
  Serial.begin(115200);
  pixy.init();
}

//************************  loop()  ****************************
//**********************  Main Loop  ***************************
//**************************************************************
void loop() {
  Ch5Check();
}

//**********************  Ch5Check()  **************************
//********************** Test Channel 5   **********************
//**************************************************************
void Ch5Check() {
  Ch6 = pulseIn(8,HIGH);
  if(Ch6 < 1600){
  Ch5 = pulseIn(7, HIGH); // Capture pulse width on Channel 5
  if (Ch5 > 1600) {
    digitalWrite(LED, HIGH);
    autonomous();
  }
  else {
    Ch1 = pulseIn(3, HIGH, 115200); // Capture pulse width on Channel 1
    Ch2 = pulseIn(4, HIGH, 115200); // Capture pulse width on Channel 2
    Ch3 = pulseIn(5, HIGH);  // Capture pulse width on Channel 3
    Ch4 = pulseIn(6, HIGH);  // Capture pulse width on Channel 4
    digitalWrite(LED, LOW);
    DriveServosRC();
//    PrintRC();
  }

  // Ch6 = pulseIn(A0,HIGH);
  // if (Ch6 > 1600){
  //   ==========================================================RC();
  // }
  }
  else if(Ch6 > 1600){
      Ch1 = pulseIn(3, HIGH, 115200); // Capture pulse width on Channel 1dr
      Ch2 = pulseIn(4, HIGH, 115200); // Capture pulse width on Channel 2
      Ch3 = pulseIn(5, HIGH);  // Capture pulse width on Channel 3

      DriveArmRC();
  }
}

// ============================================================================
// ============================= AUTONOMOUS MODE ==============================
// ============================================================================

//**************************  autonomous()  **********************
//********************** Autonomous Mode ***********************
//**************************************************************
void autonomous() {
  cx = 0;
  hit = 0;
  int dlay = 100;
  rWheel = 1500;
  lWheel = 1500;
  rSpeed = neutralRC;
  lSpeed = neutralRC;
  float lightDeadzone = 0.1;
  rcScale = 0.1;

//  pixyTrack();
  centerTot();

  if(abs(dxIR) > abs(cx)){
    cx = dxIR;
  }
  if(hit < 1){
    if(cx < -lightDeadzone){
      rWheel = -1/2 * rcScale * cx * (float)neutralRC + (float)neutralRC;
      lWheel = (float)neutralRC + rcScale * cx * (float)neutralRC;
      rSpeed = (int)rWheel;
      lSpeed = (int)lWheel;
      Serial.println("wocky slush");
    }
    if(cx > lightDeadzone){
      rWheel = (float)neutralRC + rcScale * cx * (float)neutralRC;
      lWheel = (float)neutralRC - 1/2 * rcScale * cx * (float)neutralRC;
      rSpeed = (int)rWheel;
      lSpeed = (int)lWheel;
    }
  }
//  else if(hit == 1){
//    if(proxLeft >= proxMax){
//      // Reverse
//      lSpeed = neutralRC + fast;
//      rSpeed = neutralRC - fast;
//      autoDrive(lSpeed, rSpeed, dlay); 
//      // Left Wheels Spin Forward
//
//      lSpeed = neutralRC - fast;
//      rSpeed = neutralRC;
//      autoDrive(lSpeed, rSpeed, dlay); 
//      lSpeed = neutralRC;
//      rSpeed = neutralRC;
//    }
//    if(proxRight >= proxMax){
//      // Reverse
//      lSpeed = neutralRC + fast;
//      rSpeed = neutralRC - fast;
//
//      // Right Wheels Spin Forward
//      lSpeed = neutralRC;
//      rSpeed = neutralRC + fast;
//      autoDrive(lSpeed, rSpeed, dlay);
//      lSpeed = neutralRC;
//      rSpeed = neutralRC;
//    }
//  }
  Serial.print("cx =");
  Serial.println(cx);
  Serial.print("hit = ");
  Serial.println(hit);
  Serial.print("dxIR = ");
  Serial.println(dxIR);
  Serial.print("lSpeed =");
  Serial.println(lWheel);
  Serial.print("rSpeed =");
  Serial.println(rWheel);
  printSensors();
  cx = 0;
  autoDrive(lSpeed, rSpeed, dlay);
  delay(10);
}
//**********************  Pixy Tracking  ***********************
//**************************************************************
// Tracking Regime
float pixyTrack() {
  // Serial.println("dx2");
  // static int i = 0;
  // int j;
  char buf[32];
  // grab blocks!
  blocks = 0;
  blocks = pixy.getBlocks();
  cx = 0;
  // Get Height & Width of Blocks
  if (blocks)
  {
    signature = pixy.blocks[0].signature;
    height = pixy.blocks[0].height;
    width = pixy.blocks[0].width;
    x = pixy.blocks[0].x;
    // y = pixy.ccc.blocks[0].m_y;
    cx = (x + (width / 2));
    // cy = (x + (width / 2));
    cx = mapfloat(cx, 0, 320, -1, 1);
    // cy = mapfloat(cy, 0, 200, 1, -1);
    // area = width * height;
  }
  else {
    cont += 1;
    if (cont == 100) {
      // Serial.println("dx3");
      cont = 0;
      cx = 0;
    }
    }
    return cx;
}

void driveDx()
{
  float dx = pixyTrack();
 
  // Serial.println(dx);
  if (dx > -deadZone && dx < deadZone){
    Forward(10);
  }
  if (dx <= -deadZone) {
    TLeftSlow(10);
  }
  else if (dx >= deadZone) {
    TRightSlow(10);
  }
  // Serial.print("rSpeed =");
  // Serial.println(abs(rSpeed-1500));
  // Serial.print("lSpeed =");
  // Serial.println(abs(lSpeed-1500));
}

float mapfloat(long x, long in_min, long in_max, long out_min, long out_max){
  return (float)(x-in_min)*(out_max - out_min) / (float)(in_max-in_min) + out_min;
}

// ******************** centerTot() ****************************
// *********** use proximity sensors to center Tot *************
// *************************************************************
void centerTot(){
//  hit = 0;
  float deadzoneIR = 0.1;
  proxFront = analogRead(proxFrontPin);
  proxLeft = analogRead(proxLeftPin);
  proxRight = analogRead(proxRightPin);
// 
  // take 5 samples and average
//  for (int i = 0; i <= 3; i++) {
//    proxFront = proxFront + analogRead(proxFrontPin);
//    proxLeft = proxLeft + analogRead(proxLeftPin);
//    proxRight = proxRight + analogRead(proxRightPin);
//    delay(1);
//  }
//  proxFront = proxFront / 5;
//  proxLeft = proxLeft / 5;
//  proxRight = proxRight / 5;

  proxFrontF = (float)proxFront / 550;
  proxLeftF = (float)proxLeft / 550;
  proxRightF = (float)proxRight / 550;
  // Normalize Functions
//  proxLeft = mapfloat(proxLeft, 0, 450, 0, 1);
//  proxRight = mapfloat(proxRight, 0, 450, 0, 1);

  float proxDiffF = proxRightF - proxLeftF;
//  proxDiff = proxRight - proxLeft;
 Serial.print("proxLeft =");
  Serial.println(proxLeftF);
   Serial.print("proxRight =");
  Serial.println(proxRightF);
  
  Serial.print("proxDiff =");
  Serial.println(proxDiffF);
  if(abs(proxDiffF) < deadzoneIR){
    dxIR = 0;
    hit = 0;
  }
  if(abs(proxDiffF) > deadzoneIR){
    dxIR = proxDiffF;
    hit = 0;

  }
//  if(proxDiffF >= proxMax){
//    hit = 1;
//    dxIR = 0;
//  }
//
//  if(proxDiffF <= -proxMax){
//    hit = 1;
//    dxIR = 0;
//  }
  // NOTE:
  // proxDiff == 0 , WE ARE CENTERED
  // proxDiff > 0, WE ARE BIASED LEFT >> GO RIGHT
  // proxDiff < 0, WE ARE BIASED RIGHT >> GO LEFT

  // EMERGENCY STOP
//  if (proxFront >= 300) { //changed from 500
//    Reverse(10);
//    TRightSlow(10);
//  }
  // REALIGNMENT ALGORITHM
//  if (proxDiff >= -50 || proxDiff <= 50){
//    Forward(10);
//  }
//  if (proxDiff > 50){
////    Reverse(200);
//    TRightSlow(50);
//  }
//  else if (proxDiff < -50){
////    Reverse(200);
//    TLeftSlow(50);
//  }
//   printSensors();
//  if(proxFront > 550){
//    Reverse(1000);
  }
//   delay(200);
 

//********************** setLimits() ***************************
//*******  Make sure values never exceed ranges  ***************
//******  For most all servos and like controlers  *************
//****   control must fall between 1000uS and 2000uS  **********
//**************************************************************
void setLimits() {
  if (Lwheel < 1000) {// Can be set to a value you don't wish to exceed
    Lwheel = 1000;    // to adjust maximums for your own robot
  }
  if (Lwheel > 2000) {// Can be set to a value you don't wish to exceed
    Lwheel = 2000;    // to adjust maximums for your own robot
  }
  if (Rwheel < 1000) {// Can be set to a value you don't wish to exceed
    Rwheel = 1000;    // to adjust maximums for your own robot
  }
  if (Rwheel > 2000) {// Can be set to a value you don't wish to exceed
    Rwheel = 2000;    // to adjust maximums for your own robot
  }
  // pulseMotors();
}

//********************** autoLimits() ***************************
void autoLimits() {
  if (lSpeed < 1000) {// Can be set to a value you don't wish to exceed
    lSpeed = 1000;    // to adjust maximums for your own robot
  }
  if (lSpeed > 2000) {// Can be set to a value you don't wish to exceed
    lSpeed = 2000;    // to adjust maximums for your own robot
  }
  if (rSpeed < 1000) {// Can be set to a value you don't wish to exceed
    rSpeed = 1000;    // to adjust maximums for your own robot
  }
  if (rSpeed > 2000) {// Can be set to a value you don't wish to exceed
    rSpeed = 2000;    // to adjust maximums for your own robot
  }
}

//*******************   pulseMotors  ***************************
//pulses either mapped or direct signals generated from Mixlimits
//**************************************************************
void pulseMotors() {
  //un-comment the next two line to drive the wheels directly with the MaxLimits Set
  //  R_Servo.writeMicroseconds(Rwheel);
  //  L_Servo.writeMicroseconds(Lwheel);

  //un-comment the next two to map a control range.
  //*** Take the standard range of 1000 to 2000 and frame it to your own minimum and maximum
  //*** for each wheel.
  Rwheel = map(Rwheel, 1000, 2000, 1350, 1650);
  Lwheel = map(Lwheel, 1000, 2000, 1350, 1650);
  R_Servo.writeMicroseconds(Rwheel);
  L_Servo.writeMicroseconds(Lwheel);

  // un-comment this line do display the value being sent to the motors
  //  PrintWheelCalcs(); //REMEMBER: printing values slows reaction times
}

// ============================================================================
// =========================== MOVEMENT COMMANDS ==============================
// ============================================================================

//*******************  DriveServosRC()  ************************
//******  Use the value collected from Ch1 and Ch2  ************
//******  on a single stick to relatively calculate  ***********
//****  speed and direction of two servo driven wheels *********
//**************************************************************
void DriveServosRC()
{
  int buffer = 100;
  int idleCh2 = 1500; // Ch2 = forward speed
  int idleCh1 = 1500; // Ch1 = R/L turns
  int fwdcommand = Ch2 - idleCh2, turncommand = Ch1 - idleCh1;
  int turnFactor = 10, turnMultiply;
  //  int diffNeut = idleZone - neutral;
  rSpeed = neutral;
  lSpeed = neutral;
  turnMultiply = 1; //turnFactor * (abs(plusminus)/250);
  if (fwdcommand > buffer){
    rSpeed = rSpeed + fwdcommand;
    lSpeed = lSpeed + diff*fwdcommand; //- 2 diffNeut
    if (abs(turncommand) > buffer){
      rSpeed = rSpeed - turnMultiply*turncommand;
      lSpeed = lSpeed - turnMultiply*turncommand; //- 2 * diffNeut
    }
  }
  if (fwdcommand < -buffer){
    rSpeed = rSpeed + fwdcommand;
    lSpeed = lSpeed + diff*fwdcommand; //- 2 diffNeut
    if (abs(turncommand) > buffer){
      rSpeed = rSpeed + turnMultiply*turncommand;
      lSpeed = lSpeed + turnMultiply*turncommand; //- 2 * diffNeut
    }
  }
  if (Ch2 == 0)
  {
    rSpeed = neutral;
    lSpeed = neutral;
  }
  autoLimits();
  R_Servo.writeMicroseconds(rSpeed);
  L_Servo.writeMicroseconds(lSpeed);
 
  //  if(((Ch2 - idleZone) < 0) && (Ch2 - idleZone) < (-deadZone)){
  //   rSpeed = -(Ch2 - idleZone) + neutral ;
  //   lSpeed = (neutral +  (Ch2 - idleZone)); //- 2 * diffNeut
  //   R_Servo.writeMicroseconds(rSpeed);
  //   L_Servo.writeMicroseconds(lSpeed);
  //  }
}

//*******************  Drive()  ************************
//******  Use the value collected from Ch1 and Ch2  ************
//******  on a single stick to relatively calculate  ***********
//****  speed and direction of two servo driven wheels *********
//**************************************************************
void DriveArmRC()
{
  int Neutral = 1500;
  int rcDeadZone = 150;
  int servoSpeedOffsetF = 0;
  int servoSpeedOffsetR = 10;

  int Ch2L = 1500;
  if (Ch2 > 2000) {
    Ch2 = 2000;
  }
  if (Ch2 < 1000) {
    Ch2 = 1000;
  }

  if (abs(Ch2-Neutral) < rcDeadZone){
    Ch2 = Neutral;
  }
//  Ch2 = 1500+(Ch2 - 1500)/10;
  if (Ch2 < Neutral){
    Ch2L = Ch2 + servoSpeedOffsetR;
  }
  if (Ch2 > Neutral){
    Ch2L = Ch2 - servoSpeedOffsetF;
  }
  ArmR.writeMicroseconds(Ch2L);
  ArmL.writeMicroseconds(Ch2);

  if (Ch1 > 2000) {
    Ch1 = 2000;
  }
  if (Ch1 < 1000) {
    Ch1 = 1000;
  }
  if (Ch1 > 1300 && Ch1 < 1800) {
    Ch1 = 1500;
  }
  if(Ch1 > 1500 && Ch1 < 2000){
    Ch1 = 1700;
  }
  if(Ch1 < 1500 & Ch1 > 1000){
    Ch1 = 1300;
  }
  ArmM.writeMicroseconds(Ch1);
}

//*****************  Forward(int Dlay)   ***********************
//              Move the robot Slowly Forward
//**************************************************************
void Forward(int Dlay)
{
  // PReviously doublefast
  R_Servo.writeMicroseconds(neutral+1*slow);  // sets the servo position
  L_Servo.writeMicroseconds(neutral-1*slow);   // sets the servo position
  delay(Dlay);
}
//*****************  Reverse(int Dlay)   ***********************
//                   Reverse the robot
//**************************************************************
void Reverse(int Dlay)
{
  R_Servo.writeMicroseconds(neutral-1*slow);  // sets the servo position
  L_Servo.writeMicroseconds(neutral+1*slow);   // sets the servo position
  delay(Dlay);
}
//*****************  stopBot(int Dlay)   ***********************
//                    Stop the robot
//**************************************************************
void stopBot(int Dlay)
{
  R_Servo.writeMicroseconds(neutral);  // sets the servo position
  L_Servo.writeMicroseconds(neutral);   // sets the servo position
  delay(Dlay);
}
//************* TLeftSlow(int rVal,int Dlay) *******************
//        left turn with tapering speed and a duration
//**************************************************************
void TLeftSlow(int Dlay)
{
  lSpeed = neutral;
  rSpeed = neutral + 1*fast;
  R_Servo.writeMicroseconds(rSpeed);  // sets the servo position
  L_Servo.writeMicroseconds(lSpeed);   // sets the servo position
  delay(Dlay);

}
//************* fa(int lVal,int Dlay) *******************
//        Right turn with tapering speed and a duration
//**************************************************************
void TRightSlow(int Dlay)
{
  lSpeed = neutral - 1*fast;
  rSpeed = neutral;
  R_Servo.writeMicroseconds(rSpeed);  // sets the servo position
  L_Servo.writeMicroseconds(lSpeed);   // sets the servo position
  delay(Dlay);
}

void autoDrive(int lSpeed, int rSpeed, int Dlay){
  R_Servo.writeMicroseconds(rSpeed);
  L_Servo.writeMicroseconds(lSpeed);
  delay(Dlay);
}
// ============================================================================
// =========================== RUN DIAGNOSTICS ================================
// ============================================================================

//**********************  PrintRC()  ***************************
//***  Simply print the collected RC values for diagnostics  ***
//**************************************************************
void PrintRC()
{ // print out the values you read in:
  Serial.print("lSpeed =====");
  Serial.println(lSpeed-neutral);
  Serial.print("rSpeed =====");
  Serial.println(diff*(rSpeed-neutral));

  Serial.println(" RC Control Mode ");
  Serial.print("Value Ch1 = ");
  Serial.println(Ch1);
  Serial.print("Value Ch2 = ");
  Serial.println(Ch2);
  Serial.print("Value Ch3 = ");
  Serial.println(Ch3);
  Serial.print("Value Ch4 = ");
  Serial.println(Ch4);
  Serial.print("Control = ");
  Serial.println(Ch5);
  Serial.print("Value Ch6 = ");
  Serial.println(Ch6);
  Serial.println(" ");
  delay(500);
}

//******************** printSensors() **************************
// Print the prox sensor values ***** Slows robot when in use!!!
//**************************************************************
void printSensors() {
  Serial.println("Front Prox Sensor Reads " + (String)proxFront);
  Serial.println("Left Prox Sensor Reads " + (String)proxLeft);
  Serial.println("Right Prox Sensor Reads " + (String)proxRight);
  // if (proxDiff == 0){
  //   Serial.println("Centered");
  // }
  // else if (proxDiff > 0){
  //   Serial.println("Biased to the Left");
  // }
  // else if (proxDiff < 0){
  //   Serial.println("Biased to the Right");
  // }
  Serial.println("cx = "+(String)cx);
}
