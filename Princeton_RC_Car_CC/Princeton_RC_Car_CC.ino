#include <Servo.h>
#include <Pixy2.h>
//-----------------------------------------------------------------

// Define Servo Variables
Servo R_Servo;  // Servo DC Motor Driver (Designed for RC cars)
Servo L_Servo;  // Servo DC Motor Driver (Designed for RC cars)
int Rwheel;               // Variable to hold R wheel speed
int Lwheel;               // Variable to hold L wheel speed
int LeftMaxIn;        //Variable to hold Max Data In
int RightMaxIn;      //Variable to hold Max Data In
int CalcHold;        //Variable to remp hold calculations for steering stick corections
int rSpeed, lSpeed;       // Variables to hold autonomous speed changes for each wheel

// Define RC Variables
int Ch1,Ch2,Ch3,Ch4,Ch5,Ch6;
const int LED = 13;       // Onboard LED location

// Define Pixy Variables
Pixy2 pixy;
int signature, x, y, width, height;
int cont = 0;
float cx, cy, area;
float maxArea = 315;
uint16_t blocks;

// Prox Sensor Pins
const int proxFrontPin = A0;  
const int proxLeftPin = A1;  
const int proxRightPin = A2;  
// Prox Sensor Values
int proxFront;
int proxLeft;
int proxRight;
int proxDiff;

// CALIBRATE
float deadZone = 10;
int fast = 120, slow = 80, neutral = 1500;

//**************************************************************
//*****************  Setup  ************************************
//**************************************************************
void setup() {
  // Set the pins that the transmitter will be connected to all to input
  pinMode(4, INPUT); //I connected this to Chan1 of the Receiver
  pinMode(9, INPUT); //I connected this to Chan2 of the Receiver
  pinMode(8, INPUT); //I connected this to Chan3 of the Receiver
  pinMode(7, INPUT); //I connected this to Chan4 of the Receiver
  pinMode(6, INPUT); //I connected this to Chan5 of the Receiver
  pinMode(5, INPUT); //I connected this to Chan6 of the Receiver
  pinMode(LED, OUTPUT);//Onboard LED to output for diagnostics
  
// Attach Speed controller that acts like a servo to the board
  R_Servo.attach(3);
  L_Servo.attach(11);
  rSpeed = neutral - slow;
  lSpeed = neutral + slow;
  
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
void loop()
{
  //  TestWheels();
  //  fowardSlow();
  //  DriveServosRC(); // Drive Motors under RC control
  Ch5Check(); // brightens and darkens 2 LEDs with proportional stick control
  PrintRC(); //Print Values for RC Mode Diagnostics

}

//**********************  Ch5Check()  **************************
//********************** Test Channel 5   **********************
//**************************************************************
void Ch5Check() {
  Ch5 = pulseIn(6, HIGH); // Capture pulse width on Channel 5
  if (Ch5 > 1600) {
    digitalWrite(LED, HIGH);
    autonomous();
  }
  else {
    Ch1 = pulseIn(4, HIGH); // Capture pulse width on Channel 1
    Ch2 = pulseIn(9, HIGH); // Capture pulse width on Channel 2
    Ch3 = pulseIn(8, HIGH);  // Capture pulse width on Channel 3
    Ch4 = pulseIn(7, HIGH);  // Capture pulse width on Channel 4
    digitalWrite(LED, LOW);
    DriveServosRC();
  }
}

//**********************  autoMode()  **************************
//********************** Autonomous Mode   **********************
//**************************************************************
void autonomous() {
  driveDx();
  centerTot();
  // Serial.println("dddd");
  Ch5Check();
}

//**********************  Pixy Tracking  **************************
//********************** Autonomous Mode   **********************
//**************************************************************
// Tracking Regime
float pixyTrack() {
  Serial.println("dx2");
    static int i = 0;
    int j;
    char buf[32];
    // grab blocks!
    blocks = 0;
    blocks = pixy.ccc.getBlocks();

    // Get Height & Width of Blocks
    if (blocks)
    {
      signature = pixy.ccc.blocks[0].m_signature;
      height = pixy.ccc.blocks[0].m_height;
      width = pixy.ccc.blocks[0].m_width;
      x = pixy.ccc.blocks[0].m_x;
      y = pixy.ccc.blocks[0].m_y;
      cx = (x + (width / 2));
      cy = (x + (width / 2));
      cx = mapfloat(cx, 0, 320, -1, 1);
      cy = mapfloat(cy, 0, 200, 1, -1);
      area = width * height;
    }
  else {
    cont += 1;
    if (cont == 100) {
      Serial.println("dx3");
      cont = 0;
      cx = 0;
    }
    }
    Serial.print("cx = ");
    Serial.println(cx);
    return cx;
}

void driveDx()
{
  float dx = pixyTrack();
  // Serial.println(dx);
  if (dx > -deadZone && dx < deadZone){
    Forward(1);
  }

  if (dx < 0) {
    rSpeed = rSpeed - 5;
    setLimits();
    TLeftSlow(rSpeed,1);
  }
  else if (dx > 0) {
    lSpeed = lSpeed + 5;
    setLimits();
    TLeftSlow(lSpeed,1);
  }
}

float mapfloat(long x, long in_min, long in_max, long out_min, long out_max){
  return (float)(x-in_min)*(out_max - out_min) / (float)(in_max-in_min) + out_min;
}

// ******************** centerTot() ***************************
// *********** use proximity sensors to center Tot *************
// *************************************************************
void centerTot()
{
  int front = analogRead(proxFrontPin);
  int left = analogRead(proxLeftPin); 
  int right = analogRead(proxRightPin);
  
  // take 5 samples and average
  for (int i = 0; i <= 4; i++) {
    front = front + analogRead(proxFrontPin);
    left = left + analogRead(proxLeftPin);
    right = right + analogRead(proxRightPin);
  }
  front = front / 5;
  left = left / 5;
  right = right / 5;

  proxFront = front;
  proxDiff = left - right;
  // NOTE:
  // proxDiff == 0 , WE ARE CENTERED
  // proxDiff > 0, WE ARE BIASED LEFT >> GO RIGHT
  // proxDiff < 0, WE ARE BIASED RIGHT >> GO LEFT
  // proxLeft = read[1];
  // proxRight = read[2];

  // EMERGENCY STOP
  if (proxFront >= 400) { //changed from 500
    stopBot(100);
  }

  // REALIGNMENT ALGORITHM
  if (proxDiff >= -100 || proxDiff <= 100){
    Forward(1);
  }
  else if (proxDiff > 100){
    lSpeed = lSpeed + 5;
    setLimits();
    TRightSlow(lSpeed,1);
  }
  else if (proxDiff < -100){
    rSpeed = rSpeed - 5;
    setLimits();
    TLeftSlow(rSpeed,1);
  }
}

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
  pulseMotors();
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

//*******************  DriveServosRC()  ************************
//******  Use the value collected from Ch1 and Ch2  ************
//******  on a single stick to relatively calculate  ***********
//****  speed and direction of two servo driven wheels *********
//**************************************************************
void DriveServosRC()
{
  if (Ch2 <= 1500) {
    Lwheel = Ch1 + Ch2 - 1500;
    Rwheel = Ch1 - Ch2 + 1500;
    setLimits();
  }
  if (Ch2 > 1500) {
    int Ch1_mod = map(Ch1, 1000, 2000, 2000, 1000); // Invert the Ch1 axis to keep the math similar
    Lwheel = Ch1_mod + Ch2 - 1500;
    Rwheel = Ch1_mod - Ch2 + 1500;
    setLimits();
  }
}
//*****************  Forward(int Dlay)   ***********************
//              Move the robot Slowly Forward
//**************************************************************
void Forward(int Dlay)
{
  R_Servo.writeMicroseconds(neutral-fast);  // sets the servo position
  L_Servo.writeMicroseconds(neutral+fast);   // sets the servo position
  delay(Dlay);
}
//*****************  Reverse(int Dlay)   ***********************
//                   Reverse the robot
//**************************************************************
void Reverse(int Dlay)
{
  R_Servo.writeMicroseconds(neutral+slow);  // sets the servo position
  L_Servo.writeMicroseconds(neutral-slow);   // sets the servo position
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
void TLeftSlow(int rVal, int Dlay)
{
  R_Servo.writeMicroseconds(rVal);  // sets the servo position
  L_Servo.writeMicroseconds(neutral+slow);   // sets the servo position
  delay(Dlay);
}
//************* TRightSlow(int lVal,int Dlay) *******************
//        Right turn with tapering speed and a duration
//**************************************************************
void TRightSlow(int lVal, int Dlay)
{
  R_Servo.writeMicroseconds(neutral-slow);  // sets the servo position
  L_Servo.writeMicroseconds(lVal);   // sets the servo position
  delay(Dlay);
}

//**********************  PrintRC()  ***************************
//***  Simply print the collected RC values for diagnostics  ***
//**************************************************************
void PrintRC()
{ // print out the values you read in:
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
  // Serial.println("Left Prox Sensor Reads " + (String)proxLeft);
  // Serial.println("Right Prox Sensor Reads " + (String)proxRight);
  if (proxDiff == 0){
    Serial.println("Centered");
  }
  else if (proxDiff > 0){
    Serial.println("Biased to the Left");
  }
  else if (proxDiff < 0){
    Serial.println("Biased to the Right");
  }
  Serial.println(" ");
  delay(100);
}
