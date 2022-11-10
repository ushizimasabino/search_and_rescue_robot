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
float deadZone = 0;

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

void setup()
{
// Attach Speed controller that acts like a servo to the board  
  R_Servo.attach(6); //Pin 1
  L_Servo.attach(5); //Pin 0
  pixy.init();
 
  Serial.begin(9600);
  Serial.print("Starting...\n");
}

void loop() {
  // put your main code here, to run repeatedly:
  Ch5Check();
}

// Determines whether or not in Auto mode
void Ch5Check() {
  Ch5 = pulseIn(11, HIGH, 21000); // Capture pulse width on Channel 5

  if (Ch5 > 1600) {
    digitalWrite(LED, HIGH);
    Serial.println("Naenae2");
    //autonomous();
  }
  else {
    Ch1 = pulseIn(7, HIGH, 21000); // Capture pulse width on Channel 1
    Ch2 = pulseIn(8, HIGH, 21000); // Capture pulse width on Channel 2
    Ch3 = pulseIn(9, HIGH, 21000);  // Capture pulse width on Channel 3
    Ch4 = pulseIn(10, HIGH, 21000);  // Capture pulse width on Channel 4
    digitalWrite(LED, LOW);
    DriveServosRC();
    Serial.println("fuck");
    PrintRC();
  }
}

void DriveServosRC()
{
  if (Ch3 <= 1500) {
    Lwheel = Ch1 + Ch3 - 1500;
    Rwheel = Ch1 - Ch3 + 1500;
    SetLimits();
  }
  if (Ch3 > 1500) {
    int Ch1_mod = map(Ch1, 1000, 2000, 2000, 1000); // Invert the Ch1 axis to keep the math similar
    Lwheel = Ch1_mod + Ch3 - 1500;
    Rwheel = Ch1_mod - Ch3 + 1500;
    SetLimits();
  }
}

//********************** MixLimits() ***************************
//*******  Make sure values never exceed ranges  ***************
//******  For most all servos and like controlers  *************
//****   control must fall between 1000uS and 2000uS  **********
//**************************************************************
void SetLimits() {
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

//**********************  autoMode()  **************************
//********************** Autonomous Mode   **********************
//**************************************************************
void autonomous() {
  Serial.println("Naenae");
  driveDx();
 
}

void driveDx()
{
  float dx = pixyTrack();
  Serial.println(dx);
  if (dx > -deadZone && dx < deadZone){
    // no turn
  }

  if (dx < 0) {
    R_Servo.writeMicroseconds(2000);
    L_Servo.writeMicroseconds(2000);
  }
  else if (dx > 0) {
    R_Servo.writeMicroseconds(1000);
    L_Servo.writeMicroseconds(1000);
  }
}

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

  void TLeftSlow(int rVal, int Dlay)
{
  R_Servo.writeMicroseconds(rVal);  // sets the servo position
  L_Servo.writeMicroseconds(1600);   // sets the servo position
  delay(Dlay);
}

  float mapfloat(long x, long in_min, long in_max, long out_min, long out_max){
    return (float)(x-in_min)*(out_max - out_min) / (float)(in_max-in_min) + out_min;
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
  delay(1000);
}