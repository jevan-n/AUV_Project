// AUV CONTROL SYSTEMS SOURCE CODE
// AUTHOR     :   JL NAIDOO
// DATE       :   NOVEMBER 2021
// DESCRIPTION:   This software is written for Mechanical and Mechatronic Skripsie Project 478 / 488
// PURPOSE    :   This control algorithm is used to move the AUV autonomously to a target reference position.
//                The detailed functioning of this software should be read in conjuction with the Skipsie contained at https://github.com/jevan-n/AUV_Project
// USAGE      :   The sofware is the property of the University of Stellenbosch, South Africa and is intended for reseach and development purposes and should not be used
//                commercially without explicit permission from University
// HW RQUIRED :   To operate the software, an IMU (BNO055) needs to be connected to the Arduino UNO and rotated manually, if not connected to the real AUV.
// OPERATION  :   On processing XYZ target references, the controller initially computes the Yaw-ref angle for which the AUV needs to be rotated. 
//                Once its rotated to the correct Yaw anlge, the AUV then moves straight along its body-frame X direction, for a distance of the triangulated hypotenues, 
//                at which point is X-Yact = X-Yref.
// PACKAGES   :   Please install Adafruit_Sensor, Adafruit_BNO055, PID_v1 and ESC packages into Arduino IDE to compile and run this code


// This library allows you to communicate with I2C devices. 
#include <Wire.h> 
//Adafruit Sensor library included created by Adafruit
#include <Adafruit_Sensor.h> 
//Adafruit BNO055 library included created by Adafruit
#include <Adafruit_BNO055.h> 
//IMU maths library included created by Adafruit
#include <utility/imumaths.h> 
//PID library included created by Brett Beauregard
#include <PID_v1.h>    
//ESC library included created by Eric Nantel
#include <ESC.h>       

//Thruster pin setup
#define ESC_L (5)                 // Arduino Nano Pin for left thruster ESC
#define ESC_R (6)                 // Arduino Nano Pin for right thruster ESC
#define ESC_D (9)                 // Arduino Nano Pin for depth thruster ESC
//Max, Min and Stop PWM signals setup
#define SPEED_MIN (1000)          // Set the Minimum Speed in microseconds - 5% duty cycle
#define SPEED_MAX (2000)          // Set the Minimum Speed in microseconds - 10% duty cycle
#define SPEED_STOP (1500)         // The STOP setting in mircoseconds for bi-directional ESC - 7.5% duty cycle

ESC ESC_Depth  (ESC_D, SPEED_MIN, SPEED_MAX, 500);   // ESC_Name (ESC PIN, Minimum Value, Maximum Value, Default Speed, Arm Value)
ESC ESC_Left  (ESC_L, SPEED_MIN, SPEED_MAX, 500);   // ESC_Name (ESC PIN, Minimum Value, Maximum Value, Default Speed, Arm Value)
ESC ESC_Right (ESC_R, SPEED_MIN, SPEED_MAX, 500);   // ESC_Name (ESC PIN, Minimum Value, Maximum Value, Default Speed, Arm Value)

//User sets target coordinates for AUV
double Xref = 4; //X target position
double Yref=3; //Y target position
double Zref = -2; //Z target position
double Yawref = 0; //Yaw target position based on set X&Y target coordinates.

double Xact = 0; //X actual position
double Zact = 0; //Z actual position
double YawAct=0;//Yaw actual position
double Xacc = 0; //X acceleration
double Zacc = 0; //Z acceleration


//Outputs for controls systems initialized
double U1=0; // Force in Newtons from myPIDx output
double U2 = 0; // Force in Newton meters from myPIDy output
double U3 = 0; // Force in Newtons from myPIDz output

//Other variables
double m =7.65; //mass of AUV
double g =9.81; //gravitational acceleration

//Timer variables initialized
double milliOld = 0;
double milliNew = 0;
double dt = 0;
double tick;
double t = 0;

//Speed variables initialized
int Speed = 0;
int Speed1 = 0;
int Speed2 = 0;
char Direction; //Direction variable intialized


// Setup Environmental Drag Force variables (body frame X-direction)
double Fdx = 0; // Drag force along X for each loop iteration
double Vxi = 0; // initial velocity in X direction for each loop iteration
double Vxf = 0; // final velocity in X diection for each loop iteration
double Dx = 0; // distance traveled in X direction for each loop iteration
double p = 1023.6; //density of water
double cd = 0.85; //drag coefficient
double FAx = 0.0375; // 0.25*0.15 Frontal Area of the AUV

// Setup Environmental Drag Force variables (body frame Z-direction)
double FDz = 0; // Drag force along Z for each loop iteration
double Vzi = 0; // initial velocity in Z direction for each loop iteration
double Vzf = 0; // final velocity in Z diection for each loop iteration
double Dz = 0; // distance traveled in Z direction for each loop iteration
double FAz = 0.125; // 0.5*0.25 Frontal Area of the AUV (top view Z)

//PID x direction
double Kpx=0.8, Kix=0.15, Kdx=0.9;             // modify for optimal performance  
PID myPIDx(&Xact, &U1, &Xref, Kpx, Kix, Kdx, DIRECT);  


//PID Yaw rotation
double Kpy=0.5, Kiy=0, Kdy=0.5;             // modify for optimal performance  
PID myPIDy(&YawAct, &U2, &Yawref, Kpy, Kiy, Kdy, DIRECT);  


//PID z direction
double Kpz=0.8, Kiz=0.15, Kdz=0.9;             // modify for optimal performance
PID myPIDz(&Zact, &U3, &Zref, Kpz, Kiz, Kdz, DIRECT);  



Adafruit_BNO055 bno = Adafruit_BNO055(55); //Arms IMU BNO055

void setup(void)
{
  Serial.begin(115200); //Baudrate set for serial monitor communication
  Serial.print("XE-ref = ");
  Serial.print (Xref);
  Serial.print(" ");
  Serial.print("YE-ref = ");
  Serial.print (Yref);
  Serial.print(" ");
  Serial.print("ZE-ref = ");
  Serial.print (Zref);
  Serial.print(" ");
  Serial.print('\n');
  
  
  Serial.println("Orientation Sensor Test"); Serial.println("");

  // Initialise the sensor 
  if (!bno.begin())
  {
    // There was a problem detecting the BNO055 ... check your connections 
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  delay(1000);//delay a moment so that IMU BNO055 can complete collaboration

  bno.setExtCrystalUse(true);

  // ARM THE THRUSTER ESC'S TO ACCEPT THRUSTER COMMANDS
  ESC_Left.arm();
  ESC_Right.arm();
  delay(2500); //delay a moment so that thrusters can complete arming
  
  //PID Yaw Setup
  myPIDy.SetMode(AUTOMATIC);   //set PID in Auto mode
  myPIDy.SetSampleTime(500);  // refresh rate of PID controller
  //myPIDy.SetTunings(Kpy, Kiy, Kdy);
  myPIDy.SetOutputLimits(-11.25,11.25); // this is the MAX PWM value to move thruster, here change in value reflect change in speed of motor.
  
  //PID X Setup
  myPIDx.SetMode(AUTOMATIC);   //set PID in Auto mode
  myPIDx.SetSampleTime(2000);  // refresh rate of PID controller
  //myPIDx.SetTunings(Kpx, Kix, Kdx);
  myPIDx.SetOutputLimits(-90,90); // this is the MAX PWM value to move thruster, here change in value reflect change in speed of motor.

  //PID Z Setup
  myPIDz.SetMode(AUTOMATIC);   //set PID in Auto mode
  myPIDz.SetSampleTime(2000);  // refresh rate of PID controller
  //myPIDz.SetTunings(Kpz, Kiz, Kdz);
  myPIDz.SetOutputLimits(-45,45); // this is the MAX PWM value to move thruster, here change in value reflect change in speed of motor.

  Xact = 0; // set the initial input into the PIDx controller as 0;
  YawAct = 0; // set the initial input into the PIDy controller as 0;
  Zact = 0; // set the initial input into the PIDz controller as 0;
  
  //if Yref and Xref is not 0, compute Yaw angle
  if((Yref !=0)&&(Xref!=0)){ 
  Yawref= atan(Yref/Xref)*(180/3.14159265358979); //convert to degrees
  Direction = 'L'; //set direction for most efficient turn
  
  if((Yref<0)&&(Xref>0)){ //If the angle lies in the 4th quadrant (since +X and -Y)
    Yawref = Yawref+360; //Converts angle to lie in the 4th quadrant
    Direction = 'R'; //set direction for most efficient turn
  }
  
  if((Yref>0)&&(Xref<0)){ //If the angle lies in the 2nd quadrant (since -X and +Y)
    Yawref = abs(Yawref)+90; //Converts angle to lie in the 2nd quadrant
    Direction = 'R'; //set direction for most efficient turn
  }
  
  if((Yref<0)&&(Xref<0)){ //If the angle lies in the 3rd quadrant (since -X and -Y)
    Yawref = Yawref+180; //Converts angle to lie in the 3rd quadrant
    Direction = 'L'; //set direction for most efficient turn
  }
  Xref = sqrt((Xref*Xref) + (Yref*Yref)); //Net distance is set as new Xref so AUVs route to target is simplified.
  }

  //if Yref is not 0 but Xref = 0, compute Yaw angle
  if((Yref !=0)&&(Xref==0)){ 
   if(Yref>0){ //If Yref is positive
    Yawref = 90; //Yawref set to 90 degrees. So AUV will face forward on positive Y axis 
    Direction = 'L'; //set direction for most efficient turn
   }
   if(Yref<0){ //If Yref is negative
    Yawref = 270; //Yawref set to 270 degrees. So AUV will face forward on negative Y axis 
    Direction = 'R'; //set direction for most efficient turn
   }
   Xref = Yref;  //Xref is set to Yref. After rotation AUV will move to Yref target.
  }

  Serial.print("Rotation Angle [YAW ref] = ");
  Serial.print (Yawref);
  Serial.print(" ");
  Serial.print("Triangulated XY Target [X-Y ref] = ");
  Serial.print (Xref);
  Serial.print(" ");
  Serial.print('\n');

  milliNew = millis(); // get the current time (in milliseconds) elasped since startup
  
}





void loop(void)
{
  //While loop required to ensure AUV rotates first if need be. As this is the most effective way move to target coordinates.
  while(YawAct<=Yawref){
  // Get a new sensor event
  sensors_event_t event;
  bno.getEvent(&event);
 
  delay(100);

  YawAct =  abs(event.orientation.x-360);; //Degrees of rotation read from IMU. If robot starts to rotate left, yaw angle will increase from 0 to 360.

  if(YawAct == 360){ //Ensures that 0 and 360 degrees are the same
    YawAct = 0;
  }
  myPIDy.Compute();   //Compute PID Yaw output ... this is a force in Newtons*meters
  
  // Compute the PWM signals for the thrusters
  ThrusterSpeedYaw(U2); 
  //Sends direction and PWM signals to thrusters.
  thrusterDirection(Direction,Speed);

  //Checks variables to ensure PID control system is working
  Serial.print("Yawref: ");
  Serial.print(Yawref); 
  Serial.print(' ');
  Serial.print("U2: ");
  Serial.print(U2); 
  Serial.print(' ');
//  Serial.print("t: ");
//  Serial.print(t); 
//  Serial.print(' ');
//  Serial.print("SpeedT1: ");
//  Serial.print(Speed1);
//  Serial.print(' ');
//  Serial.print("SpeedT2: ");
//  Serial.print(Speed2);
//  Serial.print(' ');
  Serial.print("YawAct: ");
  Serial.println(YawAct);
  Serial.print('\n');

  delay(2000); // just delay for a moment to all the motors to receive new PWM signals
  milliNew = millis();
  }
  
  // Calculate the elasped time for each loop cylce
  milliOld = milliNew;
  milliNew = millis();
  dt = milliNew - milliOld;
  t = (dt/1000); // t = elapsed time in seconds

  Direction = 'F'; //Sets Direction to Forward
  
  //Compute PIDx output ... this is a force in Newtons
  myPIDx.Compute();
  
  // Compute the PWM signals for the thrusters
  ThrusterSpeedStraight(U1); 
  //Sends direction and PWM signals to thrusters.
  thrusterDirection(Direction,Speed);
  
  Xacc = (U1-Fdx)/m; //compute the acceleration of the AUV using Newtons Law F = m.a (note that Fdx = drag force in body frame X direction)
  
  // calculate the distance travelled for the given acceleration and elapsed time
  Xact = Xact + 0.5*(Xacc)*t*t; // dx = (vi * t) + (0.5*Xacc*t^2) ... vi = 0 ... initial velocity at start

  // View results on serial monitor to see Xact move towards the setpoint = Xref
  // Also view how the thruster PWM (SPEED) moves from 1500 (STOP) towards 1900 full foward 
  // and then below 1500 for reverse when overshoot the target
  // when target is reached, thrusters will settle at 1500 which is the stop PWM signal
  // Kp, Ki, & Kd can be adjusted and time delay below to improve the PID response
  

  Serial.print("Xref: ");
  Serial.print(Xref); 
  Serial.print(' ');
  Serial.print("U1(N): ");
  Serial.print(U1); 
  Serial.print(' ');
  Serial.print("Xacc: ");
  Serial.print(Xacc);
  Serial.print(' ');
  Serial.print("t: ");
  Serial.print(t); 
  Serial.print(' ');
  Serial.print("PWM_Speed: ");
  Serial.print(Speed);
  Serial.print(' ');
  Serial.print("Xact: ");
  Serial.print(Xact);
  Serial.print('\n');

  delay(2000); // just delay for a moment to all the motors to receive new PWM signals
  
 
 if((abs(Xact - Xref)<0.05) && (Zref != 0)){
  // Calculate the elasped time for each loop cylce
  milliOld = milliNew;
  milliNew = millis();
  dt = milliNew - milliOld;
  t = (dt/1000); // t = elapsed time in seconds
  
  Direction = 'Z';
  
  // Compute PIDz output ... this is a force in Newtons
  // Get the value of U3 from myPIDz
  // U3 = outputx; // force for the thrusters = output of PID
  myPIDz.Compute();
  
  
  // Compute the PWM signals for the thrusters & send to thrusters
  ThrusterSpeedDepth(U3); 
  thrusterDirection(Direction,Speed);
  
  //compute the acceleration of the AUV using Newtons Law F = m . a
  //the nett force forward thruster force U3(N) - FDz(N) of the drag force ... initial drag = 0
  Zacc = (U3-FDz)/m; 

  // calculate the distance travelled for the given acceleration and elapsed time
  Zact = Zact + 0.5*(Zacc)*t*t; // Dz = (vi * t) + (0.5*Zacc*t^2) ... vi = 0 ... initial velocity at start

  //set the input into PID to be the actual distance travelled ... PID will compare to setpoint and re-compute on next loop


  // View results on serial monitor to see Zact move towards the setpoint = Zref
  // Also view how the thruster PWM (SPEED) moves from 1500 (STOP) towards 1900 full foward 
  // and then below 1500 for reverse when overshoot the target
  // when target is reached, thrusters will settle at 1500 which is the stop PWM signal
  // Kp, Ki, & Kd can be adjusted and time delay below to improve the PID response
  
  Serial.print("Zref: ");
  Serial.print(Zref); 
  Serial.print(' ');
  Serial.print("U3(N): ");
  Serial.print(U3); 
  Serial.print(' ');
  Serial.print("Zacc: ");
  Serial.print(Zacc);
  Serial.print(' ');
  Serial.print("t: ");
  Serial.print(t); 
  Serial.print(' ');
  Serial.print("PWM_Speed: ");
  Serial.print(Speed);
  Serial.print(' ');
  Serial.print("Zact: ");
  Serial.print(Zact);
  Serial.print('\n');

  delay(2000); // just delay for a moment to all the motors to receive new PWM signals
  
 }
  
}

//Function to calculate thruster rotation PWM signal
void ThrusterSpeedYaw(int U2){
  
  Speed1 = map(U2,-11.25,11.25, 1100, 1900);
  //Account for upper boundary of thruster PWM deadzone
  if((Speed1>1500) && (Speed1<1530)){ 
    Speed1 =1535;
  }
  //Account for lower boundary of thruster PWM deadzone
  if((Speed1<1500) && (Speed1>1470)){
    Speed1 =1465;
  }
  Speed2 = Speed1-(2*(Speed1-1500));
  
}

//Function to calculate thruster forward/reverse PWM signal
void ThrusterSpeedStraight(int U1){

  Speed = map(U1, -90.0, 90.0, 1100, 1900);
  //Account for upper boundary of thruster PWM deadzone
  if((Speed>1500) && (Speed<1530)){
    Speed =1535;
  }
  //Account for lower boundary of thruster PWM deadzone
  if((Speed<1500) && (Speed>1470)){
    Speed =1465;
  }
}

void ThrusterSpeedDepth(int U3){

  Speed = map(U3, -45.0, 45.0, 1100, 1900);
  //Account for upper boundary of thruster PWM deadzone
  if((Speed>1500) && (Speed<1530)){
    Speed =1535;
  }
  //Account for lower boundary of thruster PWM deadzone
  if((Speed<1500) && (Speed>1470)){
    Speed =1465;
  }
}

//ThrusterFunction
void thrusterDirection(char Direction, int Speed) {
  switch (Direction) {
    case 'F':
      ESC_Right.speed(Speed);
      ESC_Left.speed(Speed);

      break;

    case 'Z':
      ESC_Depth.speed(Speed);
    
      break;

    case 'R':
      ESC_Right.speed(Speed2);
      ESC_Left.speed(Speed1);
      

      break;

    case 'L':
    ESC_Right.speed(Speed1);
    ESC_Left.speed(Speed2);

    default:
      break;
  }
}
