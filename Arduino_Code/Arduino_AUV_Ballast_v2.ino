// AUV BALLAST CONTROLLER SOURCE CODE
// AUTHOR     :   JL NAIDOO
// DATE       :   NOVEMBER 2021
// DESCRIPTION:   This software is written for Mechanical and Mechatronic Skripsie Project 478 / 488
// PURPOSE    :   THIS CODE CONTROLS THE RELAYS FOR PUMP_IN, PUMP_OUT AND SOL_VALVE
//                THE SOL_VALVE ALWAYS IS NORMALLY CLOSED - IT OPENS TO ALLOW PUMP_IN AND PUMP_OUT TO WORK AND THEN CLOSES
//                The detailed functioning of this software should be read in conjuction with the Skipsie contained at https://github.com/jevan-n/AUV_Project
// USAGE      :   The sofware is the property of the University of Stellenbosch, South Africa and is intended for reseach and development purposes and should not be used
//                commercially without explicit permission from University
// HW RQUIRED :   To operate the software, the Ballast Tank Relays for PUMP_IN and PUMP_OUT and SOL_VALVE needs to be connected to the Arduino UNO
//                The depth/pressure sensor should be purchased and connected to the AUV [not acquired during this phase of the project]
// OPERATION  :   The Ballast Controller is designed to pump water into or out of the Ballast Tank 
//                Once its rotated to the correct Yaw anlge, the AUV then moves straight along its body-frame X direction, for a distance of the triangulated hypotenues, 
//                at which point is X-Yact = X-Yref.
// PACKAGES   :   Please install PID_v1 package and any plugins that come with the Depth/Pressure Sensor into Arduino IDE to compile and run this code

//****************************************************/ 
/***********Notice and Trouble shooting***************
1.Beware of electric shock and burn out the circuit board.
2.The input voltage of this module is 2.8~5.5V.              
****************************************************/                      
//PID library included created by Brett Beauregard
#include <PID_v1.h>    

double Zref = -5; // reference depth 
double Zact = -4.5; // to be read by the depth/pressure sensor
double Vrate = 0.000067; //The rated flowrat:240L/hour produces 0.000067m3/s
double timeOn = 3;
double pw = 1023.6; //desnity of sea water kg/m3
double V = 0.00077; //volume of ballast tank in m3
double m =7.67; //AUV total mass with ballast tank empty
double mw_in = 0.06827;
double mw_out = -0.06827;
double m_dot = 0;
double TA = 0.125; //0.5*0.25 for AUV top area.

const int SolValvePin = 7;  // SOLENOID VALVE RELAY PIN
const int PumpInPin = 3;    // PUMP_IN RELAY PIN
const int PumpOutPin = 4;   // PUMP_OUT RELAY PIN

//PID Ballast Tank Initialized
double Kp=0.5, Ki=0.1, Kd=0.2;     // modify for optimal performance  
PID myPIDBallast(&Zact, &m_dot, &Zref, Kp, Ki, Kd, DIRECT); 

void setup() {
  Serial.begin(9600);   
  pinMode(SolValvePin, OUTPUT);
  pinMode(PumpInPin, OUTPUT);
  pinMode(PumpOutPin, OUTPUT);
  digitalWrite(SolValvePin, LOW); // Close the solenoid valve
  digitalWrite(PumpInPin,LOW); //Stop the Water In Pump
  digitalWrite(PumpOutPin,LOW); // Stop the Water Out pump


  myPIDBallast.SetMode(AUTOMATIC);   //set PID in Auto mode
  myPIDBallast.SetSampleTime(500);  // refresh rate of PID controller
  //myPIDBallast.SetTunings(Kpy, Kiy, Kdy);
  myPIDBallast.SetOutputLimits(-0.06827,0.06827); // this is the MAX PWM value to pump water, here change in value reflect change in speed of motor.

  Serial.println("Starting the Ballast controller ...");
  
} 

void loop() {
//    digitalWrite(SolValvePin, HIGH); // Close the solenoid valve
//    delay(5000);
//    digitalWrite(SolValvePin, LOW); // Close the solenoid valve
//
//    delay(5000); //Allows for solenoid valve to completely close

    while(abs(Zact - Zref) > 0.2) // tolerance level for Zact and Zref is 0.2 meters difference, if within range exit loop
    {
        // Read the depth / pressure sensor once its purchased and connected
            // write code for reading sensor here
            // if sensor returns a pressure reading, convert pressure to depth from surface in meters
            // set Zact = depthsensor_reading()
                  
        myPIDBallast.Compute();

        if(Zref<Zact)
        {
          PumpWaterIn(3000);
          mw_in = mw_in*timeOn;
          Zact = Zact - ((mw_in)/(pw*TA));
        }

        if(Zref>Zact)
        {
          PumpWaterOut(3000);
          mw_out = mw_out*timeOn;
          Zact = Zact + (mw_out/(pw*TA));
        }
        
        Serial.print("Zact: ");
        Serial.print(Zact);
        Serial.print(' ');
        Serial.print("MassFlowRate: ");
        Serial.print(-m_dot);
        Serial.print(' ');
        Serial.print("Zref: ");
        Serial.print(Zref);
        Serial.print('\n');
   }
   Serial.print("AUV depth Zact is stabilised to Zref");
   Serial.print('\n');
   delay(2000);
   exit(0);

}


void PumpWaterIn (int duration) {
  timeOn = duration/1000;
  digitalWrite(SolValvePin, HIGH); // Open the solenoid valve
  delay(5000); // allow the valve to open fully
  digitalWrite(PumpInPin, HIGH); // Start the PUMP_IN
  delay (duration); // PUMP_IN water for time=duration
  digitalWrite(PumpInPin,LOW); //Stop the PUMP_IN
  digitalWrite(SolValvePin, LOW); // Close the solenoid valve
  delay(5000);
}

void PumpWaterOut (int duration) {
  timeOn = duration/1000;
  digitalWrite(SolValvePin, HIGH); // Open the solenoid valve
  delay(5000); // allow the valve to open fully
  digitalWrite(PumpOutPin, HIGH); // Start the PUMP_OUT
  delay (duration); // PUMP_IN water for time=duration
  digitalWrite(PumpOutPin,LOW); //Stop the PUMP_IN
  digitalWrite(SolValvePin, LOW); // Close the solenoid valve
  delay(5000);
}
