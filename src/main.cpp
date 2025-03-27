// Software for PatRyAut! Let's win the 2025 PBC Regatta!

#include <Arduino.h>
#include <Wire.h>
#include "QMC5883LCompass.h"
#include <math.h>
#include <TinyGPS++.h>
#include "CRSF.h"





#define PWM_TOP 39999
#define MIN_PULSE_COUNTS 2000  // 1 ms
#define MAX_PULSE_COUNTS 4000  // 2 ms
#define Main_Loop_Delay 200



// State Machine States:
enum State {INITIAL, REMOTE_CONTROL, GETTING_POS_HEADING, COMPUTING_TRAJECTORY, FOLLOWING_HEADING};
State Boat_State = INITIAL;
State Previous_State = INITIAL;

//------------------------ Globals defined here --------------------------------
float Current_Latitude = 0;
float Current_Longitude = 0;
int16_t Current_Heading = 0;
float Current_Distance_to_Waypoint = 100;

float Hardcoded_Target_Latitude[4] = {32.82922, 32.82893, 32.82877, 32.82867};
float Hardcoded_Target_Longitude[4] = {-117.22954, -117.22950, -117.23007, -117.22963}; // This is an array of 5 waypoint Longitudes, update the '5' depending on the number of waypoints
float Target_Latitude[4] = {32.82922, 32.82893, 32.82877, 32.82867};
float Target_Longitude[4] = {-117.22954, -117.22950, -117.23007, -117.22963};
int16_t Target_Heading = 0; // This is the current target heading for the boat
float Normalized_Heading_Error = 0;
int Waypoint_Index = 0; // This increments each time a new waypoint is achieved.

float Average_Speed = .3; // This is the average power to the boat. (.3 = 30%)
float Turn_Gain = .5; // This is the turn gain. Be careful.
float Waypoint_Radius = 6; // Unclear how this maps yet.
int Max_Forward = 100;
int Max_Reverse = -100;

int loops = 0; //ignore this, it's bad form to put it here but I need it.
int PID_Laps_To_Recalculate = 5; // Number of PID loops before recalculating the target heading


float Left_Motor_Effort = 0;
float Right_Motor_Effort = 0;


// Declare function prototype
void UpdatePositionandHeading();
float getHeadingToWPT(float current_lat, float current_long, float way_lat, float way_long);
float ComputeNormalizedHeadingError(float Current_Heading, float Desired_Heading);




QMC5883LCompass compass;
TinyGPSPlus gps3;
TinyGPSPlus gps2;
CRSF crsf;  // Correct instantiation—no parentheses!



void setup() {

  // Set Pins Input/Outputs etc.
  compass.setCalibrationOffsets(-216.00, 100.00, 376.00);
  compass.setCalibrationScales(1.00, 1.00, 0.95);

  Serial.begin(9600);
  while (!Serial) { /* wait for serial port to connect (needed for some boards) */ }

  // Initialize I2C
  Wire.begin();
  compass.init();
  Serial.println("Compass Initialized");

  // ----- Configure PWM pins as outputs -----
  pinMode(11, OUTPUT);  // OC1A
  pinMode(12, OUTPUT);  // OC1B

  // ----- Timer1 Configuration for 50 Hz -----
  // 1) Set PWM top value for 50 Hz
  ICR1 = PWM_TOP;

  TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM11);  // non-inverting on A & B
  TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11);

  // ----- Initialize outputs to 1.5 ms (neutral) -----
  OCR1A = 3000;  // 1.5 ms => 1500 us => 3000 counts
  OCR1B = 3000;  // same for channel B


  // Start GPS serial communication at baud rate 9600
  Serial3.begin(9600);
  Serial2.begin(9600);
  delay(1000);

  Serial2.println("$PMTK220,250*29");
  Serial.println("GNSS Initialized");

  crsf.Begin();
  Serial.println("CRSF Receiver Initialized");


  Serial.println(F("System Initialization Complete"));
}

void loop() {
  switch (Boat_State) {

    case INITIAL:
    // Initial Actions Here
    // Boat_State = GETTING_POS_HEADING;
    // Previous_State = INITIAL;

    Result_t result = crsf.GetCrsfPacket();
    Serial.print("Got Packet!");
  
    if (result == PACKET_READY) {
      crsf.UpdateChannels();
      Serial.print("Channels: ");
      for (uint8_t i = 0; i < CRSF_MAX_CHANNEL; i++) {
        Serial.print(crsf.channels[i]);
        Serial.print(" ");
      }
      Serial.println();
      if (crsf.channels[1] < 1600){
        Serial.print("HERE");
        Boat_State = REMOTE_CONTROL;
        Previous_State = INITIAL;
        break;
      }
      else{
        Boat_State = GETTING_POS_HEADING;
        Previous_State = INITIAL;
        break;
      }
    }

    case REMOTE_CONTROL:
    Serial.print("In Remote Control Mode!");
    if (result == PACKET_READY) {
      crsf.UpdateChannels();
      Left_Motor_Effort = map(crsf.channels[2], 170, 1850, -30, 30) + map(crsf.channels[0], 170, 1850, -30, 30);
      Right_Motor_Effort = map(crsf.channels[2], 170, 1850, -30, 30) - map(crsf.channels[0], 170, 1850, -30, 30);
      // Map -100..100 to 1000..2000 us
      //   For better readability, do it step by step:
      int LeftpulseMicroseconds = map(Left_Motor_Effort, -100, 100, 1000, 2000);
      int RightpulseMicroseconds = map(Right_Motor_Effort, -100, 100, 1000, 2000);
      // Convert microseconds to timer counts
      //   At prescaler=8, 1 timer count = 0.5us => multiply microseconds by 2
      int LeftpulseCounts = LeftpulseMicroseconds * 2;
      int RightpulseCounts = RightpulseMicroseconds * 2;

      // Update both channels
      OCR1A = LeftpulseCounts; 
      OCR1B = RightpulseCounts;

      if (crsf.channels[1] > 1600){
        Boat_State = GETTING_POS_HEADING;
        Previous_State = REMOTE_CONTROL;
        break;
      }

      // Check if the button is pressed (change in state)
    //   if (buttonState == HIGH && lastButtonState == LOW) {
    //     // If the button is pressed, add a value to the array
    //     if (index < 10) {
    //       array[index] = random(1, 100);  // Fill with random values between 1 and 100
    //       Serial.print("Added: ");
    //       Serial.println(array[index]);
    //       index++;  // Increment the index to fill the next array position
    //     }
    //   }
    }

    break;


    case GETTING_POS_HEADING:
    Serial.print("In GETTING POS Mode");
    UpdatePositionandHeading();
    while(Current_Latitude == 0.00){
      UpdatePositionandHeading();
      delay(15);
    }
    Serial.println("updated Position and heading!");
    // Serial.println(Current_Latitude, 6);
    Boat_State = COMPUTING_TRAJECTORY;
    Previous_State = GETTING_POS_HEADING;
    delay(15);
    break;


    case COMPUTING_TRAJECTORY:
    Target_Heading = getHeadingToWPT(Current_Latitude, Current_Longitude, Target_Latitude[Waypoint_Index], Target_Longitude[Waypoint_Index]);
    // Serial.print("Target Heading : ");
    // Serial.print(Target_Heading);
    Boat_State = FOLLOWING_HEADING;
    Previous_State = COMPUTING_TRAJECTORY;
    delay(15);
    break;

    case FOLLOWING_HEADING:
    UpdatePositionandHeading();
    Normalized_Heading_Error = ComputeNormalizedHeadingError(Current_Heading, Target_Heading);
    // Serial.print(" Normalized Heading Error : ");
    // Serial.println(Normalized_Heading_Error);

    Left_Motor_Effort = Average_Speed + Turn_Gain * Normalized_Heading_Error;
    Right_Motor_Effort = Average_Speed - Turn_Gain * Normalized_Heading_Error;

    Serial.print("Left Motor : ");
    Serial.print(Left_Motor_Effort);
    Serial.print("     |     ");
    Serial.print("Right Motor : ");
    Serial.print(Right_Motor_Effort);
    Serial.print("     |     ");
    Serial.print("Current Distance to Waypoint : ");
    Serial.println(Current_Distance_to_Waypoint);

    Left_Motor_Effort = Left_Motor_Effort * 100;
    Right_Motor_Effort = Right_Motor_Effort * 100;

    // Constrain range to -100..100
    Left_Motor_Effort = constrain(Left_Motor_Effort, Max_Reverse, Max_Forward);
    Right_Motor_Effort = constrain(Right_Motor_Effort, Max_Reverse, Max_Forward);

    // Map -100..100 to 1000..2000 us
    //   For better readability, do it step by step:
    int LeftpulseMicroseconds = map(Left_Motor_Effort, -100, 100, 1000, 2000);
    int RightpulseMicroseconds = map(Right_Motor_Effort, -100, 100, 1000, 2000);
    // Convert microseconds to timer counts
    //   At prescaler=8, 1 timer count = 0.5us => multiply microseconds by 2
    int LeftpulseCounts = LeftpulseMicroseconds * 2;
    int RightpulseCounts = RightpulseMicroseconds * 2;

    // Update both channels
    OCR1A = LeftpulseCounts; 
    OCR1B = RightpulseCounts;


    // Serial.print("Left Motor : ");
    // Serial.print(Left_Motor_Effort);
    // Serial.print("     |     ");
    // Serial.print("Right Motor : ");
    // Serial.println(Right_Motor_Effort);

    // Serial.print("Current Distance to Waypoint : ");
    // Serial.println(Current_Distance_to_Waypoint);
    if (Current_Distance_to_Waypoint <= Waypoint_Radius){
      Waypoint_Index++;
      Serial.println("WE HIT A GODDAMN WAYPOINT!!!!!!!!");
      Serial.println("ON TO THE NEXT, LIKE THE BEAST I AM");
      loops = 0;
      Boat_State = GETTING_POS_HEADING;
      Previous_State = FOLLOWING_HEADING;
      break;
    }

    loops++;
    if (loops >= PID_Laps_To_Recalculate){
      loops = 0;
      Boat_State = GETTING_POS_HEADING;
      Previous_State = FOLLOWING_HEADING;
    }
    delay(Main_Loop_Delay);
    break;


    default:
      break;

  }
}





void UpdatePositionandHeading(){
  //Serial.println("Updating Position and Heading!");
    
  // Update the compass data -------------------------------------
    compass.read();

    // Read raw x, y, z values from the library
    int x = compass.getX();
    int y = compass.getY();
    int z = compass.getZ();
  
    // Or get the heading in degrees
    int16_t heading = compass.getAzimuth();

    if (heading < 0){
      heading = heading * -1;
    }
    else {
      heading = heading * -1 + 360;
    }

    Current_Heading = heading;


    // Print out heading
    // Serial.print("  Heading: ");
    // Serial.print(heading);
    // Serial.println("°");

    // End compass stuff above, GPS stuff below --------------------

    for (int i = 0; i < 10; i++) {
      while (Serial2.available() > 0) {
        gps2.encode(Serial2.read());
      }

      if (gps2.location.isUpdated()) {
        // Serial.print("Latitude= ");
        // Serial.print(gps2.location.lat(), 6);  // Latitude with 6 decimal points
        // Serial.print(" Longitude= ");
        // Serial.println(gps2.location.lng(), 6);  // Longitude with 6 decimal points

        Current_Latitude = gps2.location.lat();
        Current_Longitude = gps2.location.lng();
      }

      // Optional: Add a small delay if desired
      delay(10);  // adjust delay as needed
    }

  return;
}



float getHeadingToWPT(float current_lat, float current_long, float way_lat, float way_long){
  float delta_lat = way_lat - current_lat;
  float delta_long = way_long - current_long;
  float scaled_delta_lat = delta_lat * 10000;
  float scaled_delta_long = delta_long * 10000;
  float heading = atan2(scaled_delta_long, scaled_delta_lat) * 180 / PI;
  float headingDegrees = fmod(heading + 360.0f, 360.0f); //converts negative to 0 - 360 deg. 
  Current_Distance_to_Waypoint = sqrt(delta_lat * delta_lat + delta_long * delta_long) / .00001; // ~1.11 meter
  return headingDegrees;
  //Debug messages
  // Serial.print("Scaled delta_lat: ");
  // Serial.println(scaled_delta_lat);
  // Serial.print("Scaled delta_long: ");
  // Serial.println(scaled_delta_long);
  // Serial.print("heading: ");
}

float ComputeNormalizedHeadingError(float Current_Heading, float Desired_Heading){
  // Serial.print("Desired Heading : ");
  // Serial.println(Desired_Heading);
  // Serial.print("Current Heading : ");
  // Serial.println(Current_Heading);
  float Delta_Theta = Desired_Heading - Current_Heading;
  if (Delta_Theta > 180){
    Delta_Theta = Delta_Theta - 360;
  }
  if (Delta_Theta < -180){
    Delta_Theta = Delta_Theta + 360;
  }
  float NormalizedHeadingError = Delta_Theta/180;
  return(NormalizedHeadingError);
}




