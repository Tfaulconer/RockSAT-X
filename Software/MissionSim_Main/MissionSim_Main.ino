/* This code acts as the integrated main function for our
   main board, the Arduino Mega 2560. This was written for a NASA
   Student Project called RockSat-X conducted by students at
   the University of Nebraska-Lincoln.

   Authors: Tom Faulconer, Ryan Green, Aaron Hayes
   Date: 4/5/2019
*/

//Libraries
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <math.h>

//BoomDistanceVariables
const double maxExtended = 6200; // Max entension is at 6ft or 72 inches
const double fullRetraction = 1; // BOOM is inside housing at 1 inch
int extendedPin = 8; //The pin we set HIGH when boom is extended
int retractedPin = 9; //The pin we set HIGH when boom is retracted

//Timers
unsigned long GSE_timer = 0; //timer starting when the GSE Line is on
unsigned long TE1_timer = 0; //timer starting when the TE1 Line is on
const unsigned long totalMissionTime = 250000 ; //total time from data collection until data sensors are off in milliseconds

//TSE LINES
int TE1_pin = 32;
int counter = 0;

//SDCARD VARIABLES & FILES
int chipSelect = 53;
File eeData;
File encData;
File mData;

//ENCODER VARIABLES
int QEM [16] = {0, -1, 1, 2, 1, 0, 2, -1, -1, 2, 0, 1, 2, 1, -1, 0};
int Old = 0;
int New = 0;
int inputA = 38;
int inputB = 40;
int Out;
int tempA;
int tempB;
int pos = 0;
double distance = 0;
const double toInchesConversion = 10.25 / 300;

//MAIN BOARD MPU VARIABLES
#define DegConvert 57.29577951

const int MPU1_addr = 0x68; // 0110 1000 from data sheet. AD0 is grounded Sec. 9.2

double accelX, accelY, accelZ;
double gForceX, gForceY, gForceZ;

double gyroX, gyroY, gyroZ;
double rotX, rotY, rotZ;

double Temp, TempF, RawTemp;

double roll;

//END EFFECTOR MPU VARIABLES
const byte numChars = 64;
char endEffectorString[numChars]; //The string we store one line of data from the End Effector

const int endEffectorStarter = 22; //The pin we set HIGH to tell End Effector to start recording data
int starterCounter = 0;
boolean newData = false;



void setup() {
  
  //Set up our Serial Ports
  Serial.begin(19200); //RS232 Serial Communication
  Serial3.begin(19200); //Communication with End Effector

  //Define needed pins as INPUT or OUTPUT
  pinMode(TE1_pin, INPUT); //Waiting for Timer Event line HIGH input
  pinMode(endEffectorStarter, OUTPUT); //Sends status to End Effector
  pinMode(extendedPin, OUTPUT); //Sends status to 
  pinMode(retractedPin, OUTPUT);

   //Initializes the pins sent to control as LOW
  digitalWrite(extendedPin, LOW);
  digitalWrite(retractedPin, LOW);

  //Required setup for our devices (See. Function Definitions)
  setup_SD();
  setup_MPU();
  setup_encoder();

  Serial.print("MISSION BEGIN");
  mData.print("MISSION BEGIN");
  eeData.print("MISSION BEGIN");
  encData.print("MISSION BEGIN");
}

void loop() {

  //The arduino will be powered on much sooner than when we need to
  //start recording data. The first timer event line signals when
  //we need to start recording data. This first conditional looks for
  //when that line is HIGH. Once it is HIGH, we can begin our mission.
  if (digitalRead(TE1_pin) == HIGH) 
  {
    
    //This timer starts when the first timer event line is powered on.
    //This is the timer that is included in all of our data recording.
    TE1_timer = millis() - GSE_timer;
    


    /**************************************READING AND PRINTING DATA************************************/
    
    //These two functions read the MPU sensor values 
    
    read_EndEffector();

    //The encoder value is stored in the distance variable
    distance = readEncoder();


    //We write to the SD files and print to Serial Port
    //all in this function.
    printData();
    
    /**************************************************************************************************/


    /*****************************************ENCODER CONDITIONALS*************************************/
    
    //When the BOOM is out of the housing, but not fully extended,
    //we set both pins LOW. See Ryan's Control code for why this is needed.
    if (distance >= fullRetraction && distance < maxExtended) {
      digitalWrite(extendedPin, LOW);
      digitalWrite(retractedPin, LOW);
    }

    //When the BOOM is fully extended, we will set the extended pin HIGH.
    //When the control receives this, the motor stops and stalls. (See Control Code)
    if (distance >= maxExtended)
    {
      digitalWrite(extendedPin, HIGH);
    }

    //When the BOOM has fully retracted and entered the housing, we set the retracted
    //pin HIGH. This tells the motor to stop, and 
    if (distance <= fullRetraction)
    {
      digitalWrite(retractedPin, HIGH);
    } else {
      digitalWrite(retractedPin, LOW);
    }
    /*************************************************************************************************/

    
    //This conditional closes our SD Card files and sets our end Effector pin 
    //LOW so that the endEffector also stops recording data.
    if (TE1_timer >= totalMissionTime) {
      closeDataFiles();
      digitalWrite(endEffectorStarter, LOW); //Tell end effector MPU to stop recording data
      Serial.print("MISSION COMPLETE!!!!");
      while(1);
    }
  }else {
    GSE_timer = millis();
  }

}

/******************************************END OF MAIN LOOP*******************************************/








/*****************************************************************************************************
 *****************************************************************************************************
                                       FUNCTION DEFINITIONS
******************************************************************************************************
******************************************************************************************************/



/*****************************************************************************************************/
//These our the setup functions needed for each device (MPU, SD Card Logger, Rotary Encoder)
/*****************************************************************************************************/

void setup_MPU()
{
  Wire.begin();
  Wire.beginTransmission(MPU1_addr); //MPU1_addr = 0x68
  Wire.write(0x6B); //Accessing the register 0x6B - Power Management (Sec. 4.28)
  Wire.write(0x00); //Setting SLEEP register to 0. (Required to wake up MPU; see Note on p. 9)
  Wire.endTransmission();
  Wire.beginTransmission(MPU1_addr); //I2C address of the MPU
  Wire.write(0x1B); //Accessing the register 0x1B - Gyroscope Configuration (Sec. 4.4)
  Wire.write(0x00); //Setting the gyro to full scale +/- 250deg./s
  Wire.endTransmission();
  Wire.beginTransmission(MPU1_addr); //I2C address of the MPU
  Wire.write(0x1C); //Accessing the register 0x1C - Acccelerometer Configuration (Sec. 4.5)
  Wire.write(0x00); //Setting the accel to +/- 2g
  Wire.endTransmission();
}

void setup_SD()
{
  pinMode(chipSelect, OUTPUT);
  SD.begin(chipSelect);
  mData = SD.open("mData.txt", FILE_WRITE);
  encData = SD.open("encData.txt", FILE_WRITE);
  eeData = SD.open("eeData.txt", FILE_WRITE);
}

void setup_encoder()
{
  pos = 0;
  distance = 0;
  pinMode(inputA, INPUT);
  pinMode(inputB, INPUT);
}



/*****************************************************************************************************/
//These functions read all of our sensor data (MPU and Rotary Encoder)
/*****************************************************************************************************/

void read_main_MPU()
{
  //Records the G-Force, Gyro, and Temperature Data of
  //our main MPU
  recordAccelRegisters();
  recordGyroRegisters();
  recordTempRegisters();

}

void read_EndEffector()
{
  static byte ndx = 0;
  char endMarker = '\n';
  char rc;

  while (Serial3.available() > 0 && newData == false) {
    rc = Serial3.read();

    if (rc != endMarker) {
      endEffectorString[ndx] = rc;
      ndx++;
      if (ndx >= numChars) {
        ndx = numChars - 1;
      }
    }
    else {
      endEffectorString[ndx] = '\0'; // terminate the string
      ndx = 0;
      newData = true;
    }
  }
}

double readEncoder()
{
  Old = New;
  New = digitalRead(inputA) * 2 + digitalRead(inputB);
  Out = QEM[Old * 4 + New];

  pos = pos + Out;
  return pos; //* toInchesConversion;
}



/*****************************************************************************************************/
//The function that takes care of writing to the SD card files and printing to Serial Port
/*****************************************************************************************************/

void printData()
{
  read_main_MPU();
  if (mData && encData && eeData) // Check that our files opened properly
  {
    if (newData == true)
      //If we include all of our writing and printing under this conditional,
      //the main MPU data will print its most recent values only when the end effector string
      //has been filled up. They can only print together now. This eliminates the problem
      //of having 5 or so main MPU data lines for every 1 endEffector MPU data line.
    {

      //First write data to all the SD files and then print to the serial port


      //Write main MPU data to the SD Card
      mData.print(rotX);
      mData.print(",");
      mData.print(rotY);
      mData.print(",");
      mData.print(rotZ);
      mData.print(",");
      mData.print(gForceX);
      mData.print(",");
      mData.print(gForceY);
      mData.print(",");
      mData.print(gForceZ);
      mData.print(",");
      mData.print(Temp);
      mData.print(",");
      mData.println(TE1_timer);

      //Write the end effector data to the SD Card
      eeData.print(endEffectorString);
      eeData.print(",");
      eeData.println(TE1_timer);

      //Write encoder data to the SD Card
      encData.print(distance);
      encData.print(",");
      encData.println(TE1_timer);


      //Now print all the data to the serial port

      //Print Main MPU Data
      Serial.print("MPU 1: ");
      Serial.print(rotX);
      Serial.print(",");
      Serial.print(rotY);
      Serial.print(",");
      Serial.print(rotZ);
      Serial.print(",");
      Serial.print(gForceX);
      Serial.print(",");
      Serial.print(gForceY);
      Serial.print(",");
      Serial.print(gForceZ);
      Serial.print(",");
      Serial.print(Temp);

      //Print End Effector MPU Data
      Serial.print("  MPU 2: ");
      Serial.print(endEffectorString);

      //Print Encoder Data
      Serial.print("  ENCODER: ");
      Serial.print(distance);

      //Print timestamp
      Serial.print("  TIMER: ");
      Serial.println(TE1_timer);

      //After printing, reset newData so that the incoming data
      //will fill up the string
      newData = false;

    }

  } else {
    Serial.println("File failed to open");
  }
}



/*****************************************************************************************************/
//This function closes the files after the mission is complete
/*****************************************************************************************************/

void closeDataFiles()
{
  mData.close();
  eeData.close();
  encData.close();
}



/*****************************************************************************************************/
//Supplementary functions for reading and processing MPU sensor data
/*****************************************************************************************************/

void recordAccelRegisters()
{
  Wire.beginTransmission(MPU1_addr); //I2C address of the MPU
  Wire.write(0x3B); //Starting register for Accel Readings
  Wire.endTransmission();
  Wire.requestFrom(MPU1_addr, 6, true); //Request Accel Registers (0x3B - 0x40)
  //while(Wire.available() < 6);
  accelX = Wire.read() << 8 | Wire.read(); //0x3B (ACCEL_XOUT_H) and 0x3C (ACCEL_XOUT_L)
  accelY = Wire.read() << 8 | Wire.read(); //0x3D (ACCEL_YOUT_H) and 0x3E (ACCEL_YOUT_L)
  accelZ = Wire.read() << 8 | Wire.read(); //0x3F (ACCEL_ZOUT_H) and 0x40 (ACCEL_ZOUT_L)
  processAccelData();
}

void processAccelData() {
  gForceX = accelX / 16384.0; // +/- 2g setting. 16834/°/g Sec 4.17 in datasheet
  gForceY = accelY / 16384.0;
  gForceZ = accelZ / 16384.0;
}

void recordGyroRegisters()
{
  Wire.beginTransmission(MPU1_addr); //I2C address of the MPU
  Wire.write(0x43); //Starting register for Gyro Readings
  Wire.endTransmission();
  Wire.requestFrom(MPU1_addr, 6, true); //Request Gyro Registers (0x43 - 0x48)
  //while(Wire.available() < 6);
  gyroX = Wire.read() << 8 | Wire.read(); //0x43 (GYRO_XOUT_H) and 0x44 (GYRO_XOUT_L)
  gyroY = Wire.read() << 8 | Wire.read(); //0x45 (GYRO_YOUT_H) and 0x46 (GYRO_YOUT_L)
  gyroZ = Wire.read() << 8 | Wire.read(); //0x47 (GYRO_ZOUT_H) and 0x48 (GYRO_ZOUT_L)
  processGyroData();
}

void processGyroData() {
  rotX = gyroX / 131.0; // The +/- 250°/sec setting --> Maximum detection = 41.67 RPM
  rotY = gyroY / 131.0;
  rotZ = gyroZ / 131.0;
  roll = atan2(rotZ, rotY) * DegConvert;
}

void recordTempRegisters() {
  Wire.beginTransmission(MPU1_addr); //I2C address of the MPU
  Wire.write(0x41); //Accessing the register 0x41 - Temperature Registers (Sec. 4.18)
  Wire.endTransmission();
  Wire.requestFrom(MPU1_addr, 2, true); // Stop transmission after request
  RawTemp = Wire.read() << 8 | Wire.read(); // Store the upper and lower byte into RawTemp
  processTemp();
}

void processTemp() {
  Temp = ((RawTemp / 340.0) + 36.532); // From section 4.18 in Register Map Datasheet
  TempF = (Temp * (9.0 / 5.0)) + 32; // Temp in F
}
