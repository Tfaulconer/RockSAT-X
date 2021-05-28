#include <Wire.h>
#include <SPI.h>
#include <math.h>



char buf[20];
unsigned int timestamp=0;

const int MPU1_addr = 0x68; // 0110 1000 from data sheet. AD0 is grounded Sec. 9.2
struct MPU {
  short accelX, accelY, accelZ;
  float gForceX, gForceY, gForceZ;
  short gyroX, gyroY, gyroZ;
  float rotX, rotY, rotZ;
  float temp, tempF;
  short rawTemp;
  float roll; };

#define DegConvert 57.29577951
MPU effMPU;
  
void setup() {
  pinMode(15,INPUT);
  digitalWrite(15,LOW);
  pinMode(13,OUTPUT);
  digitalWrite(13,HIGH);
  // put your setup code here, to run once:
  Serial1.begin(38400); //RS232 Serial1 Communication  
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
  //Wait a second so the main teensy can start up
  while(millis() < 1000);
}

void loop() {
  // put your main code here, to run repeatedly:

  
  recordAccelRegisters();
  recordGyroRegisters();
  recordTempRegisters();
  timestamp = millis();
  //Print End Effector MPU Data
  #ifdef DEBUG
  Serial1.print("MPU 2,");
  Serial1.print(timestamp);
  Serial1.print(",");
  Serial1.print(effMPU.rotX);
  Serial1.print(",");
  Serial1.print(effMPU.rotY);
  Serial1.print(",");
  Serial1.print(effMPU.rotZ);
  Serial1.print(",");
  Serial1.print(effMPU.gForceX);
  Serial1.print(",");
  Serial1.print(effMPU.gForceY);
  Serial1.print(",");
  Serial1.print(effMPU.gForceZ);
  Serial1.print(",");
  Serial1.println(effMPU.temp);  
  #else

  Serial1.write((char)0x80);
  Serial1.write((char)0x5A);
  Serial1.write((char)(timestamp>>16 & 0xFF));
  Serial1.write((char)(timestamp>>8 & 0xFF));
  Serial1.write((char)(timestamp & 0xFF));
  Serial1.write((char)(effMPU.rawTemp >> 8 & 0xFF));
  Serial1.write((char)(effMPU.rawTemp & 0xFF));
  Serial1.write((char)(effMPU.accelX >> 8 & 0xFF));
  Serial1.write((char)(effMPU.accelX & 0xFF));
  Serial1.write((char)( effMPU.accelY >> 8 & 0xFF));
  Serial1.write((char)( effMPU.accelY & 0xFF));
  Serial1.write((char)( effMPU.accelZ >> 8 & 0xFF));
  Serial1.write((char)( effMPU.accelZ & 0xFF));    
  Serial1.write((char)( effMPU.gyroX >> 8 & 0xFF));
  Serial1.write((char)( effMPU.gyroX & 0xFF));  
  Serial1.write((char)( effMPU.gyroY >> 8 & 0xFF));
  Serial1.write((char)( effMPU.gyroY & 0xFF));  
  Serial1.write((char)( effMPU.gyroZ >> 8 & 0xFF));
  Serial1.write((char)( effMPU.gyroZ & 0xFF));    
  Serial1.flush();
  #endif
}


  void recordAccelRegisters()
  {
    Wire.beginTransmission(MPU1_addr); //I2C address of the MPU
    Wire.write(0x3B); //Starting register for Accel Readings
    Wire.endTransmission();
    Wire.requestFrom(MPU1_addr, 6, true); //Request Accel Registers (0x3B - 0x40)
    while(Wire.available() < 6);
    effMPU.accelX = short(Wire.read() << 8 | Wire.read()); //0x3B (ACCEL_XOUT_H) and 0x3C (ACCEL_XOUT_L)
    effMPU.accelY = short(Wire.read() << 8 | Wire.read()); //0x3D (ACCEL_YOUT_H) and 0x3E (ACCEL_YOUT_L)
    effMPU.accelZ = short(Wire.read() << 8 | Wire.read()); //0x3F (ACCEL_ZOUT_H) and 0x40 (ACCEL_ZOUT_L)  

    #ifdef DEBUG
    processAccelData();
    #endif
  }

  void processAccelData() {
    effMPU.gForceX = effMPU.accelX / 16384.0f; // +/- 2g setting. 16834/°/g Sec 4.17 in datasheet
    effMPU.gForceY = effMPU.accelY / 16384.0f;
    effMPU.gForceZ = effMPU.accelZ / 16384.0f;
  }

  void recordGyroRegisters()
  {
    Wire.beginTransmission(MPU1_addr); //I2C address of the MPU
    Wire.write(0x43); //Starting register for Gyro Readings
    Wire.endTransmission();
    Wire.requestFrom(MPU1_addr, 6, true); //Request Gyro Registers (0x43 - 0x48)
    while(Wire.available() < 6);
    effMPU.gyroX = short(Wire.read() << 8 | Wire.read()); //0x43 (GYRO_XOUT_H) and 0x44 (GYRO_XOUT_L)
    effMPU.gyroY = short(Wire.read() << 8 | Wire.read()); //0x45 (GYRO_YOUT_H) and 0x46 (GYRO_YOUT_L)
    effMPU.gyroZ = short(Wire.read() << 8 | Wire.read()); //0x47 (GYRO_ZOUT_H) and 0x48 (GYRO_ZOUT_L) 

    #ifdef DEBUG
    processGyroData();
    #endif
  }

  void processGyroData() {
  
    effMPU.rotX = effMPU.gyroX / 131.0f; // The +/- 250°/sec setting --> Maximum detection = 41.67 RPM
    effMPU.rotY = effMPU.gyroY / 131.0f;
    effMPU.rotZ = effMPU.gyroZ / 131.0f;
    effMPU.roll = atan2f(effMPU.rotZ, effMPU.rotY) * DegConvert;
  }

  void recordTempRegisters() {

    Wire.beginTransmission(MPU1_addr); //I2C address of the MPU
    Wire.write(0x41); //Accessing the register 0x41 - Temperature Registers (Sec. 4.18)
    Wire.endTransmission();
    Wire.requestFrom(MPU1_addr, 2, true); // Stop transmission after request
    while(Wire.available() < 2);
    effMPU.rawTemp = (short)(Wire.read() << 8 | Wire.read()); // Store the upper and lower byte into RawTemp  

    #ifdef DEBUG
    processTemp();
    #endif
  }

  void processTemp() {
    effMPU.temp = ((effMPU.rawTemp / 340.0f) + 36.532f); // From section 4.18 in Register Map Datasheet
    effMPU.tempF = (effMPU.temp * (9.0f / 5.0f)) + 32.0f; // Temp in F  
  }
