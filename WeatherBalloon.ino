// Additional board library url: https://resource.heltec.cn/download/package_CubeCell_index.json

/***************************************************************
 *  Altemeter
 ***************************************************************/
 // Your sketch must #include this library, and the Wire library.
// (Wire is a standard library included with Arduino.):
// This module has a hardwired I2C address and is set to 0x77.
#include <SFE_BMP180.h>
#include <Wire.h>
SFE_BMP180 pressure;
double baseline; // baseline pressure
double alti;
int bmpAddress = 0x77; 
int mpuAddress;

/* *************************************************************
 *  MPU
 ***************************************************************/
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 mpu;

#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

/******************************************************************
 * SERVO
 ******************************************************************/
 #include <Servo.h>
 Servo leftServo;
 Servo rightServo;

 /*****************************************************************
  * ORIENTATION NORTH
  * YAW
  ****************************************************************/
  // North will be set up as the first yaw
  float north;  
  float yaw;
  float pitch;
  float roll; 
  float tempDiff;

  /**************************************************************
   * GPS VALUES
   *************************************************************/
   #include <Math.h>
   float longitude[1];
   float latitude[1]; 
   float longRise;
   float latRun;
   float GPSangle;
 
// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}


/************************************************
 * STAGING BOOLS and global variables
 ************************************************/
 bool setupStage;
 bool risingStage;
 bool landingStage;
 double height;

/**********************************************************************************
 *  Setup
 *********************************************************************************/
void setup()
{
  Serial.begin(9600);
//  setupBMP();
//  setupMPU();
  setupServos();
  setupStage = true;
  risingStage = false;
  landingStage = false;
  height = 0;
}

void loop()
{  
  if(setupStage == true);
  {
    height = getAltitude();
    gyro();    
    north = yaw;
    int pos = 0;
    leftServo.write(0);
    rightServo.write(0);
    risingStage = true;
    setupStage = false;
  }

  if(risingStage == true)
  {
    gyro();    
    alti = getAltitude();
    // If statement will give conditions on when the rising stage has ended and falling phase has begun
    if(alti > 29527.5591) // 9km in feet
    {
      landingStage = true;      
      risingStage = false;
      // detach code here
      //
      //
      //
      //
    }
  }

  if (landingStage == true)
  {
    // gps values read in only with interrupts from Heltec; 
    // sets longitude and latitude
    gyro();    
  }
}



/*******************************************************************************************************
 * SETUP AND RUN FUNCTIONS
 *******************************************************************************************************/
double getPressure()
{
  char status;
  double T,P,p0,a;

  // You must first get a temperature measurement to perform a pressure reading.
  // Start a temperature measurement:
  // If request is successful, the number of ms to wait is returned.
  // If request is unsuccessful, 0 is returned.
  status = pressure.startTemperature();
  if (status != 0)
  {
    // Wait for the measurement to complete:

    delay(status);

    // Retrieve the completed temperature measurement:
    // Note that the measurement is stored in the variable T.
    // Use '&T' to provide the address of T to the function.
    // Function returns 1 if successful, 0 if failure.

    status = pressure.getTemperature(T);
    if (status != 0)
    {
      // Start a pressure measurement:
      // The parameter is the oversampling setting, from 0 to 3 (highest res, longest wait).
      // If request is successful, the number of ms to wait is returned.
      // If request is unsuccessful, 0 is returned.
      status = pressure.startPressure(3);
      if (status != 0)
      {
        // Wait for the measurement to complete:
        delay(status);

        // Retrieve the completed pressure measurement:
        // Note that the measurement is stored in the variable P.
        // Use '&P' to provide the address of P.
        // Note also that the function requires the previous temperature measurement (T).
        // (If temperature is stable, you can do one temperature measurement for a number of pressure measurements.)
        // Function returns 1 if successful, 0 if failure.

        status = pressure.getPressure(P,T);
        if (status != 0)
        {
          return(P);
        }
        else Serial.println("error retrieving pressure measurement\n");
      }
      else Serial.println("error starting pressure measurement\n");
    }
    else Serial.println("error retrieving temperature measurement\n");
  }
  else Serial.println("error starting temperature measurement\n");
}

void setupBMP()
{
   Serial.println("REBOOT");
  // Initialize the sensor (it is important to get calibration values stored on the device).
  if (pressure.begin())
    Serial.println("BMP180 init success");
  else
  {
    // Oops, something went wrong, this is usually a connection problem,
    // see the comments at the top of this sketch for the proper connections.
    Serial.println("BMP180 init fail (disconnected?)\n\n");
  }

  // Get the baseline pressure:
  
  baseline = getPressure();
  
  Serial.print("baseline pressure: ");
  Serial.print(baseline);
  Serial.println(" mb");  
}

double getAltitude()
{
  double a,P;
  // Get a new pressure reading:
  P = getPressure();

  // Show the relative altitude difference between
  // the new reading and the baseline reading:
  a = pressure.altitude(P, baseline);
  alti = 3.28084 * a;
  Serial.print("relative altitude: ");
  if (a >= 0.0) Serial.print(" "); // add a space for positive numbers
  Serial.print(a,1);
  Serial.print(" meters, ");
  if (a >= 0.0) Serial.print(" "); // add a space for positive numbers
  Serial.print(a*3.28084,0);
  Serial.println(" feet");
}

void setupMPU()
{
  // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
    
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3V or Arduino
    // Pro Mini running at 3.3V, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
    Serial.println(F("Initializing MPU device..."));
    mpu.initialize();
//    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // wait for ready
    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
}

void setupServos()
{
  leftServo.attach(5); // attach the servo on pin 5 to the servo object
  rightServo.attach(6); // attach the servo on pin 6 to the servo object
}

void gyro()
{
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  yaw = ypr[0] * 180/M_PI;
  pitch = ypr[1] * 180/M_PI;
  roll = ypr[2] * 180/M_PI;
}

void readHeltec()
{
  // GPS data read into Lat[1] and long[1] here
  //
  //
  //
  //
  
  // create direction from GPS data
  longRise = latitude[0] - latitude[1];
  latRun = longitude[0] - longitude[1];
  GPSangle = atan2(longRise, latRun);
  latitude[0] = latitude[1];
  longitude[0] = longitude[1];
  tempDiff = GPSangle - yaw;
  if(tempDiff < -10)
  {
    rightServo.write(0);
    leftServo.write(180);
  }
  else if(tempDiff > 10)
  {
    leftServo.write(0);
    rightServo.write(180);
  }  
  else 
  {
    leftServo.write(0);
    rightServo.write(0); 
  }
}
