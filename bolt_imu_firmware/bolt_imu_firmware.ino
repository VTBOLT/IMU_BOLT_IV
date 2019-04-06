// Author:      Logan Richardson
//
// Filename:    bolt_firmware_imu.ino
//
// Description: This firmware records gyroscopic and accelerometer data from the MPU-9250
//              on the SparkFun 9DoF IMU breakout board. The collected data is placed on
//              an SD card and transmitted through UART. It is intended that the UART be
//              connected to UART6 on the MSP-EXP432E401Y SimpleLink MCU from TI. The
//              data should then be transmitted through UART7 to the XBee radio modules
//              where the data is immediately interpreted on the BOLT laptop.
//
// Revision:    0
//
// Requirements: 
// - Visit this link for extensive documentation on the Sparkfun 9DoF break-
//   out board: https://learn.sparkfun.com/tutorials/9dof-razor-imu-m0-hookup-guide/all
// - Sparkfun MPU-9250 DMP Library
// - Flash storage Arduino Library             
// - Arduino SAMD board package from Boards Manager
// - Sparkfun SAMD board package from Boards Manager
//    - NOTE: This board package requires the following URL to be entered in the "Additonal
//      Boards Manager URLs" section ins File->Preferences->Additional Boards Manager URLs:
//      https://raw.githubusercontent.com/sparkfun/Arduino_Boards/master/IDE_Board_Manager/package_sparkfun_index.json
// 
// Important Notes:
// - Sometimes (most of the time) the SAMD21 microcontroller will not enter the Arduino bootloader
//   when trying to download firmware. To force the microcontroller into the bootloader, turn the
//   power switch off, then short the SCL and GND pin with a jumper. With the pins shorted, turn on
//   the board, then remove the jumper. A blue LED should begin to pulse slowly.


// Included libraries
#include <avr/dtostrf.h>
#include <SD.h> // Manages file and hardware control on the SD storage
#include <SparkFunMPU9250-DMP.h> // Include SparkFun MPU-9250 library
#include "config.h"
#if ENABLE_NVRAM_STORAGE
#include <FlashStorage.h>
#endif

// File-scope defines
#define ACCEL 'a'
#define GYRO  'g'
#define MAG   'm'
#define EULER 'e'
#define COMPASS 'c'
#define ACCEL_BUF_SZ 5
#define GYRO_BUF_SZ 7
#define ANGLE_BUF_SZ 9
#define COMP_BUF_SZ 9

// Type definitions
typedef struct {
  float x, y, z;
} imudata_t;

typedef struct {
  float pitch, roll, yaw;
} eulerangle_t;

// Prototypes
void blinkLED();
void initHW();
void initIMU();
void initDMP();
void dbgPrintData(imudata_t, imudata_t, imudata_t, eulerangle_t, float);
void imuGetData(imudata_t*, imudata_t*, imudata_t*, eulerangle_t*, float*);
void imuSendData(imudata_t, imudata_t, eulerangle_t, float);

// Global variables
MPU9250_DMP imu; // Create instance of the MPU9250_DMP class
uint32_t gNextBlink = 0;
uint32_t gNextOutput = 0;

void setup() {
  initHW(); // Config pins and serial connections
  initIMU(); // Config accel, gyro, mag, and interrupts
  initDMP(); // Config DMP
}

void loop() {

  // Structures to contain data from IMU
  static imudata_t accel, gyro, mag;
  static eulerangle_t angle;
  static float compass;
  
  if (!imu.fifoAvailable()) {
    return; // If no new data available, then return to top of loop
  }
  if (imu.dmpUpdateFifo() != INV_SUCCESS) {
    return; // If reading data from the FIFO fails, the return to top of loop and try again...
  }
  if (imu.updateCompass() != INV_SUCCESS) {
    return; // If reading data from magnetometer fails, return to top of loop and try again...
  }
  
  imu.computeEulerAngles(); // Compute pitch, roll, yaw based on quaternion values
  imuGetData(&accel, &gyro, &mag, &angle, &compass);

  if (millis() > gNextBlink) {
    blinkLED();
    //imuSendData(accel, gyro, angle, compass);
    gNextBlink += UART_BLINK_RATE;
  }
#if DEBUG_ENABLE
  if (millis() > gNextOutput) {
    dbgPrintData(accel, gyro, mag, angle, compass);
    gNextOutput += DEBUG_OUTPUT_RATE;
  }
#endif
}

// Heartbeat
void blinkLED() {
  static bool ledState = false;
  digitalWrite(HW_LED_PIN, ledState);
  ledState = !ledState;
}

// Initialize hardware config
void initHW() {
  Serial1.begin(SERIAL_BAUD_RATE);
  while (!Serial1); // Wait for serial connection to open
#if DEBUG_ENABLE
  DEBUG.begin(DEBUG_BAUD_RATE);
  while (!DEBUG);
  DEBUG.println("Serial connection initialized");
  DEBUG.println("USB Baud rate: " + String(DEBUG_BAUD_RATE));
  DEBUG.println("Serial1 Baud rate: " + String(SERIAL_BAUD_RATE));
#endif
  pinMode(HW_LED_PIN, OUTPUT);
  digitalWrite(HW_LED_PIN, LOW);
}

void initIMU() {
#if DEBUG_ENABLE
  DEBUG.print("MPU-9250 init... ");
#endif
  if (imu.begin() != INV_SUCCESS) {
#if DEBUG_ENABLE
    DEBUG.println("ERROR");
#endif
    while(1); // Failed to initialize MPU-9250
  }
#if DEBUG_ENABLE
  DEBUG.println("SUCCESS");
  DEBUG.println("All sensors enabled");
#endif
  imu.setAccelFSR(IMU_ACCEL_FSR); // Set accelerometer range to +/- 2 Gs
#if DEBUG_ENABLE
  DEBUG.println("Accelerometer FSR: " + String(IMU_ACCEL_FSR));
#endif
  imu.setGyroFSR(IMU_GYRO_FSR); // Set gyroscope range to +/- 250 degrees per sec
#if DEBUG_ENABLE
  DEBUG.println("Gyroscope FSR: " + String(IMU_GYRO_FSR));
#endif
  imu.setCompassSampleRate(IMU_COMPASS_SAMPLE_RATE); // Set magnetometer sample rate to 42 Hz
#if DEBUG_ENABLE
  DEBUG.println("Magnetometer sample rate: " + String(IMU_COMPASS_SAMPLE_RATE));
#endif
  imu.setSampleRate(IMU_AG_SAMPLE_RATE); // Set sample rate for both accelerometer and gyroscope
#if DEBUG_ENABLE
  DEBUG.println("Accel/Gyro sample rate: " + String(IMU_AG_SAMPLE_RATE));
#endif  
  imu.setLPF(IMU_AG_LPF); // Set Low Pass Filter corner frequency to 5 Hz. (Not sure why, but it's in the example code)
#if DEBUG_ENABLE
  DEBUG.println("LPF corner freq: " + String(IMU_AG_LPF));
#endif
}

void initDMP() {
  imu.dmpBegin( DMP_FEATURE_GYRO_CAL        | // Set Gyroscope to auto calibrate after being stationary for 8 seconds
                DMP_FEATURE_6X_LP_QUAT      | // Enable 6-axis quaternion calculations
                DMP_FEATURE_SEND_RAW_ACCEL  | // Send raw accelerometer data to DMP's FIFO
                DMP_FEATURE_SEND_CAL_GYRO,    // Send calibrated gyroscope data to DMP's FIFO
                DMP_SAMPLE_RATE );            // Set sample rate of DMP
#if DEBUG_ENABLE
  DEBUG.println("Digital Motion Processor initialized");
  DEBUG.println("DMP sample rate: " + String(DMP_SAMPLE_RATE) + "Hz");
  DEBUG.println("Gyroscope set to auto-calibrate");
  DEBUG.println("6-axis quaternion calculations enabled");
  DEBUG.println("Raw accel data -> FIFO");
  DEBUG.println("Calibrated gryo data -> FIFO");
#endif
}

void imuGetData(imudata_t* accel, imudata_t* gyro, imudata_t* mag, eulerangle_t* angle, float* compass) {
    // calcAccel, calcGyro, and calcMag are required to convert signed 16-bit values to their correct units
    accel->x = imu.calcAccel(imu.ax);
    accel->y = imu.calcAccel(imu.ay);
    accel->z = imu.calcAccel(imu.az);
    gyro->x = imu.calcGyro(imu.gx);
    gyro->y = imu.calcGyro(imu.gy);
    gyro->z = imu.calcGyro(imu.gz);
    mag->x = imu.calcMag(imu.mx);
    mag->y = imu.calcMag(imu.my);
    mag->z = imu.calcMag(imu.mz);
    angle->pitch = imu.pitch;
    angle->roll = imu.roll;
    angle->yaw = imu.yaw;
    *compass = imu.computeCompassHeading();
}

void imuSendData(imudata_t accel, imudata_t gyro, eulerangle_t angle, float compass)
{
  // Write all data out through UART pins to the TI microcontroller.
  // Magnetometer data is not needed since we only care about the compass
  // data.
  char imubuf[12];

  // All values are appended with an appreviation + '|' to allow the TI microcontroller
  // to parse the data easily.
  dtostrf(accel.x, 8, 4, imubuf);
  memcpy(&imubuf[9], "ax|", 3);
  DEBUG.write(imubuf, sizeof(imubuf));

  dtostrf(accel.y, 8, 4, imubuf);
  memcpy(&imubuf[9], "ay|", 3);
  Serial1.write(imubuf, sizeof(imubuf));

  dtostrf(accel.z, 8, 4, imubuf);
  memcpy(&imubuf[9], "az|", 3);
  Serial1.write(imubuf, sizeof(imubuf));

  dtostrf(gyro.x, 8, 4, imubuf);
  memcpy(&imubuf[9], "gx|", 3);
  Serial1.write(imubuf, sizeof(imubuf));

  dtostrf(gyro.y, 8, 4, imubuf);
  memcpy(&imubuf[9], "gy|", 3);
  Serial1.write(imubuf, sizeof(imubuf));

  dtostrf(gyro.z, 8, 4, imubuf);
  memcpy(&imubuf[9], "gz|", 3);
  Serial1.write(imubuf, sizeof(imubuf));

  dtostrf(angle.pitch, 8, 4, imubuf);
  memcpy(&imubuf[9], "ep|", 3);
  Serial1.write(imubuf, sizeof(imubuf));

  dtostrf(angle.roll, 8, 4, imubuf);
  memcpy(&imubuf[9], "er|", 3);
  Serial1.write(imubuf, sizeof(imubuf));

  dtostrf(angle.yaw, 8, 4, imubuf);
  memcpy(&imubuf[9], "ez|", 3);
  Serial1.write(imubuf, sizeof(imubuf));

  dtostrf(compass, 8, 4, imubuf);
  memcpy(&imubuf[9], "co|", 2);
  Serial1.write(imubuf, sizeof(imubuf));
}

void dbgPrintData(imudata_t accel, imudata_t gyro, imudata_t mag, eulerangle_t angle, float compass) {

  static unsigned char cmd = 'a';

  // NOTE: be sure to set the Serial Monitor to not send Newline characters
  if (DEBUG.available()) {
    cmd = DEBUG.read();
  }

  switch(cmd)
  {
    case ACCEL:
      DEBUG.print("Accel  --  X: " + String(accel.x));
      DEBUG.print("  Y: " + String(accel.y));
      DEBUG.println("  Z: " + String(accel.z));
      break;
      
    case GYRO:
      DEBUG.print("Gyro  --  X: " + String(gyro.x));
      DEBUG.print("  Y: " + String(gyro.y));
      DEBUG.println("  Z: " + String(gyro.z));
      break;

    case MAG:
      DEBUG.print("Mag  --  X: " + String(mag.x));
      DEBUG.print("  Y: " + String(mag.y));
      DEBUG.println("  Z: " + String(mag.z));
      break;

    case EULER:
      DEBUG.print("pitch: " + String(angle.pitch));
      DEBUG.print("  roll: " + String(angle.roll));
      DEBUG.println("  yaw: " + String(angle.yaw));
      break;

    case COMPASS:
      DEBUG.println("Compass Heading: " + String(compass));
      break;

    default:
      DEBUG.println("Invalid command: " + cmd); // Should never get here
      break;
  }
}
