#include "BMI088.h"
#include <math.h>

#define RXD2 16
#define TXD2 17

/* accel object */
Bmi088Accel accel(SPI,15);
/* gyro object */
Bmi088Gyro gyro(SPI,27);

float phiHat_rad = 0.0f;
float thetaHat_rad = 0.0f;

float roll = 0.0f;
float pitch = 0.0f;
float rollF = 0.0f;
float pitchF = 0.0f;

//measured samping time
float dt;
unsigned long millisOld;

#define RAD_TO_DEG 57.2957795f
#define SAMPLERATE_DELAY_MS (10)
#define COMP_FILTER_ALPHA 0.95f

void setup() {
  // put your setup code here, to run once:
   int status;
  /* USB Serial to print data */
  Serial2.begin(115200, SERIAL_8N1,RXD2,TXD2); // set baud rate
  Serial.begin(115200);
  while(!Serial) {}
  /* start the sensors */
  status = accel.begin();
  if (status < 0) {
    Serial.println("Accel Initialization Error");
    Serial.println(status);
    while (1) {}
  }
  status = accel.setRange(Bmi088Accel::RANGE_3G);
  status = accel.setOdr(Bmi088Accel::ODR_100HZ_BW_40HZ);

  status = gyro.begin();
  if (status < 0) {
    Serial.println("Gyro Initialization Error");
    Serial.println(status);
    while (1) {}
  }
  status = gyro.setRange(Bmi088Gyro::RANGE_1000DPS);
  status = gyro.setOdr(Bmi088Gyro::ODR_100HZ_BW_32HZ);
}

void loop() {
  // put your main code here, to run repeatedly:
    gyro.readSensor();
  /* read the accel */
    accel.readSensor();

    roll = atanf(accel.getAccelY_mss() / accel.getAccelZ_mss());
    pitch = atanf(-accel.getAccelX_mss()/sqrt(pow(accel.getAccelY_mss(),2) + pow(accel.getAccelZ_mss(),2)));

    rollF = 0.94 * rollF + 0.06 * roll;
    pitchF = 0.94 * pitchF + 0.06 * pitch;

  /* Transform body rates to euler rates */
    float phiDot_rps = gyro.getGyroX_rads() + tanf(thetaHat_rad) * sinf(phiHat_rad) * gyro.getGyroY_rads() + cosf(phiHat_rad) * tanf(thetaHat_rad) * gyro.getGyroZ_rads();
    float thetaDot_rps =                                                              cosf(phiHat_rad) * gyro.getGyroY_rads() - sinf(phiHat_rad) * gyro.getGyroZ_rads();

  /* integreren om de pitch en roll hoeken te berekenen */
    dt=(millis()-millisOld)/1000.0;
    millisOld=millis();

    //Complementary  filter 
    phiHat_rad = COMP_FILTER_ALPHA * rollF + (1.0f - COMP_FILTER_ALPHA) * (phiHat_rad + phiDot_rps * dt);
    thetaHat_rad = COMP_FILTER_ALPHA * pitchF + (1.0f - COMP_FILTER_ALPHA) * (thetaHat_rad + thetaDot_rps * dt);

    Serial.print("Roll:");
    Serial.print(phiHat_rad * RAD_TO_DEG, 2);
    Serial.print(",");
    Serial.print(" Pitch:"); 
    Serial.print(thetaHat_rad * RAD_TO_DEG, 2);
    Serial.print(",");
    Serial.print("Reference+:"); 
    Serial.print(90);
    Serial.print(",");
    Serial.print("Reference-:"); 
    Serial.println(-90);

    String nmeaString = String("xxHPR,") + String(thetaHat_rad * RAD_TO_DEG, 2) + String(",") + String(phiHat_rad * RAD_TO_DEG, 2) + String(",") + String(0, 2);
    int checksum = 0;

    for (int i = 0; i < nmeaString.length(); i++) {
    checksum ^= nmeaString[i];
    }

    String checksumString = String(checksum, HEX);

    if (checksumString.length() < 2) {
    checksumString = "0" + checksumString;
    }

    String nmeaSentence= "$" + nmeaString + "*" + checksumString + "\r\n";
  
    Serial2.print(nmeaSentence);
     

    delay(SAMPLERATE_DELAY_MS);

}
