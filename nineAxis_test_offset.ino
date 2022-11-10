#include <Wire.h>
#include <Tone.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

/* 

   Connections
   ===========
   Connect SCL to analog 5
   Connect SDA to analog 4
   Connect VDD to 3.3V DC
   Connect GROUND to common ground

*/

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28);


int dutyCycleRight;
int dutyCycleLeft;
bool systemCalibrated;

Tone tone1;

void setup(void)
{
  
pinMode(5, OUTPUT);
pinMode(6, OUTPUT);
pinMode(8, OUTPUT);
pinMode(9, OUTPUT);
pinMode(10, OUTPUT);
pinMode(11, OUTPUT);

tone1.begin(11);
  
  Serial.begin(115200);

  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  delay(1000);

  /* Display the current temperature */
  int8_t temp = bno.getTemp();
  Serial.print("Current Temperature: ");
  Serial.print(temp);
  Serial.println(" C");
  Serial.println("");

  bno.setExtCrystalUse(true);

  Serial.println("Calibration status values: 0=uncalibrated, 3=fully calibrated");
}


void loop(void)
{
digitalWrite(6, LOW);
digitalWrite(5, LOW);
digitalWrite(8, LOW);
digitalWrite(9, LOW);
digitalWrite(10, LOW);

  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

//// calibrate system before use
//while (systemCalibrated == false){
//    /* Display calibration status for each sensor. */
//  uint8_t system, gyro, accel, mag = 0;
//  bno.getCalibration(&system, &gyro, &accel, &mag);
//  Serial.print("CALIBRATION: Sys=");
//  Serial.print(system, DEC);
//  Serial.print(" Gyro=");
//  Serial.print(gyro, DEC);
//  Serial.print(" Accel=");
//  Serial.print(accel, DEC);
//  Serial.print(" Mag=");
//  Serial.println(mag, DEC);
//
//  delay(BNO055_SAMPLERATE_DELAY_MS);
//
//  // using LEDs to model some sort of output if each sensor type is calibrated
//  if(gyro == 3){
//    digitalWrite(8, HIGH);
//    tone1.play(NOTE_A4);
//  }
//    if(accel == 3){
//    digitalWrite(9, HIGH);
//    tone1.play(NOTE_A6);
//  }
//    if(mag == 3){
//    digitalWrite(10, HIGH);
//    tone1.play(NOTE_A5);
//  }
//  // exit calibration mode
//  if (gyro == 3 && accel == 3 && mag == 3) {
//    tone1.play(NOTE_A6);
//    delay(200);
//    tone1.stop();
//    systemCalibrated = true;
//  }
//}


// control brightness of LED based on how far turned in either direction
  if(euler.x() >= 5.00 && euler.x() <= 51.00){
   // takes float returned from euler value and scales + offsets to be used as duty cycle input
    dutyCycleRight = (int)euler.x();
    dutyCycleRight = ((dutyCycleRight * 5.1) - 5.1);
    Serial.println(dutyCycleRight);
    analogWrite(6, dutyCycleRight);
    tone1.play(NOTE_A2);
  }

  if(euler.x() >= 309.00 && euler.x() <= 355.00){
    // takes float returned from euler value and scales + offsets to be used as duty cycle input
    dutyCycleLeft = (int)euler.x();
    dutyCycleLeft = ((dutyCycleLeft * (-5)) + 1795);
    Serial.println(dutyCycleLeft);    
    analogWrite(5, dutyCycleLeft);
    tone1.play(NOTE_A5);
  }  
 

if((euler.x() >= 0.00 && euler.x() < 5.00) || (euler.x() > 355.00 && euler.x() <= 360.00)){
    tone1.stop();
  }  

    if(((euler.x() >= 0.00 && euler.x() < 5.00) || (euler.x() > 355.00 && euler.x() <= 360.00)) && ((euler.z() >= 89.00) && (euler.z() <= 90.00))){
    digitalWrite(8, HIGH);
    digitalWrite(9, HIGH);
    digitalWrite(10, HIGH);
  } 
  

  // display Euler Vectors
  Serial.print("X: ");
  Serial.print(euler.x());
  Serial.print(" Y: ");
  Serial.print(euler.y());
  Serial.print(" Z: ");
  Serial.println(euler.z());

  delay(BNO055_SAMPLERATE_DELAY_MS);
}
