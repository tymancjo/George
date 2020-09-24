/*
   this fileis intended to get data fromMPU and then drive effcently stepmotors
   for self balancing robot.

   idea: 
    read the MPU
    calc the PID output as speed
    change speed to impulse delay for motors
   --> in timer based int every 20us - makepropperly spaced apart motor impulses
   
   MPU idea based on:
   Arduino and MPU6050 Accelerometer and Gyroscope Sensor Tutorial
   by Dejan, https://howtomechatronics.com
*/

#include <Wire.h>


// for MPU communication
const int MPU = 0x68; // MPU6050 I2C address
float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
float roll, pitch, yaw;
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
float elapsedTime, currentTime, previousTime;
int c = 0;

//============ stuff for PID ===========
//based on: https://wired.chillibasket.com/2015/03/pid-controller/
//
float Kp = 1;          // (P)roportional Tuning Parameter
float Ki = 0.0;          // (I)ntegral Tuning Parameter
float Kd = 0;          // (D)erivative Tuning Parameter
float iTerm = 0;       // Used to accumulate error (integral)
float lastTime = 0;    // Records the last time function was called
float maxPID = 10000;    // The maximum value that can be output
float oldValue = 0;    // The last sensor value

// target values
float target = 0.0;
const float delta = 1;
const int motMpl = 20;
const int steps_per_revolution = 200;

const int maxAngle = 20; // we stops motors if toomuch of roll.

// ============ TymancjO ============
// parametry fizyczne
const int freq = 16.0; //[MHz]
const int steps_per_rev = 200;
const float k = 2 * 3.1415 / steps_per_rev; 
const float wheel_D_mm = 90.0; // [mm]
const float R = wheel_D_mm / 2; //[mm]
const int T0_delay_counter = 39; 
const int T0 = (T0_delay_counter + 1) / (freq / 8);

// for steppers

unsigned long globaldel =  65536000;
int throttle_counter_left_motor, throttle_counter_right_motor;

void setup() {
  Serial.begin(19200);
  Wire.begin();                      // Initialize comunication
  Wire.beginTransmission(MPU);       // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);                  // Talk to the register 6B
  Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);        //end the transmission
  /*
    // Configure Accelerometer Sensitivity - Full Scale Range (default +/- 2g)
    Wire.beginTransmission(MPU);
    Wire.write(0x1C);                  //Talk to the ACCEL_CONFIG register (1C hex)
    Wire.write(0x10);                  //Set the register bits as 00010000 (+/- 8g full scale range)
    Wire.endTransmission(true);
    // Configure Gyro Sensitivity - Full Scale Range (default +/- 250deg/s)
    Wire.beginTransmission(MPU);
    Wire.write(0x1B);                   // Talk to the GYRO_CONFIG register (1B hex)
    Wire.write(0x10);                   // Set the register bits as 00010000 (1000deg/s full scale)
    Wire.endTransmission(true);
    delay(20);
  */
  // Call this function if you need to get the IMU error values for your module
  //calculate_IMU_error();
  //delay(10);

  //========== TymancjO ============
  //To create a variable pulse for controlling the stepper motors a timer is created that will execute a piece of code (subroutine) every 20us
  //This subroutine is called TIMER2_COMPA_vect
  TCCR2A = 0;                                                               //Make sure that the TCCR2A register is set to zero
  TCCR2B = 0;                                                               //Make sure that the TCCR2A register is set to zero
  TIMSK2 |= (1 << OCIE2A);                                                  //Set the interupt enable bit OCIE2A in the TIMSK2 register
  TCCR2B |= (1 << CS21);                                                    //Set the CS21 bit in the TCCRB register to set the prescaler to 8
  OCR2A = T0_delay_counter;                                                 //The compare register is set to 39 => 20us / (1s / (16.000.000MHz / 8)) - 1
  TCCR2A |= (1 << WGM21);                                                   //Set counter 2 to CTC (clear timer on compare) mode

  // preparing the pins for steppers
  pinMode(6,OUTPUT);
  pinMode(7,OUTPUT);
  pinMode(8,OUTPUT);
  pinMode(9,OUTPUT);

  Serial.println("starting...");

}


void loop() {
  // === Read acceleromter data === //
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
  //For a range of +-2g, we need to divide the raw values by 16384, according to the datasheet
  AccX = (Wire.read() << 8 | Wire.read()) / 16384.0; // X-axis value
  AccY = (Wire.read() << 8 | Wire.read()) / 16384.0; // Y-axis value
  AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0; // Z-axis value
  // Calculating Roll and Pitch from the accelerometer data
  accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) - 0.58; // AccErrorX ~(0.58) See the calculate_IMU_error()custom function for more details
  accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) + 1.58; // AccErrorY ~(-1.58)

  // === Read gyroscope data === //
  previousTime = currentTime;        // Previous time is stored before the actual time read
  currentTime = millis();            // Current time actual time read
  elapsedTime = (currentTime - previousTime) / 1000; // Divide by 1000 to get seconds
  Wire.beginTransmission(MPU);
  Wire.write(0x43); // Gyro data first register address 0x43
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 4 registers total, each axis value is stored in 2 registers
  GyroX = (Wire.read() << 8 | Wire.read()) / 131.0; // For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
  GyroY = (Wire.read() << 8 | Wire.read()) / 131.0;
  GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0;
  // Correct the outputs with the calculated error values
  GyroX = GyroX - 67.94; // GyroErrorX
  GyroY = GyroY + 0.09; // GyroErrorY
  GyroZ = GyroZ + 0.39; // GyroErrorZ

  // Currently the raw values are in degrees per seconds, deg/s, so we need to multiply by sendonds (s) to get the angle in degrees
  gyroAngleX = gyroAngleX + GyroX * elapsedTime; // deg/s * s = deg
  gyroAngleY = gyroAngleY + GyroY * elapsedTime;
  yaw =  yaw + GyroZ * elapsedTime;

  // Complementary filter - combine acceleromter and gyro angle values
  roll = 0.90 * gyroAngleX + 0.1 * accAngleX;
  pitch = 0.90 * gyroAngleY + 0.1 * accAngleY;


  // doing the PID

  float pid_output = pid(target, pitch);

  if ((pitch < target - delta) || (pitch > target + delta) && abs(pitch < maxAngle)) {
    // this sets the global variable used by int subroutine
    globaldel = time_from_speed(pid_output);
  }

// Print the values on the serial monitor
//  Serial.print(roll);
//  Serial.print("/");
//  Serial.print(pitch);
//  Serial.print("/");
//  Serial.print(yaw);
//  Serial.print("/PID: ");
//  Serial.print(pid_output);
//  Serial.print(":::");
//  Serial.println(globaldel);


  //  delay(100);
}


void calculate_IMU_error() {
  // We can call this funtion in the setup section to calculate the accelerometer and gyro data error. From here we will get the error values used in the above equations printed on the Serial Monitor.
  // Note that we should place the IMU flat in order to get the proper values, so that we then can the correct values
  // Read accelerometer values 200 times
  while (c < 200) {
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    AccX = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    AccY = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    // Sum all readings
    AccErrorX = AccErrorX + ((atan((AccY) / sqrt(pow((AccX), 2) + pow((AccZ), 2))) * 180 / PI));
    AccErrorY = AccErrorY + ((atan(-1 * (AccX) / sqrt(pow((AccY), 2) + pow((AccZ), 2))) * 180 / PI));
    c++;
  }
  //Divide the sum by 200 to get the error value
  AccErrorX = AccErrorX / 200;
  AccErrorY = AccErrorY / 200;
  c = 0;
  // Read gyro values 200 times
  while (c < 200) {
    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    GyroX = Wire.read() << 8 | Wire.read();
    GyroY = Wire.read() << 8 | Wire.read();
    GyroZ = Wire.read() << 8 | Wire.read();
    // Sum all readings
    GyroErrorX = GyroErrorX + (GyroX / 131.0);
    GyroErrorY = GyroErrorY + (GyroY / 131.0);
    GyroErrorZ = GyroErrorZ + (GyroZ / 131.0);
    c++;
  }
  //Divide the sum by 200 to get the error value
  GyroErrorX = GyroErrorX / 200;
  GyroErrorY = GyroErrorY / 200;
  GyroErrorZ = GyroErrorZ / 200;
  // Print the error values on the Serial Monitor
  Serial.print("AccErrorX: ");
  Serial.println(AccErrorX);
  Serial.print("AccErrorY: ");
  Serial.println(AccErrorY);
  Serial.print("GyroErrorX: ");
  Serial.println(GyroErrorX);
  Serial.print("GyroErrorY: ");
  Serial.println(GyroErrorY);
  Serial.print("GyroErrorZ: ");
  Serial.println(GyroErrorZ);
}


int time_from_speed( float V ){
  // this function recalculate the required delay time for impulsed for given speed

  float delayTime = ((k * R*1e-3) / abs(V))*1e6 - T0;
  int delayLoops = 2 + 2 + (int) delayTime / T0;
//  if (delayLoops < 2) delayLoops = 2;
    
  return delayLoops; // retrning number of delayloops
  } 


/**
   PID Controller
   @param  (target)  The target position/value we are aiming for
   @param  (current) The current value, as recorded by the sensor
   @return The output of the controller
*/
float pid(float target, float current) {
  // Calculate the time since function was last called
  float thisTime = micros();
  float dT = (thisTime - lastTime) * 0.000001; // to have dt in [s]
  lastTime = thisTime;

  // Calculate error between target and current values
  float error = target - current;

  // Calculate the integral term
  iTerm += error * dT;

  // Calculate the derivative term (using the simplification)
  float dTerm = (oldValue - current) / dT;

  // Set old variable to equal new ones
  oldValue = current;

  // Multiply each term by its constant, and add it all up
  float result = (error * Kp) + (iTerm * Ki) + (dTerm * Kd);

  // Limit PID value to maximum values
  if (result > maxPID) result = maxPID;
  else if (result < -maxPID) result = -maxPID;
  if (isnan(result)) result = 0;
  return result;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Interrupt routine  TIMER2_COMPA_vect
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
ISR(TIMER2_COMPA_vect){
  //Left motor pulse calculations
  throttle_counter_left_motor ++;                                           //Increase the throttle_counter_left_motor variable by 1 every time this routine is executed
  
  if(throttle_counter_left_motor > globaldel){             //If the number of loops is larger then the throttle_left_motor_memory variable
    throttle_counter_left_motor = 0;                                        //Reset the throttle_counter_left_motor variable
  }
  else if(throttle_counter_left_motor == 1)PORTD |= 0b01000000;             //Set output 2 high to create a pulse for the stepper controller
  else if(throttle_counter_left_motor == 2)PORTD &= 0b10111111;             //Set output 2 low because the pulse only has to last for 20us 
 
  //right motor pulse calculations
  throttle_counter_right_motor ++;                                          //Increase the throttle_counter_right_motor variable by 1 every time the routine is executed
  if(throttle_counter_right_motor > globaldel){           //If the number of loops is larger then the throttle_right_motor_memory variable
    throttle_counter_right_motor = 0;                                       //Reset the throttle_counter_right_motor variable
//    throttle_right_motor_memory = throttle_right_motor;                     //Load the next throttle_right_motor variable
//    if(throttle_right_motor_memory < 0){                                    //If the throttle_right_motor_memory is negative
//      PORTD |= 0b00100000;                                                  //Set output 5 low to reverse the direction of the stepper controller
//      throttle_right_motor_memory *= -1;                                    //Invert the throttle_right_motor_memory variable
//    }
//    else PORTD &= 0b11011111;                                               //Set output 5 high for a forward direction of the stepper motor
  }
  else if(throttle_counter_right_motor == 1)PORTB |= 0b00000001;            //Set output 4 high to create a pulse for the stepper controller
  else if(throttle_counter_right_motor == 2)PORTB &= 0b11111110;            //Set output 4 low because the pulse only has to last for 20us
}
