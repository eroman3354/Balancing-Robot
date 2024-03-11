#include<Wire.h>
const int MPU = 0x68;
float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY,  gyroAngleZ;
float roll, pitch, yaw;
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
float elapsedTime, currentTime, previousTime;
int c = 0;

float error, prev_error, proportion, integral, derivative;
// Tune Kp, Ki, Kd -- the scaling constants for the PID controller
float target = 0;   // the angle we want the stick at
float Kp = 0.2;     // proportional constant
float Ki = 0.05;     // integral constant
float Kd = 0.01;     // derivative constant
float voltage;      // controller output

// 3rd column is PWM values
// 1st and 2nd columns are directional values
const int wheelPins[4][3] = {
  {9, 10, 8},
  {11, 12, 13},
  {4, 3, 2},
  {6, 5, 7}
};

void setup() {
  // put your setup code here, to run once:
  Serial.begin(19200);    // ?? HOW DO I CHOSE THE VALUE HERE?
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission(true);

  // establish pinsmodes:
  for (int i = 0; i < 4; i++) {
    pinMode(wheelPins[i][0], OUTPUT);
    pinMode(wheelPins[i][1], OUTPUT);
    pinMode(wheelPins[i][2], OUTPUT);
    digitalWrite(wheelPins[i][0], LOW);
    digitalWrite(wheelPins[i][1], LOW);
  }

  // Call this function if you need to get teh IMU error values for your module
  calculate_IMU_error();
  delay(20);

}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.print("\n");
  read_MPU_data();
  pid_controller();
  Serial.print("\n");
  delay(500);
}

void read_MPU_data(){
/*
Adapted from Adruino and MPU6050 Acceleromenter and Gyroscope 
Sensor Tutorial by Dejan
https://howtomechatronics.com
*/
  // === Read accelerometer data ===//
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);      // Start with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // reads 6 registers (3 from accelerometer, 3 from gyroscope)
  
  // For a range of +- 2g, we need to divide the raw values by 16384, according to datasheet
  // ???? is this what we want for our project?
  AccX = (Wire.read() << 8 | Wire.read()) / 16384.0; // Accelermeter X value
  AccY = (Wire.read() << 8 | Wire.read()) / 16384.0; // Accelermeter Y value
  AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0; // Accelermeter Z value
  
  // ??? AccX, AccY, AccZ all reading as 0 for now

  // calculate roll and pitch from accelerometer data
  accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ,2))) * 180 / PI) - 0.58; // 0.58 is AccErrorX
  accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) + 1.58; // AccErrorY ~ -1.58
  Serial.print("accAngleX: ");
  Serial.print(accAngleX);
  Serial.print("/ accAngleY: ");
  Serial.print(accAngleY);
  Serial.print("\n");

  // === Read gyroscope data === //
  previousTime = currentTime;
  currentTime = millis();
  elapsedTime = (currentTime - previousTime)/1000; // convert ms to seconds
  Wire.beginTransmission(MPU);
  Wire.write(0x43); // Ryro data first register address 0x43
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // read 4 registers 2 for x axis, 2 for y axis
  GyroX = (Wire.read() << 8 | Wire.read()) / 131.0; // according to datasheet divide raw value by 131.0 to get 250deg/s range
  GyroY = (Wire.read() << 8 | Wire.read()) / 131.0;
  GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0;
  // correct outputs with calculated error values
  /*GyroX = GyroX + 0.56;
  GyroY = GyroY - 2 ;
  GyroZ = GyroZ + 0.79;*/
  // convert degrees/s to degrees
  gyroAngleX = gyroAngleX + GyroX * elapsedTime;
  gyroAngleY = gyroAngleY + GyroY * elapsedTime;
  // Print the values on the serial monitor
  Serial.print("gyroAngleX: ");
  Serial.print(gyroAngleX);
  Serial.print("/ gyroAngleY: ");
  Serial.print(gyroAngleY);
  Serial.print("\n");
  
  // === Complementary filter - compbine acc and gyro angle values ===//
  yaw = yaw + GyroZ * elapsedTime;
  roll = 0.96 * gyroAngleX + 0.04 * accAngleX;
  pitch = 0.96 * gyroAngleY + 0.04 * accAngleY;
  // Print the values on the serial monitor
  Serial.print("roll: ");
  Serial.print(roll);
  Serial.print("/ pitch: ");
  Serial.print(pitch);
  Serial.print("/ yaw: ");
  Serial.println(yaw);

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
  //Divide the sum by 200 to get the average error value
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
};

void pid_controller(){
  /* uses difference between target angle and measured angle to 
  calculate the voltage to the motor that spins the reaction wheel
  */
  prev_error = error;
  error = target - roll;      // target - measurement
  proportion = Kp * error;
  // integral += Ki * (error * elapsedTime);
  derivative = Kd * (error - prev_error) / elapsedTime;
  voltage = proportion + /*integral + */derivative;
  // constrain voltage from 0 to 12
  voltage = constrain(voltage, -12, 12);
  // will need to sum the remote control voltage with the balancing voltage
  
  // if the summed voltage is less than zero, then digitalWrite the motor to spin ccw
  // else if greater than or equal to zero, then digitalWrite the motor to spin cw

  //convert the voltage to PWM:
  int PWM = 0;//constrain((voltage * 255 / 12), 0, 60); // change to constrain to 255
  
  /*if(voltage < 0){
    // WRITE DIRECTIONS
    // motor 1
    digitalWrite(wheelPins[0][0], LOW);
    digitalWrite(wheelPins[0][1], HIGH);
    // motor 2
    digitalWrite(wheelPins[1][0], HIGH);
    digitalWrite(wheelPins[1][1], LOW);
    // motor 3
    digitalWrite(wheelPins[2][0], LOW);
    digitalWrite(wheelPins[2][1], HIGH);
    // motor 4
    digitalWrite(wheelPins[3][0], HIGH);
    digitalWrite(wheelPins[3][1], LOW);

  }
  else{
    // WRITE DIRECTION
    // motor 1
    digitalWrite(wheelPins[0][0], HIGH);
    digitalWrite(wheelPins[0][1], LOW);
    // motor 2
    digitalWrite(wheelPins[1][0], LOW);
    digitalWrite(wheelPins[1][1], HIGH);
    // motor 3
    digitalWrite(wheelPins[2][0], HIGH);
    digitalWrite(wheelPins[2][1], LOW);
    // motor 4
    digitalWrite(wheelPins[3][0], LOW);
    digitalWrite(wheelPins[3][1], HIGH);
  }

  // WRITE SPEED
  for(int i = 0; i < 4; i++){
    analogWrite(wheelPins[i][2], PWM);
  }*/

  // print the voltage values:
  Serial.print("voltage: ");
  Serial.print(voltage);
}
