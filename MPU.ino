
#include <Wire.h>           // Communication with MPU6050
#include <LiquidCrystal.h>  // Operation of LCD
const int BTN_PIN = 1;      // For mode button
const int MPU = 0x68;       // MPU6050 I2C address
float AccX, AccY, AccZ;     // Raw accelerometer data 
float GyroX, GyroY, GyroZ;  // Raw gyroscope data
float accAngleX, accAngleY; // Calculated accelerometer angles
float gyroAngleX, gyroAngleY, gyroAngleZ;  // Calculated gyroscope angles
float roll, pitch;
unsigned long elapsedTime = 0;
unsigned long currentTime = 0;
unsigned long previousTime = 0;
unsigned long devTime = 0;

bool mode = true;           // true == sensing mode; false == pause mode
bool bat = true;            // true == battery high; false == battery low, must switch
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
int c = 0;
float roll_bound = 1.0;
float pitch_bound = 1.0;


float cal_roll = 0.0;
float cal_pitch = 0.0;
float offset_roll = 0.0;
float offset_pitch = 0.0;

// initialize the library by associating any needed LCD interface pin
// with the arduino pin number it is connected to
const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 2;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

void setup() {
  Serial.begin(19200);
  pinMode(BTN_PIN, INPUT);           // Initialize mode button
  
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
  calculate_IMU_error();


  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);
  
  delay(20);
}





void loop() {
  checkMode();  
  LCD();    
  if(mode == false){  
    goto bailout;
  }
  // sensing
  bailout:
  delay(10);
  
  // Print the values on the serial monitor
  Serial.print(roll);
  Serial.print("/");
  Serial.print(pitch);

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
    AccX = (Wire.read() << 8 | Wire.read()) / 8192 ;
    AccY = (Wire.read() << 8 | Wire.read()) / 8192 ;
    AccZ = (Wire.read() << 8 | Wire.read()) / 8192 ;
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
void read_acc(){
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
  Wire.requestFrom(MPU, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
  GyroX = (Wire.read() << 8 | Wire.read()) / 131.0; // For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
  GyroY = (Wire.read() << 8 | Wire.read()) / 131.0;
  GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0;
  // Correct the outputs with the calculated error values
  GyroX = GyroX + 0.56; // GyroErrorX ~(-0.56)
  GyroY = GyroY - 2; // GyroErrorY ~(2)
  GyroZ = GyroZ + 0.79; // GyroErrorZ ~ (-0.8)
  
  // Currently the raw values are in degrees per seconds, deg/s, so we need to multiply by sendonds (s) to get the angle in degrees
  gyroAngleX = gyroAngleX + GyroX * elapsedTime; // deg/s * s = deg
  gyroAngleY = gyroAngleY + GyroY * elapsedTime;
  
  // Complementary filter - combine acceleromter and gyro angle values
  gyroAngleX = 0.96 * gyroAngleX + 0.04 * accAngleX;
  gyroAngleY = 0.96 * gyroAngleY + 0.04 * accAngleY;

  roll = gyroAngleX;
  pitch = gyroAngleY;
  
  //return roll, pitch;

  
}

long sum(float arr[], int size_of_array) {
  long sum = 0;
  for (int i=0; i < size_of_array; i++) {
      sum = sum + arr[i];
  }

  return sum;
}

float stan_dev(float arr[], float mean_arr,int size_of_array) {
  float std1 = 0.0;
  float sum_num=0.0;
  float sub = 0.0;
  float sq_sub = 0.0;
  float alm_std = 0.0;
  for (int i=0; i < size_of_array; i++) {
      sub = arr[i] - mean_arr;
      sq_sub = sq(sub);
      sum_num += sq_sub;
  }

  alm_std = sum_num / size_of_array;
  std1 = sqrt(alm_std);

  return std1;

}

void calibrate() { 
  int cal_count = 100; // Number of samples to take

  // Create 3 lists to store readings
  float roll_arr[cal_count];
  float pitch_arr[cal_count];
  //vecotr<float> yaw_vect [];
  // Get readings from gyroscope
  for (int i = 0; i < cal_count-1; i++) {      
    read_acc();

    roll_arr[i] = roll;
    pitch_arr[i] = pitch;
    //roll_vect.push_back(roll);
    //pitch_vect.push_back(pitch);
    //yaw_vect.push_back(yaw);

    delay(100);
  }

  float sum_roll =  sum(roll_arr, cal_count);
  float sum_pitch = sum(pitch_arr, cal_count);


  // Take the average of all values after 5 seconds and Standard deviation
  cal_roll = sum_roll / cal_count;
  cal_pitch = sum_pitch / cal_count;
  //cal_yaw = average(yaw_vect);
  float flat = 5.0;
  offset_roll  = stan_dev(roll_arr, cal_roll, cal_count) + flat;
  offset_pitch = stan_dev(pitch_arr, cal_pitch, cal_count) + flat;

  // Store averages and std's and return them
  return cal_roll, cal_pitch, offset_roll, offset_pitch;
}





void compare(float roll, float pitch){
  
}



void LCD(){
  
  //setCursor(column, row) indexed starting at 0
  
  lcd.setCursor(0, 0);    // sets cursor to first pixel of first line
  if(mode){               // prints sense if mode is true
    lcd.print("sense");
  }
  else{                   // prints pause if mode is false
    lcd.print("pause");
  }
  lcd.setCursor(9,0);    // sets cursor to 10th pixel of the first line
  if(bat){                // prints "BAT: HI" if bat is true
    lcd.print("BAT: HI");
  }
  else{                   // prints "BAT: LO" if bat is false
    lcd.print("BAT: LO");
  }
  lcd.setCursor(0,1);     // sets cursor to the first pixel of the second line
  lcd.print("time: ");
  lcd.print(devTime);
}



void checkBat(){
  int sensorValue = analogRead(A0);
  float voltage = sensorValue * (9.0/1023.0);

  if (voltage <=7.4){
    bat = false;
  }
  else{
    bat = true;
  }
}

void checkMode(){
  uint8_t state = digitalRead(BTN_PIN);
  if(state = HIGH){     //HIGH == sense
    mode = true;
  }
  else{
    mode = false;      //LOW == pause
  }
}
