
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
unsigned long devTime = 0;  // time deviated from baseline 

bool mode = true;           // true == sensing mode; false == pause mode
bool bat = true;            // true == battery high; false == battery low, must switch
bool dev = false;           // true == deviating from good posture; false = posture is good
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
int c = 0;


float cal_roll = 0.0;       // calibrated roll value
float cal_pitch = 0.0;      // calibrated pitch value
// gives maximum deviation allowed 
float offset_roll = 0.0;
float offset_pitch = 0.0;   

// initialize the library by associating any needed LCD interface pin
// with the arduino pin number it is connected to
const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 2;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

// output pin for haptic motor
const int motor = 6;
 

void setup() {
  Serial.begin(19200);
  pinMode(BTN_PIN, INPUT);           // Initialize mode button
  pinMode(motor, OUTPUT);            // Initialize haptic motor 
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

  
  // Setup function to find IMU error
  // need to only run for one time
  calculate_IMU_error();


  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);

  // finds calibrated values
  calibrate();
  delay(20);
}





void loop() {
  
  checkMode();                // updates the current mode of device (pause/sense)
  checkBat();                 // checks the battery status
  LCD();                      // displays updated mode
  
  if(mode == false){          // if on pause mode, skips to bailout
    goto bailout;
  }
  
  read_acc();
  compare();                   // checks the position state of the user
  // if true, user is deviating from standard posure and the haptic motor will turn on
  if(dev){
    digitalWrite(motor, HIGH);
    delay(100);
    // updates time deviated from posture
    devTime = devTime + 100;
  }
  // turns off motor after alerting the user
  digitalWrite(motor, LOW);
  //updates LCD 
  LCD();

  bailout:
  delay(10);                  // delays 10 ms and goes onto the next loop
  
  
}
void calculate_IMU_error() {
  
  // calculates the accelerometer and gyro data error. IMU should be placed flat 
  // Read accelerometer values 200 times
  while (c < 200) {
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    AccX = (Wire.read() << 8 | Wire.read()) / 8192.0 ;
    AccY = (Wire.read() << 8 | Wire.read()) / 8192.0 ;
    AccZ = (Wire.read() << 8 | Wire.read()) / 8192.0 ;
    
    // Sum all readings
    // accAngleX and accAngleY should be 0, any deviations calculated is considered aas error
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
    // All gyro readings should be 0 as well
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
  Wire.beginTransmission(MPU);                  // connects to MPU
  Wire.write(0x3B);                             // Start with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);                  // continues wire transmission
  Wire.requestFrom(MPU, 6, true);               // Read 6 registers total, each axis value is stored in 2 registers
  //+-2g requires division by 8192 according to the data sheet
  AccX = (Wire.read() << 8 | Wire.read()) / 8192.0; // X-axis value
  AccY = (Wire.read() << 8 | Wire.read()) / 8192.0; // Y-axis value
  AccZ = (Wire.read() << 8 | Wire.read()) / 8192.0; // Z-axis value
  // Calculating Roll and Pitch from the accelerometer data
  accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) - 0.58;      // AccErrorX ~(0.58) See the calculate_IMU_error()custom function for more details
  accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) + 1.58; // AccErrorY ~(-1.58)
  // === Read gyroscope data === //
  previousTime = currentTime;                        // Previous time is stored before the actual time read
  currentTime = millis();                            // Current time actual time read
  elapsedTime = (currentTime - previousTime) / 1000; // Divide by 1000 to get seconds
  Wire.beginTransmission(MPU);                  // connects to MPU
  Wire.write(0x43);                             // Gyro data first register address 0x43
  Wire.endTransmission(false);                  // continues wire transmission
  Wire.requestFrom(MPU, 6, true);               // Read 6 registers total, each axis value is stored in 2 registers
  GyroX = (Wire.read() << 8 | Wire.read()) / 131.0; // For a 250deg/s range divide first the raw value by 131.0, according to the datasheet
  GyroY = (Wire.read() << 8 | Wire.read()) / 131.0;
  GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0;
  // Correct the outputs with the calculated error values
  GyroX = GyroX + 0.56; // GyroErrorX ~(-0.56)
  GyroY = GyroY - 2; // GyroErrorY ~(2)
  GyroZ = GyroZ + 0.79; // GyroErrorZ ~ (-0.8)
  
  // current units are in deg/s. Multiply by seconds to get degrees
  gyroAngleX = gyroAngleX + GyroX * elapsedTime; // deg/s * s = deg
  gyroAngleY = gyroAngleY + GyroY * elapsedTime;
  
  // Complementary filter - combine acceleromter and gyro angle values
  // Gyroscope is more accurate but tend to deviate over time, accelerometer makes sures time error takes longer to build up 
  gyroAngleX = 0.96 * gyroAngleX + 0.04 * accAngleX;
  gyroAngleY = 0.96 * gyroAngleY + 0.04 * accAngleY;

  // Updates roll and pitch values
  roll = gyroAngleX;
  pitch = gyroAngleY;
  
 

  
}

// returns the sum of values in an array
float sum(float arr[], int size_of_array) {
  float sum = 0;
  for (int i=0; i < size_of_array; i++) {
      sum = sum + arr[i];
  }

  return sum;
}

// returns the standard deviation of an array
float stan_dev(float arr[], float mean_arr,int size_of_array) {
  float std1 = 0.0;       // the standard deviation
  float sub = 0.0;        // temporary value to hold the difference between each iteration and the mean
  
  float sq_sub = 0.0;     // holds the square of the difference
  float sum_num=0.0;      // holds the sum of the square differences   
  float alm_std = 0.0;    
  
  for (int i=0; i < size_of_array; i++) {
      sub = arr[i] - mean_arr;
      sq_sub = sq(sub);
      sum_num += sq_sub;
  }

  // applying standard deviation formula
  alm_std = sum_num / size_of_array;
  std1 = sqrt(alm_std);

  return std1;

}

void calibrate() { 
  int cal_count = 100; // Number of samples to take

  // Create 3 lists to store readings
  float roll_arr[cal_count];
  float pitch_arr[cal_count];

  // delays 10 seconds before taking readings
  delay(10000);

  // turns motor on to alert user calibration is occuring
  digitalWrite(motor, HIGH);

  // Get readings from MPU6050
  for (int i = 0; i < cal_count-1; i++) {    

    // updates roll and pitch values
    read_acc();


    // adds roll and pitch values into the array
    roll_arr[i] = roll;
    pitch_arr[i] = pitch;
   

    delay(100);
  }
 
  // sums the arrays
  float sum_roll =  sum(roll_arr, cal_count);
  float sum_pitch = sum(pitch_arr, cal_count);


  // Take the average of all values after 5 seconds and Standard deviation
  cal_roll = sum_roll / cal_count;
  cal_pitch = sum_pitch / cal_count;

  // flat is the predetermined flat offset from mean of calibrated values 
  float flat = 0.1;

  // offset includes the standard deviation of the calibrated values 
  offset_roll  = stan_dev(roll_arr, cal_roll, cal_count) + flat;
  offset_pitch = stan_dev(pitch_arr, cal_pitch, cal_count) + flat;

  // Store averages and std's and return them
  return cal_roll, cal_pitch, offset_roll, offset_pitch;

  // turns motor off to indicate callibration has ended
  digitalWrite(motor, LOW);
}





void compare(){
  // if distance between calibrated and current position values is greater than offset for either roll or pitch
  // sets dev to true
  if(abs(roll-cal_roll) >= offset_roll){
    dev = true;
  }
  else{
    dev = false;
  }
   if(abs(pitch-cal_pitch) >= offset_pitch){
    dev = true;
  }
  else{
    dev = false;
  }
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
  // Prints time deviated from baseline posture 
  lcd.print("time: ");    
  lcd.print(devTime);
}



void checkBat(){
  // reads input voltage levels
  int sensorValue = analogRead(A0);

  // applies formula to find voltage
  float voltage = sensorValue * (9.0/1023.0);

  // if the voltage is below 7.4 volts, returns false, else returns true
  if (voltage <=7.4){
    bat = false;
  }
  else{
    bat = true;
  }
}

void checkMode(){
  //reads the input state on the mode button
  uint8_t state = digitalRead(BTN_PIN);
  if(state = HIGH){     //HIGH == sense
    mode = true;
  }
  else{
    mode = false;      //LOW == pause
  }
}
