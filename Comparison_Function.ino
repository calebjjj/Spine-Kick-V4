unsigned long Time_elapsed = 0;

// calibrated values for XYZ representing the good posture
float cal_acc_AngleX
float cal_acc_AngleY
float cal_acc_AngleZ
float cal_gyro_AngleY
float cal_gyro_AngleZ

void compare(float  accAngleX, float  accAngleY, float gyroAngleX, float gyroAngleY, float  gyroAngleZ){
  unsigned long initial = millis();

  if( accAngleX - cal_acc_AngleX >= set_bound ){
    //vibrate motor
    //turn timer on:
    //Time_elapsed = Time_elapsed + (millis()-initial);

    
  }
if( accAngleY - cal_acc_AngleY >= set_bound ){
    //vibrate motor
    //turn timer on
  }
if( gyroAngleX - cal_gyro_AngleX >= set_bound ){
    //vibrate motor
    //turn timer on
  }
if( gyroAngleY - cal_gyro_AngleY >= set_bound ){
    //vibrate motor
    //turn timer on
  }

if( gyroAngleZ - cal_gyro_AngleZ >= set_bound ){
    //vibrate motor
    //turn timer on
  }


}
