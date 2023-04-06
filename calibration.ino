long cal_roll = 0;
long cal_pitch = 0;
long offset_roll = 0;
long offset_pitch = 0;

long sum(arr, size_of_array) {
  long sum = 0;
  for (int i=0; i < size_of_array; i++) {
      sum = sum + arr[i];
  }

  return sum;
}

long std(arr, mean_arr, size_of_array) {
  long std = 0;
  long sum_num=0;

  for (int i=0; i < size_of_array; i++) {
      long sub = arr[i] - mean_arr;
      long sq_sub = sq(sub);
      sum_num += sq_sub;
  }

  long alm_std = sum_num / size_of_array;
  long std = sqrt(alm_std);

  return std;

}

void calibrate() {//, yaw_vect) {//could have it take the vectors as inputs instead) 
  int cal_count = 100; // Number of samples to take

  // Create 3 lists to store readings
  long roll_arr[cal_count];
  long pitch_arr[cal_count];
  //vecotr<float> yaw_vect [];
  // Get readings from gyroscope
  for (int i = 0; i < cal_count-1; i++) {      
    roll, pitch = read_acc();

    roll_arr[i] = roll;
    pitch_arr[i] = pitch;
    //roll_vect.push_back(roll);
    //pitch_vect.push_back(pitch);
    //yaw_vect.push_back(yaw);

    delay(100);
  }

  long sum_roll =  sum(roll_arr, cal_count);
  long sum_pitch = sum(pitch_arr, cal_count);


  // Take the average of all values after 5 seconds and Standard deviation
  cal_roll = sum_roll / cal_count;
  cal_pitch = sum_pitch / cal_count;
  //cal_yaw = average(yaw_vect);
  offset_roll  = 5 + std(roll_arr, cal_roll, cal_count);
  offset_pitch = 5 + std(pitch_arr, cal_pitch, cal_count);

  // Store averages and std's and return them
  return cal_roll, cal_pitch, offset_roll, offset_pitch;
}
