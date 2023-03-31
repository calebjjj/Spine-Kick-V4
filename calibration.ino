#include <numeric>
#include <vector>

double average(vecotr<float> const& v) {
  if(v.empty()) {
    return 0;
  }

  auto const count = static_cast<float>(v.size());
  return (reduce(v.begin(), v.end()) / count);
}


void calibrate(roll_vect, pitch_vect, yaw_vect) {//could have it take the vectors as inputs instead) 

  // Create 3 lists to store readings
  vector<float> roll_vect = [];
  vector<float> pitch_vect = [];
  vecotr<float> yaw_vect [];
  // Get readings from gyroscope
  for (int i = 0; i < 100; i++) {      
    roll, pitch, yaw = read_acc();

    roll_vect.push_back(roll);
    pitch_vect.push_back(pitch;)
    yaw_vect.push_back(yaw);

    delay(100)
}

  // Take the average of all values after 5 seconds and Standard deviation
  cal_roll = average(roll_vect);
  cal_pitch = average(pitch_vect);
  cal_yaw = average(yaw_vect);
  // Store averages and std's and return them

  return cal_roll, cal_pitch, cal_yaw
}
