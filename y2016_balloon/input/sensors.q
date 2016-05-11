package y2016_balloon.sensors;

message Sensors {
	double bmp180_temp;
  double bmp180_pressure;
  double bmp180_altitude;

  int16_t mpu_temp;
  int16_t ax;
  int16_t ay;
  int16_t az;

  int16_t cx;
  int16_t cy;
  int16_t cz;

  int32_t ms5803_temp;
  int32_t ms5803_pressure;

  double gps_latitude;
  double gps_longitude;
  double gps_speed;
  double gps_altitude;
  double gps_course;
};
queue Sensors sensors;
