package y2016_balloon.gps;

message Gps {
  double latitude;
  double longitude;
  double speed;
  double altitude;
  double course;
};
queue Gps gps;
