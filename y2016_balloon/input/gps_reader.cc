#include <gps.h>

#include "aos/linux_code/init.h"
#include "y2016_balloon/input/gps.q.h"

int main(int argc, char **argv) {
  ::aos::Init();
  gps_init();
  loc_t gps_data;

  while (true) {
    gps_location(&gps_data);

    ::y2016_balloon::gps::gps.MakeWithBuilder()
        .latitude(gps_data.latitude)
        .longitude(gps_data.longitude)
        .speed(gps_data.speed)
        .altitude(gps_data.altitude)
        .course(gps_data.course)
        .Send();
  }

  ::aos::Cleanup();
  return 0;
}
