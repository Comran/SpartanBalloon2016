#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <time.h>
#include <string.h>
#include <string>
#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>
#include <fcntl.h>
#include <dirent.h>
#include <mntent.h>

#include <map>
#include <unordered_set>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

#include "aos/linux_code/init.h"
#include "aos/common/time.h"

#include "y2016_balloon/input/sensors.q.h"
#include "y2016_balloon/input/gps.q.h"

// Whether to store a local copy of the log data on the SD card.
//#define BACKUP_LOG

int main() {
  aos::InitNRT();

  // Log file, in comma-separated value format to make it easy to spreadsheet
  // the data.
  ::std::ofstream log_file("/media/logger/log.csv", ::std::ios::app);
  log_file << ::std::endl << "BEGIN NEW LOG" << ::std::endl;
#ifdef BACKUP_LOG
  ::std::ofstream log_file_backup("/home/comran/log.csv", ::std::ios::app);
  log_file_backup << ::std::endl << "BEGIN NEW LOG" << ::std::endl;
#endif

  while (true) {
    ::aos::time::Time now = ::aos::time::Time::Now();

    ::std::stringstream new_log_data;
    new_log_data << ::std::endl;
    new_log_data << now.sec() << "." << now.nsec() << ",";

    // Poll our sensor queues for new data.
    ::y2016_balloon::sensors::sensors.FetchNextBlocking();
    new_log_data.precision(10);
    new_log_data << "s,";
    new_log_data << ::y2016_balloon::sensors::sensors->bmp180_temp << ",";
    new_log_data << ::y2016_balloon::sensors::sensors->bmp180_pressure << ",";
    new_log_data << ::y2016_balloon::sensors::sensors->bmp180_altitude << ",";

    new_log_data << ::y2016_balloon::sensors::sensors->ax << ",";
    new_log_data << ::y2016_balloon::sensors::sensors->ay << ",";
    new_log_data << ::y2016_balloon::sensors::sensors->az << ",";

    new_log_data << ::y2016_balloon::sensors::sensors->cx << ",";
    new_log_data << ::y2016_balloon::sensors::sensors->cy << ",";
    new_log_data << ::y2016_balloon::sensors::sensors->cz << ",";

    new_log_data << ::y2016_balloon::sensors::sensors->ms5803_temp << ",";
    new_log_data << ::y2016_balloon::sensors::sensors->ms5803_pressure;

    // Throw in the GPS data if there's a new message.
    if (::y2016_balloon::gps::gps.FetchLatest()) {
      new_log_data << ::std::endl << "g,";
      new_log_data << ::y2016_balloon::gps::gps->latitude << ",";
      new_log_data << ::y2016_balloon::gps::gps->longitude << ",";
      new_log_data << ::y2016_balloon::gps::gps->speed << ",";
      new_log_data << ::y2016_balloon::gps::gps->altitude << ",";
      new_log_data << ::y2016_balloon::gps::gps->course;
    }

    // Write to file.
    log_file << new_log_data.str();
#ifdef BACKUP_LOG
    log_file_backup << new_log_data.str();
#endif
  }

  aos::Cleanup();
  return 0;
}
