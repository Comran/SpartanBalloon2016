#include "y2016_balloon/http_status/http_status.h"

#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "seasocks/Server.h"

#include "aos/linux_code/init.h"
#include "aos/common/logging/logging.h"
#include "aos/common/time.h"
#include "aos/common/util/phased_loop.h"
#include "aos/common/mutex.h"

#include "y2016_balloon/http_status/embedded.h"
#include "y2016_balloon/input/sensors.q.h"
#include "y2016_balloon/input/gps.q.h"

namespace frc971 {
namespace http_status {

// TODO(comran): Make some of these separate libraries & document them better.

HTTPStatusMessage::HTTPStatusMessage()
    : sample_id_(0),
      measure_index_(0),
      overflow_id_(200) {}

void HTTPStatusMessage::NextSample() {
  int32_t adjusted_index = GetIndex(sample_id_);

  ::aos::time::Time time_now = ::aos::time::Time::Now();

  if (sample_id_ < overflow_id_) {
    sample_times_.emplace_back(time_now);
    data_values_.emplace_back(::std::vector<double>());
  } else {
    sample_times_[adjusted_index] = time_now;
  }

  sample_id_++;
  measure_index_ = 0;

  CHECK(!mutex_.Lock());  // Lock the mutex so measures can be added.
}

void HTTPStatusMessage::EndSample() { mutex_.Unlock(); }

int32_t HTTPStatusMessage::GetIndex(int32_t sample_id) {
  return sample_id % overflow_id_;
}

void HTTPStatusMessage::AddMeasure(::std::string name, double value) {
  // Mutex should be locked when this method is called to synchronize packets.
  CHECK(mutex_.OwnedBySelf()); //TODO(comran): Used to be assert(...);

  int32_t index = GetIndex(sample_id_ - 1);

  if (measure_index_ >= static_cast<int32_t>(data_names_.size())) {
    data_names_.emplace_back(name);
  }

  if (measure_index_ >= static_cast<int32_t>(data_values_.at(index).size())) {
    data_values_.at(index).emplace_back(value);
  } else {
    data_values_.at(index).at(measure_index_) = value;
  }
  measure_index_++;
}

::std::string HTTPStatusMessage::Fetch(size_t from_sample) {
  ::aos::MutexLocker locker(&mutex_);

  ::std::stringstream message;
  message.precision(10);  // Cap how precise the time/measurement data is.

  // To save space, data is being sent with a custom protocol over to the
  // client.
  // Initially, a message containing all the names of the measurements is sent
  // and is preceeded with a *.
  // Names begin with a star and are split with commas.

  // Example: *test,test2
  if (static_cast<int32_t>(from_sample) == -1) {
    message << "*";
    for (int32_t cur_data_name = 0;
         cur_data_name < static_cast<int32_t>(data_names_.size());
         cur_data_name++) {
      if (cur_data_name > 0) {
        message << ",";
      }
      message << data_names_.at(cur_data_name);
    }
    return message.str();
  }

  // Data packets are sent, with raw data being placed at the same index as the
  // original index of the measurement name sent in the initial packet.
  // Samples are split with dollar signs, info with percent signs, and
  // measurements with commas.
  // This special format system is helpful for debugging issues and looping
  // through the data on the client side.

  // Example of two samples that correspond with the initialized example:
  // 289%2803.135127%10,67$290%2803.140109%12,68
  int32_t cur_sample = sample_id_ - 2;
  message << "$";

  int32_t adjusted_index = GetIndex(cur_sample);

  message << cur_sample << "%" << sample_times_.at(adjusted_index).ToSeconds()
          << "%";
  for (int32_t cur_measure = 0;
       cur_measure < static_cast<int32_t>(data_names_.size());
       cur_measure++) {
    if (cur_measure > 0) {
      message << ",";
    }
    message << data_values_.at(adjusted_index).at(cur_measure);
  }
  return message.str();
}

DataCollector::DataCollector() : cur_raw_data_("no data") {}

void DataCollector::RunIteration() {
  message_.NextSample();
  // Add recorded data here. /////
  // NOTE: Try to use fewer than 30 measures, or the whole thing will lag.
  // Abbreviate names if long, otherwise just use the command to get the value
  // from the queue.

  // TODO(comran): Make it so that the name doesn't have to be copied as a
  // string.

  ::y2016_balloon::sensors::sensors.FetchNextBlocking();
  message_.AddMeasure("bmp180 temp", ::y2016_balloon::sensors::sensors->bmp180_temp);
  message_.AddMeasure("bmp180 pressure", ::y2016_balloon::sensors::sensors->bmp180_pressure);
  message_.AddMeasure("bmp180 altitude", ::y2016_balloon::sensors::sensors->bmp180_altitude);

  message_.AddMeasure("ax", ::y2016_balloon::sensors::sensors->ax);
  message_.AddMeasure("ay", ::y2016_balloon::sensors::sensors->ay);
  message_.AddMeasure("az", ::y2016_balloon::sensors::sensors->az);

  message_.AddMeasure("cx", ::y2016_balloon::sensors::sensors->cx);
  message_.AddMeasure("cy", ::y2016_balloon::sensors::sensors->cy);
  message_.AddMeasure("cz", ::y2016_balloon::sensors::sensors->cz);

  message_.AddMeasure("ms5803 temp", ::y2016_balloon::sensors::sensors->ms5803_temp);
  message_.AddMeasure("ms5803 pressure", ::y2016_balloon::sensors::sensors->ms5803_pressure);

  static double lat(0.0), lon(0.0), speed(0.0), alt(0.0), course(0.0);
  if(::y2016_balloon::gps::gps.FetchLatest()) {
    lat = ::y2016_balloon::gps::gps->latitude;
    lon = ::y2016_balloon::gps::gps->longitude;
    speed = ::y2016_balloon::gps::gps->speed;
    alt = ::y2016_balloon::gps::gps->altitude;
    course = ::y2016_balloon::gps::gps->course;
  }
  message_.AddMeasure("gps latitude", lat);
  message_.AddMeasure("gps longitude", lon);
  message_.AddMeasure("gps speed", speed);
  message_.AddMeasure("gps altitude", alt);
  message_.AddMeasure("gps course", course);

  // End recorded data. /////
  message_.EndSample();
}

::std::string DataCollector::GetData(int32_t from_sample) {
  return message_.Fetch(from_sample);
}

void DataCollector::operator()() {
  ::aos::SetCurrentThreadName("HTTPStatusData");

  while (run_) {
    ::aos::time::PhasedLoopXMS(10, 0);
    RunIteration();
  }
}

SocketHandler::SocketHandler()
    : data_collector_thread_(::std::ref(data_collector_)) {}

void SocketHandler::onConnect(seasocks::WebSocket* connection) {
  connections_.insert(connection);
  LOG(INFO, "Connected: %s : %s\n", connection->getRequestUri().c_str(),
      seasocks::formatAddress(connection->getRemoteAddress()).c_str());
}

void SocketHandler::onData(seasocks::WebSocket* connection, const char* data) {
  int32_t from_sample = atoi(data);

  ::std::string send_data = data_collector_.GetData(from_sample);
  connection->send(send_data.c_str());
}

void SocketHandler::onDisconnect(seasocks::WebSocket* connection) {
  connections_.erase(connection);
  LOG(INFO, "Disconnected: %s : %s\n", connection->getRequestUri().c_str(),
      seasocks::formatAddress(connection->getRemoteAddress()).c_str());
}

void SocketHandler::Quit() {
  data_collector_.Quit();
  data_collector_thread_.join();
}

SeasocksLogger::SeasocksLogger(Level min_level_to_log)
    : PrintfLogger(min_level_to_log) {}

void SeasocksLogger::log(Level level, const char* message) {
  // Convert Seasocks error codes to AOS.
  log_level aos_level;
  switch (level) {
    case seasocks::Logger::INFO:
      aos_level = INFO;
      break;
    case seasocks::Logger::WARNING:
      aos_level = WARNING;
      break;
    case seasocks::Logger::ERROR:
    case seasocks::Logger::SEVERE:
      aos_level = ERROR;
      break;
    case seasocks::Logger::DEBUG:
    case seasocks::Logger::ACCESS:
    default:
      aos_level = DEBUG;
      break;
  }
  LOG(aos_level, "Seasocks: %s\n", message);
}

}  // namespace http_status
}  // namespace frc971

int main(int, char* []) {
  ::aos::InitNRT();

  seasocks::Server server(::std::shared_ptr<seasocks::Logger>(
      new frc971::http_status::SeasocksLogger(seasocks::Logger::INFO)));
  frc971::http_status::SocketHandler socket_handler;

  server.addWebSocketHandler(
      "/ws",
      ::std::shared_ptr<frc971::http_status::SocketHandler>(&socket_handler));
  server.serve("www", 8080);

  socket_handler.Quit();

  ::aos::Cleanup();
  return 0;
}
