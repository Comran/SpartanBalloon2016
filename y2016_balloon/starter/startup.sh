#!/bin/bash

echo Starting up the code. Timestamp: $(date) >> /home/comran/startup_log.txt
cd /home/comran
LD_LIBRARY_PATH=/opt/mjpg-streamer/ /opt/mjpg-streamer/mjpg_streamer -i "input_raspicam.so -fps 10 -q 15 -x 595 -y 415" -o "output_http.so -p 8081 -w /opt/mjpg-streamer/www" &
sleep 1
./core &
sleep 1
./sensor_reader &
sleep 10
./http_status &
midori -e Fullscreen -a 127.0.0.1:8080 &
echo "running startup logger mounter in bg..." >> /home/comran/startup_log.txt;
sudo ./startup_logger_mounter.sh &
echo "running startup logger in bg..." >> /home/comran/startup_log.txt;
sudo ./startup_logger.sh &
echo "running gps_reader in background..." >> /home/comran/startup_log.txt;
while true; do ./gps_reader;done
