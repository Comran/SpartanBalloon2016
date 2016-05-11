package(default_visibility = ['//visibility:public'])

load('/aos/build/queues', 'queue_library')

queue_library(
  name = 'sensors',
  srcs = [
    'sensors.q',
  ],
)

queue_library(
  name = 'gps',
  srcs = [
    'gps.q',
  ],
)

cc_binary(
  name = 'gps_reader',
  visibility = ['//visibility:public'],
  srcs = [
    'gps_reader.cc',
  ],
  deps = [
    ':gps',
    '//aos/linux_code:init',
    '//third_party/libgps:libgps',
  ],
  copts = [
    '-Wno-unused-parameter',
  ],
)

cc_binary(
  name = 'sensor_reader',
  visibility = ['//visibility:public'],
  srcs = [
    'sensor_reader.cc',
    'sensor_reader.h',
  ],
  deps = [
    ':sensors',
    '//aos/common/logging:queue_logging',
    '//aos/linux_code:init',
    '//aos/common/util:phased_loop',
    '//third_party/MPU6050:MPU6050',
    '//third_party/ADXL345:ADXL345',
    '//third_party/HMC5883L:HMC5883L',
    '//third_party/libgps:libgps',
  ],
  copts = [
    '-Wno-unused-parameter',
    '-Wno-write-strings',
    '-Wno-narrowing',
    '-Wno-sign-compare',
    '-Wno-parentheses',
  ],
)
