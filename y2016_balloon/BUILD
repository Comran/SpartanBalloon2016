package(default_visibility = ['//visibility:public'])

cc_binary(
  name = 'logger',
  visibility = ['//visibility:public'],
  srcs = [
    'logger.cc',
  ],
  deps = [
    '//aos/linux_code:init',
    '//aos/common:die',
    '//aos/common:time',
    '//aos/common/logging:logging',
    '//aos/common/logging:implementations',
    '//y2016_balloon/input:sensors',
    '//y2016_balloon/input:gps',
  ],
)
