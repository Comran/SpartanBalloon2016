load('/aos/externals/seasocks/gen_embedded', 'gen_embedded')

gen_embedded(
  name = 'gen_embedded',
  srcs = glob(['www_defaults/**/*'], ['www/**/*']),
)

cc_binary(
  name = 'http_status',
  visibility = ['//visibility:public'],
  srcs = [
    'http_status.cc',
    'http_status.h',
  ],
  deps = [
    ':gen_embedded',
    '//aos/linux_code:init',
    '//aos/common/logging',
    '//third_party/seasocks',
    '//aos/common/util:phased_loop',
    '//aos/common:time',
    '//y2016_balloon/input:sensors',
    '//y2016_balloon/input:gps',
  ],
  copts = [
    '-Wno-sign-compare',
  ],
)
