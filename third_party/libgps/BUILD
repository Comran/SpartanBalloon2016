licenses(['notice'])

cc_library(
  name = 'libgps',
  visibility = ['//visibility:public'],
  srcs = glob(['src/*.cc',]),
  hdrs = glob(['src/*.h',]),
  includes = ['src'],
  copts = [
    '-Ithird_party/libgps/src',
    '-Wno-unused-parameter',
  ],
)
