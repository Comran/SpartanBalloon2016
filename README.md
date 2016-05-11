# Introduction
Code for a telemetry system used on a weather balloon for a high school engineering project. This project uses FRC971's 2015 code repository as an underlying backbone for the project's structure.

Steps to set up a computer to build the code:
  0. Set up the required APT repositories:
     Download
	 [frc971.list](http://robotics.mvla.net/files/frc971/packages/frc971.list)
	 and
	 [llvm.org.list](http://robotics.mvla.net/files/frc971/packages/llvm.org.list)
	 and put them in `/etc/apt/sources.list.d/`.
  1. Install the required packages:
```console
apt-get install python libpython-dev bazel ruby clang-format-3.5 clang-3.6 gfortran libblas-dev liblapack-dev python-scipy python-matplotlib
```
  2. Allow Bazel's sandboxing to work

Some useful Bazel commands:
  * Build and test everything (on the host system):
```console
bazel test //... -- $(cat NO_BUILD_AMD64)
bazel build --cpu=roborio //... -- $(cat NO_BUILD_ROBORIO)
```
    The NO_BUILD_{AMD64,ROBORIO} files contain lists of the targets which are intentionally not built for the various CPUs.
  * Build the code for a specific robot:
```console
bazel build --cpu=roborio --compilation_mode=opt //y2015/...
```
  * Download code to a robot:
```console
bazel run --cpu=roborio --compilation_mode=opt //y2015:download roboRIO-971.local
```
