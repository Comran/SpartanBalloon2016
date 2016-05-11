# Introduction
Code for a telemetry system used on a weather balloon for a high school engineering project. This project uses FRC971's 2015 code repository as an underlying backbone for the project's structure.

## Building the code
The currently supported operating system for building the code is amd64 Debian Jessie. It is likely to work on any x86\_64 GNU/Linux system, but that's not at all well-tested.

We use [Bazel](http://bazel.io) to build the code. Bazel has [extensive](http://bazel.io/docs/build-ref.html) [docs](http://bazel.io/docs/build-encyclopedia.html) and does a nice job with fast, correct increment rebuilds.

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
bazel test //...
bazel build --cpu=raspi //...
```
