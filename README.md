MSCL-Embedded
=============

Welcome to the Embedded version of the MicroStrain Communication Library.

Features
--------

* MIP packet parsing and field iteration
* MIP packet creation


How to Build
------------

### Prerequisites

* CMake version 3.10 or later
* A working C compiler
* Doxygen, if building documentation

### Build configuration

The following options may be specified when configuring the build with CMake (e.g. `cmake .. -DOPTION=VALUE`):
* BUILD_TESTING - If enabled (`-DBUILD_TESTING=ON`), the test programs in the /test directory will be compiled and linked. Run the tests with `ctest`.
* BUILD_DOCUMENTATION - If enabled, the documentation will be built with doxygen. You must have doxygen installed.
* BUILD_DOCUMENTATION_FULL - Builds internal documentation.
* BUILD_DOCUMENTATION_QUIET - Suppress standard doxygen output (default enabled).

### Compilation on Linux

1. Create the build directory (e.g. `mkdir build`).
2. In the build directory, run `cmake .. -G 'Unix Makefiles'`
   * An alternative generator may be used, such as ninja, code blocks, etc.
   * You can use `cmake-gui ..` if you'd prefer to use the GUI tool (and have it installed).
3. Invoke `make` in the build directory.
4. (Optional, if BUILD_TESTS was enabled) Run `ctest` to verify the test cases pass.

### Compilation on Windows

Todo
