
MIP SDK Change Log
==================

The version number scheme for the MIP SDK is MAJOR.MINOR.PATCH.

* The MAJOR number is incremented when breaking changes are made which are not backwards compatible.
  This includes public API changes and especially behavioral changes. It is likely that existing code
  will not work properly and/or may not compile without changes.
* The MINOR number is incremented when a new feature is added or a current feature is improved.
  Minor revisions may incorporate bug fixes and other patches.
* The PATCH number is incremented when a bug is fixed or a small, non-breaking change is made.
  Patches will not significantly affect the behavior of existing code, except where such behavior
  is unintentional or erroneous.

Major revisions will specify what caused the non-backwards compatible change. These will be specified like so:
CHANGED - A non-backwards compatible change was made to an existing function/class.
RENAMED - A function/class has been renamed.
REMOVED - A function/class has been removed.

Forthcoming
-----------
### New Features
* CMake find_package config files
  * Added CMake find_package config files for each module
  * Each module supports the components feature of find_package
### Interface Changes
### Bug Fixes

V3.1.0
-----------
### New Features
* Mip parser:
  * Improved performance: typically 2-5x faster parsing in both desktop and embedded applications.
  * Stand-alone parser doesn't require a parse buffer anymore.
### Interface Changes
* Mip Parser:
  * Constructor no longer takes a parse buffer.
  * Removed optional limit on max packets per parse call. Users may limit the number of bytes passed to the parser instead.
  * `mip_parser_parse` and related functions return void because they always consume the entire buffer.
  * Packet callback now must return void
  * `mip_interface_user_recv_from_device` doesn't take a data buffer parameter anymore. Instead, users should pass
    their own data buffer to `mip_parser_parse`. The buffer may be transient.
### Bug Fixes
  * None

v3.0.0
------

### New Features
* Metadata for C++ template programming (beta)
* Pretty-printing example (beta)
* Packet processing examples
* `microstrain::Span` implementation of `std::span` (interchangeable, see readme/documentation)
* Improved serialization system in c++

### Interface Changes
* Reorganized directory structure and libraries
  * Separated base ‘microstrain’ library from ‘mip’ library
  * Separated C and C++ code
  * Migration:
    * #include .h files for C and .hpp files for C++.
    * Add these include paths to your project:
      * `src/c`
      * `src/cpp`
    * Include files as `#include <mip/mip_*.h>` or `#include <microstrain/*.h>`, or .hpp for c++.
    * Most files have just been moved, but a few things have been broken out into new files
* Introduced microstrain namespace
  * Used for common code that’s not mip-specific
  * Which namespace to use depends on the location of the corresponding #include file
* Renamed some CMake variables (see readme)

### Bug Fixes
* Revamped C++ serialization system to avoid huge error messages due to large number of insert/extract overloads.
* Improved CMake scripts
* Cleaned up warnings

v2.0.0
------

### New features
* CV7-INS support
* GV7-INS support
* Logging capability (`mip_logging.h`)
* Diagnostic counters in mip parser and mip interface for debugging (define `MIP_ENABLE_DIAGNOSTICS`)
* User-defined values in CmdResult
* Additional metadata in C++ command structs
* `mip::PacketBuf` - implements `mip::PacketRef` and includes a data buffer
* Extra helper utilities
  * `CompositeResult` - stores a std::vector of CmdResults and associated command descriptors
  * `Index` - Helps prevent off-by-one errors when using 1-based MIP and 0-based arrays
  * `RecordingConnection` - Intermediate connection which logs sent/received data to files

### Interface Changes

#### Renamed
* CMake:
  * `WITH_SERIAL` &rarr; `MIP_USE_SERIAL`
  * `WITH_TCP` &rarr; `MIP_USE_TCP`
* C++
  * `mip::Packet` &rarr; `mip::PacketRef`
  * `CMD_GPS_TIME_BROADCAST_NEW` &rarr; `CMD_GPS_TIME_UPDATE`
* C
  * `timestamp_type` &rarr; `mip_timestamp`
  * `timeout_type` &rarr; `mip_timeout`
  * `renaming_count` &rarr; `int` (typedef removed)
  * `packet_length` &rarr; `uint_least16_t` (typedef removed)
  * `MIP_CMD_DESC_BASE_GPS_TIME_BROADCAST_NEW` &rarr; `MIP_CMD_DESC_BASE_GPS_TIME_UPDATE`

#### Changed
* The following 2 extern functions have been changed to callbacks to better support shared libraries.
  Supply your callbacks to `mip_interface_init`.
  * `mip_interface_user_send_to_device`
  * `mip_interface_user_recv_from_device`
* The interface for certain commands from files in `mip/definitions` have been modified:
  * Vectors and Quaternions are now explicitly-defined types.
    * In C, these are typedef'd to arrays.
    * In C++, these are simple structs offering conversion to/from plain arrays and some
      other helpful features such as `fill`.
  * Command/response structs (e.g. `mip::commands_base::DeviceDescriptors::Response`) now have
    arrays embedded rather than pointers. This change simplifies user code and reduces bugs due
    to dangling pointers.
* The C standard in CMake has been switched to `C11` from `C99` to reflect actual usage and fix
  some warnings.
* `serial_port_init` must now be called before any of the other serial port functions.

### Bug Fixes
* Use `NULL` payload for `mip_field_from_header_ptr` if input field isn't long enough
* Properly de-queue pending commands in `mip_interface_wait_for_reply` if `mip_interface_update` fails.
* `mip_packet_cancel_last_field` now computes the new header length correctly
* Serial Ports
  * Serial ports now close themselves properly if an error occurs while reading from the port. This happens when the device is connected via USB and is unplugged.
  * The port is now opened exclusively in Posix (Linux, Mac) systems
  * The port is now closed properly if setup fails during `serial_port_open`.
  * Removed `handle->is_open` member to avoid it becoming out of date.
* TCP connections are now supported on Windows.


v1.0.0
------
* Initial release of the MIP SDK
