MIP SDK  {#mainpage}
=================

Welcome to the official MIP Software Development Kit. This software package
provides everything you need to communicate with any MIP-compatible
MicroStrain inertial sensor.

### Main Features

* MIP packet creation
* Send commands using a single function
* Packet parsing and field iteration
* Data field deserialization
* Can be used to parse offline binary files
* Dual C and C++ API for maximum usability, safety, flexibility, and convenience.
* Suitable for bare-metal microcontrollers:
  * Minimal code size and memory footprint
  * No dynamic memory allocation
  * No dependence on any RTOS or threads

Quick Reference [C++]  {#quickref_cpp}
---------------------

All C++ functions and classes reside within the mip namespace.
The C functions can be accessed via the mip::C namespace.
Within this documentation, links to C++ entities are shown in green.

* [C++ API Overview](@ref mip_cpp)
* mip::Interface  Top-level MIP interface class.
* mip::PacketView An interface to a MIP packet for either transmission or reception.
* mip::PacketBuf  Similar to PacketRef but includes the data buffer.
* mip::FieldView  An interface to a MIP field within a packet.
* mip::Parser     MIP parser class for converting received bytes into packets.
* mip::CmdResult  Stores the status or result of a MIP command.

Quick Reference [C]  {#quickref_c}
-------------------

C does not support the equivalent of C++ namespaces, so all definitions are
global. Most names start with `mip_` to avoid conflicts.
In these documentation pages, objects are referred to by their fully-
qualified C++ names for clarity. Within this documentation, links to C entities are shown in red.

* [C API Overview](@ref mip_c)
* [mip_interface ](@ref mip_interface_c)
* [mip_packet    ](@ref mip::C::mip_packet_view)
* [mip_field     ](@ref mip::C::mip_field_view)
* [mip_parser    ](@ref mip::C::mip_parser)
* [mip_cmd_result](@ref mip::C::mip_cmd_result)
