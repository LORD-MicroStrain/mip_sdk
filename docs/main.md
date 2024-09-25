MIP SDK  {#mainpage}
=================

Welcome to the official MIP Software Development Kit. This software package
provides everything you need to communicate with any MIP-compatible
MicroStrain inertial sensor.
See @ref mip_interface for details on how to get started.

### Main Features

* MIP packet creation
* Send commands using a single function
* Packet parsing and field iteration
* Data field deserialization
* Simple interface requires only two functions to be defined
* Can be used to parse offline binary files
* Dual C and C++ API for maximum usability, safety, flexibility, and convenience.
* Suitable for bare-metal microcontrollers (Minimal code size and memory footprint, No dynamic memory allocation, No dependence on any RTOS or threading)

Quick Reference [C++]  {#quickref_cpp}
---------------------

All C++ functions and classes reside within the mip namespace.
The C functions can be accessed via the mip::C namespace.

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
qualified C++ names for clarity.

* [mip_interface (C)](mip_interface_c)
* [mip_packet (C)](mip_packet_c)
* [mip_field (C)](mip_field_c)
* [mip_parser (C)](mip_parser_c)
* [mip_cmd_result (C)](mip::C::mip_cmd_result)

