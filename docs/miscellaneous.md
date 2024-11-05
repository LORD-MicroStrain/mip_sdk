Other Considerations  {#other}
====================

Known Issues and Workarounds  {#known_issues}
----------------------------

### suppress_ack parameters are not supported

Some commands accept a parameter named `suppress_ack` which prevents
the standard ack/nack reply from being returned by the device, e.g. the
3DM Poll Data command. Use of this parameter is not fully supported by the MIP SDK
and will cause the command to appear to time out after a short delay.

If you wish to use this feature, (i.e. just send the command without waiting for an ACK/NACK),
you can build and send the command manually:
~~~~~~~~{.cpp}
// Create the command with required parameters.
mip::commands_3dm::PollData cmd;

cmd.desc_set = mip::data_sensor::DESCRIPTOR_SET;
cmd.suppress_ack = true;  // We can set this since we're not calling the standard cmd handling functions.
cmd.num_descriptors = 2;
cmd.descriptors[0] = mip::data_sensor::ScaledAccel::FIELD_DESCRIPTOR;
cmd.descriptors[1] = mip::data_sensor::ScaledGyro::FIELD_DESCRIPTOR;

// Build a packet.
mip::PacketBuf packet(cmd);

// Send it to the device.
device.sendCommand(packet);
~~~~~~~~

### Some commands take longer and may time out

This applies to the following commands:
* Device Settings (mip::commands_3dm::DeviceSettings), in particular saving settings.
* Commanded Built-In Test (mip::commands_base::BuiltInTest)
* Capture Gyro Bias (mip::commands_3dm::CaptureGyroBias)

The device timeout must be sufficiently long when sending these commands.
There are 3 potential ways to avoid erroneous timeouts:
* Set a high overall device timeout. This is the easiest solution but may cause excess
  delays in your application if the device is unplugged, not powered, etc.
* Temporarily set the timeout higher, and restore it after running the long command.
* If using C++, use the mip::Interface::runCommand function and pass a large enough
  value for the `additionalTime` parameter. This raises the timeout specifically for that
  one command instance and is the recommended option.
