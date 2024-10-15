MIP Packet Cheatsheet  {#mip_packet_processing_cpp}
=====================

New Packet Creation
-------------------

mip::PacketView can construct a new MIP packet using any of the
constructor overloads taking a descriptor set parameter.

### Create a new packet using an existing buffer for storage

    uint8_t buffer[mip::PACKET_LENGTH_MAX];

    mip::PacketView packet(buffer, sizeof(buffer), descriptor_set);

Using a Span:

    microstrain::Span<uint8_t> buffer_span(buffer, sizeof(buffer));
    mip::PacketView packet(buffer_span, descriptor_set);

### Create a new packet with a built-in buffer

    mip::PacketBuf packet( descriptor_set );

If you know the max size of the packet is less than PACKET_LENGTH_MAX,
you may use `SizedPacketBuf<SIZE>` instead:

    mip::SizedPacketBuf<SIZE> packet(descriptor_set);

A PacketBuf inherits all of the methods from PacketRef and may be used
interchangeably.

### Clear/restart a packet

This effectively resets the packet as if the constructor was called again with
the same parameters (buffer, size, and descriptor set).

    packet.reset();

The descriptor set may be changed if desired:

    packet.reset( descriptor_set );

### Create a packet from a field using a C++ struct

    void send_command(mip::commands_base::CommSpeed& cmd)
    {
        mip::PacketBuf packet( cmd );

        // send packet
    }

The packet is complete and ready to go after construction; the checksum has been computed in this case.

Similar to the above example, multiple fields may be appended.
In this case, two message format commands are added to one packet.

    mip::PacketBuf packet(mip::commands_3dm::DESCRIPTOR_SET);

    mip::commands_3dm::MessageFormat sensor_format;
    mip::commands_3dm::MessageFormat filter_format;
    // initialize the commands here

    packet.addField(sensor_format);
    packet.addField(filter_format);

    packet.finalize();


Existing Packet Processing
--------------------------

### Create a PacketView over an existing packet buffer

    void handle_packet(const uint8_t* buffer, size_t length)
    {
        mip::PacketView packet(existing_buffer, sizeof(existing_buffer));
    }

Using a span

    void handle_packet(microstrain::Span<const uint8_t> buffer)
    {
        mip::PacketView packet(buffer);
    }

### Validation

A PacketView can only be used if the packet structure makes sense, i.e. if
PacketView::isSane() returns true. This checks the following:
1. The buffer is not NULL.
2. The size of the buffer is at least PACKET_LENGTH_MIN.
3. The payload length does not exceed the size of the buffer.

Additional checks are performed by PacketView::isValid():
1. The packet is sane, i.e. isSane() returns true.
2. The descriptor set is not invalid (0x00).
3. Computing the checksum matches the value in the buffer.

You may compute the checksum manually as well:

    bool chk_valid = packet.checksumValue() == packet.computeChecksum();


### PacketView properties

Once a PacketView is created, it may be used to inspect the packet.

    uint8_t desc_set       = packet.descriptorSet();
    size_t  total_length   = packet.totalLength();
    uint8_t payload_length = packet.payloadLength();

    // Pointer to raw bytes (e.g. to send to the device)
    // Do not modify data unless you can be sure the buffer passed
    // to the constructor was not const! I.e. only if using PacketBuf
    // or the descriptor_set version of PacketView's ctor.
    uint8_t* data = packet.pointer();

    // Pointer to the start of the paylaod.
    // The caution about constness above applies here as well.
    uint8_t* payload = packet.payload();

    // This is the value that was passed to the constructor.
    size_t buffer_size = packet.bufferSize();

    // How much of the buffer's capacity has been used.
    // Note that the max size is also limited by PACKET_SIZE_MAX.
    // If this is negative, the packet is not "sane", i.e. 
    // PacketView::isSane() is false and you cannot safely iterate fields
    // without risking reading past the end of the buffer.
    // See "validation" above.
    int remaining_space = packet.remainingSpace();

### Reading fields in a packet

Generally, field offsets aren't known ahead of time so they must be iterated.

The easiest way to do this is with a range-based for loop:

    void iter_fields(const mip::PacketView& packet)
    {
        for(mip::FieldView field : packet)
        {
            process_field(field);
        }
    }

This is effectively the same as

    void iter_fields(const mip::PacketView& packet)
    {
        for(mip::FieldIterator fi = packet.begin(); fi != packet.end(); ++fi)
        {
            process_field(*field);
        }
    }


#### Manually searching for a field without FieldIterator

The next field in a packet can be obtained directly with FieldView::next or FieldView::nextAfter.
Note that the validity needs to be checked in case there is no next field.
Here is an example code which checks if response data follows an ack/nack reply:

    FieldView first = packet.firstField();
    if(first.isValid())
    {
        if( first.is_reply() )
        {
            FieldView response = first.nextAfter();

            if(response.isValid())
                printf("Got response field: 0x%02X\n", response.fieldDescriptor());
        }
    }

### Field properties

FieldViews have properties similar to PacketViews:

    // Unsafe to access payload data or descriptor info unless valid.
    // Generally this is already checked during field iteration.
    bool valid = isValid();

    uint8_t descriptor_set = field.descriptorSet();  // Same as the packet
    uint8_t field_desc     = field.fieldDescriptor();

    // Combination of descriptor set and field descriptor.
    CompositeDescriptor desc = field.descriptor();

    uint8_t payload_length = field.payloadLength();
    const uint8_t* payload = field.payload();

    // Span version of payload
    microstrain::Span<const uint8_t> payload = field.payloadSpan();

    // Determining the type of field, e.g. for routing it to the right function in your app.
    bool is_data_field    = field.isData();     // Data field, e.g. accel data
    bool is_command       = field.isCommand();  // Command to device
    bool is_reply         = field.isReply();    // Ack/nack field from device
    bool is_response_data = field.isResponse(); // Reply from device containing data

### Decoding field data

After checking the descriptor, field data can be deserialized.

    if(field.descriptor() == mip::data_sensor::ScaledAccel::DESCRIPTOR)
    {
        mip::data_sensor::ScaledAccel data;

        if(field.extract(data))
            printf("Accel data = %f %f %f\n", data.scaled_accel[0], data.scaled_accel[1], data.scaled_accel[2]);
    }

You may also deserialize it manually.

    mip::Serializer serializer(field.payloadSpan());

    // Same as prior example
    //serializer.extract(data);

    // Separate parameters for scaled accel
    float x,y,z;
    serializer.extract(x,y,z);
