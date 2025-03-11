#pragma once

#include "mip_field.hpp"

#include <mip/mip_packet.h>
#include <mip/mip_offsets.h>

#include <microstrain/serialization.hpp>

#include <assert.h>
#include <cstring>


namespace mip
{
////////////////////////////////////////////////////////////////////////////////
///@addtogroup mip_cpp
///@{


////////////////////////////////////////////////////////////////////////////////
///@brief C++ class representing a view of a MIP packet.
///
/// This is a thin wrapper around the mip_packet_view C structure. Like the C
/// version, it does not contain or own the data buffer. Any of the C functions
/// can be used with the C++ packet class because it inherits from the C struct.
///
/// Fields may be iterated over using the C-style methods, with an iterator, or
/// with a range-based for loop:
///@code{.cpp}
/// for(PacketRef::Iterator iter = packet.begin(); iter != packet.end(); ++iter) { ... }
/// for(Field field : packet) { ... }
///@endcode
///
class PacketView : public C::mip_packet_view
{
public:
    static constexpr size_t PAYLOAD_LENGTH_MAX = C::MIP_PACKET_PAYLOAD_LENGTH_MAX;
    static constexpr size_t PACKET_SIZE_MIN    = C::MIP_PACKET_LENGTH_MIN;
    static constexpr size_t PACKET_SIZE_MAX    = C::MIP_PACKET_LENGTH_MAX;

    class FieldIterator;

public:
    ///@copydoc mip::C::mip_packet_create
    PacketView(uint8_t* buffer, size_t bufferSize, uint8_t descriptorSet) { C::mip_packet_create(this, buffer, bufferSize, descriptorSet); }
    ///@copydoc mip::C::mip_packet_from_buffer
    PacketView(const uint8_t* buffer, size_t length) { C::mip_packet_from_buffer(this, const_cast<uint8_t*>(buffer), length); }
    /// Constructs a C++ %PacketRef class from the base C object.
    PacketView(const C::mip_packet_view* other) { std::memcpy(static_cast<C::mip_packet_view*>(this), other, sizeof(*this)); }
    /// Constructs a C++ %PacketRef class from the base C object.
    PacketView(const C::mip_packet_view& other) { std::memcpy(static_cast<C::mip_packet_view*>(this), &other, sizeof(*this)); }

    ///@brief Create a new MIP packet in an existing buffer.
    ///@param buffer        Place to store the MIP packet bytes.
    ///@param descriptorSet Initializes the packet to this descriptor set.
    PacketView(microstrain::Span<uint8_t> buffer, uint8_t descriptorSet) { C::mip_packet_create(this, buffer.data(), buffer.size(), descriptorSet); }

    ///@brief Create a reference to an existing MIP packet.
    ///@param buffer Buffer containing an existing MIP packet.
    ///@warning Do not call functions which modify the packet (addField, finalize, reset, etc) unless you know the buffer is not const.
    PacketView(microstrain::Span<const uint8_t> buffer) { C::mip_packet_from_buffer(this, const_cast<uint8_t*>(buffer.data()), buffer.size()); }

    //
    // C function wrappers
    //

    uint8_t        descriptorSet() const { return C::mip_packet_descriptor_set(this); }  ///<@copydoc mip::C::mip_packet_descriptor_set
    uint_least16_t totalLength()   const { return C::mip_packet_total_length(this);   }  ///<@copydoc mip::C::mip_packet_total_length
    uint8_t        payloadLength() const { return C::mip_packet_payload_length(this); }  ///<@copydoc mip::C::mip_packet_payload_length

    bool isData() const { return C::mip_packet_is_data(this); }

    const uint8_t* pointer() const { return C::mip_packet_pointer(this); }  ///<@copydoc mip::C::mip_packet_pointer
    const uint8_t* payload() const { return C::mip_packet_payload(this); }  ///<@copydoc mip::C::mip_packet_payload

    uint16_t checksumValue() const { return C::mip_packet_checksum_value(this); }     ///<@copydoc mip::C::mip_packet_checksum_value
    uint16_t computeChecksum() const { return C::mip_packet_compute_checksum(this); } ///<@copydoc mip::C::mip_packet_compute_checksum

    bool isSane() const { return C::mip_packet_is_sane(this); }    ///<@copydoc mip::C::mip_packet_is_sane
    bool isValid() const { return C::mip_packet_is_valid(this); }  ///<@copydoc mip::C::mip_packet_is_valid
    bool isEmpty() const { return C::mip_packet_is_empty(this); }  ///<@copydoc mip::C::mip_packet_is_empty

    uint_least16_t bufferSize() const { return C::mip_packet_buffer_size(this); }  ///<@copydoc mip::C::mip_packet_buffer_size
    int remainingSpace() const { return C::mip_packet_remaining_space(this); }  ///<@copydoc mip::C::mip_packet_remaining_space

    bool addField(uint8_t fieldDescriptor, const uint8_t* payload, uint8_t payloadLength) { return C::mip_packet_add_field(this, fieldDescriptor, payload, payloadLength); }  ///<@copydoc mip::C::mip_packet_add_field
    Serializer createField(uint8_t fieldDescriptor, uint8_t length) { uint8_t* ptr; if(C::mip_packet_create_field(this, fieldDescriptor, length, &ptr) < 0) length =0; return Serializer{ptr, length}; }
    //std::tuple<uint8_t*, size_t> createField(uint8_t fieldDescriptor) { uint8_t* ptr; int max_size = C::mip_packet_alloc_field(this, fieldDescriptor, 0, &ptr); return max_size >= 0 ? std::make_tuple(ptr, max_size) : std::make_tuple(nullptr, 0); }  ///<@copydoc mip::C::mip_packet_alloc_field
    //int finishLastField(uint8_t* payloadPtr, uint8_t newPayloadLength) { return C::mip_packet_realloc_last_field(this, payloadPtr, newPayloadLength); }  ///<@copydoc mip::C::mip_packet_realloc_last_field
    //int cancelLastField(uint8_t* payloadPtr) { return C::mip_packet_cancel_last_field(this, payloadPtr); }  ///<@copydoc mip::C::mip_packet_cancel_last_field

    void finalize() { C::mip_packet_finalize(this); }  ///<@copydoc mip::C::mip_packet_finalize

    void reset(uint8_t descSet) { C::mip_packet_reset(this, descSet); }  ///<@copydoc mip::C::mip_packet_reset
    void reset() { reset(descriptorSet()); }  ///<@brief Resets the packet using the same descriptor set.

    //
    // C++ additions
    //

    ///@brief Gets a span over the whole packet.
    ///
    microstrain::Span<const uint8_t> totalSpan() const { return {pointer(), totalLength()}; }

    ///@brief Gets a span over just the payload.
    ///
    microstrain::Span<const uint8_t> payloadSpan() const { return {payload(), payloadLength()}; }


    class AllocatedField : public Serializer
    {
    public:
        AllocatedField(mip::PacketView& packet, uint8_t* buffer, size_t space) : Serializer(buffer, space), m_packet(packet) {}
        //AllocatedField(const AllocatedField&) = delete;
        AllocatedField& operator=(const AllocatedField&) = delete;

        uint8_t* allocateOrCancel(size_t length)
        {
            uint8_t* ptr = getPtrAndAdvance(length);
            if(!ptr)
                cancel();
            return ptr;
        }

        bool commit()
        {
            assert(capacity() <= FIELD_PAYLOAD_LENGTH_MAX);

            bool ok = isOk();

            if(ok)
                ok &= C::mip_packet_update_last_field_length(&m_packet, basePointer(), (uint8_t) usedLength()) >= 0;

            if(!ok && basePointer())
                C::mip_packet_cancel_last_field(&m_packet, basePointer());

            return ok;
        }

        void cancel() { if(basePointer()) C::mip_packet_cancel_last_field(&m_packet, basePointer()); }

    private:
        PacketView& m_packet;
    };

    AllocatedField createField(uint8_t fieldDescriptor)
    {
        uint8_t* ptr;
        size_t max_size = std::max<int>(0, C::mip_packet_create_field(this, fieldDescriptor, 0, &ptr));
        return {*this, ptr, max_size};
    }

    uint8_t operator[](unsigned int index) const { return pointer()[index]; }

    //
    // Additional functions which have no C equivalent
    //

    /// Returns a forward iterator to the first field in the packet.
    ///
    FieldIterator begin() const { return firstField(); }

    /// Returns a sentry object representing the end of fields in the packet.
    ///
#if __cpp_range_based_for >= 201603
    // After 201603, for loops allow different clases for begin and end.
    // Using nullptr is simpler and more efficient than creating an end iterator.
    std::nullptr_t end() const { return nullptr; }
#else
    FieldIterator end() const { return FieldView(); }
#endif

    ///@brief Returns the first field in the packet.
    ///
    /// Subsequent fields can be obtained via the returned Field class,
    /// but iteration is best done with begin()/end() or the range-based for loop.
    ///
    ///@note Packets can be empty, so make sure that the returned field is
    ///      valid before using it.
    ///
    ///@returns A Field instance representing the first field (if any).
    ///
    FieldView firstField() const { return FieldView(C::mip_field_first_from_packet(this)); }

    ///@brief Adds a field of the given type to the packet.
    ///
    ///@tparam FieldType Any field class from a file in the mip/definitions directory.
    ///
    ///@param field           Instance of the field to add to the packet.
    ///@param fieldDescriptor If specified, overrides the field descriptor.
    ///
    ///@returns True if the field was added, false if the packet has insufficient space.
    ///
    template<class FieldType>
    bool addField(const FieldType& field, uint8_t fieldDescriptor=INVALID_FIELD_DESCRIPTOR)
    {
        if( fieldDescriptor == INVALID_FIELD_DESCRIPTOR )
            fieldDescriptor = FieldType::FIELD_DESCRIPTOR;

        AllocatedField buffer = createField(fieldDescriptor);
        buffer.insert(field);
        return buffer.commit();
    }

    ///@brief Creates a new PacketRef containing a single MIP field from an instance of the field type.
    ///
    /// This works just like the addField<FieldType>() function but also initializes and finalizes the packet.
    /// It is assumed that the field will fit in an empty packet; otherwise the field can't ever be used.
    /// The field classes are predefined so this doesn't need runtime checking.
    ///
    ///@tparam FieldType Any field class from a file in the mip/definitions directory.
    ///
    ///@param buffer          Buffer to hold the packet bytes.
    ///@param bufferSize      Size of buffer in bytes.
    ///@param field           Instance of the field to add to the packet.
    ///@param fieldDescriptor If specified, overrides the field descriptor.
    ///
    ///@returns A PacketRef object containing the field.
    ///
    template<class FieldType>
    static PacketView createFromField(uint8_t* buffer, size_t bufferSize, const FieldType& field, uint8_t fieldDescriptor=INVALID_FIELD_DESCRIPTOR)
    {
        if( fieldDescriptor == INVALID_FIELD_DESCRIPTOR )
            fieldDescriptor = FieldType::FIELD_DESCRIPTOR;
        PacketView packet(buffer, bufferSize, FieldType::DESCRIPTOR_SET);
        packet.addField<FieldType>(field, fieldDescriptor);
        packet.finalize();
        return packet;
    }


    /// Iterator class for use with the range-based for loop or iterators.
    ///
    /// You should generally use the begin()/end() functions on the PacketRef
    /// class instead of using this class directly.
    ///
    class FieldIterator
    {
    public:
        /// Empty iterator, which represents the "end" iterator of a packet.
        FieldIterator() {}

        /// Create an iterator given the first field to iterate in a packet.
        /// Technically this can be any field, not just the first field.
        FieldIterator(const FieldView& first) : mField(first) {}

        /// Comparison between any two iterators.
        /// This works even for iterators over different packets, which will
        /// never be the same (except all null/end iterators are equivalent).
        bool operator==(const FieldIterator& other) const {
            // Required to make invalid fields equivalent for range-based for loop
            if( !mField.isValid() && !other.mField.isValid() )
                return true;
            return mField.descriptorSet() == other.mField.descriptorSet() && mField.fieldDescriptor() == other.mField.fieldDescriptor() && mField.payload() == other.mField.payload();
        }
        bool operator!=(const FieldIterator& other) const { return !(*this == other); }

        /// Comparison with std::nullptr is checking if the iterator points to
        /// a valid field (i.e. not the end).
        bool operator==(std::nullptr_t) const { return !mField.isValid(); }
        bool operator!=(std::nullptr_t) const { return mField.isValid(); }

        /// Dereference the iterator as a Field instance.
        const FieldView& operator*() const { return mField; }

        /// Advance to the next field.
        FieldIterator& operator++() { mField.next(); return *this; }

    private:
        FieldView mField;
    };

    ///@brief Copies this packet to an external buffer.
    ///
    /// This packet must be sane (see isSane()). Undefined behavior otherwise due to lookup of totalLength().
    ///
    ///@param buffer    Data is copied into this location.
    ///@param maxLength Maximum number of bytes to copy.
    ///
    ///@returns true if successful.
    ///@returns false if maxLength is too short.
    ///
    bool copyPacketTo(uint8_t* buffer, size_t maxLength) { assert(isSane()); size_t copyLength = this->totalLength(); if(copyLength > maxLength) return false; std::memcpy(buffer, pointer(), copyLength); return true; }

    ///@brief Copies this packet to an external buffer (span version).
    ///
    /// This packet must be sane (see isSane()). Undefined behavior otherwise due to lookup of totalLength().
    ///
    ///@param buffer Data is copied to this buffer.
    ///
    ///@returns true if successful.
    ///@returns false if maxLength is too short.
    ///
    bool copyPacketTo(microstrain::Span<uint8_t> buffer) { return copyPacketTo(buffer.data(), buffer.size()); }
};


////////////////////////////////////////////////////////////////////////////////
///@brief A mip packet with a self-contained buffer (useful with std::vector).
///
template<size_t BufferSize>
class SizedPacketBuf : public PacketView
{
    static_assert(BufferSize >= PACKET_LENGTH_MIN, "BufferSize must be at least PACKET_LENGTH_MIN bytes");

public:
    SizedPacketBuf(uint8_t descriptorSet=INVALID_DESCRIPTOR_SET) : PacketView(mData, sizeof(mData), descriptorSet) {}

    ///@brief Creates a PacketBuf by copying existing data.
    ///
    explicit SizedPacketBuf(const uint8_t* data, size_t length) : PacketView(mData, sizeof(mData)) { copyFrom(data, length); }
    explicit SizedPacketBuf(const PacketView& packet) : PacketView(mData, sizeof(mData)) { copyFrom(packet); }

    ///@brief Construct from a span.
    explicit SizedPacketBuf(microstrain::Span<const uint8_t> data) : SizedPacketBuf(data.data(), data.size()) {}

    ///@brief Copy constructor
    SizedPacketBuf(const SizedPacketBuf& other) : PacketView(mData, sizeof(mData)) { copyFrom(other); }


    ///@brief Copy constructor (required to insert packets into std::vector in some cases).
    template<size_t OtherSize>
    explicit SizedPacketBuf(const SizedPacketBuf<OtherSize>& other) : PacketView(mData, sizeof(mData)) { copyFrom(other); };

    ///@brief Copy assignment operator
    SizedPacketBuf& operator=(const SizedPacketBuf& other) { copyFrom(other); return *this; }

    ///@brief Assignment operator, copies data from another buffer to this one.
    template<size_t OtherSize>
    SizedPacketBuf& operator=(const SizedPacketBuf<OtherSize>& other) { copyFrom(other); return *this; }

    ///@brief Create a packet containing just the given field.
    ///
    ///@tparam FieldType Type of the MIP field. This can't be explicitly specified due to being a constructor.
    ///
    ///@param field           The field object to serialize.
    ///@param fieldDescriptor If specified (not INVALID_FIELD_DESCRIPTOR), overrides the field descriptor.
    ///
    template<class FieldType>
    SizedPacketBuf(
        const FieldType& field,
        uint8_t fieldDescriptor=INVALID_FIELD_DESCRIPTOR,
        typename std::enable_if<std::is_class<FieldType>::value, void>::type* = nullptr
    ) : PacketView(mData, sizeof(mData))
    {
        createFromField<FieldType>(mData, sizeof(mData), field, fieldDescriptor);
    }


    ///@brief Explicitly obtains a reference to the packet data.
    ///
    PacketView ref() { return *this; }

    ///@brief Explicitly obtains a const reference to the packet data.
    ///
    const PacketView& ref() const { return *this; }

    ///@brief Returns a pointer to the underlying buffer.
    /// This is technically the same as PacketRef::pointer but is writable.
    uint8_t* buffer() { return mData; }

    ///@brief Returns a Span covering the entire buffer.
    ///
    microstrain::Span<uint8_t, BufferSize> bufferSpan() { return microstrain::Span<uint8_t, BufferSize>{buffer(), BufferSize}; }

    ///@brief Copies the data from the pointer to this buffer. The data is not inspected.
    ///
    ///@param data   Pointer to the start of the packet.
    ///@param length Total length of the packet.
    ///
    void copyFrom(const uint8_t* data, size_t length) { assert(length <= sizeof(mData)); std::memcpy(mData, data, length); }

    ///@brief Copies an existing packet. The packet is assumed to be valid (undefined behavior otherwise).
    ///
    ///@param packet A "sane" (isSane()) mip packet.
    ///
    void copyFrom(const PacketView& packet) { assert(packet.isSane()); copyFrom(packet.pointer(), packet.totalLength()); }

private:
    uint8_t mData[BufferSize];
};

////////////////////////////////////////////////////////////////////////////////
///@brief Typedef for SizedPacketBuf of max possible size.
///
/// Generally you should use this instead of SizedPacketBuf directly, unless you
/// know the maximum size of your packet will be less than PACKET_LENGTH_MAX.
///
typedef SizedPacketBuf<mip::PACKET_LENGTH_MAX> PacketBuf;



///@}
////////////////////////////////////////////////////////////////////////////////
} // namespace mip
