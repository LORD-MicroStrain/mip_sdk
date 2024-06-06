#pragma once

#include "mip_field.hpp"

#include "mip_packet.h"
#include "mip_offsets.h"

#include <microstrain/common/serialization.hpp>

#include <assert.h>
#include <cstring>


namespace mip
{
////////////////////////////////////////////////////////////////////////////////
///@addtogroup mip_cpp
///@{


////////////////////////////////////////////////////////////////////////////////
///@brief C++ class representing a MIP PacketRef.
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
class PacketRef : public C::mip_packet_view
{
public:
    static constexpr size_t PAYLOAD_LENGTH_MAX = C::MIP_PACKET_PAYLOAD_LENGTH_MAX;
    static constexpr size_t PACKET_SIZE_MIN    = C::MIP_PACKET_LENGTH_MIN;
    static constexpr size_t PACKET_SIZE_MAX    = C::MIP_PACKET_LENGTH_MAX;

    class FieldIterator;

public:
    ///@copydoc mip::C::mip_packet_create
    PacketRef(uint8_t* buffer, size_t bufferSize, uint8_t descriptorSet) { C::mip_packet_create(this, buffer, bufferSize, descriptorSet); }
    ///@copydoc mip_packet_from_buffer
    PacketRef(uint8_t* buffer, size_t length) { C::mip_packet_from_buffer(this, buffer, length); }
    /// Constructs a C++ %PacketRef class from the base C object.
    PacketRef(const C::mip_packet_view* other) { std::memcpy(static_cast<C::mip_packet_view*>(this), other, sizeof(*this)); }
    /// Constructs a C++ %PacketRef class from the base C object.
    PacketRef(const C::mip_packet_view& other) { std::memcpy(static_cast<C::mip_packet_view*>(this), &other, sizeof(*this)); }


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
    microstrain::Serializer createField(uint8_t fieldDescriptor, uint8_t length) { uint8_t * ptr; if(C::mip_packet_alloc_field(this, fieldDescriptor, length, &ptr) < 0) length =0; return microstrain::Serializer{ptr, length}; }
    //std::tuple<uint8_t*, size_t> createField(uint8_t fieldDescriptor) { uint8_t* ptr; int max_size = C::mip_packet_alloc_field(this, fieldDescriptor, 0, &ptr); return max_size >= 0 ? std::make_tuple(ptr, max_size) : std::make_tuple(nullptr, 0); }  ///<@copydoc mip::C::mip_packet_alloc_field
    //int finishLastField(uint8_t* payloadPtr, uint8_t newPayloadLength) { return C::mip_packet_realloc_last_field(this, payloadPtr, newPayloadLength); }  ///<@copydoc mip::C::mip_packet_realloc_last_field
    //int cancelLastField(uint8_t* payloadPtr) { return C::mip_packet_cancel_last_field(this, payloadPtr); }  ///<@copydoc mip::C::mip_packet_cancel_last_field

    class AllocatedField : public microstrain::Serializer
    {
    public:
        AllocatedField(mip::PacketRef& packet, uint8_t* buffer, size_t space) : Serializer(buffer, space), m_packet(packet) {}
        //AllocatedField(const AllocatedField&) = delete;
        AllocatedField& operator=(const AllocatedField&) = delete;

        bool commit()
        {
            assert(capacity() <= FIELD_PAYLOAD_LENGTH_MAX);

            bool ok = isOk();

            if(ok)
                ok &= C::mip_packet_realloc_last_field(&m_packet, basePointer(), (uint8_t)length()) >= 0;

            if(!ok)
                C::mip_packet_cancel_last_field(&m_packet, basePointer());

            return ok;
        }

    private:
        PacketRef& m_packet;
    };

    AllocatedField createField(uint8_t fieldDescriptor) { uint8_t* ptr; size_t max_size = std::max<int>(0, C::mip_packet_alloc_field(this, fieldDescriptor, 0, &ptr)); return {*this, ptr, max_size}; }

    void finalize() { C::mip_packet_finalize(this); }  ///<@copydoc mip::C::mip_packet_finalize

    void reset(uint8_t descSet) { C::mip_packet_reset(this, descSet); }  ///<@copydoc mip::C::mip_packet_reset
    void reset() { reset(descriptorSet()); }  ///<@brief Resets the packet using the same descriptor set.

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
    FieldIterator end() const { return Field(); }
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
    Field firstField() const { return Field(C::mip_field_first_from_packet(this)); }

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
    static PacketRef createFromField(uint8_t* buffer, size_t bufferSize, const FieldType& field, uint8_t fieldDescriptor=INVALID_FIELD_DESCRIPTOR)
    {
        if( fieldDescriptor == INVALID_FIELD_DESCRIPTOR )
            fieldDescriptor = FieldType::FIELD_DESCRIPTOR;
        PacketRef packet(buffer, bufferSize, FieldType::DESCRIPTOR_SET);
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
        FieldIterator(const Field& first) : mField(first) {}

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
        const Field& operator*() const { return mField; }

        /// Advance to the next field.
        FieldIterator& operator++() { mField.next(); return *this; }

    private:
        Field mField;
    };

};


////////////////////////////////////////////////////////////////////////////////
///@brief A mip packet with a self-contained buffer (useful with std::vector).
///
template<size_t BufferSize>
class SizedPacketBuf : public PacketRef
{
public:
    SizedPacketBuf(uint8_t descriptorSet=INVALID_DESCRIPTOR_SET) : PacketRef(mData, sizeof(mData), descriptorSet) {}

    ///@brief Creates a PacketBuf by copying existing data.
    ///
    explicit SizedPacketBuf(const uint8_t* data, size_t length) : PacketRef(mData, sizeof(mData)) { copyFrom(data, length); }
    explicit SizedPacketBuf(const PacketRef& packet) : PacketRef(mData, sizeof(mData)) { copyFrom(packet); }

    ///@brief Copy constructor
    SizedPacketBuf(const SizedPacketBuf& other) : PacketRef(mData, sizeof(mData)) { copyFrom(other); }

    ///@brief Copy constructor (required to put packets into std::vector in some cases).
    template<size_t OtherSize>
    explicit SizedPacketBuf(const SizedPacketBuf<OtherSize>& other) : PacketRef(mData, sizeof(mData)) { copyFrom(other); };

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
    SizedPacketBuf(const FieldType& field, uint8_t fieldDescriptor=INVALID_FIELD_DESCRIPTOR) : PacketRef(mData, sizeof(mData)) { createFromField<FieldType>(mData, sizeof(mData), field, fieldDescriptor); }


    ///@brief Explicitly obtains a reference to the packet data.
    ///
    PacketRef ref() { return *this; }

    ///@brief Explicitly obtains a const reference to the packet data.
    ///
    const PacketRef& ref() const { return *this; }

    ///@brief Returns a pointer to the underlying buffer.
    /// This is technically the same as PacketRef::pointer but is writable.
    uint8_t* buffer() { return mData; }

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
    void copyFrom(const PacketRef& packet) { assert(packet.isSane()); copyFrom(packet.pointer(), packet.totalLength()); }

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
    bool copyTo(uint8_t* buffer, size_t maxLength) { assert(isSane()); size_t copyLength = this->totalLength(); if(copyLength > maxLength) return false; std::memcpy(buffer, mData, copyLength); return true; }

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
