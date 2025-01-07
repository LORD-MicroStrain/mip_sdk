#pragma once

#include "readwrite.hpp"

#include "../span.hpp"

#include <array>

#include <stdint.h>
#include <stddef.h>

#ifdef MICROSTRAIN_HAS_OPTIONAL
#include <optional>
#endif

#if __cpp_lib_apply >= 201603L
#include <functional>
#endif

namespace microstrain
{

////////////////////////////////////////////////////////////////////////////////
///@brief Represents a view of a buffer of bytes of known capacity.
///
/// The class maintains 3 attributes:
///@li A pointer to the first byte of the buffer.
///@li A capacity indicating the total size of the buffer.
///@li An offset, representing the number of bytes read or written.
///
/// This class works for either reading or writing data. As data is written/
/// read, the offset is incremented. Through functions such as pointer() and
/// getPtrAndAdvance(), the next readable/writable location can be accessed.
/// Both of these take a 'required_size' parameter indicating the amount of
/// data intending to be read/written. If sufficient data/space remains,
/// a valid pointer is returned. Otherwise, NULL is returned. The offset is
/// incremented in getPtrAndAdvance() regardless. This means the offset can
/// exceed the size/capacity of the buffer. This is an "overrun" condition,
/// and it can be checked with the isOk(), isFinished(), or hasRemaining()
/// functions. This behavior prevents bugs where an object is partially
/// read/written in the case where a larger parameter (say u32) fails but
/// then a smaller one (say u8) succeeds. With this class an overflowing
/// access will cause every following access to fail. Note that "overrun"
/// here does NOT mean that an out-of-bounds access has occurred.
///
///@note This class can be constructed with a const buffer, however constness
///      is not enforced. It is up to the user to ensure const buffers are
///      not written.
///
class SerializerBase
{
public:
    SerializerBase() = default;
    SerializerBase(uint8_t* ptr, size_t capacity, size_t offset=0) : m_ptr(ptr), m_size(capacity), m_offset(offset) {}
    SerializerBase(const uint8_t* ptr, size_t size, size_t offset=0) : m_ptr(const_cast<uint8_t*>(ptr)), m_size(size), m_offset(offset) {}
    SerializerBase(microstrain::Span<const uint8_t> buffer, size_t offset=0) : m_ptr(const_cast<uint8_t*>(buffer.data())), m_size(buffer.size()), m_offset(offset) {}

    size_t capacity()   const { return m_size;                 }  ///< Returns the total size of the buffer.
    size_t offset()     const { return m_offset;               }  ///< Returns the current read or write offset.
    size_t usedLength() const { return offset();               }  ///< Returns the number of bytes read/written.
    int remaining()     const { return int(m_size - m_offset); }  ///< Returns the number of byte remaining (negative if overflowed).

    bool isOverrun()                  const { return m_offset > m_size;        }  ///< Returns true if offset has exceeded the size/capacity.
    bool isOk()                       const { return !isOverrun();             }  ///< Returns true if not overrun, i.e. !isOverrun().
    bool isFinished()                 const { return m_offset == m_size;       }  ///< Returns true if the entire buffer (and no more) has been read/written.
    bool hasRemaining(size_t count=1) const { return m_offset+count <= m_size; }  ///< Returns true if at least 'count' bytes remain unread/unwritten.

    uint8_t*       basePointer()       { return m_ptr; }  ///< Returns a pointer to the start of the buffer.
    const uint8_t* basePointer() const { return m_ptr; }  ///< Returns a pointer to the start of the buffer.

    ///@brief Obtains a pointer to the current offset for reading/writing a value of the specified size.
    ///       This function does NOT advance the offset value. Generally, you should use getPtrAndAdvance() instead.
    ///@param required_size How many bytes will be read/written to the pointer.
    ///@returns A valid pointer if required_size bytes are available. NULL otherwise.
    uint8_t*       getPointer(size_t required_size)       { return hasRemaining(required_size) ? (m_ptr+m_offset) : nullptr; }
    const uint8_t* getPointer(size_t required_size) const { return hasRemaining(required_size) ? (m_ptr+m_offset) : nullptr; }  ///<@copydoc pointer(size_t required_size)

    ///@brief Obtains a pointer to the current offset for reading/writing a value of specified size, and post-increments the offset by that size.
    ///       Use this function just like pointer().
    ///@param size How many bytes will be read/written to the pointer. The offset is increased by this amount.
    ///@returns A valid pointer if size bytes are available. NULL otherwise.
    ///Example usage:
    ///@code{.cpp}
    /// uint32_t value = 5;
    /// if(uint8_t* ptr = serializer.getPointerAndAdvance(sizeof(value)))
    ///     std::memcpy(ptr, &value, sizeof(value));
    ///@endcode
    uint8_t* getPtrAndAdvance(size_t size) { uint8_t* ptr = hasRemaining(size) ? (m_ptr+m_offset) : nullptr; m_offset += size; return ptr; }

    ///@brief Marks the buffer as invalid, i.e. overrun/error state.
    ///       All further accesses via pointer(), getPtrAndAdvance(), etc. will fail. (basePointer() and capacity() remain valid)
    void invalidate() { m_offset = m_size+1; }

    ///@brief Sets a new offset and returns the old value.
    ///       This can be used to save/restore the current offset.
    ///       Calling setOffset() after an overrun with an in-range (i.e. non-overrun) value restores the non-overrun status.
    ///@param offset New offset. Allowed to exceed the size, i.e. an overrun state.
    ///@returns The old offset value.
    size_t setOffset(size_t offset) { std::swap(m_offset, offset); return offset; }

private:
    uint8_t* m_ptr    = nullptr;
    size_t   m_size   = 0;
    size_t   m_offset = 0;
};


////////////////////////////////////////////////////////////////////////////////
///@brief Serializes or deserializes data to/from a byte buffer.
///
/// Create one of these to handle serialization or deserialization of one or
/// more values to/from a buffer of bytes.
///
/// You can use `this->insert`/`extract` or directly call the `insert`/`extract` non-
///member functions taking a serializer reference.
///
/// The endianness is included as part of the %Serializer's type so that it
/// doesn't have to be repeatedly re-specified. It is not included as a
/// runtime parameter for performance reasons (most of the time only a buffer
/// of a known, specific endianness is required).
///
///@tparam E Endianness of the target buffer.
///
template<serialization::Endian E>
class Serializer : public SerializerBase
{
public:
    using SerializerBase::SerializerBase;

    static const serialization::Endian ENDIAN = E;

    template<typename... Ts> bool insert (const Ts&... values);
    template<typename... Ts> bool extract(Ts&... values);

    template<class T, class S> bool extract_count(T& count, S max_count);
    template<class T, class S> bool extract_count(T* count, S max_count) { return extract_count(*count, max_count); }
};

using BigEndianSerializer    = Serializer<serialization::Endian::big>;
using LittleEndianSerializer = Serializer<serialization::Endian::little>;


//
//
// Non-member functions which the user may overload
//
//

////////////////////////////////////////////////////////////////////////////////
//
// LEVEL 1 SERIALIZATION
//
////////////////////////////////////////////////////////////////////////////////

//
// Built-in types (bool, int, float, ...)
//

////////////////////////////////////////////////////////////////////////////////
///@brief Inserts a numeric value to a Serializer.
///
///@tparam E Endianness of buffer. Automatically deduced from the serializer parameter.
///@tparam T Type of the value to write. Automatically deduced from the value parameter.
///
///@param buffer Serializer object pointing to the destination buffer.
///@param value A number to write to the buffer. Can be bool, int, uint32_t, float, etc.
///
///@returns The size of T, for consistency.
///
template<serialization::Endian E, class T>
typename std::enable_if<std::is_arithmetic<T>::value, size_t>::type
/*size_t*/ insert(Serializer<E>& buffer, T value)
{
    if(auto ptr = buffer.getPtrAndAdvance(sizeof(T)))
        serialization::write<E>(ptr, value);

    return sizeof(T);
}

////////////////////////////////////////////////////////////////////////////////
///@brief Reads a numeric value from a Serializer.
///
///@tparam E Endianness of buffer. Automatically deduced from the serializer parameter.
///@tparam T Type of the value to read. Automatically deduced from the value parameter.
///
///@param buffer Serializer object pointing to the source buffer.
///@param[out] value The read value is stored in this variable. Can be a reference to bool, int, uint32_t, float, etc.
///
///@returns The size of T, for consistency.
///
template<serialization::Endian E, class T>
typename std::enable_if<std::is_arithmetic<T>::value, size_t>::type
/*size_t*/ extract(Serializer<E>& buffer, T& value)
{
    if(auto ptr = buffer.getPtrAndAdvance(sizeof(T)))
        serialization::read<E>(ptr, value);

    return sizeof(T);
}


//
// Enums
//

////////////////////////////////////////////////////////////////////////////////
///@brief Writes an enum to a Serializer.
/// For this to work properly, the enum must have its underlying type specified, i.e.,
/// like `enum Foo : uint32_t { /*...*/ };`
///
///@tparam E Endianness of buffer. Automatically deduced from the serializer parameter.
///@tparam T Type of the enum. Automatically deduced from the underlying type of the enum.
///
///@param buffer Serializer object pointing to the destination buffer.
///@param value The enum value to write to the buffer.
///
///@returns The size of T, for consistency.
///
template<serialization::Endian E, class T>
typename std::enable_if<std::is_enum<T>::value, size_t>::type
/*size_t*/ insert(Serializer<E>& buffer, T value)
{
    using BaseType = typename std::underlying_type<T>::type;

    if(auto ptr = buffer.getPtrAndAdvance(sizeof(BaseType)))
        serialization::write<E>(ptr, static_cast<BaseType>(value));

    return sizeof(BaseType);
}

////////////////////////////////////////////////////////////////////////////////
///@brief Reads an enum from a Serializer.
/// For this to work properly, the enum must have its underlying type specified, i.e.,
/// like `enum Foo : uint32_t { /*...*/ };`
///
///@tparam E Endianness of buffer. Automatically deduced from the serializer parameter.
///@tparam T Type of the enum. Automatically deduced from the underlying type of the enum.
///
///@param buffer Serializer object pointing to the source buffer.
///@param[out] value The value will be stored in this variable.
///
///@returns The size of T, for consistency.
///
template<serialization::Endian E, class T>
typename std::enable_if<std::is_enum<T>::value, size_t>::type
/*size_t*/ extract(Serializer<E>& buffer, T& value)
{
    using BaseType = typename std::underlying_type<T>::type;

    if(auto ptr = buffer.getPtrAndAdvance(sizeof(BaseType)))
    {
        BaseType base;
        serialization::read<E>(ptr, base);
        value = static_cast<T>(base);
    }

    return sizeof(BaseType);
}


//
// Classes - if they have member functions "insert" and "extract"
//

////////////////////////////////////////////////////////////////////////////////
///@brief Writes a class object to a Serializer.
/// This overload is available to any class which defines a method
/// `insert`. It shall take a Serializer reference as the sole parameter.
///
/// Example:
///@code{.cpp}
/// class Foo
/// {
///     int x, y;
///     float z;
///
///     // E.g. for big endian serialization:
///     void insert(microstrain::BigEndianSerializer& serializer) const {
///         insert(serializer, x, y, z);  // Write x, y, and z parameters.
///     }
///     // OR, if both big and little serialization are desired:
///     template<microstrain::Endian E>
///     void insert(microstrain::Serializer<E>& serializer) const {
///         insert(serializer, x, y, z);  // Write x, y, and z parameters.
///     }
/// };
///@endcode
///
///@tparam E Endianness of buffer. Automatically deduced from the serializer parameter.
///@tparam T Type of the class. Automatically deduced from the object parameter.
///
///@param serializer Serializer object pointing to the destination buffer.
///@param object The class to write to the buffer.
///
///@returns The number of bytes written.
///
template<serialization::Endian E, class T, decltype(&T::insert) = nullptr>
typename std::enable_if<std::is_class<T>::value , size_t>::type
/*size_t*/ insert(microstrain::Serializer<E>& serializer, const T& object)
{
    size_t offset = serializer.offset();
    object.insert(serializer);
    return serializer.offset() - offset;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Reads a class object from a Serializer.
/// This overload is available to any class which defines a method
/// `extract`. It shall take a Serializer reference as the sole parameter.
///
/// Example:
///@code{.cpp}
/// class Foo
/// {
///     int x, y;
///     float z;
///
///     // E.g. for big endian serialization:
///     void extract(microstrain::BigEndianSerializer& serializer) {
///         extract(serializer, x, y, z);  // Read x, y, and z parameters.
///     }
///     // OR, if both big and little serialization are desired:
///     template<microstrain::Endian E>
///     void insert(microstrain::Serializer<E>& serializer) {
///         extract(serializer, x, y, z);  // Read x, y, and z parameters.
///     }
/// };
///@endcode
///
///@tparam E Endianness of buffer. Automatically deduced from the serializer parameter.
///@tparam T Type of the class. Automatically deduced from the object parameter.
///
///@param serializer Serializer object pointing to the source buffer.
///@param object The class to read from the buffer.
///
///@returns The number of bytes read.
///
template<serialization::Endian E, class T, decltype(&T::extract) = nullptr>
typename std::enable_if<std::is_class<T>::value , size_t>::type
/*size_t*/ extract(microstrain::Serializer<E>& serializer, T& object)
{
    size_t offset = serializer.offset();
    object.extract(serializer);
    return serializer.offset() - offset;
}


////////////////////////////////////////////////////////////////////////////////
//
// LEVEL 2 SERIALIZATION
//
////////////////////////////////////////////////////////////////////////////////

//
// std::tuple - only supported if std::apply can be used
//
#if __cpp_lib_apply >= 201603L

////////////////////////////////////////////////////////////////////////////////
///@brief Writes all values from a std::tuple to a Serializer.
///
/// This overload is only enabled if std::apply from c++17 is available.
///
///@tparam E Endianness of buffer. Automatically deduced from the serializer parameter.
///@tparam Ts Types contained in the tuple. Deduced from the values parameter.
///
///@param serializer Serializer object pointing to the destination buffer.
///@param values     Tuple to serialize.
///
///@returns The total sizes of each type in the enum (i.e. Ts).
///
template<serialization::Endian E, class... Ts>
size_t insert(Serializer<E>& serializer, const std::tuple<Ts...>& values)
{
    auto lambda = [&serializer](const Ts&... args) {
        return insert(serializer, args...);
    };

    return std::apply(lambda, values);
}

////////////////////////////////////////////////////////////////////////////////
///@brief Reads values from a Serializer to a series of references wrapped in a std::tuple.
///
/// This overload is only enabled if std::apply from c++17 is available.
///
///@tparam E Endianness of buffer. Automatically deduced from the serializer parameter.
///@tparam Ts Types contained in the tuple. Deduced from the values parameter.
///
///@param serializer Serializer object pointing to the source buffer.
///@param values     Tuple of references (i.e. via std::ref or std::reference_wrapper) to the destination variables.
///
///@returns The total sizes of each type in the enum (i.e. Ts).
///
template<serialization::Endian E, class... Ts>
size_t extract(Serializer<E>& serializer, const std::tuple<std::reference_wrapper<Ts>...>& values)
{
    auto lambda = [&serializer](auto&... args) {
        return extract(serializer, args...);
    };

    return std::apply(lambda, values);
}

#endif



//
// Arrays of runtime length
//

////////////////////////////////////////////////////////////////////////////////
///@brief Writes an array to a Serializer.
///
///@tparam E Endianness of buffer. Automatically deduced from the serializer parameter.
///@tparam T Type of array elements. Automatically deduced from the values parameter.
///
///@param serializer Serializer object pointing to the destination buffer.
///@param values     Pointer to the first value in the array.
///@param count      Number of elements in the array to write.
///
///@returns The total number of bytes written.
///
template<serialization::Endian E, class T>
size_t insert(Serializer<E>& serializer, const T* values, size_t count)
{
    // For arithmetic types the size is fixed so it can be optimized.
    IF_CONSTEXPR(std::is_arithmetic<T>::value)
    {
        const size_t size = sizeof(T)*count;
        if(auto ptr = serializer.getPtrAndAdvance(size))
        {
            for(size_t i=0; i<count; i++)
                serialization::write<E>(ptr+i*sizeof(T), values[i]);
        }
        return size;
    }
    else  // Unknown size, have to check length every time.
    {
        size_t offset = serializer.offset();
        for(size_t i=0; i<count; i++)
            serializer.insert(values[i]);
        return serializer.offset() - offset;
    }
}

////////////////////////////////////////////////////////////////////////////////
///@brief Reads an array from a Serializer.
///
///@tparam E Endianness of buffer. Automatically deduced from the serializer parameter.
///@tparam T Type of array elements. Automatically deduced from the values parameter.
///
///@param serializer Serializer object pointing to the source buffer.
///@param values     Pointer to the first value in the array.
///@param count      Number of elements in the array to read.
///
///@returns The total number of bytes read.
///
template<serialization::Endian E, class T>
size_t extract(Serializer<E>& serializer, T* values, size_t count)
{
    // For arithmetic types the size is fixed so it can be optimized.
    IF_CONSTEXPR(std::is_arithmetic<T>::value)
    {
        const size_t size = sizeof(T)*count;
        if(auto ptr = serializer.getPtrAndAdvance(size))
        {
            for(size_t i=0; i<count; i++)
                serialization::read<E>(ptr+i*sizeof(T), values[i]);
        }
        return size;
    }
    else  // Unknown size, have to check length every time.
    {
        size_t offset = serializer.offset();
        for(size_t i=0; i<count; i++)
            serializer.extract(values[i]);
        return serializer.offset() - offset;
    }
}

////////////////////////////////////////////////////////////////////////////////
///@brief Writes an array to a Serializer via a Span.
///
///@tparam E Endianness of buffer. Automatically deduced from the serializer parameter.
///@tparam T Type of array elements. Automatically deduced from the values parameter.
///
///@param serializer Serializer object pointing to the destination buffer.
///@param values     Span containing pointer and count.
///
///@returns The total number of bytes written.
///
template<serialization::Endian E, class T>
size_t insert(Serializer<E>& serializer, microstrain::Span<const T> values)
{
    return insert(serializer, values.data(), values.size());
}

////////////////////////////////////////////////////////////////////////////////
///@brief Reads an array from a Serializer via a Span.
///
///@tparam E Endianness of buffer. Automatically deduced from the serializer parameter.
///@tparam T Type of array elements. Automatically deduced from the values parameter.
///
///@param serializer Serializer object pointing to the source buffer.
///@param values     Span containing pointer and count.
///
///@returns The total number of bytes read.
///
template<serialization::Endian E, class T>
size_t extract(Serializer<E>& serializer, microstrain::Span<const T> values)
{
    return extract(serializer, values.data(), values.size());
}


//
// Arrays of fixed size
//

////////////////////////////////////////////////////////////////////////////////
///@brief Writes a fixed-size array to a Serializer.
///
///@tparam E Endianness of buffer. Automatically deduced from the serializer parameter.
///@tparam T Type of array elements. Automatically deduced from the values parameter.
///
///@param serializer Serializer object pointing to the destination buffer.
///@param values     C-style array of values
///
///@returns The total number of bytes written.
///
template<serialization::Endian E, class T, size_t N>
size_t insert(Serializer<E>& serializer, const T(&values)[N])
{
    return insert(serializer, values, N);
}

////////////////////////////////////////////////////////////////////////////////
///@brief Reads a fixed-size array from a Serializer.
///
///@tparam E Endianness of buffer. Automatically deduced from the serializer parameter.
///@tparam T Type of array elements. Automatically deduced from the values parameter.
///
///@param serializer Serializer object pointing to the source buffer.
///@param values     C-style array of values
///
///@returns The total number of bytes read.
///
template<serialization::Endian E, class T, size_t N>
size_t extract(Serializer<E>& serializer, T(&values)[N])
{
    return extract(serializer, values, N);
}



////////////////////////////////////////////////////////////////////////////////
///@brief Writes a std::array to a Serializer.
///
///@tparam E Endianness of buffer. Automatically deduced from the serializer parameter.
///@tparam T Type of array elements. Automatically deduced from the values parameter.
///
///@param serializer Serializer object pointing to the destination buffer.
///@param values     std::array of values
///
///@returns The total number of bytes written.
///
template<serialization::Endian E, class T, size_t N>
size_t insert(Serializer<E>& serializer, const std::array<T,N>& values)
{
    return insert(serializer, values.data(), values.size());
}

////////////////////////////////////////////////////////////////////////////////
///@brief Reads a std::array from a Serializer.
///
///@tparam E Endianness of buffer. Automatically deduced from the serializer parameter.
///@tparam T Type of array elements. Automatically deduced from the values parameter.
///
///@param serializer Serializer object pointing to the source buffer.
///@param values     std::array of values
///
///@returns The total number of bytes read.
///
template<serialization::Endian E, class T, size_t N>
size_t extract(Serializer<E>& serializer, const std::array<T,N>& values)
{
    return extract(serializer, values.data(), values.size());
}

//
// Multiple values at once - more efficient since it avoids multiple size checks
//

////////////////////////////////////////////////////////////////////////////////
///@brief Writes multiple values to a Serializer.
///
/// This function is equivalent to calling insert on each parameter in sequence.
///
/// This function can make use of C++17 fold expressions to improve performance
/// by combining the buffer bounds checks when all types are of known size (i.e. numeric types).
///
///@tparam E Endianness of buffer. Automatically deduced from the serializer parameter.
///@tparam Ts Types of values. Automatically deduced from the values parameters.
///
///@param serializer Serializer object pointing to the destination buffer.
///@param values     One or more parameters to be written.
///
///@returns The total number of bytes written.
///
#if __cpp_fold_expressions >= 201603L && __cpp_if_constexpr >= 201606L
template<serialization::Endian E, typename... Ts>
typename std::enable_if<(sizeof...(Ts) > 1), size_t>::type
/*size_t*/ insert(Serializer<E>& serializer, Ts... values)
{
    if constexpr( (std::is_arithmetic<Ts>::value && ...) )
    {
        const size_t size = ( ... + sizeof(Ts) );

        if(uint8_t* ptr = serializer.getPtrAndAdvance(size))
        {
            size_t offset = 0;
            ( ..., (offset += serialization::write<E>(ptr+offset, values)) );
            return offset;
        }

        return size;
    }
    else  // Class types may not have fixed sizes, can't optimize them
        return ( ... + insert(serializer, values) );
}
#else
template<serialization::Endian E, typename T0, typename T1, typename... Ts>
size_t insert(Serializer<E>& serializer, const T0& value0, const T1& value1, Ts... values)
{
    return insert(serializer, value0) + insert(serializer, value1, values...);
}
#endif


////////////////////////////////////////////////////////////////////////////////
///@brief Reads multiple values from a Serializer.
///
/// This function is equivalent to calling extract on each parameter in sequence.
///
/// This function can make use of C++17 fold expressions to improve performance
/// by combining the buffer bounds checks when all types are of known size (i.e. numeric types).
///
///@tparam E Endianness of buffer. Automatically deduced from the serializer parameter.
///@tparam Ts Types of values. Automatically deduced from the values parameters.
///
///@param serializer Serializer object pointing to the destination buffer.
///@param values     One or more parameters to be read.
///
///@returns The total number of bytes read.
///
#if __cpp_fold_expressions >= 201603L && __cpp_if_constexpr >= 201606L
template<serialization::Endian E, typename... Ts>
typename std::enable_if<(sizeof...(Ts) > 1), size_t>::type
/*size_t*/ extract(Serializer<E>& serializer, Ts&... values)
{
    if constexpr( (std::is_arithmetic<Ts>::value && ...) )
    {
        const size_t size = ( ... + sizeof(Ts) );

        if(uint8_t* ptr = serializer.getPtrAndAdvance(size))
        {
            size_t offset = 0;
            ( ..., (offset += serialization::read<E>(ptr+offset, values)) );
            return offset;
        }

        return size;
    }
    else  // Class types may not have fixed sizes, can't optimize them
        return ( ... + extract(serializer, values) );
}
#else
template<serialization::Endian E, typename T0, typename T1, typename... Ts>
size_t extract(Serializer<E>& serializer, T0& value0, T1& value1, Ts&... values)
{
    return extract(serializer, value0) + extract(serializer, value1, values...);
}
#endif


//
// Raw buffer - avoids the need to create a serializer yourself.
//

////////////////////////////////////////////////////////////////////////////////
///@brief Serializes a value to a raw byte buffer.
///
/// Use this overload to write a single value without needing to
/// manually construct a Serializer object.
///
/// Example:
///@code{.cpp}
/// uint8_t buffer[256];
/// uint32_t value = 16909060; // 0x01020304
/// insert<microstrain::Endian::big>(value, buffer, sizeof(buffer), 0, false);
/// // Buffer = [0x01, 0x02, 0x03, 0x04]
/// insert<microstrain::Endian::little>(value, buffer, sizeof(buffer), 0, false);
/// // Buffer = [0x04, 0x03, 0x02, 0x01]
///@endcode
///
///@tparam E Endianness of buffer. Must be manually specified.
///@tparam T Type of value. Automatically deduced from the value parameter.
///
///@param value         Parameter to serialize. This can be any serializable type.
///@param buffer        Pointer to first element of the byte buffer.
///@param buffer_length Length/size of buffer.
///@param offset        Starting offset. Default 0.
///@param exact_size    Returns true only if exactly buffer_length plus offset bytes are written. Default false.
///
///@returns False if the buffer isn't large enough.
///@returns False if exact_size is true and the number of bytes written didn't equal buffer_length.
///@returns True otherwise.
///
template<serialization::Endian E, class T>
bool insert(const T& value, uint8_t* buffer, size_t buffer_length, size_t offset=0, bool exact_size=false)
{
    Serializer<E> serializer(buffer, buffer_length, offset);
    serializer.insert(value);
    return exact_size ? serializer.isFinished() : serializer.isOk();
}

////////////////////////////////////////////////////////////////////////////////
///@brief Deserializes a value from a raw byte buffer.
///
/// Use this overload to read a single value without needing to
/// manually construct a Serializer object.
///
/// Example:
///@code{.cpp}
/// uint8_t buffer[4] = {0x01, 0x02, 0x03, 0x04};
/// uint32_t value_BE;
/// uint32_t value_LE;
/// extract<microstrain::Endian::big>(value_BE, buffer, sizeof(buffer), 0, true);
/// extract<microstrain::Endian::little>(value_LE, buffer, sizeof(buffer), 0, true);
/// // value_BE == 0x01020304
/// // value_LE == 0x04030201
///@endcode
///
///@tparam E Endianness of buffer. Must be manually specified.
///@tparam T Type of value. Automatically deduced from the value parameter.
///
///@param value         Parameter to deserialize. This can be any serializable type.
///@param buffer        Pointer to first element of the byte buffer.
///@param buffer_length Length/size of buffer.
///@param offset        Starting offset (default 0).
///@param exact_size    Returns true only if exactly buffer_length-offset bytes are read. Default false.
///
///@returns False if the buffer doesn't have enough data.
///@returns False if exact_size is true and the number of bytes read plus offset didn't equal buffer_length.
///@returns True otherwise.
///
template<serialization::Endian E, class T>
bool extract(T& value, const uint8_t* buffer, size_t buffer_length, size_t offset=0, bool exact_size=false)
{
    Serializer<E> serializer(buffer, buffer_length, offset);
    extract(serializer, value);
    return exact_size ? serializer.isFinished() : serializer.isOk();
}


//
// Raw buffer - Span version
//

////////////////////////////////////////////////////////////////////////////////
///@brief Serializes a value to a raw byte buffer (span version).
///
/// Use this overload to write a single value without needing to
/// manually construct a Serializer object.
///
///@tparam E Endianness of buffer. Must be manually specified.
///@tparam T Type of value. Automatically deduced from the value parameter.
///
///@param value         Parameter to serialize. This can be any serializable type.
///@param buffer        Source buffer span.
///@param offset        Starting offset. Default 0.
///@param exact_size    Returns true only if exactly buffer.size()-offset bytes are written. Default false.
///
///@returns False if the buffer isn't large enough.
///@returns False if exact_size is true and the number of bytes written didn't equal buffer_length.
///@returns True otherwise.
///
template<serialization::Endian E, class T>
bool insert(T value, microstrain::Span<uint8_t> buffer, size_t offset=0, bool exact_size=false)
{
    return insert<E,T>(value, buffer.data(), buffer.size(), offset, exact_size);
}

////////////////////////////////////////////////////////////////////////////////
///@brief Deserializes a value from a raw byte buffer (span version).
///
/// Use this overload to read a single value without needing to
/// manually construct a Serializer object.
///
///@tparam E Endianness of buffer. Must be manually specified.
///@tparam T Type of value. Automatically deduced from the value parameter.
///
///@param value         Parameter to deserialize. This can be any serializable type.
///@param buffer        Source buffer span.
///@param offset        Starting offset (default 0).
///@param exact_size    Returns true only if exactly buffer.size()-offset bytes are read. Default false.
///
///@returns False if the buffer doesn't have enough data.
///@returns False if exact_size is true and the number of bytes read plus offset didn't equal buffer_length.
///@returns True otherwise.
///
template<serialization::Endian E, class T>
bool extract(T& value, microstrain::Span<const uint8_t> buffer, size_t offset=0, bool exact_size=false)
{
    return extract<E,T>(value, buffer.data(), buffer.size(), offset, exact_size);
}

//
// Special Deserialization
//

#ifdef MICROSTRAIN_HAS_OPTIONAL
////////////////////////////////////////////////////////////////////////////////
///@brief Reads a value from a serializer and returns it via std::optional.
///
/// This overload is only enabled if std::optional is supported.
///
///@tparam E Endianness of buffer. Automatically deduced from the serializer parameter.
///@tparam T Type of value. Must be manually specified.
///
///@param buffer Serializer object pointing to the buffer.
///
///@returns The value read from the buffer, or std::nullopt if it couldn't be read.
///
template<class T, serialization::Endian E>
std::optional<T> extract(Serializer<E>& serializer)
{
    T value;
    if(extract<E,T>(serializer, value))
        return value;
    else
        return std::nullopt;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Reads a value from a raw byte buffer and returns it via std::optional.
///
/// This overload is only enabled if std::optional is supported.
///
///@see bool extract(T& value, const uint8_t* buffer, size_t buffer_length, size_t offset=0, bool exact_size=false)
///
///@tparam E Endianness of buffer. Must be manually specified.
///@tparam T Type of value. Must be manually specified.
///
///@param buffer        Pointer to first element of the byte buffer.
///@param buffer_length Length/size of buffer.
///@param offset        Starting offset (default 0).
///@param exact_size    Returns a value only if exactly buffer_length-offset bytes are read. Default false.
///
///@returns The value read from the buffer, or std::nullopt if it couldn't be read.
///
template<class T, serialization::Endian E>
std::optional<T> extract(const uint8_t* buffer, size_t length, size_t offset, bool exact_size=false)
{
    T value;
    if(extract<E,T>(value, buffer, length, offset, exact_size))
        return value;
    else
        return std::nullopt;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Reads a value from a raw byte span and returns it via std::optional.
///
/// This overload is only enabled if std::optional is supported.
///
///@see bool extract(T& value, microstrain::Span<const uint8_t> buffer, size_t offset=0, bool exact_size=false)
///
///@tparam E Endianness of buffer. Must be manually specified.
///@tparam T Type of value. Must be manually specified.
///
///@param buffer        Source buffer span.
///@param offset        Starting offset (default 0).
///@param exact_size    Returns a value only if exactly buffer.size()-offset bytes are read. Default false.
///
///@returns The value read from the buffer, or std::nullopt if it couldn't be read.
///
template<class T, serialization::Endian E>
std::optional<T> extract(microstrain::Span<const uint8_t> buffer, size_t offset, bool exact_size=false)
{
    T value;
    if(extract<E,T>(value, buffer.data(), buffer.size(), offset, exact_size))
        return value;
    else
        return std::nullopt;
}
#endif


////////////////////////////////////////////////////////////////////////////////
//
// Serializer member functions which depend on the above overloads.
//
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
///@brief Serializes one or more values.
///
/// This function defers to the non-member function microstrain::insert. By
/// doing so, it allows the user to provide custom overloads for their own
/// types. Any type supported by the non-member functions may be used here.
///
///@tparam Ts Types of values to serialize.
///
///@param values One or more parameters to serialize.
///
///@returns True if all values were successfully written.
///
template<serialization::Endian E>
template<typename... Ts>
bool Serializer<E>::insert(const Ts&... values)
{
    // Prevents infinite recursion but allows ADL
    // https://stackoverflow.com/questions/13407205/calling-nonmember-instead-of-member-function
    using microstrain::insert;

    insert(*this, values...);

    return isOk();
}

////////////////////////////////////////////////////////////////////////////////
///@brief Deserializes one or more values.
///
/// This function defers to the non-member function microstrain::extract. By
/// doing so, it allows the user to provide custom overloads for their own
/// types. Any type supported by the non-member functions may be used here.
///
///@tparam Ts Types of values to deserialize.
///
///@param[out] values One or more parameters to deserialize.
///
///@returns True if all values were successfully read.
///
template<serialization::Endian E>
template<typename... Ts>
bool Serializer<E>::extract(Ts&... values)
{
    // Prevents infinite recursion but allows ADL
    // https://stackoverflow.com/questions/13407205/calling-nonmember-instead-of-member-function
    using microstrain::extract;

    extract(*this, values...);

    return isOk();
}

////////////////////////////////////////////////////////////////////////////////
///@brief Deserializes an integer with maximum permissible value.
///
/// This function is intended to be used with protocols which have embedded
/// length fields. This function returns false and invalidates the serializer
/// if the count is too large.
///
///@tparam T Type of value to read. Expected to be integer-like and must
///          support `bool operator<=(S)`.
///@tparam S Type of the maximum value. Usually the same integral type as T,
///          but the only requirement is that it can be compared with T.
///
///@param[out] count Value read from the buffer. Set to 0 on failure.
///@param max_count  Maximum permissible value for count (inclusive).
///
///@returns True if the count was successfully read and is less than
///         or equal to max_count.
///
template<serialization::Endian E>
template<class T, class S>
bool Serializer<E>::extract_count(T& count, S max_count)
{
    if( this->extract(count) )
    {
        if( count <= max_count )
            return true;

        invalidate();
    }

    count = 0;

    return false;
}


} // namespace microstrain
