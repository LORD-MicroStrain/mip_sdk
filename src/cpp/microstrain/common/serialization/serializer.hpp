#pragma once

#include "readwrite.hpp"

#include <array>

#include <stdint.h>
#include <stddef.h>

#ifdef HAS_OPTIONAL
#include <optional>
#endif
#ifdef HAS_SPAN
#include <span>
#endif

#if __cpp_lib_apply >= 201603L
#include <functional>
#endif



namespace microstrain
{


class SerializerBase
{
public:
    SerializerBase() = default;
    SerializerBase(uint8_t* ptr, size_t capacity, size_t offset=0) : m_ptr(ptr), m_size(capacity), m_offset(offset) {}
    SerializerBase(const uint8_t* ptr, size_t size, size_t offset=0) : m_ptr(const_cast<uint8_t*>(ptr)), m_size(size), m_offset(offset) {}
#ifdef HAS_SPAN
    SerializerBase(std::span<const uint8_t> buffer, size_t offset=0) : m_ptr(const_cast<uint8_t*>(buffer.data())), m_size(buffer.size()), m_offset(offset) {}
#endif

    size_t capacity() const { return m_size; }
    size_t length() const { return m_size; }
    size_t offset() const { return m_offset; }
    int remaining() const { return int(m_size - m_offset); }

    bool isOverrun() const { return m_offset > m_size; }
    bool isOk() const { return !isOverrun(); }
    bool isFinished() const { return m_offset == m_size; }
    bool hasRemaining(size_t count=0) const { return m_offset+count <= m_size; }

    const uint8_t* basePointer() const { return m_ptr; }
    uint8_t* basePointer() { return m_ptr; }

    const uint8_t* pointer(size_t required_size) const { return hasRemaining(required_size) ? (m_ptr+m_offset) : nullptr; }
    uint8_t* pointer(size_t required_size) { return hasRemaining(required_size) ? (m_ptr+m_offset) : nullptr; }

    uint8_t* getPtrAndAdvance(size_t size) { uint8_t* ptr = hasRemaining(size) ? (m_ptr+m_offset) : nullptr; m_offset += size; return ptr; }

    void invalidate() { m_offset = size_t(-1); }

    // Sets a new offset and returns the old value.
    size_t setOffset(size_t offset) { std::swap(m_offset, offset); return offset; }

private:
    uint8_t* m_ptr    = nullptr;
    size_t   m_size   = 0;
    size_t   m_offset = 0;
};


// Serializer that has the endianness built-in
template<serialization::Endian E>
class Serializer : public SerializerBase
{
public:
    using SerializerBase::SerializerBase;

    static const serialization::Endian ENDIAN = E;

    template<typename... Ts> bool insert (const Ts&... values);
    template<typename... Ts> bool extract(Ts&... values);

    template<class T, class S> bool extract_count(T& value, S max_value);
    template<class T, class S> bool extract_count(T* value, S max_value) { return extract_count(*value, max_value); }
};

using BigEndianSerializer    = Serializer<serialization::Endian::big>;
using LittleEndianSerializer = Serializer<serialization::Endian::little>;


//
//
// Non-member functions which the user may overload
//
//

//
// Built-in types (bool, int, float, ...)
//

template<serialization::Endian E, class T>
typename std::enable_if<std::is_arithmetic<T>::value, size_t>::type
/*size_t*/ insert(Serializer<E>& buffer, T value)
{
    if(auto ptr = buffer.getPtrAndAdvance(sizeof(T)))
        serialization::write<E>(ptr, value);

    return sizeof(T);
}

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

template<serialization::Endian E, class T>
typename std::enable_if<std::is_enum<T>::value, size_t>::type
/*size_t*/ insert(Serializer<E>& buffer, T value)
{
    using BaseType = typename std::underlying_type<T>::type;

    if(auto ptr = buffer.getPtrAndAdvance(sizeof(BaseType)))
        serialization::write<E>(ptr, static_cast<BaseType>(value));

    return sizeof(BaseType);
}

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
// std::tuple - only supported if std::apply can be used
//
#if __cpp_lib_apply >= 201603L

template<serialization::Endian E, class... Ts>
size_t insert(Serializer<E>& serializer, const std::tuple<Ts...>& values)
{
    auto lambda = [&serializer](const Ts&... args) {
        return insert(serializer, args...);
    };

    return std::apply(lambda, values);
}

// std::tuple of references
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
// Classes - if they have member functions "insert" and "extract"
//

// Generic classes which have an "insert" method.
template<serialization::Endian E, class T, decltype(&T::insert) = nullptr>
typename std::enable_if<std::is_class<T>::value , size_t>::type
/*size_t*/ insert(microstrain::Serializer<E>& serializer, const T& object)
{
    size_t offset = serializer.offset();
    object.insert(serializer);
    return serializer.offset() - offset;
}

// Generic classes which have an "extract" method.
template<serialization::Endian E, class T, decltype(&T::extract) = nullptr>
typename std::enable_if<std::is_class<T>::value , size_t>::type
/*size_t*/ extract(microstrain::Serializer<E>& serializer, T& object)
{
    size_t offset = serializer.offset();
    object.extract(serializer);
    return serializer.offset() - offset;
}

//
// Arrays of runtime length
//

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

#ifdef HAS_SPAN
template<serialization::Endian E, class T>
size_t insert(Serializer<E>& serializer, std::span<const T> values)
{
    return insert(serializer, values.data(), values.size());
}
template<serialization::Endian E, class T>
size_t extract(Serializer<E>& serializer, std::span<const T> values)
{
    return extract(serializer, values.data(), values.size());
}
#endif

//
// Arrays of fixed size
//

template<serialization::Endian E, class T, size_t N>
size_t insert(Serializer<E>& serializer, const T(&values)[N])
{
    return insert(serializer, values, N);
}
template<serialization::Endian E, class T, size_t N>
size_t extract(Serializer<E>& serializer, T(&values)[N])
{
    return extract(serializer, values, N);
}

template<serialization::Endian E, class T, size_t N>
size_t insert(Serializer<E>& serializer, const std::array<T,N>& values)
{
    return insert(serializer, values.data(), values.size());
}
template<serialization::Endian E, class T, size_t N>
size_t extract(Serializer<E>& serializer, const std::array<T,N>& values)
{
    return extract(serializer, values.data(), values.size());
}

//
// Multiple values at once - more efficient since it avoids multiple size checks
//

#if __cpp_fold_expressions >= 201603L && __cpp_if_constexpr >= 201606L
template<serialization::Endian E, typename... Ts>
typename std::enable_if<(sizeof...(Ts) > 1), size_t>::type
/*size_t*/ insert(Serializer<E>& buffer, Ts... values)
{
    if constexpr( (std::is_arithmetic<Ts>::value && ...) )
    {
        const size_t size = ( ... + sizeof(Ts) );

        if(uint8_t* ptr = buffer.getPtrAndAdvance(size))
        {
            size_t offset = 0;
            ( ..., (offset += serialization::write<E>(ptr+offset, values)) );
            return offset;
        }

        return size;
    }
    else  // Class types may not have fixed sizes, can't optimize them
        return ( ... + insert(buffer, values) );
}
#else
template<serialization::Endian E, typename T0, typename... Ts>
size_t insert(Serializer<E>& serializer, T0 value0, Ts... values)
{
    return insert(serializer, value0) + insert(serializer, values...);
}
#endif


#if __cpp_fold_expressions >= 201603L && __cpp_if_constexpr >= 201606L
template<serialization::Endian E, typename... Ts>
typename std::enable_if<(sizeof...(Ts) > 1), size_t>::type
/*size_t*/ extract(Serializer<E>& buffer, Ts&... values)
{
    if constexpr( (std::is_arithmetic<Ts>::value && ...) )
    {
        const size_t size = ( ... + sizeof(Ts) );

        if(uint8_t* ptr = buffer.getPtrAndAdvance(size))
        {
            size_t offset = 0;
            ( ..., (offset += serialization::read<E>(ptr+offset, values)) );
            return offset;
        }

        return size;
    }
    else  // Class types may not have fixed sizes, can't optimize them
        return ( ... + extract(buffer, values) );
}
#else
template<serialization::Endian E, typename T0, typename... Ts>
size_t extract(Serializer<E>& serializer, T0 value0, Ts... values)
{
    return extract(serializer, value0) + extract(serializer, values...);
}
#endif


//
// Raw buffer - avoids the need to create a serializer yourself.
//

template<serialization::Endian E, class T>
bool insert(const T& value, uint8_t* buffer, size_t buffer_length, size_t offset=0, bool exact_size=true)
{
    Serializer<E> serializer(buffer, buffer_length, offset);
    serializer.insert(value);
    return exact_size ? serializer.isFinished() : serializer.isOk();
}

template<serialization::Endian E, class T>
bool extract(T& value, const uint8_t* buffer, size_t buffer_length, size_t offset=0, bool exact_size=false)
{
    Serializer<E> serializer(buffer, buffer_length, offset);
    extract(serializer, value);
    return exact_size ? serializer.isFinished() : serializer.isOk();
}



//
// Special Deserialization
//


// Deserialize and return by value

#ifdef HAS_OPTIONAL
template<class T, serialization::Endian E>
std::optional<T> extract(Serializer<E>& serializer)
{
    T value;
    if(extract<E,T>(serializer, value))
        return value;
    else
        return std::nullopt;
}

template<class T, serialization::Endian E>
std::optional<T> extract(const uint8_t* buffer, size_t length, size_t offset, bool exact_size=false)
{
    T value;
    if(extract<E,T>(value, buffer, length, offset, exact_size))
        return value;
    else
        return std::nullopt;
}
#endif


//
//
// Serializer member functions which depend on the above overloads.
//
//

template<serialization::Endian E>
template<typename... Ts>
bool Serializer<E>::insert(const Ts&... values)
{
    // Prevents infinite recursion but allows ADL
    // https://stackoverflow.com/questions/13407205/calling-nonmember-instead-of-member-function
    using microstrain::insert;

    return insert(*this, values...);
}

template<serialization::Endian E>
template<typename... Ts>
bool Serializer<E>::extract(Ts&... values)
{
    // Prevents infinite recursion but allows ADL
    // https://stackoverflow.com/questions/13407205/calling-nonmember-instead-of-member-function
    using microstrain::extract;

    return extract(*this, values...);
}

// Extract a counter with maximum value.
template<serialization::Endian E>
template<class T, class S>
bool Serializer<E>::extract_count(T& count, S max_count)
{
    this->extract(count);

    if(count <= max_count)
        return true;

    count = 0;
    invalidate();

    return false;
}

} // namespace microstrain