#pragma once

#include <stdint.h>
#include <stddef.h>

#include <type_traits>
#include <tuple>


namespace microstrain
{


//
// Basic Insertion
//

template<class T>
typename std::enable_if<std::is_arithmetic<T>::value && sizeof(T)==1, size_t>::type
/*size_t*/ write(uint8_t* buffer, T value)
{
    buffer[0] = reinterpret_cast<const uint8_t*>(&value)[0];
    return sizeof(T);
}

template<class T>
typename std::enable_if<std::is_arithmetic<T>::value && sizeof(T)==2, size_t>::type
/*size_t*/ write(uint8_t* buffer, T value)
{
    buffer[0] = reinterpret_cast<const uint8_t*>(&value)[1];
    buffer[1] = reinterpret_cast<const uint8_t*>(&value)[0];
    return sizeof(T);
}

template<class T>
typename std::enable_if<std::is_arithmetic<T>::value && sizeof(T)==4, size_t>::type
/*size_t*/ write(uint8_t* buffer, T value)
{
    buffer[0] = reinterpret_cast<const uint8_t*>(&value)[3];
    buffer[1] = reinterpret_cast<const uint8_t*>(&value)[2];
    buffer[2] = reinterpret_cast<const uint8_t*>(&value)[1];
    buffer[3] = reinterpret_cast<const uint8_t*>(&value)[0];
    return sizeof(T);
}

template<class T>
typename std::enable_if<std::is_arithmetic<T>::value && sizeof(T)==8, size_t>::type
/*size_t*/ write(uint8_t* buffer, T value)
{
    buffer[0] = reinterpret_cast<const uint8_t*>(&value)[7];
    buffer[1] = reinterpret_cast<const uint8_t*>(&value)[6];
    buffer[2] = reinterpret_cast<const uint8_t*>(&value)[5];
    buffer[3] = reinterpret_cast<const uint8_t*>(&value)[4];
    buffer[4] = reinterpret_cast<const uint8_t*>(&value)[3];
    buffer[5] = reinterpret_cast<const uint8_t*>(&value)[2];
    buffer[6] = reinterpret_cast<const uint8_t*>(&value)[1];
    buffer[7] = reinterpret_cast<const uint8_t*>(&value)[0];
    return sizeof(T);
}

//
// Basic Extraction
//

template<class T>
typename std::enable_if<std::is_arithmetic<T>::value && sizeof(T)==1, size_t>::type
/*size_t*/ read(const uint8_t* buffer, T& value)
{
    reinterpret_cast<uint8_t*>(&value)[0] = buffer[0];
    return sizeof(T);
}

template<class T>
typename std::enable_if<std::is_arithmetic<T>::value && sizeof(T)==2, size_t>::type
/*size_t*/ read(const uint8_t* buffer, T& value)
{
    reinterpret_cast<uint8_t*>(&value)[0] = buffer[1];
    reinterpret_cast<uint8_t*>(&value)[1] = buffer[0];
    return sizeof(T);
}

template<class T>
typename std::enable_if<std::is_arithmetic<T>::value && sizeof(T)==4, size_t>::type
/*size_t*/ read(const uint8_t* buffer, T& value)
{
    reinterpret_cast<uint8_t*>(&value)[0] = buffer[3];
    reinterpret_cast<uint8_t*>(&value)[1] = buffer[2];
    reinterpret_cast<uint8_t*>(&value)[2] = buffer[1];
    reinterpret_cast<uint8_t*>(&value)[3] = buffer[0];
    return sizeof(T);
}

template<class T>
typename std::enable_if<std::is_arithmetic<T>::value && sizeof(T)==8, size_t>::type
/*size_t*/ read(const uint8_t* buffer, T& value)
{
    reinterpret_cast<uint8_t*>(&value)[0] = buffer[7];
    reinterpret_cast<uint8_t*>(&value)[1] = buffer[6];
    reinterpret_cast<uint8_t*>(&value)[2] = buffer[5];
    reinterpret_cast<uint8_t*>(&value)[3] = buffer[4];
    reinterpret_cast<uint8_t*>(&value)[4] = buffer[3];
    reinterpret_cast<uint8_t*>(&value)[5] = buffer[2];
    reinterpret_cast<uint8_t*>(&value)[6] = buffer[1];
    reinterpret_cast<uint8_t*>(&value)[7] = buffer[0];
    return sizeof(T);
}


class Serializer
{
public:
    Serializer() = default;
    Serializer(uint8_t* ptr, size_t capacity, size_t offset=0) : m_ptr(ptr), m_size(capacity), m_offset(offset) {}
    Serializer(const uint8_t* ptr, size_t size, size_t offset=0) : m_ptr(const_cast<uint8_t*>(ptr)), m_size(size), m_offset(offset) {}

    size_t capacity() const { return m_size; }
    size_t length() const { return m_size; }
    size_t offset() const { return m_offset; }
    int remaining() const { return int(m_size - m_offset); }

    bool noRemaining() const { return m_offset == m_size; }
    bool isOverrun() const { return m_offset > m_size; }
    bool isOk() const { return !isOverrun(); }
    bool hasRemaining(size_t count) const { return m_offset+count <= m_size; }

    const uint8_t* basePointer() const { return m_ptr; }
    uint8_t* basePointer() { return m_ptr; }

    const uint8_t* pointer(size_t required_size) const { return hasRemaining(required_size) ? (m_ptr+m_offset) : nullptr; }
    uint8_t* pointer(size_t required_size) { return hasRemaining(required_size) ? (m_ptr+m_offset) : nullptr; }

    uint8_t* getPtrAndAdvance(size_t size) { uint8_t* ptr = hasRemaining(size) ? (m_ptr+m_offset) : nullptr; m_offset += size; return ptr; }

    void invalidate() { m_offset = size_t(-1); }

    template<typename... Ts>
    bool insert(const Ts&... values);

    template<typename... Ts>
    bool extract(Ts&... values);

    template<typename T>
    struct Counter
    {
        T& count;
        size_t max_count = 0;
    };

private:
    uint8_t* m_ptr    = nullptr;
    size_t   m_size   = 0;
    size_t   m_offset = 0;
};



//template<class T, typename=void>
//struct fixed_size {};
//
//template<class T>
//struct fixed_size<T, std::void_t<decltype(T::SIZE_BYTES)>> : std::integral_constant<size_t, T::SIZE_BYTES> {};


//
// General Insertion
//

// Built-in types (bool, int, float, ...)
template<class T>
typename std::enable_if<std::is_arithmetic<T>::value, size_t>::type
/*size_t*/ insert(Serializer& buffer, T value)
{
    if(auto ptr = buffer.getPtrAndAdvance(sizeof(T)))
        write(ptr, value);

    return sizeof(T);
}

// Enums
template<class T>
typename std::enable_if<std::is_enum<T>::value, size_t>::type
/*size_t*/ insert(Serializer& buffer, T value)
{
    using BaseType = typename std::underlying_type<T>::type;

    if(auto ptr = buffer.getPtrAndAdvance(sizeof(BaseType)))
        write(ptr, static_cast<BaseType>(value));

    return sizeof(BaseType);
}

// std::tuple
template<class... Ts>
size_t insert(Serializer& serializer, const std::tuple<Ts...>& values)
{
    auto lambda = [&serializer](const auto&... args) {
        return insert(serializer, args...);
    };

    return std::apply(lambda, values);
}


// Multiple values at once
#if __cpp_fold_expressions >= 201603L && __cpp_if_constexpr >= 201606L
template<typename... Ts>
typename std::enable_if<(sizeof...(Ts) > 1), size_t>::type
/*size_t*/ insert(Serializer& buffer, Ts... values)
{
    if constexpr( (std::is_arithmetic<Ts>::value && ...) )
    {
        const size_t size = ( ... + sizeof(Ts) );

        if(uint8_t* ptr = buffer.getPtrAndAdvance(size))
        {
            size_t offset = 0;
            ( ..., (offset += write(ptr+offset, values)) );
            return offset;
        }

        return size;
    }
    else
        return ( ... + insert(buffer, values) );
}
#else
template<typename T0, typename... Ts>
size_t insert(Buffer& buffer, T0 value0, Ts... values)
{
    return insert(buffer, value0) + insert(buffer, values...);
}
#endif


//template<typename... Ts>
//size_t insert(Buffer& buffer, Ts... values) { return detail::insert(buffer, values...); }


//// Class types with known size
//template<class T, size_t Size = fixed_size<T>::value>
//size_t insert(Buffer& buffer, T value)
//{
//    if(auto ptr = buffer.getPtrAndAdvance(Size))
//        insert
//}

//
// General Extraction
//

// Built-in types (bool, int, float, ...)
template<class T>
typename std::enable_if<std::is_arithmetic<T>::value, size_t>::type
/*size_t*/ extract(Serializer& buffer, T& value)
{
    if(auto ptr = buffer.getPtrAndAdvance(sizeof(T)))
        read(ptr, value);

    return sizeof(T);
}

// Enums
template<class T>
typename std::enable_if<std::is_enum<T>::value, size_t>::type
/*size_t*/ extract(Serializer& buffer, T& value)
{
    using BaseType = typename std::underlying_type<T>::type;

    if(auto ptr = buffer.getPtrAndAdvance(sizeof(BaseType)))
    {
        BaseType base;
        read(ptr, base);
        value = static_cast<T>(base);
    }

    return sizeof(BaseType);
}

// std::tuple of references
template<class... Ts>
size_t extract(Serializer& serializer, const std::tuple<std::reference_wrapper<Ts>...>& values)
{
    auto lambda = [&serializer](auto&... args) {
        return extract(serializer, args...);
    };

    return std::apply(lambda, values);
}
// // Generic class types - assume class has a `size_t serialize(Buffer& buffer) const` function.
// template<class T>
// typename std::enable_if<std::is_class<T>::value, size_t>::type
// /*size_t*/ extract(Serializer& buffer, T& value)
// {
//     return value.deserialize(buffer);
// }

#if __cpp_fold_expressions >= 201603L && __cpp_if_constexpr >= 201606L
template<typename... Ts>
typename std::enable_if<(sizeof...(Ts) > 1), size_t>::type
/*size_t*/ extract(Serializer& buffer, Ts&... values)
{
    if constexpr( (std::is_arithmetic<Ts>::value && ...) )
    {
        const size_t size = ( ... + sizeof(Ts) );

        if(uint8_t* ptr = buffer.getPtrAndAdvance(size))
        {
            size_t offset = 0;
            ( ..., (offset += read(ptr+offset, values)) );
            return offset;
        }

        return size;
    }
    else
        return ( ... + extract(buffer, values) );
}
#else
template<typename T0, typename... Ts>
size_t extract(Buffer& buffer, T0 value0, Ts... values)
{
    return extract(buffer, value0) + extract(buffer, values...);
}
#endif



//template<class T>
//size_t extract(Buffer& buffer, Buffer::Counter<T> counter)
//{
//    size_t size = extract(counter.count);
//
//    if(counter.count > counter.max_count)
//        buffer.invalidate();
//
//    return size;
//}

template<class T>
size_t extract_count(Serializer& buffer, T& count, size_t max_count)
{
    T tmp;
    size_t size = extract(buffer, tmp);

    if(tmp <= max_count)
        count = tmp;
    else
        buffer.invalidate();

    return size;
}
template<class T>
size_t extract_count(Serializer& buffer, T* count, size_t max_count) { return extract_count(buffer, *count, max_count); }



template<typename... Ts>
bool Serializer::insert(const Ts&... values) { return microstrain::insert(*this, values...); }

template<typename... Ts>
bool Serializer::extract(Ts&... values) { return microstrain::extract(*this, values...); }


} // namespace microstrain
