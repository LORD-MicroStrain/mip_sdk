Serialization  {#serialization}
=============

Serialization in C  {#serialization_c}
------------------

Serialization infrastructure in C is very basic and is currently limited to built-in types and big-endian protocols.

To (de)serialize a buffer, follow these steps:
1. Create a serializer and initialize it via @ref microstrain_serializer_init_insertion
   or @ref microstrain_serializer_init_extraction, depending on whether you're 
   writing or reading data.
2. Call `microstrain_insert_*` or `microstrain_extract_*` for each parameter. E.g. [microstrain_extract_u32](@ref microstrain::C::microstrain_extract_u32).
3. Call @ref microstrain_serializer_is_ok to check if all the data was written/read successfully (i.e. fit in the buffer).
   Alternatively, to verify if exactly buffer_size bytes were read/written, use @ref microstrain_serializer_is_complete.
4. Transmit the written buffer or use the deserialized parameters.

When reading an array length from a buffer, it is recommended to use @ref microstrain_extract_count to specify a maximum
length. This helps avoid buffer overrun bugs and associated security vulnerabilities.

Serialization in C++  {#serialization_cpp}
--------------------

The MIP SDK includes a complete serialization system in C++. It supports both big- and little-endian buffers and user-
defined types.

### Basic usage

To (de)serialize a buffer, follow these steps:
1. Create a microstrain::Serializer, passing in a pointer to your buffer and the size. A starting offset may also be
   specified for convenience.
2. Call microstrain::Serializer::insert or microstrain::Serializer.extract with the values to be (de)serialized. Multiple calls may be made to
   these functions if needed. When reading an array length from a buffer, it is recommended to use
   microstrain::Serializer::extract_count to specify a maximum count. This helps avoid buffer overrun bugs and associated security
   vulnerabilities.
3. Check if the data was written/read successfully (i.e. fit in the buffer) by calling microstrain::Serializer::isOk or
   microstrain::Serializer::isFinished (use the latter if the entire buffer should have been used).
4. Transmit the written buffer or use the deserialized parameters.

Example:

~~~~~~~~{.cpp}
int main()
{
    // Byte buffers
    uint8_t buffer_le[128];
    uint8_t buffer_be[128];
    
    // Create serializers
    microstrain::BigEndianSerializer    bes(buffer_be, sizeof(buffer_be));
    microstrain::LittleEndianSerializer les(buffer_le, sizeof(buffer_le));
    
    // Alternatively, specify the endianness via template argument.
    //microstrain::Serializer<microstrain::Endian::big   > bes2(buffer_be, sizeof(buffer_be));
    //microstrain::Serializer<microstrain::Endian::little> les2(buffer_le, sizeof(buffer_le));
    
    // Some variables to serialize
    uint8_t  a = 22;
    int8_t   b = -33;
    uint16_t c = 1024;
    int32_t  d = -1000000;
    uint64_t e = 0x81726354AABBCCDD;
    float    f = 1.25f;
    double   g = -1.1;
    
    // Serialize all the basic variables.
    bes.insert(a,b,c,d,e,f,g);
    les.insert(a,b,c,d,e,f,g);
    
    // buffer_be = [0x16, 0xDF, 0x04,0x00, 0xFF,0xF0,0xBD,0xC0, 0x81,0x72,0x63,0x54,0xAA,0xBB,0xCC,0xDD, 0x3F,0xA0,0x00,0x00, 0xBF,0xF1,0x99,0x99,0x99,0x99,0x99,0x9a]
    // buffer_le = [0x16, 0xDF, 0x00,0x04, 0xC0,0xBD,0xF0,0xFF, 0xDD,0xCC,0xBB,0xAA,0x54,0x63,0x72,0x81, 0x00,0x00,0xA0,0x3F, 0x9a,0x99,0x99,0x99,0x99,0x99,0xF1,0xBF]
    
    // Serialize 20 u64s using C-style array or pointer and size.
    uint64_t too_much[20] = {0};
    bes.insert(too_much);          // Size is deduced from C-style array.
    les.insert(&too_much[0], 20);  // 20 items, not 20 bytes.
    
    // Too much data! Note: no actual overrun / invalid access of the buffer has occurred.
    // (20*sizeof(uint64_t) = 160)  >  (sizeof(buffer) = 128)
    assert(!bes.isOk() && !les.isOk());
    
    // Jump back to the start of the buffer for deserialization.
    // Note: this clears the overrun condition.
    bes.setOffset(0);
    les.setOffset(0);
    assert(bes.isOk() && les.isOk());
    
    // Deserialize the values.
    bes.extract(a,b,c,d,e,f,g);
    les.extract(a,b,c,d,e,f,g);
    
    // Check if everything was deserialized successfully.
    if(!bes.isOk() || !les.isOk())
     return 1;
    
    // Jump to specific offset.
    bes.setOffset(4);
    les.setOffset(4);
    
    // Try to read a value using std::optional (subject to compiler support for C++17).
    std::optional vg = microstrain::extract<int32_t>(bes);
    assert( vg.has_value() && *vg == d );
    
    // Reset again
    bes.setOffset(0);
    les.setOffset(0);
    
    // See following examples
    
    return 0;
}
~~~~~~~~

### Supported Types

The serialization library has support for the following basic types:
* Booleans (as a u8; false->[0x00], true->[0x01]; 0x00 reads as false, anything else as true)
* Signed and unsigned integers of various sizes (u8, s8, u16, ..., u64, s64)
* Floating point values (float and double)
* Enums, provided they have an underlying type specified

Additionally, the following compound types are supported:
* Arrays, both fixed-size and runtime size
  * `std::array<T, N>`
  * `microstrain::Span<T>` / `std::span<T>`
  * C-style arrays of fixed, known size
  * Pointer and size
* std::tuple (with c++17 support)
* User-defined types (see below)

 Adding to the example above, we have:
~~~~~~~~{.cpp}
    // A strongly-typed enum
    enum class MyEnum : uint8_t { ZERO=0, ONE=1, TWO=2, THREE, FOUR };
    MyEnum me = MyEnum::TWO;
    
    auto basics = std::make_tuple(a,b,c,d,e,f,g);
    std::array<float, 4> vector4 = {1.0f, 2.0f, 3.0f, 4.0f};
    microstrain::Span<float> vector3(vector4[0], 3);
    
    /// Serialize a custom enum and the contents of a std::tuple.
    bes.insert(me, basics);
    les.insert(me, basics);
    
    /// Serialize an array and Span using the non-member functions.
    /// This is equivalent to `s.insert(vector4, vector3)`.
    microstrain::insert(bes, vector4, vector3);
    microstrain::insert(les, vector4, vector3);
~~~~~~~~

### Serialization "One-liners"

For convenience, a few additional methods are provided for serialization in a single line.
* insert to raw buffer
* extract from raw buffer
* extract to std::optional

~~~~~~~~{.cpp}
void one_liners()
{
     uint8_t buffer[4];
     microstrain::Span<uint8_t> span(buffer, sizeof(buffer));

     int32_t x = -501;

     // Write a single value to a buffer.
     bool ok1 = microstrain::insert<microstrain::Endian::big>(x, span);
     bool ok2 = microstrain::insert<microstrain::Endian::big>(x, buffer, sizeof(buffer));
     bool ok3 = microstrain::insert<microstrain::Endian::big>(x, buffer, sizeof(buffer), 0, true);  // Enforces all bytes used

     assert(ok1 && ok2 && ok3);

     // Read a single value from the buffer
     int32_t y1,y2,y3;
     ok1 = microstrain::extract<microstrain::Endian::big>(y1, span);
     ok2 = microstrain::extract<microstrain::Endian::big>(y2, span, 0, true);  // Enforces all bytes read
     ok3 = microstrain::extract<microstrain::Endian::big>(y3, buffer, sizeof(buffer));

     assert(ok1 && ok2 && ok3 && y1 == x && y2 == x && y3 == x);

     // Read a value of the specified type from the buffer.
     // Note: only available with std::optional support from C++17.
     std::optional<float> value1 = microstrain::extract<microstrain::Endian::big, int32_t>(span);
     std::optional<float> value2 = microstrain::extract<microstrain::Endian::big, int32_t>(buffer, sizeof(buffer));

     assert(value1.has_value() && value2.has_value());
     assert(*value1 == x && *value2 == x);
}
~~~~~~~~

### User-defined types

#### Classes and structs

Classes and structs may include one or more of the following member functions to enable serialization support:
* `void insert(microstrain::BigEndianSerializer& serializer) const`
* `void insert(microstrain::LittleEndianSerializer& serializer) const`
* `template<microstrain::Endian E> void insert(microstrain::Serializer<E>& serializer) const`
* `void extract(microstrain::BigEndianSerializer& serializer)`
* `void extract(microstrain::LittleEndianSerializer& serializer)`
* `template<microstrain::Endian E> void extract(microstrain::Serializer<E>& serializer)`

In addition, the serialization non-member functions may be overloaded as described next.

#### Other user-defined types

All serialization goes through `microstrain::insert` / `microstrain::extract`. These are non-member functions and are
overloaded for various data types. This makes it possible to extend serialization to new types.
Serialization for any custom type may be implemented by overloading the `insert` or `extract`
functions. For example:

~~~~~~~~{.cpp}
namespace custom
{
    // An enum which is not typed and thus cannot use the regular enum methods.
    enum Foo { A=0, B, C, MAX_FOO };
    
    // Insert Foo function which just converts to u8.
    template<microstrain::Endian E>
    size_t insert(microstrain::Serializer<E>& serializer, Foo foo)
    {
        return microstrain::insert(serializer, uint8_t(foo));
    }
    
    // Extract Foo function which range-checks and converts a u8.
    template<microstrain::Endian E>
    size_t extract(microstrain::Serializer<E>& serializer, Foo& foo)
    {
        uint8_t value;
        size_t size = microstrain::extract(serializer, value);
        if(serializer.isOk())  // Optional check
        {
            if(value < MAX_FOO)  // Another optional check
                foo = value;
            else
                serializer.invalidate();
        }
        return size;
    }

}  // end namespace custom

// Test function
void write_read_foo(custom::Foo foo)
{
    // A byte buffer of 8 bytes.
    uint8_t buffer[8];
    
    // Create the serializer, passing in the buffer.
    microstrain::BigEndianSerializer serializer(buffer, sizeof(buffer));
    
    // Write foo to the buffer.
    // This calls Serializer::insert, which calls the non-member function 'insert'.
    // Despite being in the 'custom' namespace, the 'insert(Serializer&, Foo)'
    // function will be found via argument-dependent lookup because
    // 'Foo' is also in that namespace.
    serializer.insert(foo);
    
    // Jump back to start of buffer for reading.
    serializer.setOffset(0);
    
    // Read foo back out and compare to original.
    custom::Foo foo2;
    serializer.extract(foo2);
    
    assert(foo2 == foo);
}
~~~~~~~~

### Serialization System Architecture

The MIP library implements many custom types and heavily leverages the serialization system. It uses `insert`/`extract`
overloads, class methods, strongly-typed enums, and arrays.

The Serializer uses read/write functions from the microstrain::serialization namespace to handle endianness /
byteswapping and packing at the lowest level.

This diagram describes the architecture of the serialization system:

![Serialization Diagram](serialization.svg)
