using namespace microstrain::C;
////////////////////////////////////////////////////////////////////////////////
///@page Serialization
///
////////////////////////////////////////////////////////////////////////////////
///@section serialization_C Serialization in C
///
///@see microstrain::C::microstrain_serializer
///@see microstrain_serializer
///
/// Serialization infrastructure in C is very basic and is currently limited to
/// built-in types and big-endian protocols.
///
/// To (de)serialize a buffer, follow these steps:
/// 1. Create a serializer and initialize it via microstrain_serializer_init_insertion
///    or microstrain_serializer_init_extraction, depending on whether you're
///    writing or reading data.
/// 2. Call microstrain_insert_* or microstrain_extract_* for each parameter.
/// 3. Call microstrain_serializer_is_ok to check if all the data was
///    written/read successfully (i.e. fit in the buffer). Alternatively, to
///    verify if exactly buffer_size bytes were read/written, use microstrain_serializer_is_complete.
/// 4. Transmit the written buffer or use the deserialized parameters.
///
/// When reading an array length from a buffer, it is recommended to use
/// microstrain_extract_count to specify a maximum length. This helps avoid
/// buffer overrun bugs and associated security vulnerabilities.
///
////////////////////////////////////////////////////////////////////////////////
///@section serialization_CPP Serialization in C++
///
/// The MIP SDK includes a complete serialization system in C++. It supports
/// both big- and little-endian buffers and user-defined types.
///
///@subsection basic_usage Basic usage
///
/// To (de)serialize a buffer, follow these steps:
/// 1. Create a microstrain::Serializer, passing in a pointer to your buffer
///    and the size. A starting offset may also be specified for convenience.
/// 2. Call serializer.insert or serializer.extract with the values to be
///    (de)serialized. Multiple calls may be made to these functions if needed.
///    When reading an array length from a buffer, it is recommended to use
///    Serializer::extract_count to specify a maximum count. This helps avoid
///    buffer overrun bugs and associated security vulnerabilities.
/// 3. Check if the data was written/read successfully (i.e. fit in the buffer)
///    by calling Serializer::isOk or Serializer::isFinished (use the latter if
///    the entire buffer should have been used).
/// 4. Transmit the written buffer or use the deserialized parameters.
///
///@subsection user_types User-defined types
///
/// All serialization goes through microstrain::insert / microstrain::extract.
/// These are non-member functions and are overloaded for various data types.
/// This makes it possible to extend serialization to new types.
///
///@par Classes and structs
///
/// Classes and structs may include one or more of the following member
/// functions to enable serialization support:
///
///@li `void insert(microstrain::BigEndianSerializer& serializer) const`
///@li `void insert(microstrain::LittleEndianSerializer& serializer) const`
///@li `template<microstrain::Endian E> void insert(microstrain::Serializer<E>& serializer) const`
///
///@li `void extract(microstrain::BigEndianSerializer& serializer)`
///@li `void extract(microstrain::LittleEndianSerializer& serializer)`
///@li `template<microstrain::Endian E> void extract(microstrain::Serializer<E>& serializer)`
///
/// In addition, the serialization non-member functions may be overloaded as
/// described next.
///
///@par Other user-defined types
///
/// Serialization for any custom type may be implemented by overloading
/// the microstrain::insert / microstrain::extract functions. For example:
///@code{.cpp}
/// namespace custom {
/// // An enum which is not typed and thus cannot use the regular enum methods.
/// enum Foo { A=0, B, C, MAX_FOO };
///
/// template<microstrain::Endian E>
/// size_t insert(microstrain::Serializer<E>& serializer, Foo foo)
/// {
///     return microstrain::insert(serializer, uint8_t(foo));
/// }
///
/// template<microstrain::Endian E>
/// size_t extract(microstrain::Serializer<E>& serializer, Foo& foo)
/// {
///     uint8_t value;
///     size_t size = microstrain::extract(serializer, value);
///     if(serializer.isOk())  // Optional check
///     {
///         if(value < MAX_FOO)  // Another optional check
///             foo = value;
///         else
///             serializer.invalidate();
///     }
///     return size;
/// }
/// }  // namespace custom
///
/// // Test function
/// void read_write_foo(custom::Foo foo)
/// {
///     uint8_t buffer[8];
///     microstrain::BigEndianSerializer serializer(buffer, sizeof(buffer));
///     serializer.insert(foo);
///
///     custom::Foo foo2;
///     serializer.setOffset(0);  // Jump back to start of buffer
///     serializer.extract(foo2);
///     assert(foo2 == foo);
/// }
///
///@endcode
///
///@image html serialization.svg
///
