#pragma once

#include <array>
#include <type_traits>
#include <stdint.h>
#include <stddef.h>
#include <assert.h>

namespace microstrain
{


static constexpr size_t DYNAMIC_EXTENT = SIZE_MAX;

////////////////////////////////////////////////////////////////////////////////
///@brief Represents a view over a contiguous array of objects, similar to
///       std::span, and is implemented as a pointer and length.
///
/// It can be used as a function parameter to accommodate std::vector,
/// std::array, and C-style arrays in one function signature. It's acceptable
/// to pass ArrayViews by value, which is equivalent to passing separate pointer
/// and length parameters. The underlying data would not be copied.
///
/// Note that because ArrayView doesn't store the underlying data, the original
/// buffer must remain intact while the view is in use.
///
/// Some functions provide both pointer+length and ArrayView versions, while
/// others provide just the ArrayView version. To call an ArrayView-only
/// function with a pointer and length, just put braces around the pointer and
/// length arguments. This will create an ArrayView implicitly with no
/// performance impact:
/// `theFunctionToCall( {pointer, length} );`
///
/// To pass an ArrayView to a function taking separate pointer and length
/// parameters, call `.data()` and `.size()` for each, like so:
/// `theFunctionToCall( view.data(), view.size() );`
///
/// This class is intended to be mostly compatible and interchangeable with
/// std::span, but certain features from span may be missing and additional ones
/// may be added.
///
/// https://en.cppreference.com/w/cpp/container/span
///
template<class T, size_t Extent=DYNAMIC_EXTENT>
struct ArrayView
{
    static constexpr size_t extent = Extent;

    using element_type = T;
    using value_type = typename std::remove_cv<T>::type;
    using difference_type = ptrdiff_t;
    using pointer = T*;
    using reference = T&;
    using iterator = T*;
    using const_pointer = const T*;
    using const_reference = const T&;
    using const_iterator = const T*;

    constexpr ArrayView(pointer ptr, size_t count) : m_ptr(ptr) { assert(count==extent); }
    constexpr ArrayView(const std::array<T, Extent>& data) : m_ptr(data) {}

    constexpr pointer begin() const noexcept { return m_ptr; }
    constexpr pointer end() const noexcept { return m_ptr+extent; }

    constexpr element_type front() const noexcept { return *m_ptr; }
    constexpr element_type back() const noexcept { return *m_ptr[extent-1]; }

    constexpr reference operator[](size_t idx) noexcept { return m_ptr[idx]; }
    constexpr const_reference operator[](size_t idx) const noexcept { return m_ptr[idx]; }

    constexpr pointer data() const noexcept { return m_ptr; }

    [[nodiscard]] constexpr size_t size() const noexcept { return extent; }
    [[nodiscard]] constexpr bool empty() const noexcept { return extent == 0; }

    [[nodiscard]] constexpr ArrayView<T, DYNAMIC_EXTENT> subspan(size_t index, size_t length) const { return {m_ptr+index, length}; }
    [[nodiscard]] constexpr ArrayView<T, DYNAMIC_EXTENT> subspan(size_t index) const { return {m_ptr+index, extent-index}; }
    template<size_t Offset, size_t Count = DYNAMIC_EXTENT>
    [[nodiscard]] constexpr ArrayView<T, Count == DYNAMIC_EXTENT ? DYNAMIC_EXTENT : Extent-Count> subspan() const { return {m_ptr+Offset}; }

    [[nodiscard]] constexpr ArrayView<T, DYNAMIC_EXTENT> first(size_t count) const { return {m_ptr, count};}
    template<size_t Count>
    [[nodiscard]] constexpr ArrayView<T, Count> first() const { static_assert(Count<=Extent, "Count out of range"); return {m_ptr}; }

    [[nodiscard]] constexpr ArrayView<T, DYNAMIC_EXTENT> last(size_t count) const { return {m_ptr+(size()-count), count};}
    template<size_t Count>
    [[nodiscard]] constexpr ArrayView<T, Count> last() const { static_assert(Count<=Extent, "Count out of range"); return {m_ptr+(Extent-Count)}; }

    operator ArrayView<const T, Extent>() const { return {m_ptr, Extent}; }

private:
    pointer m_ptr = nullptr;
};


template<class T>
struct ArrayView<T, DYNAMIC_EXTENT>
{
    using element_type = T;
    using value_type = typename std::remove_cv<T>::type;
    using difference_type = ptrdiff_t;
    using pointer = T*;
    using reference = T&;
    using iterator = T*;
    using const_pointer = const T*;
    using const_reference = const T&;
    using const_iterator = const T*;

    constexpr ArrayView() = default;
    constexpr ArrayView(pointer ptr, size_t cnt) : m_ptr(ptr), m_cnt(cnt) {}

    template<size_t N>
    constexpr ArrayView(T (&arr)[N]) : m_ptr(arr), m_cnt(N) {}

    template<size_t N>
    constexpr ArrayView(const std::array<T,N>& data) : m_ptr(data.data()), m_cnt(N) {}

    constexpr pointer begin() const noexcept { return m_ptr; }
    constexpr pointer end() const noexcept { return m_ptr+m_cnt; }

    constexpr element_type front() const noexcept { return *m_ptr; }
    constexpr element_type back() const noexcept { return *m_ptr[m_cnt-1]; }

    constexpr reference operator[](size_t idx) noexcept { return m_ptr[idx]; }
    constexpr const_reference operator[](size_t idx) const noexcept { return m_ptr[idx]; }

    constexpr pointer data() const noexcept { return m_ptr; }

    [[nodiscard]] constexpr size_t size() const noexcept { return m_cnt; }
    [[nodiscard]] constexpr bool empty() const noexcept { return m_cnt == 0; }

    [[nodiscard]] constexpr ArrayView<T, DYNAMIC_EXTENT> subspan(size_t index, size_t length) const { return {m_ptr+index, length}; }
    [[nodiscard]] constexpr ArrayView<T, DYNAMIC_EXTENT> subspan(size_t index) const { return {m_ptr+index, m_cnt-index}; }
    template<size_t Offset, size_t Count = DYNAMIC_EXTENT>
    [[nodiscard]] constexpr ArrayView<T, Count> subspan() const { return {m_ptr+Offset, Count}; }

    [[nodiscard]] constexpr ArrayView<T, DYNAMIC_EXTENT> first(size_t count) const { return {m_ptr, count};}
    template<size_t Count>
    [[nodiscard]] constexpr ArrayView<T, Count> first() const { return {m_ptr}; }

    [[nodiscard]] constexpr ArrayView<T, DYNAMIC_EXTENT> last(size_t count) const { return {m_ptr+(size()-count), count};}
    template<size_t Count>
    [[nodiscard]] constexpr ArrayView<T, Count> last() const { return {m_ptr+(size()-Count)}; }

    operator ArrayView<const T, DYNAMIC_EXTENT>() const { return {m_ptr, m_cnt}; }

private:
    pointer m_ptr   = nullptr;
    size_t  m_cnt = 0;
};


template<class T>
using ConstArrayView = ArrayView<const T>;

using U8ArrayView      = ArrayView<uint8_t>;
using ConstU8ArrayView = ConstArrayView<uint8_t>;

using CharArrayView        = ArrayView<char>;
using ConstCharArrayView   = ConstArrayView<char>;


} // namespace microstrain
