#pragma once

#include <type_traits>
#include <stdint.h>
#include <stddef.h>
#include <assert.h>

namespace microstrain
{


static constexpr size_t DYNAMIC_EXTENT = SIZE_MAX;

////////////////////////////////////////////////////////////////////////////////
///@brief Represents a view over a contiguous array of objects, similar to
///       std::span.
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

private:
    pointer m_ptr   = nullptr;
    size_t  m_cnt = 0;
};


template<class T>
using ConstArrayView = ArrayView<const T>;

using BufferView      = ArrayView<uint8_t>;
using ConstBufferView = ConstArrayView<uint8_t>;

using CharView        = ArrayView<char>;
using ConstCharView   = ConstArrayView<char>;


} // namespace microstrain
