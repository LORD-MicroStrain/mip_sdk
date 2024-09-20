#pragma once

#if MICROSTRAIN_USE_STD_SPAN

#include <span>

namespace microstrain
{

    static inline constexpr size_t DYNAMIC_EXTENT = std::dynamic_extent;

    template<class T, size_t Size=DYNAMIC_EXTENT>
    using Span = std::span<T, Size>;

}

#else // MICROSTRAIN_USE_STD_SPAN

#include <type_traits>
#include <stddef.h>

namespace microstrain
{


static constexpr size_t DYNAMIC_EXTENT = -1;

template<class T, size_t Extent=DYNAMIC_EXTENT>
struct Span
{
    static constexpr size_t extent = Extent;

    using element_type = T;
    using value_type = typename std::remove_cv<T>::type;
    using difference_type = ptrdiff_t;
    using pointer = T*;
    using reference = T&;
    using const_pointer = const T*;
    using const_reference = const T&;

    Span(pointer ptr) : m_ptr(ptr) {}

    constexpr pointer begin() const noexcept { return m_ptr; }
    constexpr pointer end() const noexcept { return m_ptr+extent; }

    constexpr element_type front() const noexcept { return *m_ptr; }
    constexpr element_type back() const noexcept { return *m_ptr[extent-1]; }

    constexpr reference operator[](size_t idx) noexcept { return m_ptr[idx]; }
    constexpr const_reference operator[](size_t idx) const noexcept { return m_ptr[idx]; }

    constexpr pointer data() const noexcept { return m_ptr; }

    [[nodiscard]] constexpr size_t size() const noexcept { return extent; }
    [[nodiscard]] constexpr bool empty() const noexcept { return extent == 0; }

private:
    const pointer m_ptr = nullptr;
};


template<class T>
struct Span<T, DYNAMIC_EXTENT>
{
    using element_type = T;
    using value_type = typename std::remove_cv<T>::type;
    using difference_type = ptrdiff_t;
    using pointer = T*;
    using reference = T&;
    using const_pointer = const T*;
    using const_reference = const T&;

    Span(pointer ptr, size_t cnt) : m_ptr(ptr), m_cnt(cnt) {}

    constexpr pointer begin() const noexcept { return m_ptr; }
    constexpr pointer end() const noexcept { return m_ptr+m_cnt; }

    constexpr element_type front() const noexcept { return *m_ptr; }
    constexpr element_type back() const noexcept { return *m_ptr[m_cnt-1]; }

    constexpr reference operator[](size_t idx) noexcept { return m_ptr[idx]; }
    constexpr const_reference operator[](size_t idx) const noexcept { return m_ptr[idx]; }

    constexpr pointer data() const noexcept { return m_ptr; }

    [[nodiscard]] constexpr size_t size() const noexcept { return m_cnt; }
    [[nodiscard]] constexpr bool empty() const noexcept { return m_cnt == 0; }

private:
    pointer const m_ptr   = nullptr;
    size_t  const m_cnt = 0;
};


}  // namespace microstrain

#endif // MICROSTRAIN_USE_STD_SPAN
