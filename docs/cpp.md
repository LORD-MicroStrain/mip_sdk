C++ Standard Support   {#cpp_standards}
====================

The MIP SDK requires at least C++14 support, but some parts of the MIP SDK can make use of newer C++ features.
These parts are guarded by `#ifdefs` for the `__cplusplus` version or [feature test macros](https://en.cppreference.com/w/cpp/feature_test)
and they will be safely ignored if your compiler lacks support.

In a few cases however, this can be dangerous when the MIP SDK is built as a library (e.g., MicroStrain
pre-built versions from github) and then gets used in a project with a different C++ standard version. If the `#ifdef`
guards cause a different implementation to be selected, then there is a mismatch and a violation of the one-definition
rule. This can cause linker errors or mysterious crashes and odd behavior. To avoid this, the MIP SDK requires that the
user explicitly opt-in to certain C++ features. This "opt-in" choice must be made identically when the library is
compiled and then in the project where it is used (this includes anywhere the MIP SDK headers are included).

std::span
---------

A "span" in C++ parlance is the combination of a pointer and length, typically representing a view of an array. The
MIP SDK provides an implementation called `microstrain::Span`, which tries to be compatible with `std::span` from C++20.
It includes most, but not all, features from the standard version. If your compiler supports it, you may use `std::span`
instead by defining `MICROSTRAIN_USE_STD_SPAN`. This will replace the microstrain span implementation with a
typedef to `std::span`.

https://en.cppreference.com/w/cpp/container/span

std::endian
-----------

The serialization system requires a notion of endianness. By default, an enum called `microstrain::Endian` is defined
which provides integer constants representing big and little endianness. If your compiler supports `std::endian` from
C++20, you may define `MICROSTRAIN_USE_STD_ENDIAN` in your project. This will replace the microstrain enum with a
typedef to `std::endian`.

https://en.cppreference.com/w/cpp/types/endian

Other Optional C++ Features
---------------------------

These features can be used with the MIP SDK if your compiler supports them. No special `#defines` are necessary as
compiled library code is not affected by the choice. They are used by header-only features and no alternative
implementations are supplied with which they could conflict.

* [std::apply](https://en.cppreference.com/w/cpp/utility/apply)
* [fold expressions](https://en.cppreference.com/w/cpp/language/fold)
* [constexpr if](https://en.cppreference.com/w/cpp/language/if)
* [std::optional](https://en.cppreference.com/w/cpp/utility/optional)
