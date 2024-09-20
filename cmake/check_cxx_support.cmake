
include(CheckIncludeFileCXX)

CHECK_INCLUDE_FILE_CXX("span" MICROSTRAIN_COMPILER_SUPPORTS_SPAN)
CHECK_INCLUDE_FILE_CXX("bit" MICROSTRAIN_COMPILER_SUPPORTS_BIT)


include(CheckCXXSourceCompiles)

check_cxx_source_compiles("

#if __cpp_constexpr < 201603L
#error \"Metadata not supported\"
#endif

struct Foo
{
    int i = 0;
    const char* s = nullptr;
};

static inline constexpr Foo foo = {5, \"12345\"};

int main()
{
    static_assert(foo.i == 5 && foo.s != nullptr, \"foo has wrong value\");
    return 0;
}

" MIP_COMPILER_SUPPORTS_METADATA)
