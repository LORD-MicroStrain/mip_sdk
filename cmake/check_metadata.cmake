
include(CheckCXXSourceCompiles)

check_cxx_source_compiles("
#include <span>

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

#set(MIP_COMPILER_SUPPORTS_METADATA ${COMPILER_SUPPORTS_METADATA} PARENT_SCOPE)
