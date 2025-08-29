
#include <stdarg.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
#include <type_traits>
extern "C" {
#endif

extern unsigned int num_errors;

void print_buffer(FILE* file, const uint8_t* buffer, size_t length);

bool check(bool value, const char* fmt, ...);
bool check_equal(int a, int b, const char* fmt, ...);


#ifdef __cplusplus
} // extern "C"

inline void printT(FILE* file, int value) { fprintf(file, "%d", value); }
inline void printT(FILE* file, unsigned int value) { fprintf(file, "%u", value); }
inline void printT(FILE* file, std::enable_if<!std::is_same<size_t, unsigned int>::value, size_t>::type value) { fprintf(file, "%zu", value); }
inline void printT(FILE* file, const void* value) { fprintf(file, "%p", value); }

template<typename A, typename B>
bool check_equal(A a, B b, const char* fmt, ...)
{
    if( a == b )
        return true;

    va_list argptr;
    va_start(argptr, fmt);
    vfprintf(stderr, fmt, argptr);
    va_end(argptr);

    fprintf(stderr, "( ");
    printT(stderr, a);
    fprintf(stderr, " != ");
    printT(stderr, b);
    fprintf(stderr, ")\n");

    num_errors++;
    return false;
}

#endif
