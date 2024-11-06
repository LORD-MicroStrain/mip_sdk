
#include "test.h"

#include <stdarg.h>
#include <string.h>


unsigned int num_errors = 0;

void print_buffer(FILE* file, const uint8_t* buffer, size_t length)
{
    for(unsigned int i=0; i<length; i++)
    {
        fprintf(file, " %02X", buffer[i]);
    }

    fputc('\n', file);
}

bool check(bool condition, const char* fmt, ...)
{
    if( condition )
        return true;

    va_list argptr;
    va_start(argptr, fmt);
    vfprintf(stderr, fmt, argptr);
    va_end(argptr);

    fputc('\n', stderr);

    //print_buffer(stderr);

    num_errors++;
    return false;
}

bool check_equal(int a, int b, const char* fmt, ...)
{
    if( a == b )
        return true;

    va_list argptr;
    va_start(argptr, fmt);
    vfprintf(stderr, fmt, argptr);
    va_end(argptr);

    fprintf(stderr, " (%d != %d)", a, b);

    fputc('\n', stderr);

    //print_buffer(stderr);

    num_errors++;
    return false;
}
