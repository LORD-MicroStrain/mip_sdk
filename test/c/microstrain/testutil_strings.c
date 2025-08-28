
#include <assert.h>
#include <ctype.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>


unsigned int g_fail_count = 0;


void print_buffer(const char* buffer, size_t buffer_size)
{
    fputs("      \"", stderr);
    for(size_t i=0; i<buffer_size; i++)
    {
        char c = buffer[i];
        if( isprint((unsigned char)c) )  // Printable characters only
            fputc(c, stderr);
        else if( c == '\0' )
            fputs("\\0", stderr);
        else
            fputc('?', stderr);
    }
    fputs("\"\n", stderr);
}

void print_buffer_context(const char* buffer, size_t buffer_size, size_t where, size_t context)
{
    fputs("      \"", stderr);

    const size_t start = (where > context) ? (where - context) : 0;
    const size_t stop  = (where+context < buffer_size) ? (where+context) : buffer_size;

    for(size_t i=start; i<stop; i++)
    {
        // if(where + i < context)  // Negative index
        //     fputc(' ', stderr);
        // else if(where + i >= buffer_size)  // Exceeds buffer_size
        //     fputc(' ', stderr);
        // else
        // {
        char c = buffer[i];
        if( isprint((unsigned char)c) )  // Printable characters only
            fputc(c, stderr);
        else if( c == '\0' )
            fputs("\\0", stderr);
        else
            fputc('?', stderr);
        // }
    }
    fputs("\"\n", stderr);
}



void assert_condition(bool cond, const char* cond_str, const char* func, const char* note)
{
    if(!cond)
    {
        fprintf(stderr, "FAIL: %s (%s)\n      '%s' failed\n", func, note, cond_str);
        g_fail_count++;
    }
}

void assert_eq_zz(size_t a, size_t b, const char* cond_str, const char* func, const char* note)
{
    if(a != b)
    {
        fprintf(stderr, "FAIL: %s (%s)\n      '%s' failed: %zu != %zu\n", func, note, cond_str, a, b);
        g_fail_count++;
    }
}

void assert_buffer_matches(const char* buffer, const char* compare, size_t count, const char* func, const char* note)
{
    for(size_t i=0; i<count; i++)
    {
        if(buffer[i] != compare[i])
        {
            g_fail_count++;
            fprintf(stderr, "FAIL: %s (%s)\n      Buffer does not match expected result at index %zu\n", func, note, i);
            print_buffer_context(compare, count, i, 20);
            print_buffer_context(buffer,  count, i, 20);
            return;
        }
    }
}

void assert_buffer_not_overrun(const char* buffer, size_t size, size_t offset, const char* func, const char* note)
{
    for(size_t i=offset; i<size; i++)
    {
        char value = buffer[i];
        if( value != '_' )
        {
            fprintf(stderr, "FAIL: %s (%s)\n      Buffer corrupted to '%c' (0x%02X) at offset %zu (%zu past end))\n", func, note, value, value, i, i-offset);
            print_buffer_context(buffer, size, offset, 20);
            g_fail_count++;
            break;
        }
    }
}

void assert_buffer_terminated(const char* buffer, size_t buffer_size, size_t where, const char* func, const char* note)
{
    if(where != (size_t)-1)
    {
        assert(where < buffer_size);  // where must be valid
        if(buffer[where] == '\0')
            return;

        fprintf(stderr, "FAIL: %s (%s)\n      Buffer is not terminated at offset %zu\n", func, note, where);
        print_buffer_context(buffer, buffer_size, where, 20);
    }
    else
    {
        for(size_t i = 0; i < buffer_size; i++)
        {
            if(buffer[i] == '\0')
                return;
        }

        fprintf(stderr, "FAIL: %s (%s)\n      Buffer is never terminated\n", func, note);
        print_buffer_context(buffer, buffer_size, buffer_size, 20);
    }

    g_fail_count++;
}
