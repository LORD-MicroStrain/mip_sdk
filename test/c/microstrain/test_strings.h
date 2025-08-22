#pragma once

#include <stdbool.h>

#define STRINGIFY(x) #x
#define TEST_ASSERT(cond, note) assert_condition(cond, #cond, __func__, note)
#define TEST_ASSERT_EQ(a, b, note) assert_eq_zz(a, b, #a " == " #b, __func__, note)
#define TEST_ASSERT_BUFFER_COMPARE(buff, comp, count, note) assert_buffer_matches(buff, comp, count, __func__, note)
#define TEST_ASSERT_BUFFER_TERMINATED(buff, size, where, note) assert_buffer_terminated(buff, size, where, __func__, note)
#define TEST_ASSERT_BUFFER_NOT_OVERRUN(buff, size, offset, note) assert_buffer_not_overrun(buff, size, offset, __func__, note)

void print_buffer(const char* buffer, size_t buffer_size);
void print_buffer_context(const char* buffer, size_t buffer_size, size_t where, size_t context);
void assert_condition(bool cond, const char* cond_str, const char* func, const char* note);
void assert_eq_zz(size_t a, size_t b, const char* cond_str, const char* func, const char* note);
void assert_buffer_matches(const char* buffer, const char* compare, size_t count, const char* func, const char* note);
void assert_buffer_not_overrun(const char* buffer, size_t size, size_t offset, const char* func, const char* note);
void assert_buffer_terminated(const char* buffer, size_t buffer_size, size_t where, const char* func, const char* note);

extern unsigned int g_fail_count;
