
#include "testutil_strings.h"

#include <microstrain/strings.h>

#include <string.h>










int main()
{
    format_bytes_to_blank_unterminated_buffer_works();
    format_bytes_with_group1_works_and_has_no_extra_spaces();
    format_bytes_with_group2_works_and_has_no_extra_spaces();
    format_bytes_with_group2_works_with_partial_group();
    format_bytes_with_group4_works_and_has_no_extra_spaces();
    format_bytes_works_when_no_data();
    format_bytes_at_offset_works();
    format_bytes_at_offset_fails_gracefully_when_buffer_too_small();

    return (int)g_fail_count;
}