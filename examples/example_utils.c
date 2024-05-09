#include "example_utils.h"


void displayFilterState(const mip_filter_mode filter_state, char **current_state, bool isFiveSeries) 
{
    char *read_state = "";
    if (filter_state == MIP_FILTER_MODE_INIT) {
        read_state = false ? "GX5_INIT (1)" : "INIT (1)";
    } else if (filter_state == MIP_FILTER_MODE_VERT_GYRO) {
        read_state = false ? "GX5_RUN_SOLUTION_VALID (2)" : "VERT_GYRO (2)";
    } else if (filter_state == MIP_FILTER_MODE_AHRS) {
        read_state = false ? "GX5_RUN_SOLUTION_ERROR (3)" : "AHRS (3)";
    } else if (filter_state == MIP_FILTER_MODE_FULL_NAV) {
        read_state = "FULL_NAV (4)";
    } else {
        read_state = "STARTUP (0)";
    }

    if (strcmp(read_state, *current_state) != 0) {
        printf("FILTER STATE: %s\n", read_state);
        *current_state = read_state;
    }
}